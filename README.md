# LSLIDAR N10P ROS 2 驱动

本目录是镭神激光雷达的 ROS 2 驱动工作区，当前重点支持 LSN10P/N10P，并保留
M10、M10_GPS、M10_P、M10_PLUS、N10、L10 和 M10_DOUBLE 等型号的兼容路径。
工程使用 `ament_cmake`，由两个 ROS 2 包组成：

- `lslidar_msgs`：雷达原始包、扫描、双回波点云和 LCP 调试消息定义。
- `lslidar_driver`：网络、PCAP、串口输入，协议解析，ROS 消息发布以及 LCP 建系。

## 处理流程

```text
UDP / PCAP / UART
        │
        ▼
输入层（只负责取得原始字节）
        │
        ▼
polling()：帧头同步、长度判断、CRC 校验、DIFOP 分流
        │
        ├─ 普通型号解析 → 通用扫描发布
        │
        └─ N10PModule：108 字节 N10P 包 → 16 组双回波 → 累积完整 360° 扫描
                                                        │
                                                        ▼
                              发布线程复制完整扫描（与解析线程解耦）
                                  ├─ LaserScan（首回波）
                                  ├─ PointCloud2（首回波 + 次回波）
                                  └─ LcpCore（矩形环境建系与跟踪）
```

输入层不理解雷达字段；`N10PModule` 是不依赖 ROS 的纯 C++ 协议模块。解析线程
完成一圈后，通过互斥锁和条件变量把扫描快照交给发布线程，避免 ROS 发布或 LCP
几何计算读取正在变化的半圈缓存。

## N10P 协议逻辑

N10P 每包固定 108 字节：

```text
0..1       5..6       7..102                              105..106  107
帧头       包首角度   16 × [首回波距离/强度 + 次回波距离/强度]  包尾角度  CRC
```

- 距离单位为毫米，角度为百分之一度，字段按大端读取。
- CRC 是前 107 字节累加和的低 8 位。
- 无效距离 `0xffff` 不进入首回波累积，但保留包内角度采样顺序。
- 角度由包首、包尾线性插值得到；检测到 `359° → 0°` 后输出上一圈。
- 首回波存放在累积区起始位置，次回波存放在偏移 3000 的区域，以兼容原有双回波发布布局。
- 量程和可用角度窗口在整圈边界确认后统一过滤。

## ROS 接口

默认参数文件中的 N10P 运行接口如下：

| 类型 | 接口 | 说明 |
| --- | --- | --- |
| 发布 | `/scan` (`sensor_msgs/LaserScan`) | 首回波扫描，可由 `pubScan` 控制 |
| 发布 | `/lslidar_point_cloud` (`sensor_msgs/PointCloud2`) | 首、次回波点云，可由 `pubPointCloud2` 控制 |
| 订阅 | `lslidar_order` (`std_msgs/Int8`) | 雷达开关控制，具体行为由型号和输入方式决定 |
| 发布 | `/lcp/status` (`std_msgs/UInt8`) | LCP 状态：0 等待、1 建系、2 地图锁定、3 不健康 |
| 发布 | `/lcp/odometry` (`nav_msgs/Odometry`) | LCP 锁定后的 XY 与 yaw |
| 发布 | `/lcp/yaw` (`std_msgs/Float32`) | LCP 锁定后的偏航角（弧度） |
| 发布 | `/lcp/debug` (`lslidar_msgs/LcpDebug`) | 四面墙距离、地图尺寸和当前姿态 |
| 服务 | `/lcp/start_initialization` (`std_srvs/Trigger`) | 请求清空旧地图并开始建系 |

雷达坐标系由 `frame_id` 配置，默认参数文件为 `laser`（代码默认值为 `laser_link`，
以参数文件为准）。点云使用 `x` 向前、`y` 向左；协议角度方向会在发布时转换为 ROS
角度索引。

## LCP 建系与跟踪

LCP 将每一圈扫描转换为平面点，过滤局部邻域中的远端回波，然后拟合四面矩形墙面。
建系需要：

1. 飞控已连接、未解锁，并报告 `ON_GROUND`。
2. 通过 `/lcp/start_initialization` 成功发起请求。
3. MAVROS 状态、IMU 姿态和扫描数据保持新鲜，横滚/俯仰不超过配置阈值。
4. 默认连续 20 圈满足四墙数量、残差和尺寸稳定性要求。

默认 `lcp_initial_heading_is_north=true` 时，初始化前应将雷达/机体前方对准真北。
输出坐标为 `lcp_nwu`：`+X` 向北、`+Y` 向西/左、`+Z` 向上，初始化时 yaw 为 0。
关闭该选项时输出 frame 为 `lcp_map`。

状态 3 表示当前姿态、数据新鲜度或几何匹配失败；已锁定地图不会因此自动清除，恢复
健康后可以回到状态 2。只有在地面安全条件满足时再次调用初始化服务，才会清空旧地图。

## 构建

```bash
cd /home/pi/LSLIDARN10P
source /opt/ros/<你的 ROS 2 发行版>/setup.bash
colcon build --symlink-install --packages-select lslidar_msgs lslidar_driver
source install/setup.bash
```

主要依赖包括 `rclcpp`、`sensor_msgs`、`nav_msgs`、`mavros_msgs`、`PCL`、`libpcap`、
`Boost::thread`、`diagnostic_updater` 和 `rosidl_default_generators`。

## 启动

网络输入（使用 `2368/2369` 端口及网络参数）：

```bash
ros2 launch lslidar_driver lsn10p_net_launch.py
```

串口输入（默认读取 `/dev/ttyAMA0`，可在参数文件中修改）：

```bash
ros2 launch lslidar_driver lsn10p_launch.py
```

对应参数文件分别为：

- `lslidar_driver/params/lidar_net_ros2/lsn10p_net.yaml`
- `lslidar_driver/params/lidar_uart_ros2/lsn10p.yaml`

常用配置：

| 参数 | 作用 |
| --- | --- |
| `interface_selection` | `net` 使用 UDP，`serial` 使用串口或文本回放 |
| `lidar_name` | 型号选择；N10P 使用 `N10_P` |
| `device_ip` / `msop_port` | 网络雷达源地址和 MSOP 端口 |
| `serial_port_` | 串口设备路径 |
| `min_range` / `max_range` | 距离过滤范围，单位米 |
| `angle_disable_min` / `angle_disable_max` | 角度屏蔽区；相等时表示不裁剪 |
| `pubScan` / `pubPointCloud2` | 控制两类输出 |
| `pcap` | 网络 PCAP 离线回放文件；为空时使用实时 UDP |
| `in_file_name` | 非网络模式下的文本输入文件 |

## 测试

测试包含 LCP 几何状态机、现场扫描回放和 N10P 协议解码：

```bash
cd /home/pi/LSLIDARN10P
source /opt/ros/<你的 ROS 2 发行版>/setup.bash
colcon build --symlink-install --packages-select lslidar_msgs lslidar_driver
colcon test --packages-select lslidar_driver
colcon test-result --verbose
```

现场回放数据位于 `lslidar_driver/test/data/`；N10P 协议模块本身不依赖 ROS，适合
单独进行 CRC、包长、双回波和跨零点测试。

## 目录结构

```text
src/
├── lslidar_msgs/                         ROS 消息定义
├── lslidar_driver/
│   ├── include/lslidar_driver/           驱动、N10P 和 LCP 公共头文件
│   ├── src/input/                        UDP、PCAP 输入
│   ├── src/serial/                       串口输入
│   ├── src/n10p/                         N10P 纯协议解析
│   ├── src/driver/                       ROS 驱动编排和发布
│   ├── src/lcp/                          纯 C++ LCP 几何核心
│   ├── launch/                           启动文件
│   ├── params/                           网络/串口参数文件
│   ├── test/                             单元测试和现场回放数据
│   └── docs/n10p_data_flow.md            N10P 详细数据流
├── version.txt
└── wheeltec_udev.sh
```

## 注意事项

- 修改 `lidar_name`、输入方式或量程后应同步检查对应参数文件，避免旧配置与协议不匹配。
- N10P 的完整一圈由解析线程累积、发布线程消费；不要在协议模块中直接创建或发布 ROS 消息。
- LCP 依赖真实 MAVROS 状态、ExtendedState 和 IMU 数据。没有飞控状态时只能发布雷达数据，
  不能通过初始化服务进入有效建系状态。
- 运行真实飞行控制前，应单独核验传感器来源、坐标系方向、PX4/MAVROS 配置和安全区域。
