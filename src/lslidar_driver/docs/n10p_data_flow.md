# N10P 数据流说明

本文说明 LSN10P/N10P 从雷达原始字节到 ROS 话题和 LCP 本地位姿的完整路径。
核心原则是：传输层只搬运字节，协议模块只解码 N10P，ROS 发布线程才生成消息。

## 总体流程

```text
雷达
  │
  ├─ UDP 实时 ──> input/InputSocket::getPacket()
  ├─ PCAP 回放 ─> input/InputPCAP::getPacket()
  └─ UART 串口 ─> serial/LSIOSR::read()
                         │
                         ▼
              driver/LslidarDriver::polling()
              1. 帧头同步/长度确定
              2. 复制到 packet_bytes[500]
              3. CRC 校验和 DIFOP 分流
                         │
                         ▼
              driver::data_processing_2()
                         │
              lidar_name == N10_P
                         │
                         ▼
              n10p/N10PModule::processPacket()
              1. 读取 5/105 包首包尾角度
              2. 读取 16 组双回波
              3. 距离 mm -> m，角度插值
              4. 检测 359° -> 0° 整圈边界
                         │
              未完成一圈 ─┴─> 继续缓存下一包
                         │
              完成一圈
                         ▼
              publishN10PSweep()
              解析线程 ──mutex_──> scan_points_bak_
                        └─condition_variable──> 发布线程
                         │
                         ▼
              driver::pubScanThread()
              1. getScan() 复制完整一圈
              2. N10P 调用 processLcpScan()
              3. 生成 /scan
              4. 生成点云
                         │
                         ├─> /scan
                         ├─> /lslidar_point_cloud
                         └─> LcpCore -> /lcp/status
                                      ├─> /lcp/odometry
                                      ├─> /lcp/yaw
                                      └─> /lcp/debug
```

## 1. 输入层

`InputSocket`、`InputPCAP` 和 `LSIOSR` 的职责都只是返回一段原始字节：

- UDP：等待可读事件，检查来源 IP，写入 `LslidarPacket::data`。
- PCAP：读取 UDP 负载，去除抓包头后写入同一个 `data` 缓冲区。
- UART：按请求读取字节，处理串口超时和重连，不理解 N10P 字段。

因此上层 `polling()` 可以统一处理三种输入，不需要在协议模块里判断数据来源。

## 2. `polling()` 帧处理

网络输入有些设备会把 `0xA5` 帧头剥掉，只留下以 `0x5A` 开头的数据。
`polling()` 会把数据整体右移一字节并补回 `0xA5`，随后 N10P 固定使用 108 字节。

串口输入由 `receive_data()` 逐步同步 `0xA5 0x5A`，N10P 不读取通用型号的动态长度，
直接收满 108 字节。

CRC 通过后，普通设备包进入 `data_processing()`；N10P 和 M10_DOUBLE 进入
`data_processing_2()`。DIFOP 只更新角度补偿，不进入扫描点累积。

## 3. N10P 包解码

每个 N10P 包布局如下：

```text
0       1       5..6       7..102                         105..106  107
┌───────┬───────┬──────────┬──────────────────────────────┬──────────┬─────┐
│  A5   │  5A   │ start°  │ 16 × [r1(2),i1(1),r2(2),i2(1)]│ end°     │ CRC │
└───────┴───────┴──────────┴──────────────────────────────┴──────────┴─────┘
```

角度是百分之一度的大端整数，距离是毫米的大端整数。`N10PModule` 不修改输入包，
也不释放 `packet_bytes`；输入缓冲区的所有权始终由 `polling()` 持有。

模块内部维护 6000 个 `ScanPoint`：

- `[0, point_count)`：首回波。
- `[3000, 3000 + point_count)`：次回波。

当当前角度小于上一角度、当前角度小于 5° 且上一角度大于 355° 时，认为完成一圈。
当前“跨零点”的点属于下一圈，上一圈只输出它之前的点，保持原驱动兼容行为。

## 4. 线程交接

协议解析发生在 `polling()` 所在线程，不能在这里构造和发布 ROS 消息。
完整一圈完成后：

1. `publishN10PSweep()` 在 `mutex_` 保护下替换 `scan_points_bak_`。
2. 设置 `count_num` 和扫描时间 `pre_time_`。
3. 释放 `mutex_` 后调用 `pubscan_cond_.notify_one()`。
4. `pubScanThread()` 被唤醒，用 `getScan()` 复制一份本圈数据。

这样解析线程可以继续接收下一包，发布线程也不会读取正在变化的半圈缓存。

## 5. 发布和 LCP

发布线程使用同一份完整扫描：

- `/scan`：首回波按 `360 - degree` 映射到 ROS 角度索引。
- `/lslidar_point_cloud`：首回波和次回波都转换为 `x=range*cos(angle)`、
  `y=-range*sin(angle)`，并保留强度和时间。
- LCP：N10P 先把完整扫描交给 `LcpCore::processScan()`，再发布 `/scan`，
  因此建系和对应的扫描发布处在同一个发布周期。

LCP 的 MAVROS 状态由 ROS 回调缓存，`processLcpScan()` 复制状态快照后交给 Core：

```text
/mavros/state ─────────────┐
/mavros/extended_state ────┼─> mavros_snapshot_ -> LcpCore
/mavros/imu/data ──────────┘
```

只有服务在“飞控已连接、未解锁、ON_GROUND、状态新鲜”时才会清空旧地图并进入
STATUS=1。锁定地图后，倾斜、超时或矩形匹配失败进入 STATUS=3，但不会重建坐标轴；
恢复健康并重新匹配后恢复 STATUS=2。
