/*
 * This file is part of lslidar driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

// 进程入口只负责创建 ROS 节点和驱动轮询，不承载协议解析业务。
// polling() 每次处理一个输入包；spin_some() 让参数、服务和 MAVROS
// 回调在同一进程内得到调度。
#include "rclcpp/rclcpp.hpp"
#include "lslidar_driver/lslidar_driver.h"

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lslidar_driver::LslidarDriver>();
  // DF-00：主线程交替执行一个雷达包轮询和一次 ROS 回调处理。
  // 雷达包由 polling() 向下游传递，服务/MAVROS 回调由 spin_some() 更新状态缓存。
  while (rclcpp::ok() && node->polling()) {
        rclcpp::spin_some(node);
  }
  //rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
