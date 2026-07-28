// 驱动生命周期模块：构造节点、启动初始化、在析构时停止扫描发布线程。
// 具体参数、协议、扫描和 LCP 逻辑分散在同目录的其他编译单元中，
// 这样每个文件都只负责一种运行阶段。
#include "lslidar_driver/lslidar_driver.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>

namespace lslidar_driver
{

namespace
{
void signalHandler(int signal_number)
{
	std::printf("sig: %d", signal_number);
	std::abort();
}
} // namespace

LslidarDriver::LslidarDriver() : LslidarDriver(rclcpp::NodeOptions()) {}

LslidarDriver::LslidarDriver(const rclcpp::NodeOptions &options)
	: Node("lslidar_driver_node", options), diagnostics(this)
{
	std::signal(SIGINT, signalHandler);
	if (!initialize())
		RCLCPP_ERROR(get_logger(), "Could not initialize the driver...");
	else
		RCLCPP_INFO(get_logger(), "Successfully initialized driver...");
}

LslidarDriver::~LslidarDriver()
{
	shutting_down_.store(true);
	pubscan_cond_.notify_all();
	if (pubscan_thread_ != nullptr) {
		pubscan_thread_->join();
		delete pubscan_thread_;
		pubscan_thread_ = nullptr;
	}
}

} // namespace lslidar_driver
