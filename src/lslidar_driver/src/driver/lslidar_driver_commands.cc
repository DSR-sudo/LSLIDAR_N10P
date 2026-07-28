// 设备控制模块：封装启动/停止、DIFOP 和串口控制帧。
// N10P 没有独立 DIFOP 解码，但仍复用统一的 lslidar_order 控制入口。
#include "lslidar_driver/lslidar_driver.h"

#include <cstdio>
#include <unistd.h>

namespace lslidar_driver
{

void LslidarDriver::lidar_difop()
{
	// L10/N10/N10P 没有独立 DIFOP 配置流程。
	if (lidar_name == "L10" || lidar_name == "N10" || lidar_name == "N10_P")
		return;
	if (interface_selection == "net") {
		msop_input_->UDP_difop();
		return;
	}
	for (int retry = 0; retry < 10; ++retry) {
		unsigned char data[188] = {};
		data[0] = 0xa5; data[1] = 0x5a; data[2] = 0x55;
		data[184] = 0x08; data[185] = 0x01; data[186] = 0xfa; data[187] = 0xfb;
		if (serial_->send(reinterpret_cast<const char *>(data), 188) >= 0)
			return;
	}
	std::printf("start scan error !\n");
}

void LslidarDriver::lidar_order(const std_msgs::msg::Int8::SharedPtr msg)
{
	if (lidar_name == "L10")
		return;
	const int order = msg->data;
	is_start = order != 0;
	if (interface_selection == "net") {
		msop_input_->UDP_order(*msg);
		return;
	}
	for (int retry = 0; retry < 10; ++retry) {
		unsigned char data[188] = {};
		data[0] = 0xa5; data[1] = 0x5a; data[2] = 0x55;
		data[186] = 0xfa; data[187] = 0xfb;
		if (lidar_name == "M10" || lidar_name == "M10_GPS" || lidar_name == "M10_P"
			|| lidar_name == "M10_DOUBLE") {
			if (order <= 1) {
				data[184] = 0x01; data[185] = static_cast<unsigned char>(order);
			} else if (order >= 2 && order <= 4) {
				data[181] = static_cast<unsigned char>(8 + order);
				data[184] = 0x06; data[185] = is_start ? 1 : 0;
			} else if (order == 100) {
				data[184] = 0x08; data[185] = 0x01;
			} else {
				return;
			}
		} else if (lidar_name == "M10_PLUS") {
			data[184] = 0x0a; data[185] = 0x01;
			const unsigned char rpm[][2] = {{0, 0}, {0, 0}, {1, 0x2c}, {1, 0x68},
				{0, 0}, {2, 0x58}, {0, 0}, {2, 0xd0}, {0, 0}, {3, 0x84}, {0, 0}, {4, 0xb0}};
			if (order >= 5 && order <= 12) {
				data[141] = rpm[order - 1][0]; data[142] = rpm[order - 1][1];
			} else if (order <= 1) {
				data[184] = 0x01; data[185] = static_cast<unsigned char>(order);
			} else if (order == 100) {
				data[184] = 0x08; data[185] = 0x01;
			} else {
				return;
			}
		} else if (lidar_name == "N10" || lidar_name == "N10_P") {
			if (order <= 1) {
				data[184] = 0x01; data[185] = static_cast<unsigned char>(order);
			} else if (order >= 6 && order <= 12) {
				data[172] = static_cast<unsigned char>(order);
				data[184] = 0x0a; data[185] = 0x01;
			} else {
				return;
			}
		}
		if (serial_->send(reinterpret_cast<const char *>(data), 188) >= 0) {
			if (order == 1) usleep(1000000);
			is_start = order != 0;
			return;
		}
	}
	std::printf("start scan error !\n");
}

} // namespace lslidar_driver
