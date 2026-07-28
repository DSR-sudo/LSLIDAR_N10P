// 网络输入基类及 UDP 控制帧模块。
// Input 只保存设备地址、端口和型号等连接信息；数据接收由 socket/pcap 子类完成。
#include "lslidar_driver/input.h"

#include <cstdio>
#include <cstring>

extern volatile sig_atomic_t flag;

namespace lslidar_driver
{
namespace
{
constexpr size_t kInputPacketSize = 400;
}

Input::Input(rclcpp::Node *private_nh, uint16_t port)
	: private_nh_(private_nh), port_(port), cur_rpm_(0), return_mode_(1),
	  npkt_update_flag_(false), add_multicast(false), group_ip("224.1.1.2"),
	  UDP_PORT_NUMBER_DIFOP(2369), socket_id_difop(0), sockfd_(-1),
	  devip_str_difop("192.168.1.200")
{
	devip_str_ = "192.168.1.102";
	lidar_name = "M10";
	private_nh->declare_parameter<std::string>("device_ip", devip_str_);
	private_nh->declare_parameter<std::string>("device_ip_difop", devip_str_difop);
	private_nh->declare_parameter<bool>("add_multicast", false);
	private_nh->declare_parameter<std::string>("group_ip", group_ip);
	private_nh->declare_parameter<int>("difop_port", UDP_PORT_NUMBER_DIFOP);
	private_nh->get_parameter("lidar_name", lidar_name);
	private_nh->get_parameter("device_ip", devip_str_);
	private_nh->get_parameter("device_ip_difop", devip_str_difop);
	private_nh->get_parameter("add_multicast", add_multicast);
	private_nh->get_parameter("group_ip", group_ip);
	private_nh->get_parameter("difop_port", UDP_PORT_NUMBER_DIFOP);
	if (!devip_str_.empty())
		RCLCPP_INFO(private_nh->get_logger(), "[driver][input] accepting packets from IP %s port %d",
			devip_str_.c_str(), port);
}

void Input::UDP_difop()
{
	sockaddr_in server{};
	server.sin_family = AF_INET;
	server.sin_port = htons(UDP_PORT_NUMBER_DIFOP);
	server.sin_addr.s_addr = inet_addr(devip_str_.c_str());
	for (int retry = 0; retry < 10; ++retry) {
		unsigned char data[188] = {};
		data[0] = 0xa5; data[1] = 0x5a; data[2] = 0x55;
		data[184] = 0x08; data[185] = 0x01; data[186] = 0xfa; data[187] = 0xfb;
		if (sendto(sockfd_, data, 188, 0, reinterpret_cast<sockaddr *>(&server), sizeof(server)) >= 0)
			return;
	}
	std::printf("start scan error !\n");
}

void Input::UDP_order(const std_msgs::msg::Int8 msg)
{
	const int order = msg.data;
	sockaddr_in server{};
	server.sin_family = AF_INET;
	server.sin_port = htons(UDP_PORT_NUMBER_DIFOP);
	server.sin_addr.s_addr = inet_addr(devip_str_.c_str());
	for (int retry = 0; retry < 10; ++retry) {
		unsigned char data[188] = {};
		data[0] = 0xa5; data[1] = 0x5a; data[2] = 0x55; data[186] = 0xfa; data[187] = 0xfb;
		if (lidar_name == "M10" || lidar_name == "M10_GPS" || lidar_name == "M10_P") {
			if (order <= 1) { data[184] = 1; data[185] = order; }
			else if (order >= 2 && order <= 4) { data[181] = 8 + order; data[184] = 6; data[185] = 1; }
			else if (order == 100) { data[184] = 8; data[185] = 1; }
			else return;
		} else if (lidar_name == "M10_PLUS") {
			data[184] = 0x0a; data[185] = 1;
			if (order == 5) { data[141] = 1; data[142] = 0x2c; }
			else if (order == 6) { data[141] = 1; data[142] = 0x68; }
			else if (order == 8) { data[141] = 1; data[142] = 0xe0; }
			else if (order == 10) { data[141] = 2; data[142] = 0x58; }
			else if (order == 12) { data[141] = 2; data[142] = 0xd0; }
			else if (order == 15) { data[141] = 3; data[142] = 0x84; }
			else if (order == 20) { data[141] = 4; data[142] = 0xb0; }
			else if (order <= 1) { data[184] = 1; data[185] = order; }
			else if (order == 100) { data[184] = 8; data[185] = 1; }
			else return;
		} else if (lidar_name == "N10" || lidar_name == "N10_P") {
			if (order <= 1) { data[184] = 1; data[185] = order; }
			else if (order >= 6 && order <= 12) { data[172] = order; data[184] = 0x0a; data[185] = 1; }
			else return;
		}
		if (sendto(sockfd_, data, 188, 0, reinterpret_cast<sockaddr *>(&server), sizeof(server)) >= 0) {
			if (order == 1) usleep(3000000);
			return;
		}
	}
	std::printf("start scan error !\n");
}

} // namespace lslidar_driver
