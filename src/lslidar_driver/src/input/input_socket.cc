// 实时 UDP 输入：负责 socket、绑定端口、可选组播和来源 IP 过滤。
// 本文件不判断 N10P 的 108 字节布局，避免网络层和协议层耦合。
#include "lslidar_driver/input.h"

#include <cstdio>
#include <cstring>

extern volatile sig_atomic_t flag;

namespace lslidar_driver
{

InputSocket::InputSocket(rclcpp::Node *private_nh, uint16_t port) : Input(private_nh, port)
{
	if (!devip_str_.empty()) {
		inet_aton(devip_str_.c_str(), &devip_);
		inet_aton(devip_str_difop.c_str(), &devip_difop);
	}
	RCLCPP_INFO(private_nh_->get_logger(), "[driver][socket] Opening UDP socket: port %d", port);
	sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
	if (sockfd_ < 0) { perror("socket"); return; }
	int option = 1;
	if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option)) < 0) {
		perror("setsockopt error"); return;
	}
	sockaddr_in address{};
	address.sin_family = AF_INET;
	address.sin_port = htons(port);
	address.sin_addr.s_addr = INADDR_ANY;
	if (bind(sockfd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
		perror("bind"); return;
	}
	if (add_multicast) {
		ip_mreq group{};
		group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
		group.imr_interface.s_addr = htonl(INADDR_ANY);
		if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &group, sizeof(group)) < 0) {
			perror("Adding multicast group error");
			close(sockfd_);
			return;
		}
	}
	if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
		perror("non-block");
}

InputSocket::~InputSocket()
{
	if (sockfd_ >= 0) close(sockfd_);
}

int InputSocket::getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet)
{
	// socket 只负责等待和来源过滤；帧头、包长、CRC 由 driver polling 层处理。
	pollfd descriptor{sockfd_, POLLIN, 0};
	while (flag == 1) {
		const int poll_result = poll(&descriptor, 1, 2000);
		if (poll_result < 0) {
			if (errno != EINTR) RCLCPP_ERROR(private_nh_->get_logger(), "poll() error: %s", strerror(errno));
			return 0;
		}
		if (poll_result == 0) {
			RCLCPP_WARN(private_nh_->get_logger(), "lslidar poll() timeout, port: %d", port_);
			return 0;
		}
		if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
			RCLCPP_ERROR(private_nh_->get_logger(), "poll() reports lslidar error");
			return 0;
		}
		if (!(descriptor.revents & POLLIN)) continue;
		sockaddr_in sender{};
		socklen_t sender_length = sizeof(sender);
		const ssize_t bytes = recvfrom(sockfd_, packet->data.data(), 400, 0,
			reinterpret_cast<sockaddr *>(&sender), &sender_length);
		if (bytes < 0) {
			if (errno != EWOULDBLOCK) {
				perror("recvfail");
				return 1;
			}
			continue;
		}
		if (bytes >= 50 && bytes <= 400
			&& (devip_str_.empty() || sender.sin_addr.s_addr == devip_.s_addr))
			return static_cast<int>(bytes);
	}
	if (flag == 0) abort();
	return 0;
}

} // namespace lslidar_driver
