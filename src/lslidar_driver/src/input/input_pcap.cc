// PCAP 离线输入：把 UDP 负载复制到统一的 LslidarPacket 缓冲区。
// 上层 polling() 不需要区分实时网卡和抓包文件，仍按同一协议流程校验和解析。
#include "lslidar_driver/input.h"

#include <cstring>

extern volatile sig_atomic_t flag;

namespace lslidar_driver
{
namespace
{
constexpr size_t kInputPacketSize = 400;
}

InputPCAP::InputPCAP(rclcpp::Node *private_nh, uint16_t port, double packet_rate,
	std::string filename)
	: Input(private_nh, port), packet_rate_(packet_rate), filename_(filename), pcap_(nullptr),
	  empty_(true), read_once_(false), read_fast_(false), repeat_delay_(0.0)
{
	private_nh->get_parameter("read_once", read_once_);
	private_nh->get_parameter("read_fast", read_fast_);
	private_nh->get_parameter("repeat_delay", repeat_delay_);
	RCLCPP_INFO(private_nh->get_logger(), "Opening PCAP file %s", filename_.c_str());
	pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
	if (pcap_ == nullptr) {
		RCLCPP_WARN(private_nh->get_logger(), "Error opening lslidar socket dump file.");
		return;
	}
	std::stringstream filter;
	if (!devip_str_.empty()) filter << "src host " << devip_str_ << "&&";
	filter << "udp dst port " << port;
	pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
}

InputPCAP::~InputPCAP()
{
	if (pcap_ != nullptr) pcap_close(pcap_);
}

int InputPCAP::getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet)
{
	// PCAP 每次返回一个 UDP 负载，保持和 InputSocket 相同的 packet->data 布局。
	pcap_pkthdr *header = nullptr;
	const u_char *data = nullptr;
	while (flag == 1) {
		const int result = pcap_next_ex(pcap_, &header, &data);
		if (result >= 0) {
			if (!devip_str_.empty() && pcap_offline_filter(&pcap_packet_filter_, header, data) == 0)
				continue;
			if (!read_fast_) packet_rate_.sleep();
			std::memcpy(packet->data.data(), data + 42, kInputPacketSize);
			empty_ = false;
			return 0;
		}
		if (empty_) {
			RCLCPP_WARN(private_nh_->get_logger(), "Error %d reading lslidar packet: %s",
				result, pcap_geterr(pcap_));
			return -1;
		}
		if (read_once_) {
			RCLCPP_WARN(private_nh_->get_logger(), "end of file reached -- done reading.");
			return -1;
		}
		if (repeat_delay_ > 0.0) {
			RCLCPP_WARN(private_nh_->get_logger(), "end of file reached -- delaying %.3f seconds.", repeat_delay_);
			usleep(static_cast<useconds_t>(repeat_delay_ * 1000000.0));
		}
		RCLCPP_WARN(private_nh_->get_logger(), "replaying lslidar dump file");
		pcap_close(pcap_);
		pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
		empty_ = true;
	}
	if (flag == 0) abort();
	return 0;
}

} // namespace lslidar_driver
