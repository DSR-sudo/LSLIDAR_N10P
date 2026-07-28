// ROS/传输资源模块：创建诊断对象、UDP/PCAP 输入或串口资源。
// 这里不解析雷达数据，只建立 polling() 后续使用的输入对象。
#include "lslidar_driver/lslidar_driver.h"

#include <cstdlib>
#include <fstream>

namespace lslidar_driver
{

bool LslidarDriver::createRosIO()
{
	UDP_PORT_NUMBER = 2368;
	declare_parameter<int>("msop_port", 2368);
	get_parameter("msop_port", UDP_PORT_NUMBER);
	RCLCPP_INFO_STREAM(get_logger(), "Opening UDP socket: port " << UDP_PORT_NUMBER);
	dump_file.clear();
	declare_parameter<std::string>("pcap", "");
	get_parameter("pcap", dump_file);
	diagnostics.setHardwareID("Lslidar");
	const double expected_frequency = 12 * 24;
	diag_max_freq = expected_frequency;
	diag_min_freq = expected_frequency;
	RCLCPP_INFO(get_logger(), "expected frequency: %.3f (Hz)", expected_frequency);
	using namespace diagnostic_updater;
	diag_topic.reset(new TopicDiagnostic(
		"lslidar_packets", diagnostics,
		FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10), TimeStampStatusParam()));
	int hz = lidar_name == "M10_P" ? 12 : (lidar_name == "M10_PLUS" ? 20 : 10);
	const double packet_rate = hz * 24;
	if (!dump_file.empty())
		msop_input_.reset(new InputPCAP(this, UDP_PORT_NUMBER, packet_rate, dump_file));
	else
		msop_input_.reset(new InputSocket(this, UDP_PORT_NUMBER));
	return true;
}

void LslidarDriver::open_serial()
{
	diagnostics.setHardwareID("Lslidar");
	serial_port_ = "/dev/ttyUSB0";
	declare_parameter<std::string>("serial_port_", "/dev/ttyUSB0");
	get_parameter("serial_port_", serial_port_);
	serial_ = LSIOSR::instance(serial_port_, baud_rate_);
	if (serial_->init() != 0) {
		RCLCPP_ERROR(get_logger(), "open_port %s ERROR!", serial_port_.c_str());
		rclcpp::shutdown();
		std::exit(0);
	}
	RCLCPP_INFO(get_logger(), "open_port %s OK!", serial_port_.c_str());
}

bool LslidarDriver::initialize()
{
	if (!loadParameters()) {
		RCLCPP_ERROR(get_logger(), "Cannot load all required ROS parameters...");
		return false;
	}
	if (interface_selection == "net") {
		if (!createRosIO()) {
			RCLCPP_ERROR(get_logger(), "Cannot create all ROS IO...");
			return false;
		}
	} else {
		in_file_name.clear();
		declare_parameter<std::string>("in_file_name", "");
		get_parameter("in_file_name", in_file_name);
		if (in_file_name.empty()) {
			open_serial();
		} else {
			std::ifstream reader(in_file_name);
			if (!reader.is_open()) {
				RCLCPP_ERROR(get_logger(), "Cannot open the input file");
				return false;
			}
		}
	}
	if (pubscan_thread_ == nullptr)
		pubscan_thread_ = new boost::thread(boost::bind(&LslidarDriver::pubScanThread, this));
	RCLCPP_INFO(get_logger(), "Initialised lslidar without error");
	return true;
}

} // namespace lslidar_driver
