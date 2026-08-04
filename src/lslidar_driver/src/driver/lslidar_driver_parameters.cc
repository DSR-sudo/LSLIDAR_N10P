// 参数模块按三层加载配置：通用驱动参数、LCP 质量门限、型号协议参数。
// N10P 的字节偏移只在最后一段绑定到 N10PModule，避免散落在 ROS 参数代码中。
#include "lslidar_driver/lslidar_driver.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace lslidar_driver
{

bool LslidarDriver::loadParameters()
{
	interface_selection = "net";
	frame_id = "laser_link";
	scan_topic = "/scan";
	lidar_name = "M10";
	pointcloud_topic = "/lslidar_point_cloud";
	is_start = true;
	min_range = 0.3;
	max_range = 100.0;
	use_gps_ts = true;
	compensation = true;
	pubScan = true;
	pubPointCloud2 = true;
	angle_disable_min = 0.0;
	angle_disable_max = 0.0;

	this->declare_parameter<std::string>("lidar_name", "M10");
	this->declare_parameter<std::string>("frame_id", "laser_link");
	this->declare_parameter<std::string>("scan_topic", "/scan");
	this->declare_parameter<std::string>("pointcloud_topic", "/lslidar_point_cloud");
	this->declare_parameter<double>("min_range", 0.3);
	this->declare_parameter<double>("max_range", 100.0);
	this->declare_parameter<bool>("use_gps_ts", false);
	this->declare_parameter<bool>("high_reflection", false);
	this->declare_parameter<bool>("compensation", false);
	this->declare_parameter<bool>("pubScan", true);
	this->declare_parameter<bool>("pubPointCloud2", false);
	this->declare_parameter<double>("angle_disable_min", 0.0);
	this->declare_parameter<double>("angle_disable_max", 0.0);
	this->declare_parameter<std::string>("interface_selection", "net");
	this->declare_parameter<bool>("lcp_enabled", true);
	this->declare_parameter<bool>("lcp_initial_heading_is_north", true);
	this->declare_parameter<int>("lcp_init_scans", 20);
	this->declare_parameter<double>("lcp_nearest_return_window_deg", 8.0);
	this->declare_parameter<double>("lcp_nearest_return_margin_m", 0.35);
	this->declare_parameter<double>("lcp_tilt_max_deg", 15.0);
	this->declare_parameter<int>("lcp_attitude_timeout_ms", 500);
	this->declare_parameter<int>("lcp_flight_state_timeout_ms", 2000);
	this->declare_parameter<int>("lcp_scan_timeout_ms", 500);
	this->declare_parameter<int>("lcp_min_valid_points", 24);
	this->declare_parameter<int>("lcp_min_wall_points", 3);
	this->declare_parameter<int>("lcp_min_quadrant_points", 3);
	this->declare_parameter<int>("lcp_max_fit_points", 360);
	this->declare_parameter<double>("lcp_wall_inlier_threshold_m", 0.15);
	this->declare_parameter<double>("lcp_max_wall_residual_m", 0.10);
	this->declare_parameter<double>("lcp_stability_threshold_m", 0.15);
	this->declare_parameter<double>("lcp_stability_size_ratio", 0.05);
	this->declare_parameter<double>("lcp_map_match_threshold_m", 0.30);
	this->declare_parameter<double>("lcp_map_size_tolerance_m", 0.30);
	this->declare_parameter<double>("lcp_map_size_tolerance_ratio", 0.10);
	this->declare_parameter<std::string>("mavros_state_topic", "/mavros/state");
	this->declare_parameter<std::string>("mavros_extended_state_topic", "/mavros/extended_state");
	this->declare_parameter<std::string>("mavros_imu_topic", "/mavros/imu/data");
	this->declare_parameter<std::string>("lcp_mavros_state_topic", "");
	this->declare_parameter<std::string>("lcp_mavros_extended_state_topic", "");
	this->declare_parameter<std::string>("lcp_mavros_imu_topic", "");

	this->get_parameter("lidar_name", lidar_name);
	this->get_parameter("frame_id", frame_id);
	this->get_parameter("scan_topic", scan_topic);
	this->get_parameter("pointcloud_topic", pointcloud_topic);
	this->get_parameter("min_range", min_range);
	this->get_parameter("max_range", max_range);
	this->get_parameter("use_gps_ts", use_gps_ts);
	this->get_parameter("high_reflection", high_reflection);
	this->get_parameter("compensation", compensation);
	this->get_parameter("pubScan", pubScan);
	this->get_parameter("pubPointCloud2", pubPointCloud2);
	this->get_parameter("angle_disable_min", angle_disable_min);
	this->get_parameter("angle_disable_max", angle_disable_max);
	this->get_parameter("interface_selection", interface_selection);

	// LCP 的参数先独立读取并创建 Core；Core 只接收普通 C++ 数据，
	// MAVROS 回调和 ROS publisher 在 driver/lslidar_driver_lcp.cc 中使用这些对象。
	lcp::Config lcp_config;
	int init_scans = static_cast<int>(lcp_config.init_scans);
	int min_valid = static_cast<int>(lcp_config.min_valid_points);
	int min_wall = static_cast<int>(lcp_config.min_wall_points);
	int min_quadrant = static_cast<int>(lcp_config.min_quadrant_points);
	int max_fit = static_cast<int>(lcp_config.max_fit_points);
	double nearest_return_window_deg = lcp_config.nearest_return_window_deg;
	double nearest_return_margin_m = lcp_config.nearest_return_margin_m;
	double tilt_deg = 15.0;
	int attitude_timeout = 500;
	int flight_timeout = 2000;
	int scan_timeout = 500;
	this->get_parameter("lcp_enabled", lcp_config.enabled);
	this->get_parameter("lcp_initial_heading_is_north", lcp_config.initial_heading_is_north);
	this->get_parameter("lcp_init_scans", init_scans);
	this->get_parameter("lcp_nearest_return_window_deg", nearest_return_window_deg);
	this->get_parameter("lcp_nearest_return_margin_m", nearest_return_margin_m);
	this->get_parameter("lcp_tilt_max_deg", tilt_deg);
	this->get_parameter("lcp_attitude_timeout_ms", attitude_timeout);
	this->get_parameter("lcp_flight_state_timeout_ms", flight_timeout);
	this->get_parameter("lcp_scan_timeout_ms", scan_timeout);
	this->get_parameter("lcp_min_valid_points", min_valid);
	this->get_parameter("lcp_min_wall_points", min_wall);
	this->get_parameter("lcp_min_quadrant_points", min_quadrant);
	this->get_parameter("lcp_max_fit_points", max_fit);
	this->get_parameter("lcp_wall_inlier_threshold_m", lcp_config.wall_inlier_threshold_m);
	this->get_parameter("lcp_max_wall_residual_m", lcp_config.max_wall_residual_m);
	this->get_parameter("lcp_stability_threshold_m", lcp_config.stability_threshold_m);
	this->get_parameter("lcp_stability_size_ratio", lcp_config.stability_size_ratio);
	this->get_parameter("lcp_map_match_threshold_m", lcp_config.map_match_threshold_m);
	this->get_parameter("lcp_map_size_tolerance_m", lcp_config.map_size_tolerance_m);
	this->get_parameter("lcp_map_size_tolerance_ratio", lcp_config.map_size_tolerance_ratio);
	this->get_parameter("mavros_state_topic", mavros_state_topic_);
	this->get_parameter("mavros_extended_state_topic", mavros_extended_state_topic_);
	this->get_parameter("mavros_imu_topic", mavros_imu_topic_);
	std::string alias;
	this->get_parameter("lcp_mavros_state_topic", alias);
	if (!alias.empty()) mavros_state_topic_ = alias;
	this->get_parameter("lcp_mavros_extended_state_topic", alias);
	if (!alias.empty()) mavros_extended_state_topic_ = alias;
	this->get_parameter("lcp_mavros_imu_topic", alias);
	if (!alias.empty()) mavros_imu_topic_ = alias;

	lcp_config.init_scans = static_cast<unsigned>(std::max(1, init_scans));
	lcp_config.nearest_return_window_deg = std::max(0.0, nearest_return_window_deg);
	lcp_config.nearest_return_margin_m = std::max(0.0, nearest_return_margin_m);
	lcp_config.min_valid_points = static_cast<unsigned>(std::max(12, min_valid));
	lcp_config.min_wall_points = static_cast<unsigned>(std::max(2, min_wall));
	lcp_config.min_quadrant_points = static_cast<unsigned>(std::max(1, min_quadrant));
	lcp_config.max_fit_points = static_cast<unsigned>(std::max(72, max_fit));
	lcp_config.tilt_max_rad = tilt_deg * M_PI / 180.0;
	lcp_config.min_range_m = std::max(0.001, min_range);
	lcp_config.max_range_m = std::max(lcp_config.min_range_m, max_range);
	lcp_config.attitude_timeout_sec = std::max(0.001, attitude_timeout / 1000.0);
	lcp_config.flight_state_timeout_sec = std::max(0.001, flight_timeout / 1000.0);
	lcp_config.scan_timeout_sec = std::max(0.001, scan_timeout / 1000.0);
	lcp_core_.reset(new lcp::LcpCore(lcp_config));

	lcp_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("/lcp/odometry", 10);
	lcp_yaw_pub_ = create_publisher<std_msgs::msg::Float32>("/lcp/yaw", 10);
	lcp_status_pub_ = create_publisher<std_msgs::msg::UInt8>("/lcp/status", 10);
	lcp_debug_pub_ = create_publisher<lslidar_msgs::msg::LcpDebug>("/lcp/debug", 10);
	mavros_state_sub_ = create_subscription<mavros_msgs::msg::State>(mavros_state_topic_, 10,
		std::bind(&LslidarDriver::mavrosStateCallback, this, std::placeholders::_1));
	mavros_extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(mavros_extended_state_topic_, 10,
		std::bind(&LslidarDriver::mavrosExtendedStateCallback, this, std::placeholders::_1));
	// MAVROS publishes IMU data with BEST_EFFORT reliability.  State the
	// compatible profile explicitly rather than relying on a process default.
	const auto mavros_imu_qos = rclcpp::QoS(rclcpp::KeepLast(5))
		.best_effort()
		.durability_volatile();
	mavros_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(mavros_imu_topic_, mavros_imu_qos,
		std::bind(&LslidarDriver::mavrosImuCallback, this, std::placeholders::_1));
	lcp_start_service_ = create_service<std_srvs::srv::Trigger>("/lcp/start_initialization",
		std::bind(&LslidarDriver::startLcpCallback, this, std::placeholders::_1, std::placeholders::_2));

	// 角度裁剪采用“可用区间”表达：允许 angle_able_max 大于 360，
	// 用来表示从 angle_able_min 跨过 0° 后继续到下一圈的区间。
	while (angle_disable_min < 0) angle_disable_min += 360;
	while (angle_disable_max < 0) angle_disable_max += 360;
	while (angle_disable_min > 360) angle_disable_min -= 360;
	while (angle_disable_max > 360) angle_disable_max -= 360;
	if (angle_disable_max == angle_disable_min) {
		angle_able_min = 0;
		angle_able_max = 360;
	} else if (angle_disable_min < angle_disable_max) {
		angle_able_min = angle_disable_max;
		angle_able_max = angle_disable_min == 0 ? 360 : angle_disable_min + 360;
	} else {
		angle_able_min = angle_disable_max;
		angle_able_max = angle_disable_min;
	}

	// 6000 个元素分成两个区域：首回波从 0 开始，次回波从 3000 开始。
	// 这个布局是发布线程和旧 M10_DOUBLE 兼容所需，不是 N10P 包的原始布局。
	count_num = 0;
	scan_points_.assign(N10PModule::kAccumulatorSize, ScanPoint{});
	scan_points_bak_.assign(N10PModule::kAccumulatorSize, ScanPoint{});
	// 下面只绑定各型号的协议元数据；N10P 的具体解码不在此处实现。
	if (lidar_name == "M10") {
		use_gps_ts = false; PACKET_SIZE = 92; package_points = 42;
		data_bits_start = 6; degree_bits_start = 2; rpm_bits_start = 4;
		baud_rate_ = 460800; points_size_ = 1008;
	} else if (lidar_name == "M10_P") {
		PACKET_SIZE = 160; package_points = 70; data_bits_start = 8;
		degree_bits_start = 4; rpm_bits_start = 6; baud_rate_ = 500000; points_size_ = 2000;
	} else if (lidar_name == "M10_PLUS") {
		PACKET_SIZE = 104; package_points = 41; data_bits_start = 8;
		degree_bits_start = 4; rpm_bits_start = 6; points_size_ = 5000; baud_rate_ = 921600;
	} else if (lidar_name == "M10_GPS") {
		PACKET_SIZE = 102; package_points = 42; data_bits_start = 6;
		degree_bits_start = 2; rpm_bits_start = 4; baud_rate_ = 460800; points_size_ = 1008;
	} else if (lidar_name == "N10" || lidar_name == "L10") {
		PACKET_SIZE = 58; package_points = 16; data_bits_start = 7;
		degree_bits_start = 5; end_degree_bits_start = 55; baud_rate_ = 230400;
		points_size_ = 2000; use_gps_ts = false; compensation = false;
	} else if (lidar_name == "M10_DOUBLE") {
		PACKET_SIZE = 300; package_points = 70; data_bits_start = 8;
		degree_bits_start = 4; rpm_bits_start = 6; points_size_ = 3000; baud_rate_ = 921600;
	} else if (lidar_name == "N10_P") {
		// N10P: 108 字节、16 个双回波点，角度字段为包首/包尾各 2 字节。
		PACKET_SIZE = N10PModule::kPacketSize; package_points = N10PModule::kPointsPerPacket;
		data_bits_start = N10PModule::kPrimaryDataOffset;
		degree_bits_start = N10PModule::kStartDegreeOffset;
		end_degree_bits_start = N10PModule::kEndDegreeOffset;
		baud_rate_ = 460800; points_size_ = N10PModule::kMaxSweepPoints;
		use_gps_ts = false; compensation = false;
	}
	N10PModule::Config n10p_config;
	n10p_config.min_range = min_range; n10p_config.max_range = max_range;
	n10p_config.angle_able_min = angle_able_min; n10p_config.angle_able_max = angle_able_max;
	n10p_module_.configure(n10p_config);

	RCLCPP_INFO_STREAM(get_logger(), "Lidar is " << lidar_name);
	if (pubScan) scan_pub = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);
	if (pubPointCloud2) point_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
	difop_switch = create_subscription<std_msgs::msg::Int8>("lslidar_order", 1,
		std::bind(&LslidarDriver::lidar_order, this, std::placeholders::_1));
	return true;
}

} // namespace lslidar_driver
