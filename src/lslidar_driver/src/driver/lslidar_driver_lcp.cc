// LCP ROS 适配层：缓存 MAVROS 状态、接收初始化服务，并把纯 C++
// LcpCore 的结果转换为 /lcp/status、/lcp/odometry 和 /lcp/yaw。
// LcpCore 本身不依赖 ROS，因此锁和消息发布只在此文件处理。
#include "lslidar_driver/lslidar_driver.h"

#include <algorithm>
#include <cmath>

namespace lslidar_driver
{

double LslidarDriver::steadyNowSec()
{
	const auto now = std::chrono::steady_clock::now().time_since_epoch();
	return std::chrono::duration<double>(now).count();
}

void LslidarDriver::mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(mavros_mutex_);
	mavros_snapshot_.state.received = true;
	mavros_snapshot_.state.connected = msg->connected;
	mavros_snapshot_.state.armed = msg->armed;
	mavros_snapshot_.state.received_at_sec = steadyNowSec();
}

void LslidarDriver::mavrosExtendedStateCallback(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(mavros_mutex_);
	mavros_snapshot_.extended_state.received = true;
	mavros_snapshot_.extended_state.landed_state = msg->landed_state;
	mavros_snapshot_.extended_state.received_at_sec = steadyNowSec();
}

void LslidarDriver::mavrosImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
	const double x = msg->orientation.x;
	const double y = msg->orientation.y;
	const double z = msg->orientation.z;
	const double w = msg->orientation.w;
	const double norm = std::sqrt(x * x + y * y + z * z + w * w);
	if (!std::isfinite(norm) || norm < 1e-6)
		return;
	const double nx = x / norm;
	const double ny = y / norm;
	const double nz = z / norm;
	const double nw = w / norm;
	const double sin_roll = 2.0 * (nw * nx + ny * nz);
	const double cos_roll = 1.0 - 2.0 * (nx * nx + ny * ny);
	const double sin_pitch = std::max(-1.0, std::min(1.0, 2.0 * (nw * ny - nz * nx)));
	std::lock_guard<std::mutex> lock(mavros_mutex_);
	mavros_snapshot_.imu.received = true;
	mavros_snapshot_.imu.roll_rad = std::atan2(sin_roll, cos_roll);
	mavros_snapshot_.imu.pitch_rad = std::asin(sin_pitch);
	mavros_snapshot_.imu.received_at_sec = steadyNowSec();
}

void LslidarDriver::startLcpCallback(
	const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
	std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
	(void)request;
	// 服务只负责发起一次建系请求；是否接受由最新的 MAVROS 连接、解锁和
	// ON_GROUND 状态决定。地图清空操作封装在 LcpCore 的互斥区内。
	lcp::MavrosSnapshot snapshot;
	{
		std::lock_guard<std::mutex> lock(mavros_mutex_);
		snapshot = mavros_snapshot_;
	}
	std::string reason;
	bool accepted = false;
	{
		std::lock_guard<std::mutex> lock(lcp_mutex_);
		if (lcp_core_ != nullptr)
			accepted = lcp_core_->startInitialization(snapshot, steadyNowSec(), reason);
		else
			reason = "LCP core is not initialized";
	}
	response->success = accepted;
	response->message = accepted ? "LCP initialization started" : reason;
}

void LslidarDriver::processLcpScan(const std::vector<ScanPoint> &points,
	float scan_duration, const rclcpp::Time &stamp)
{
	if (lcp_core_ == nullptr || lcp_status_pub_ == nullptr)
		return;
	// 回调缓存和 Core 分别使用不同的锁：MAVROS 回调不会阻塞几何拟合，
	// 但每圈处理使用的是同一时刻复制出的完整状态快照。
	lcp::MavrosSnapshot snapshot;
	{
		std::lock_guard<std::mutex> lock(mavros_mutex_);
		snapshot = mavros_snapshot_;
	}
	lcp::ProcessResult result;
	{
		std::lock_guard<std::mutex> lock(lcp_mutex_);
		result = lcp_core_->processScan(points, scan_duration, steadyNowSec(), snapshot);
	}
	std_msgs::msg::UInt8 status;
	status.data = result.status;
	lcp_status_pub_->publish(status);
	if (result.status != lcp::kStatusLocked || !result.pose_valid || !result.debug.valid) {
		// 非锁定状态不发布位姿，但保留限频诊断，便于区分姿态、扫描和几何拟合门禁。
		if (result.status == lcp::kStatusUnhealthy) {
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 2000, "LCP status=%u: %s",
				static_cast<unsigned int>(result.status), result.diagnostic.c_str());
		} else {
			RCLCPP_INFO_THROTTLE(
				get_logger(), *get_clock(), 2000, "LCP status=%u: %s",
				static_cast<unsigned int>(result.status), result.diagnostic.c_str());
		}
		return;
	}
	const char *const output_frame = lcp_core_->initialHeadingIsNorth() ? "lcp_nwu" : "lcp_map";

	nav_msgs::msg::Odometry odometry;
	odometry.header.stamp = stamp;
	odometry.header.frame_id = output_frame;
	odometry.child_frame_id = frame_id;
	odometry.pose.pose.position.x = result.pose.x_m;
	odometry.pose.pose.position.y = result.pose.y_m;
	odometry.pose.pose.orientation.z = std::sin(0.5 * result.pose.yaw_rad);
	odometry.pose.pose.orientation.w = std::cos(0.5 * result.pose.yaw_rad);
	odometry.pose.covariance[0] = 0.01;
	odometry.pose.covariance[7] = 0.01;
	odometry.pose.covariance[35] = 0.02;
	lcp_odometry_pub_->publish(odometry);
	std_msgs::msg::Float32 yaw;
	yaw.data = static_cast<float>(result.pose.yaw_rad);
	lcp_yaw_pub_->publish(yaw);

	lslidar_msgs::msg::LcpDebug debug;
	debug.header.stamp = stamp;
	debug.header.frame_id = output_frame;
	debug.status = result.status;
	debug.map_locked = result.map_locked;
	debug.pose_valid = result.pose_valid;
	debug.position_x_m = static_cast<float>(result.pose.x_m);
	debug.position_y_m = static_cast<float>(result.pose.y_m);
	debug.yaw_rad = static_cast<float>(result.pose.yaw_rad);
	debug.front_distance_m = static_cast<float>(result.debug.front_distance_m);
	debug.rear_distance_m = static_cast<float>(result.debug.rear_distance_m);
	debug.left_distance_m = static_cast<float>(result.debug.left_distance_m);
	debug.right_distance_m = static_cast<float>(result.debug.right_distance_m);
	debug.map_size_x_m = static_cast<float>(result.debug.map_size_x_m);
	debug.map_size_y_m = static_cast<float>(result.debug.map_size_y_m);
	lcp_debug_pub_->publish(debug);
}

} // namespace lslidar_driver
