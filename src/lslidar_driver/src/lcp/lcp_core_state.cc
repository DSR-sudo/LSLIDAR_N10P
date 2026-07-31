// LCP 状态机模块：处理服务准入、姿态/飞控/扫描健康门限、连续稳定帧计数，
// 并在地图锁定后输出当前 XY 与 yaw。STATUS=3 时保留地图但暂停位姿输出。
#include "lslidar_driver/lcp_core.hpp"
#include "lcp_core_internal.hpp"

#include <algorithm>
#include <cmath>

namespace lslidar_driver
{
namespace lcp
{

LcpCore::LcpCore(const Config &config) : config_(config)
{
	config_.init_scans = std::max(1u, config_.init_scans);
	config_.min_valid_points = std::max(12u, config_.min_valid_points);
	config_.min_wall_points = std::max(2u, config_.min_wall_points);
	config_.min_quadrant_points = std::max(1u, config_.min_quadrant_points);
	config_.max_fit_points = std::max(72u, config_.max_fit_points);
}

double LcpCore::wrapPi(double angle)
{
	while (angle > detail::kPi) angle -= detail::kTwoPi;
	while (angle <= -detail::kPi) angle += detail::kTwoPi;
	return angle;
}

double LcpCore::angleDifference(double a, double b)
{
	return std::fabs(wrapPi(a - b));
}

int LcpCore::sideIndex(unsigned axis, int sign)
{
	return axis == 0 ? (sign > 0 ? detail::kSidePositiveX : detail::kSideNegativeX)
		: (sign > 0 ? detail::kSidePositiveY : detail::kSideNegativeY);
}

bool LcpCore::healthyForInitialization(const MavrosSnapshot &mavros, double now_sec,
	std::string &reason) const
{
	if (!mavros.state.received || !mavros.extended_state.received) {
		reason = "MAVROS flight state is not available"; return false;
	}
	if (!mavros.state.connected) { reason = "MAVROS flight controller is not connected"; return false; }
	if (now_sec < mavros.state.received_at_sec
		|| now_sec - mavros.state.received_at_sec > config_.flight_state_timeout_sec
		|| now_sec < mavros.extended_state.received_at_sec
		|| now_sec - mavros.extended_state.received_at_sec > config_.flight_state_timeout_sec) {
		reason = "MAVROS flight state is stale"; return false;
	}
	if (mavros.state.armed) { reason = "flight controller is armed"; return false; }
	if (mavros.extended_state.landed_state != kLandedStateOnGround) {
		reason = "flight controller does not report ON_GROUND"; return false;
	}
	return true;
}

bool LcpCore::healthyForScan(const MavrosSnapshot &mavros, double now_sec,
	double scan_duration_sec, std::string &reason) const
{
	// 建系前和锁定后的健康检查共用飞控/IMU/扫描门限；锁定后失败只进
	// STATUS=3，不清除 locked_map_，恢复水平和匹配后可回到 STATUS=2。
	if (!mavros.state.received || !mavros.extended_state.received || !mavros.state.connected) {
		reason = "MAVROS flight state is unavailable or disconnected"; return false;
	}
	if (now_sec < mavros.state.received_at_sec
		|| now_sec - mavros.state.received_at_sec > config_.flight_state_timeout_sec
		|| now_sec < mavros.extended_state.received_at_sec
		|| now_sec - mavros.extended_state.received_at_sec > config_.flight_state_timeout_sec) {
		reason = "MAVROS flight state is stale"; return false;
	}
	if (!mavros.imu.received || now_sec < mavros.imu.received_at_sec
		|| now_sec - mavros.imu.received_at_sec > config_.attitude_timeout_sec) {
		reason = "IMU attitude is stale"; return false;
	}
	if (std::fabs(mavros.imu.roll_rad) > config_.tilt_max_rad
		|| std::fabs(mavros.imu.pitch_rad) > config_.tilt_max_rad) {
		reason = "vehicle tilt exceeds configured limit"; return false;
	}
	if (scan_duration_sec > config_.scan_timeout_sec) {
		reason = "scan duration exceeds timeout"; return false;
	}
	if (last_scan_arrival_sec_ >= 0.0 && (now_sec < last_scan_arrival_sec_
		|| now_sec - last_scan_arrival_sec_ > config_.scan_timeout_sec)) {
		reason = "scan stream is stale"; return false;
	}
	return true;
}

bool LcpCore::startInitialization(const MavrosSnapshot &mavros, double now_sec,
	std::string &reason)
{
	reason.clear();
	if (!config_.enabled) { reason = "LCP is disabled"; return false; }
	if (!healthyForInitialization(mavros, now_sec, reason)) return false;
	initialization_requested_ = true;
	map_locked_ = false;
	stable_scans_ = 0;
	status_ = kStatusInitializing;
	pending_map_ = MapGeometry{};
	locked_map_ = MapGeometry{};
	previous_pose_valid_ = false;
	north_reference_valid_ = false;
	last_scan_arrival_sec_ = -1.0;
	return true;
}

bool LcpCore::fitPassesQuality(const Fit &fit, std::string &reason) const
{
	if (!fit.valid) { reason = "four-wall rectangle fit failed"; return false; }
	if (fit.inlier_count < 4 * config_.min_wall_points) {
		reason = "not all four walls have enough inliers"; return false;
	}
	for (unsigned side = 0; side < 4; ++side) {
		if (fit.wall_inliers[side] < config_.min_wall_points) {
			reason = "one of the four walls has insufficient directional support"; return false;
		}
	}
	if (fit.wall_residual_m > config_.max_wall_residual_m) {
		reason = "wall fit residual is too large"; return false;
	}
	if (fit.corner_closure_gap_m > config_.max_rectangle_residual_m) {
		reason = "rectangle corner closure gap is too large"; return false;
	}
	return true;
}

bool LcpCore::geometryStable(const MapGeometry &a, const MapGeometry &b) const
{
	if (!a.valid || !b.valid) return false;
	const double x_tolerance = std::max(config_.stability_threshold_m,
		config_.stability_size_ratio * std::max(a.size_x, 1.0));
	const double y_tolerance = std::max(config_.stability_threshold_m,
		config_.stability_size_ratio * std::max(a.size_y, 1.0));
	if (std::fabs(a.size_x - b.size_x) > x_tolerance || std::fabs(a.size_y - b.size_y) > y_tolerance)
		return false;
	for (unsigned side = 0; side < 4; ++side) {
		const double tolerance = side == 0 || side == 2 ? x_tolerance : y_tolerance;
		if (std::fabs(a.wall_lines[side] - b.wall_lines[side]) > tolerance) return false;
	}
	return true;
}

ProcessResult LcpCore::processScan(const std::vector<ScanPoint> &points,
	double scan_duration_sec, double now_sec, const MavrosSnapshot &mavros)
{
	// 状态转移：0 等待服务，1 累积稳定矩形，2 跟踪锁定地图，3 暂停输出。
	// 空中不会重新回到 1；只有服务在地面受理时才会清空旧地图。
	ProcessResult result;
	result.status = status_; result.map_locked = map_locked_; result.stable_scans = stable_scans_;
	if (!config_.enabled) {
		status_ = kStatusBeforeInitialization; result.status = status_; result.diagnostic = "LCP is disabled"; return result;
	}
	std::string reason;
	const bool healthy = healthyForScan(mavros, now_sec, scan_duration_sec, reason);
	last_scan_arrival_sec_ = now_sec;
	if (!initialization_requested_ && !map_locked_) {
		status_ = kStatusBeforeInitialization; result.status = status_;
		result.diagnostic = healthy ? "waiting for /lcp/start_initialization" : reason; return result;
	}
	if (!healthy) {
		status_ = kStatusUnhealthy; result.status = status_; result.map_locked = map_locked_;
		result.stable_scans = stable_scans_;
		if (!map_locked_) { stable_scans_ = 0; pending_map_ = MapGeometry{}; }
		result.diagnostic = reason; return result;
	}
	std::vector<Vec2> fit_points;
	std::vector<Vec2> quality_points;
	Fit fit{};
	if (!makeFitPoints(points, fit_points, quality_points) || !fitRectangle(fit_points, quality_points, fit)) {
		status_ = kStatusUnhealthy; result.status = status_; result.map_locked = map_locked_;
		result.stable_scans = stable_scans_; result.diagnostic = "four-wall rectangle fit failed";
		if (!map_locked_) stable_scans_ = 0;
		return result;
	}
	if (!fitPassesQuality(fit, reason)) {
		status_ = kStatusUnhealthy; result.status = status_; result.map_locked = map_locked_;
		result.stable_scans = stable_scans_; result.diagnostic = reason;
		if (!map_locked_) stable_scans_ = 0;
		return result;
	}
	if (!map_locked_) {
		const MapGeometry candidate = initialMapGeometry(fit);
		if (!candidate.valid) {
			status_ = kStatusUnhealthy; result.status = status_; result.diagnostic = "initial map geometry is invalid";
			stable_scans_ = 0; return result;
		}
		if (stable_scans_ == 0 || !pending_map_.valid) { pending_map_ = candidate; stable_scans_ = 1; }
		else if (geometryStable(pending_map_, candidate)) ++stable_scans_;
		else { pending_map_ = candidate; stable_scans_ = 1; }
		if (stable_scans_ >= config_.init_scans) {
			locked_map_ = pending_map_; map_locked_ = true; initialization_requested_ = false;
			previous_map_x_body_yaw_ = pending_map_.map_x_body_yaw; previous_pose_valid_ = true;
			const Pose map_pose{0.0, 0.0, wrapPi(-pending_map_.map_x_body_yaw)};
			if (config_.initial_heading_is_north) {
				north_reference_map_yaw_rad_ = map_pose.yaw_rad;
				north_reference_valid_ = true;
			}
			status_ = kStatusLocked; result.pose = outputPoseFromMapPose(map_pose);
			result.pose_valid = true;
			result.debug = debugGeometry(map_pose);
		} else {
			status_ = kStatusInitializing;
		}
	} else {
		MapGeometry current;
		Pose map_pose;
		double match_error = 0.0;
		if (!matchLockedMap(fit, current, map_pose, match_error)) {
			status_ = kStatusUnhealthy; result.status = status_; result.map_locked = true;
			result.stable_scans = stable_scans_; result.diagnostic = "current rectangle does not match locked map"; return result;
		}
		status_ = kStatusLocked; previous_map_x_body_yaw_ = current.map_x_body_yaw;
		previous_pose_valid_ = true; result.pose = outputPoseFromMapPose(map_pose); result.pose_valid = true;
		result.debug = debugGeometry(map_pose);
	}
	result.status = status_; result.map_locked = map_locked_; result.stable_scans = stable_scans_;
	if (result.status == kStatusLocked && !result.pose_valid) {
		status_ = kStatusUnhealthy; result.status = status_; result.diagnostic = "locked map pose association failed";
	}
	return result;
}

} // namespace lcp
} // namespace lslidar_driver
