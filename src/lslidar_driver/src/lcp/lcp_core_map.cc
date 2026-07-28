// LCP 地图关联模块：初始化时确定最近墙面的切线方向，锁定后只枚举
// 等价轴向并与已锁定墙线匹配，绝不在飞行中重建地图坐标轴。
#include "lslidar_driver/lcp_core.hpp"
#include "lcp_core_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lslidar_driver
{
namespace lcp
{

Pose LcpCore::outputPoseFromMapPose(const Pose &map_pose) const
{
	if (!config_.initial_heading_is_north || !north_reference_valid_)
		return map_pose;

	// Rotate the arbitrary wall-map basis into the declared lcp_nwu basis.
	// At initialization +X is the placed-forward direction (declared north),
	// and +Y is the vehicle's left side (west).  XY and yaw must use the same
	// rotation; changing yaw alone would make position and attitude disagree.
	const double c = std::cos(north_reference_map_yaw_rad_);
	const double s = std::sin(north_reference_map_yaw_rad_);
	return Pose{
		c * map_pose.x_m + s * map_pose.y_m,
		-s * map_pose.x_m + c * map_pose.y_m,
		wrapPi(map_pose.yaw_rad - north_reference_map_yaw_rad_)};
}

LcpCore::MapGeometry LcpCore::transformToMap(const Fit &fit,
	unsigned map_x_axis, int map_x_sign) const
{
	MapGeometry geometry{};
	const unsigned map_y_axis = map_x_axis == 0 ? 1u : 0u;
	const int map_y_sign = map_x_axis == 0 ? map_x_sign : -map_x_sign;
	geometry.map_x_body_yaw = wrapPi(fit.axis_yaw_body + (map_x_axis == 0 ? 0.0 : detail::kHalfPi)
		+ (map_x_sign > 0 ? 0.0 : detail::kPi));
	for (unsigned side = 0; side < 4; ++side) {
		const unsigned axis = side == detail::kSidePositiveX || side == detail::kSideNegativeX ? 0 : 1;
		const int side_sign = side == detail::kSidePositiveX || side == detail::kSidePositiveY ? 1 : -1;
		const unsigned candidate_axis = axis == 0 ? map_x_axis : map_y_axis;
		const int candidate_sign = axis == 0 ? map_x_sign : map_y_sign;
		const int candidate_side = sideIndex(candidate_axis, side_sign * candidate_sign);
		geometry.wall_lines[side] = candidate_sign * fit.wall_lines[candidate_side];
	}
	geometry.size_x = geometry.wall_lines[detail::kSidePositiveX]
		- geometry.wall_lines[detail::kSideNegativeX];
	geometry.size_y = geometry.wall_lines[detail::kSidePositiveY]
		- geometry.wall_lines[detail::kSideNegativeY];
	geometry.valid = geometry.size_x > 0.1 && geometry.size_y > 0.1;
	return geometry;
}

LcpCore::MapGeometry LcpCore::initialMapGeometry(const Fit &fit) const
{
	// 先找距离最近的已拟合墙，再把它的切线定义为 lcp_map 的 +X，
	// +Y 固定指向远离该墙的一侧，从根源上消除正方形的 90°/180°歧义。
	double nearest_distance = std::numeric_limits<double>::max();
	unsigned nearest_side = detail::kSidePositiveX;
	for (unsigned side = 0; side < 4; ++side) {
		const bool positive = side == detail::kSidePositiveX || side == detail::kSidePositiveY;
		const double distance = positive ? fit.wall_lines[side] : -fit.wall_lines[side];
		if (distance > 0.0 && distance < nearest_distance) {
			nearest_distance = distance;
			nearest_side = side;
		}
	}
	// +Y 远离最近墙面，+X 取该墙面的切线，固定矩形的 ±90° 歧义。
	unsigned map_x_axis = 1;
	int map_x_sign = 1;
	switch (nearest_side) {
	case detail::kSidePositiveX: map_x_axis = 1; map_x_sign = 1; break;
	case detail::kSideNegativeX: map_x_axis = 1; map_x_sign = -1; break;
	case detail::kSidePositiveY: map_x_axis = 0; map_x_sign = -1; break;
	case detail::kSideNegativeY: map_x_axis = 0; map_x_sign = 1; break;
	default: break;
	}
	return transformToMap(fit, map_x_axis, map_x_sign);
}

bool LcpCore::matchLockedMap(const Fit &fit, MapGeometry &geometry,
	Pose &pose, double &match_error) const
{
	match_error = std::numeric_limits<double>::max();
	bool found = false;
	MapGeometry best_geometry{};
	Pose best_pose{};
	const double tolerance_x = std::max(config_.map_size_tolerance_m,
		config_.map_size_tolerance_ratio * std::max(locked_map_.size_x, 1.0));
	const double tolerance_y = std::max(config_.map_size_tolerance_m,
		config_.map_size_tolerance_ratio * std::max(locked_map_.size_y, 1.0));
	for (unsigned axis = 0; axis < 2; ++axis) {
		for (int sign : {1, -1}) {
			const MapGeometry candidate = transformToMap(fit, axis, sign);
			if (!candidate.valid) continue;
			const double dx1 = locked_map_.wall_lines[0] - candidate.wall_lines[0];
			const double dx2 = locked_map_.wall_lines[2] - candidate.wall_lines[2];
			const double dy1 = locked_map_.wall_lines[1] - candidate.wall_lines[1];
			const double dy2 = locked_map_.wall_lines[3] - candidate.wall_lines[3];
			const double px = 0.5 * (dx1 + dx2), py = 0.5 * (dy1 + dy2);
			double line_error = 0.0;
			for (unsigned side = 0; side < 4; ++side) {
				const double delta = locked_map_.wall_lines[side] - candidate.wall_lines[side]
					- ((side == 0 || side == 2) ? px : py);
				line_error += delta * delta;
			}
			line_error = std::sqrt(line_error / 4.0);
			const double size_error = std::fabs(candidate.size_x - locked_map_.size_x)
				/ std::max(locked_map_.size_x, 0.1)
				+ std::fabs(candidate.size_y - locked_map_.size_y) / std::max(locked_map_.size_y, 0.1);
			const double yaw_error = previous_pose_valid_
				? angleDifference(candidate.map_x_body_yaw, previous_map_x_body_yaw_) : 0.0;
			const double cost = line_error + 0.05 * size_error + 0.01 * yaw_error;
			if (cost < match_error) {
				match_error = cost; best_geometry = candidate;
				best_pose = Pose{px, py, wrapPi(-candidate.map_x_body_yaw)}; found = true;
			}
		}
	}
	if (!found || std::fabs(best_geometry.size_x - locked_map_.size_x) > tolerance_x
		|| std::fabs(best_geometry.size_y - locked_map_.size_y) > tolerance_y)
		return false;
	const double dx1 = locked_map_.wall_lines[0] - best_geometry.wall_lines[0];
	const double dx2 = locked_map_.wall_lines[2] - best_geometry.wall_lines[2];
	const double dy1 = locked_map_.wall_lines[1] - best_geometry.wall_lines[1];
	const double dy2 = locked_map_.wall_lines[3] - best_geometry.wall_lines[3];
	const double px = 0.5 * (dx1 + dx2), py = 0.5 * (dy1 + dy2);
	double residual = 0.0;
	for (unsigned side = 0; side < 4; ++side) {
		const double delta = locked_map_.wall_lines[side] - best_geometry.wall_lines[side]
			- ((side == 0 || side == 2) ? px : py);
		residual += delta * delta;
	}
	if (std::sqrt(residual / 4.0) > config_.map_match_threshold_m) return false;
	geometry = best_geometry;
	pose = best_pose;
	return true;
}

DebugGeometry LcpCore::debugGeometry(const Pose &pose) const
{
	DebugGeometry debug;
	if (!locked_map_.valid)
		return debug;

	const auto distance_to_boundary = [this, &pose](double dx, double dy) {
		double distance = std::numeric_limits<double>::infinity();
		if (dx > 1e-9)
			distance = std::min(distance,
				(locked_map_.wall_lines[detail::kSidePositiveX] - pose.x_m) / dx);
		else if (dx < -1e-9)
			distance = std::min(distance,
				(locked_map_.wall_lines[detail::kSideNegativeX] - pose.x_m) / dx);
		if (dy > 1e-9)
			distance = std::min(distance,
				(locked_map_.wall_lines[detail::kSidePositiveY] - pose.y_m) / dy);
		else if (dy < -1e-9)
			distance = std::min(distance,
				(locked_map_.wall_lines[detail::kSideNegativeY] - pose.y_m) / dy);
		return std::isfinite(distance) && distance >= 0.0
			? distance : std::numeric_limits<double>::quiet_NaN();
	};

	const double forward_x = std::cos(pose.yaw_rad);
	const double forward_y = std::sin(pose.yaw_rad);
	const double left_x = -forward_y;
	const double left_y = forward_x;
	debug.front_distance_m = distance_to_boundary(forward_x, forward_y);
	debug.rear_distance_m = distance_to_boundary(-forward_x, -forward_y);
	debug.left_distance_m = distance_to_boundary(left_x, left_y);
	debug.right_distance_m = distance_to_boundary(-left_x, -left_y);
	debug.map_size_x_m = locked_map_.size_x;
	debug.map_size_y_m = locked_map_.size_y;
	debug.valid = std::isfinite(debug.front_distance_m) && std::isfinite(debug.rear_distance_m)
		&& std::isfinite(debug.left_distance_m) && std::isfinite(debug.right_distance_m)
		&& std::isfinite(debug.map_size_x_m) && std::isfinite(debug.map_size_y_m);
	return debug;
}

} // namespace lcp
} // namespace lslidar_driver
