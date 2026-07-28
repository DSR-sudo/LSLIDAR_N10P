// LCP 几何拟合模块：把极坐标扫描投影到机体平面，寻找四条近墙线，
// 再用方向支撑、墙面残差和矩形角点残差过滤缺墙/障碍物场景。
#include "lslidar_driver/lcp_core.hpp"
#include "lcp_core_internal.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lslidar_driver
{
namespace lcp
{

bool LcpCore::makeFitPoints(const std::vector<ScanPoint> &points,
	std::vector<Vec2> &fit_points, std::array<unsigned, 4> &quadrant_counts) const
{
	// 先用完整扫描统计四个机体象限，再对拟合点做均匀抽样，
	// 防止某一面墙过密时占满 CPU 或掩盖缺失墙面。
	fit_points.clear();
	quadrant_counts = {};
	std::vector<Vec2> all_points;
	all_points.reserve(points.size());
	struct PolarPoint
	{
		double degree;
		double range;
	};
	std::vector<PolarPoint> valid_points;
	valid_points.reserve(points.size());
	for (const ScanPoint &scan_point : points) {
		if (!std::isfinite(scan_point.degree) || !std::isfinite(scan_point.range)
			|| scan_point.range < config_.min_range_m || scan_point.range > config_.max_range_m)
			continue;
		valid_points.push_back({detail::clampAngle(scan_point.degree * detail::kPi / 180.0)
			* 180.0 / detail::kPi, scan_point.range});
	}
	const double half_window_deg = 0.5 * std::max(0.0, config_.nearest_return_window_deg);
	for (const PolarPoint &candidate : valid_points) {
		double nearest_range = candidate.range;
		if (half_window_deg > 0.0) {
			for (const PolarPoint &neighbour : valid_points) {
				double difference = std::fabs(candidate.degree - neighbour.degree);
				difference = std::min(difference, 360.0 - difference);
				if (difference <= half_window_deg)
					nearest_range = std::min(nearest_range, neighbour.range);
			}
		}
		if (candidate.range > nearest_range + config_.nearest_return_margin_m)
			continue;
		const double angle = candidate.degree * detail::kPi / 180.0;
		const Vec2 point{candidate.range * std::cos(angle), -candidate.range * std::sin(angle)};
		if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
		all_points.push_back(point);
		const unsigned quadrant = std::min(3u, static_cast<unsigned>(
			detail::clampAngle(std::atan2(point.y, point.x)) / detail::kHalfPi));
		++quadrant_counts[quadrant];
	}
	if (all_points.size() < config_.min_valid_points) return false;
	for (unsigned count : quadrant_counts)
		if (count < config_.min_quadrant_points) return false;
	const size_t stride = std::max<size_t>(1, (all_points.size() + config_.max_fit_points - 1)
		/ config_.max_fit_points);
	fit_points.reserve((all_points.size() + stride - 1) / stride);
	for (size_t i = 0; i < all_points.size(); i += stride)
		fit_points.push_back(all_points[i]);
	return fit_points.size() >= config_.min_valid_points;
}

bool LcpCore::initializeWallLines(const std::vector<Vec2> &projected,
	std::array<double, 4> &wall_lines, std::array<unsigned, 4> &supports) const
{
	wall_lines = {};
	supports = {};
	const double threshold = std::max(1e-3, config_.wall_inlier_threshold_m);
	for (unsigned axis = 0; axis < 2; ++axis) {
		for (int sign : {1, -1}) {
			std::vector<double> values;
			for (const Vec2 &point : projected) {
				const unsigned side = axis == 0 ? detail::kSidePositiveX : detail::kSidePositiveY;
				const double coordinate = detail::sideCoordinate(point, side);
				if (sign * coordinate >= 0.0) values.push_back(coordinate);
			}
			if (values.empty()) return false;
			std::sort(values.begin(), values.end());
			const int side_index = sideIndex(axis, sign);
			size_t best_begin = 0, best_end = 0, end = 0;
			for (size_t begin = 0; begin < values.size(); ++begin) {
				end = std::max(end, begin);
				while (end + 1 < values.size() && values[end + 1] - values[begin] <= 2 * threshold)
					++end;
				const unsigned support = static_cast<unsigned>(end - begin + 1);
				const double center = 0.5 * (values[begin] + values[end]);
				const double best_center = 0.5 * (values[best_begin] + values[best_end]);
				const bool farther = sign > 0 ? center > best_center : center < best_center;
				if (support > supports[side_index] || (support == supports[side_index] && farther)) {
					best_begin = begin; best_end = end; supports[side_index] = support;
				}
			}
			if (supports[side_index] < config_.min_wall_points) return false;
			const double center = 0.5 * (values[best_begin] + values[best_end]);
			double sum = 0.0;
			unsigned count = 0;
			for (double value : values) {
				if (std::fabs(value - center) <= threshold) { sum += value; ++count; }
			}
			if (count < config_.min_wall_points) return false;
			wall_lines[side_index] = sum / count;
			supports[side_index] = count;
		}
	}
	return true;
}

bool LcpCore::refineWallLines(const std::vector<Vec2> &projected,
	std::array<double, 4> &wall_lines, std::array<unsigned, 4> &supports) const
{
	const double threshold = std::max(1e-3, config_.wall_inlier_threshold_m);
	for (unsigned iteration = 0; iteration < 2; ++iteration) {
		std::array<double, 4> sums{};
		std::array<unsigned, 4> counts{};
		for (const Vec2 &point : projected) {
			double residual = 0.0;
			const unsigned side = detail::nearestSide(point, wall_lines, residual);
			if (residual <= threshold) {
				sums[side] += detail::sideCoordinate(point, side);
				++counts[side];
			}
		}
		for (unsigned side = 0; side < 4; ++side) {
			if (counts[side] < config_.min_wall_points) return false;
			wall_lines[side] = sums[side] / counts[side];
		}
		supports = counts;
	}
	return true;
}

bool LcpCore::fitRectangle(const std::vector<Vec2> &points,
	const std::array<unsigned, 4> &quadrant_counts, Fit &fit) const
{
	fit = Fit{};
	if (points.size() < config_.min_valid_points) return false;
	double best_cost = std::numeric_limits<double>::max();
	for (unsigned step = 0; step <= 90; ++step) {
		// 矩形只需搜索 0..90°；其余方向与轴交换或符号翻转等价。
		const double axis_yaw = step * detail::kPi / 180.0;
		const double c = std::cos(axis_yaw), s = std::sin(axis_yaw);
		std::vector<Vec2> projected;
		projected.reserve(points.size());
		for (const Vec2 &point : points)
			projected.push_back({point.x * c + point.y * s, -point.x * s + point.y * c});
		std::array<double, 4> lines{};
		std::array<unsigned, 4> supports{};
		if (!initializeWallLines(projected, lines, supports) || !refineWallLines(projected, lines, supports))
			continue;
		const double size_x = lines[detail::kSidePositiveX] - lines[detail::kSideNegativeX];
		const double size_y = lines[detail::kSidePositiveY] - lines[detail::kSideNegativeY];
		if (size_x <= 0.1 || size_y <= 0.1) continue;
		double wall_error = 0.0;
		unsigned inliers = 0;
		std::array<unsigned, 4> directional_support{};
		for (const Vec2 &point : projected) {
			double residual = 0.0;
			detail::nearestSide(point, lines, residual);
			if (residual <= config_.wall_inlier_threshold_m) {
				wall_error += residual * residual; ++inliers;
			}
			const double range = std::hypot(point.x, point.y);
			if (range < 1e-6) continue;
			for (unsigned side = 0; side < 4; ++side) {
				const int sign = side == detail::kSidePositiveX || side == detail::kSidePositiveY ? 1 : -1;
				// A wall must be observed close to head-on.  A shallow oblique return
				// from an adjacent wall must not fabricate support for a missing side.
				if (std::fabs(detail::sideCoordinate(point, side) - lines[side]) <= config_.wall_inlier_threshold_m
					&& sign * detail::sideCoordinate(point, side) / range > 0.70)
					++directional_support[side];
			}
		}
		bool supported = inliers > 0;
		for (unsigned side = 0; side < 4; ++side)
			supported = supported && directional_support[side] >= config_.min_wall_points;
		if (!supported) continue;

		const Vec2 corners[] = {{lines[0], lines[1]}, {lines[2], lines[1]},
			{lines[2], lines[3]}, {lines[0], lines[3]}};
		std::array<std::vector<std::pair<double, Vec2>>, 4> extremes;
		for (const Vec2 &point : points) {
			const unsigned quadrant = std::min(3u, static_cast<unsigned>(
				detail::clampAngle(std::atan2(point.y, point.x)) / detail::kHalfPi));
			const Vec2 projected_point{point.x * c + point.y * s, -point.x * s + point.y * c};
			extremes[quadrant].push_back({point.x * point.x + point.y * point.y, projected_point});
		}
		double corner_error = 0.0;
		unsigned corner_count = 0;
		for (unsigned quadrant = 0; quadrant < 4; ++quadrant) {
			if (quadrant_counts[quadrant] < config_.min_quadrant_points || extremes[quadrant].empty()) continue;
			std::sort(extremes[quadrant].begin(), extremes[quadrant].end(),
				[](const std::pair<double, Vec2> &a, const std::pair<double, Vec2> &b) { return a.first < b.first; });
			const size_t index = static_cast<size_t>(0.95 * (extremes[quadrant].size() - 1));
			const Vec2 extreme = extremes[quadrant][index].second;
			double nearest = std::numeric_limits<double>::max();
			for (const Vec2 &corner : corners) {
				const double dx = extreme.x - corner.x, dy = extreme.y - corner.y;
				nearest = std::min(nearest, dx * dx + dy * dy);
			}
			corner_error += nearest; ++corner_count;
		}
		const double wall_residual = std::sqrt(wall_error / inliers);
		const double corner_residual = corner_count ? std::sqrt(corner_error / corner_count) : 0.0;
		const double outlier_ratio = static_cast<double>(points.size() - inliers) / points.size();
		const double cost = wall_residual * wall_residual + 0.02 * corner_residual * corner_residual
			+ 0.04 * outlier_ratio;
		if (cost < best_cost) {
			best_cost = cost; fit.valid = true; fit.axis_yaw_body = axis_yaw; fit.wall_lines = lines;
			fit.wall_inliers = directional_support; fit.quadrant_counts = quadrant_counts;
			fit.size_x = size_x; fit.size_y = size_y; fit.wall_residual_m = wall_residual;
			fit.rectangle_residual_m = corner_residual; fit.residual_m = std::sqrt(cost);
			fit.inlier_count = inliers; fit.outlier_count = points.size() - inliers;
		}
	}
	return fit.valid;
}

} // namespace lcp
} // namespace lslidar_driver
