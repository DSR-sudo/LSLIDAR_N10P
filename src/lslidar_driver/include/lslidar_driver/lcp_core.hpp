#ifndef LSLIDAR_DRIVER_LCP_CORE_HPP_
#define LSLIDAR_DRIVER_LCP_CORE_HPP_

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "lslidar_driver/scan_point.hpp"

namespace lslidar_driver
{

namespace lcp
{

// 状态值直接对应 /lcp/status：0 等待服务，1 累积建系，2 地图锁定，
// 3 健康检查或几何匹配失败。STATUS=3 不会自动清除已锁定地图。
static constexpr uint8_t kStatusBeforeInitialization = 0;
static constexpr uint8_t kStatusInitializing = 1;
static constexpr uint8_t kStatusLocked = 2;
static constexpr uint8_t kStatusUnhealthy = 3;
static constexpr uint8_t kLandedStateOnGround = 1;

struct Config
{
	// 几何拟合、稳定性、地图关联和姿态/扫描新鲜度全部参数化，
	// 便于台架测试时收紧门限而不改算法代码。
	bool enabled{true};
	// If the operator places the laser/vehicle forward axis on true north
	// before accepting /lcp/start_initialization, rotate the public pose so
	// that the accepted heading is yaw=0.  The public frame is then lcp_nwu:
	// +X north, +Y west/left, +Z up.  Internal wall-map axes remain unchanged.
	bool initial_heading_is_north{true};
	unsigned init_scans{20};
	unsigned min_valid_points{24};
	unsigned min_wall_points{3};
	unsigned min_quadrant_points{3};
	unsigned max_fit_points{360};

	double min_range_m{0.05};
	double max_range_m{100.0};
	// In a mesh enclosure, a ray that misses the mesh can reach a remote wall.
	// Before line fitting, discard a return that is much longer than the
	// nearest return in its local angular neighbourhood.
	double nearest_return_window_deg{8.0};
	double nearest_return_margin_m{0.35};
	double tilt_max_rad{15.0 * 3.14159265358979323846 / 180.0};
	double attitude_timeout_sec{0.5};
	double flight_state_timeout_sec{2.0};
	double scan_timeout_sec{0.5};

	// The line estimator uses this as a consensus half-width.
	double wall_inlier_threshold_m{0.15};
	double max_wall_residual_m{0.10};
	// Maximum unobserved gap between an implied corner and the endpoints of
	// both adjacent fitted walls. This is a corner-closure check, not the
	// distance from an arbitrary far return to a corner.
	double max_rectangle_residual_m{0.60};
	double stability_threshold_m{0.15};
	double stability_size_ratio{0.05};
	double map_match_threshold_m{0.30};
	double map_size_tolerance_m{0.30};
	double map_size_tolerance_ratio{0.10};
};

struct FlightState
{
	bool received{false};
	bool connected{false};
	bool armed{false};
	double received_at_sec{-1.0};
};

struct ExtendedFlightState
{
	bool received{false};
	uint8_t landed_state{0};
	double received_at_sec{-1.0};
};

struct ImuAttitude
{
	bool received{false};
	double roll_rad{0.0};
	double pitch_rad{0.0};
	double received_at_sec{-1.0};
};

struct MavrosSnapshot
{
	// 三类 MAVROS 回调在 ROS 线程中更新，驱动复制快照后交给 LcpCore，
	// Core 因此不需要依赖 ROS 消息类型或 ROS executor。
	FlightState state{};
	ExtendedFlightState extended_state{};
	ImuAttitude imu{};
};

struct Pose
{
	double x_m{0.0};
	double y_m{0.0};
	double yaw_rad{0.0};
};

struct DebugGeometry
{
	// Valid only together with a valid STATUS=2 pose.  Directions are expressed
	// in the current body/laser frame, not by assuming lcp_map axes are aligned
	// with the vehicle.
	bool valid{false};
	double front_distance_m{std::numeric_limits<double>::quiet_NaN()};
	double rear_distance_m{std::numeric_limits<double>::quiet_NaN()};
	double left_distance_m{std::numeric_limits<double>::quiet_NaN()};
	double right_distance_m{std::numeric_limits<double>::quiet_NaN()};
	double map_size_x_m{std::numeric_limits<double>::quiet_NaN()};
	double map_size_y_m{std::numeric_limits<double>::quiet_NaN()};
};

struct ProcessResult
{
	uint8_t status{kStatusBeforeInitialization};
	bool pose_valid{false};
	bool map_locked{false};
	unsigned stable_scans{0};
	Pose pose{};
	DebugGeometry debug{};
	std::string diagnostic{};
};

class LcpCore
{
public:
	explicit LcpCore(const Config &config = Config());

	const Config &config() const { return config_; }

	// This is deliberately the only operation that can clear a locked map.
	// It is expected to be called by the ROS service callback after taking the
	// driver's LCP mutex.
	bool startInitialization(const MavrosSnapshot &mavros, double now_sec, std::string &reason);

	// 每个完整 360° 扫描调用一次；返回当前状态以及在 STATUS=2 时有效的姿态。
	ProcessResult processScan(const std::vector<ScanPoint> &points,
					 double scan_duration_sec,
					 double now_sec,
					 const MavrosSnapshot &mavros);

	uint8_t status() const { return status_; }
	bool mapLocked() const { return map_locked_; }
	unsigned stableScans() const { return stable_scans_; }
	bool initialHeadingIsNorth() const { return config_.initial_heading_is_north; }

private:
	struct Vec2
	{
		double x{0.0};
		double y{0.0};
	};

	struct Fit
	{
		bool valid{false};
		double axis_yaw_body{0.0};
		std::array<double, 4> wall_lines{}; // +X, +Y, -X, -Y in fit axes
		std::array<unsigned, 4> wall_inliers{};
		std::array<unsigned, 4> quadrant_counts{};
		double size_x{0.0};
		double size_y{0.0};
		double wall_residual_m{0.0};
		double corner_closure_gap_m{0.0};
		double residual_m{0.0};
		unsigned inlier_count{0};
		unsigned outlier_count{0};
	};

	struct MapGeometry
	{
		bool valid{false};
		double map_x_body_yaw{0.0};
		std::array<double, 4> wall_lines{}; // +X, +Y, -X, -Y in lcp_map
		double size_x{0.0};
		double size_y{0.0};
	};

	static double wrapPi(double angle);
	static double angleDifference(double a, double b);
	static int sideIndex(unsigned axis, int sign);

	bool makeFitPoints(const std::vector<ScanPoint> &points,
				  std::vector<Vec2> &fit_points,
				  std::vector<Vec2> &quality_points) const;
	/// Measure the largest gap between a fitted corner and both adjacent wall segments.
	double cornerClosureGap(const std::vector<Vec2> &points, double cos_axis, double sin_axis,
					const std::array<double, 4> &wall_lines) const;
	bool fitRectangle(const std::vector<Vec2> &points,
				  const std::vector<Vec2> &quality_points,
				  Fit &fit) const;
	bool initializeWallLines(const std::vector<Vec2> &projected,
					std::array<double, 4> &wall_lines,
					std::array<unsigned, 4> &supports) const;
	bool refineWallLines(const std::vector<Vec2> &projected,
				 std::array<double, 4> &wall_lines,
				 std::array<unsigned, 4> &supports) const;

	MapGeometry transformToMap(const Fit &fit, unsigned map_x_axis, int map_x_sign) const;
	MapGeometry initialMapGeometry(const Fit &fit) const;
	bool matchLockedMap(const Fit &fit, MapGeometry &geometry, Pose &pose, double &match_error) const;
	Pose outputPoseFromMapPose(const Pose &map_pose) const;
	DebugGeometry debugGeometry(const Pose &pose) const;
	bool healthyForInitialization(const MavrosSnapshot &mavros, double now_sec,
					 std::string &reason) const;
	bool healthyForScan(const MavrosSnapshot &mavros, double now_sec,
				   double scan_duration_sec, std::string &reason) const;
	bool fitPassesQuality(const Fit &fit, std::string &reason) const;
	bool geometryStable(const MapGeometry &a, const MapGeometry &b) const;

	Config config_{};
	uint8_t status_{kStatusBeforeInitialization};
	bool initialization_requested_{false};
	bool map_locked_{false};
	unsigned stable_scans_{0};
	double last_scan_arrival_sec_{-1.0};
	MapGeometry pending_map_{};
	MapGeometry locked_map_{};
	double previous_map_x_body_yaw_{0.0};
	bool previous_pose_valid_{false};
	double north_reference_map_yaw_rad_{0.0};
	bool north_reference_valid_{false};
};

} // namespace lcp
} // namespace lslidar_driver

#endif // LSLIDAR_DRIVER_LCP_CORE_HPP_
