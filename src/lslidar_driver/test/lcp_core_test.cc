#include "lslidar_driver/lcp_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
using lslidar_driver::ScanPoint;
using lslidar_driver::lcp::Config;
using lslidar_driver::lcp::LcpCore;
using lslidar_driver::lcp::MavrosSnapshot;

constexpr double kPi = 3.14159265358979323846;

MavrosSnapshot healthyMavros(double received_at = 0.0)
{
	MavrosSnapshot snapshot;
	snapshot.state.received = true;
	snapshot.state.connected = true;
	snapshot.state.armed = false;
	snapshot.state.received_at_sec = received_at;
	snapshot.extended_state.received = true;
	snapshot.extended_state.landed_state = lslidar_driver::lcp::kLandedStateOnGround;
	snapshot.extended_state.received_at_sec = received_at;
	snapshot.imu.received = true;
	snapshot.imu.received_at_sec = received_at;
	return snapshot;
}

std::vector<ScanPoint> makeScan(double size_x,
					 double size_y,
					 double position_x,
					 double position_y,
					 double body_yaw_map,
					 unsigned samples = 720)
{
	std::vector<ScanPoint> scan;
	scan.reserve(samples);
	const double half_x = 0.5 * size_x;
	const double half_y = 0.5 * size_y;
	for (unsigned i = 0; i < samples; ++i) {
		const double body_angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(samples);
		const double dx_body = std::cos(body_angle);
		const double dy_body = std::sin(body_angle);
		const double c = std::cos(body_yaw_map);
		const double s = std::sin(body_yaw_map);
		const double dx_map = c * dx_body - s * dy_body;
		const double dy_map = s * dx_body + c * dy_body;
		double range = std::numeric_limits<double>::infinity();
		if (dx_map > 1e-9)
			range = std::min(range, (half_x - position_x) / dx_map);
		else if (dx_map < -1e-9)
			range = std::min(range, (-half_x - position_x) / dx_map);
		if (dy_map > 1e-9)
			range = std::min(range, (half_y - position_y) / dy_map);
		else if (dy_map < -1e-9)
			range = std::min(range, (-half_y - position_y) / dy_map);
		const double degree = std::fmod(-body_angle * 180.0 / kPi + 360.0, 360.0);
		scan.push_back(ScanPoint{degree, range, 0.0});
	}
	return scan;
}

std::vector<ScanPoint> makeMeshAndOuterScan()
{
	// One out of every six rays reflects from the inner mesh.  The other rays
	// pass through it and reach a larger, rotated external enclosure.
	const auto mesh = makeScan(4.0, 3.0, 0.20, -0.55, 0.0);
	const auto outer = makeScan(10.0, 8.0, 0.20, -0.55, 0.55);
	std::vector<ScanPoint> scan = outer;
	for (size_t index = 0; index < scan.size(); ++index) {
		if (index % 6 == 0)
			scan[index] = mesh[index];
	}
	return scan;
}

std::vector<ScanPoint> replaceBodyAngleInterval(
	const std::vector<ScanPoint> &inner, const std::vector<ScanPoint> &replacement,
	double start_deg, double end_deg)
{
	EXPECT_EQ(inner.size(), replacement.size());
	auto scan = inner;
	for (size_t index = 0; index < scan.size(); ++index) {
		const double body_angle = std::fmod(-scan[index].degree + 360.0, 360.0);
		if (body_angle >= start_deg && body_angle <= end_deg)
			scan[index].range = replacement[index].range;
	}
	return scan;
}

Config testConfig(unsigned init_scans = 3)
{
	Config config;
	config.init_scans = init_scans;
	config.max_range_m = 20.0;
	config.max_wall_residual_m = 0.12;
	config.max_rectangle_residual_m = 0.8;
	config.attitude_timeout_sec = 2.0;
	config.flight_state_timeout_sec = 2.0;
	config.scan_timeout_sec = 2.0;
	return config;
}

void startAt(LcpCore &core, const MavrosSnapshot &mavros)
{
	std::string reason;
	ASSERT_TRUE(core.startInitialization(mavros, 0.1, reason)) << reason;
}

TEST(LcpCore, RejectsInitializationUnlessDisarmedOnGroundAndFresh)
{
	LcpCore core(testConfig());
	MavrosSnapshot bad = healthyMavros(0.0);
	bad.state.armed = true;
	std::string reason;
	EXPECT_FALSE(core.startInitialization(bad, 0.1, reason));
	EXPECT_EQ(core.status(), lslidar_driver::lcp::kStatusBeforeInitialization);

	bad.state.armed = false;
	bad.extended_state.landed_state = 2;
	EXPECT_FALSE(core.startInitialization(bad, 0.1, reason));

	bad = healthyMavros(0.0);
	EXPECT_FALSE(core.startInitialization(bad, 3.0, reason));

	startAt(core, healthyMavros(0.0));
	EXPECT_EQ(core.status(), lslidar_driver::lcp::kStatusInitializing);
}

TEST(LcpCore, InitializesLocksAndTracksFixedMap)
{
	LcpCore core(testConfig(3));
	const MavrosSnapshot mavros = healthyMavros(0.0);
	startAt(core, mavros);
	const auto scan = makeScan(4.0, 3.0, 0.4, -0.25, 0.35);
	for (unsigned i = 0; i < 2; ++i) {
		auto result = core.processScan(scan, 0.1, 0.2 + i * 0.1, mavros);
		EXPECT_EQ(result.status, lslidar_driver::lcp::kStatusInitializing);
	}
	auto locked = core.processScan(scan, 0.1, 0.4, mavros);
	ASSERT_EQ(locked.status, lslidar_driver::lcp::kStatusLocked);
	ASSERT_TRUE(locked.pose_valid);
	EXPECT_NEAR(locked.pose.x_m, 0.0, 0.12);
	EXPECT_NEAR(locked.pose.y_m, 0.0, 0.12);
	// The initial placed-forward direction is declared north, so public yaw is
	// zero even though the internal wall-map yaw is about 0.35 rad.
	EXPECT_NEAR(locked.pose.yaw_rad, 0.0, 0.12);

	const auto moved = makeScan(4.0, 3.0, 0.7, -0.05, 0.50);
	const auto tracked = core.processScan(moved, 0.1, 0.5, mavros);
	ASSERT_EQ(tracked.status, lslidar_driver::lcp::kStatusLocked);
	ASSERT_TRUE(tracked.pose_valid);
	EXPECT_NEAR(std::hypot(tracked.pose.x_m, tracked.pose.y_m), std::hypot(0.3, 0.2), 0.18);
	EXPECT_NEAR(tracked.pose.yaw_rad, 0.15, 0.15);
}

TEST(LcpCore, NorthReferenceRotatesXyAndYawTogether)
{
	Config north_config = testConfig(1);
	north_config.initial_heading_is_north = true;
	Config legacy_config = north_config;
	legacy_config.initial_heading_is_north = false;
	LcpCore north_core(north_config);
	LcpCore legacy_core(legacy_config);
	const MavrosSnapshot mavros = healthyMavros(0.0);
	startAt(north_core, mavros);
	startAt(legacy_core, mavros);
	const auto initial = makeScan(4.0, 3.0, 0.4, -0.25, 0.35);
	const auto north_initial = north_core.processScan(initial, 0.1, 0.1, mavros);
	const auto legacy_initial = legacy_core.processScan(initial, 0.1, 0.1, mavros);
	ASSERT_TRUE(north_initial.pose_valid);
	ASSERT_TRUE(legacy_initial.pose_valid);

	const auto moved = makeScan(4.0, 3.0, 0.7, -0.05, 0.50);
	const auto north = north_core.processScan(moved, 0.1, 0.2, mavros);
	const auto legacy = legacy_core.processScan(moved, 0.1, 0.2, mavros);
	ASSERT_TRUE(north.pose_valid);
	ASSERT_TRUE(legacy.pose_valid);
	const double reference_yaw = legacy_initial.pose.yaw_rad;
	const double c = std::cos(reference_yaw);
	const double s = std::sin(reference_yaw);
	EXPECT_NEAR(north.pose.x_m, c * legacy.pose.x_m + s * legacy.pose.y_m, 0.12);
	EXPECT_NEAR(north.pose.y_m, -s * legacy.pose.x_m + c * legacy.pose.y_m, 0.12);
	EXPECT_NEAR(north.pose.yaw_rad, legacy.pose.yaw_rad - reference_yaw, 0.15);
}

TEST(LcpCore, ReportsBodyFrameWallDistancesAndLockedMapSize)
{
	LcpCore core(testConfig(1));
	const MavrosSnapshot mavros = healthyMavros(0.0);
	startAt(core, mavros);
	const auto scan = makeScan(4.0, 3.0, 0.0, 0.0, 0.0);
	const auto locked = core.processScan(scan, 0.1, 0.1, mavros);
	ASSERT_EQ(locked.status, lslidar_driver::lcp::kStatusLocked);
	ASSERT_TRUE(locked.pose_valid);
	ASSERT_TRUE(locked.debug.valid);
	EXPECT_NEAR(locked.debug.front_distance_m, 2.0, 0.12);
	EXPECT_NEAR(locked.debug.rear_distance_m, 2.0, 0.12);
	EXPECT_NEAR(locked.debug.left_distance_m, 1.5, 0.12);
	EXPECT_NEAR(locked.debug.right_distance_m, 1.5, 0.12);
	EXPECT_NEAR(locked.debug.map_size_x_m, 4.0, 0.12);
	EXPECT_NEAR(locked.debug.map_size_y_m, 3.0, 0.12);
}

TEST(LcpCore, NoiseAndOutliersAreToleratedButMissingWallIsNot)
{
	LcpCore core(testConfig(1));
	const MavrosSnapshot mavros = healthyMavros(0.0);
	startAt(core, mavros);
	auto scan = makeScan(4.0, 3.0, 0.0, 0.0, 0.0);
	for (size_t i = 0; i < scan.size(); ++i) {
		scan[i].range += 0.01 * std::sin(static_cast<double>(i));
	}
	scan[12].range = 0.5;
	scan[300].range = 15.0;
	auto locked = core.processScan(scan, 0.1, 0.1, mavros);
	ASSERT_EQ(locked.status, lslidar_driver::lcp::kStatusLocked);

	LcpCore missing_core(testConfig(1));
	startAt(missing_core, mavros);
	auto missing = makeScan(4.0, 3.0, 0.0, 0.0, 0.0);
	for (auto &point : missing) {
		const double a = std::fmod(-point.degree + 360.0, 360.0);
		// At the centered pose, all rays in this interval terminate on the
		// +Y wall. Leave the other three walls intact.
		if (a > 35.0 && a < 145.0)
			point.range = 0.0;
	}
	const auto result = missing_core.processScan(missing, 0.1, 0.1, mavros);
	EXPECT_EQ(result.status, lslidar_driver::lcp::kStatusUnhealthy);
}

TEST(LcpCore, PrefersNearMeshWallsOverMoreFrequentOuterReturns)
{
	LcpCore core(testConfig(1));
	const MavrosSnapshot mavros = healthyMavros(0.0);
	startAt(core, mavros);
	const auto scan = makeMeshAndOuterScan();
	const auto locked = core.processScan(scan, 0.1, 0.1, mavros);
	ASSERT_EQ(locked.status, lslidar_driver::lcp::kStatusLocked);
	ASSERT_TRUE(locked.pose_valid);
	// The inner mesh has yaw 0.  The external enclosure is rotated by 0.55 rad
	// and supplies five times as many returns, so this fails with the former
	// support-count-first / farthest-tie-break selection rule.
	EXPECT_NEAR(locked.pose.yaw_rad, 0.0, 0.12);
}

TEST(LcpCore, UsesWallEndpointsInsteadOfDistantQuadrantReturnsForCornerClosure)
{
	LcpCore core(testConfig(1));
	const MavrosSnapshot mavros = healthyMavros(0.0);
	startAt(core, mavros);
	const auto inner = makeScan(4.0, 3.0, 0.0, 0.0, 0.0);
	const auto outer = makeScan(9.0, 7.0, 0.0, 0.0, 0.0);
	// This continuous 20 degree distant wedge survives the local nearest-return
	// filter. The inner X and Y wall segments still extend to the corner.
	const auto scan = replaceBodyAngleInterval(inner, outer, 30.0, 50.0);

	const auto locked = core.processScan(scan, 0.1, 0.1, mavros);
	EXPECT_EQ(locked.status, lslidar_driver::lcp::kStatusLocked);
	EXPECT_TRUE(locked.pose_valid);
}

TEST(LcpCore, RejectsCornerWithoutBothAdjacentWallEndpoints)
{
	Config config = testConfig(1);
	config.max_rectangle_residual_m = 0.20;
	LcpCore core(config);
	const MavrosSnapshot mavros = healthyMavros(0.0);
	startAt(core, mavros);
	auto scan = makeScan(4.0, 3.0, 0.0, 0.0, 0.0);
	const auto invalid = std::vector<ScanPoint>(scan.size(), ScanPoint{0.0, 0.0, 0.0});
	// Remove the actual +X/+Y corner wedge while retaining ample points on all
	// four wall lines. A valid rectangle must not be inferred across this gap.
	scan = replaceBodyAngleInterval(scan, invalid, 5.0, 80.0);

	const auto result = core.processScan(scan, 0.1, 0.1, mavros);
	EXPECT_EQ(result.status, lslidar_driver::lcp::kStatusUnhealthy);
	EXPECT_EQ(result.diagnostic, "rectangle corner closure gap is too large");
}

TEST(LcpCore, TiltAndTimeoutSuppressPoseThenRecoverWithoutRelocking)
{
	Config config = testConfig(1);
	config.scan_timeout_sec = 0.5;
	LcpCore core(config);
	MavrosSnapshot mavros = healthyMavros(0.0);
	startAt(core, mavros);
	const auto scan = makeScan(4.0, 3.0, 0.0, 0.0, 0.0);
	auto locked = core.processScan(scan, 0.1, 0.1, mavros);
	ASSERT_EQ(locked.status, lslidar_driver::lcp::kStatusLocked);

	mavros.imu.roll_rad = 0.5;
	auto tilted = core.processScan(scan, 0.1, 0.2, mavros);
	EXPECT_EQ(tilted.status, lslidar_driver::lcp::kStatusUnhealthy);
	EXPECT_FALSE(tilted.pose_valid);
	mavros.imu.roll_rad = 0.0;
	auto recovered = core.processScan(scan, 0.1, 0.3, mavros);
	EXPECT_EQ(recovered.status, lslidar_driver::lcp::kStatusLocked);
	EXPECT_TRUE(recovered.pose_valid);

	auto timeout = core.processScan(scan, 0.1, 1.0, mavros);
	EXPECT_EQ(timeout.status, lslidar_driver::lcp::kStatusUnhealthy);
	EXPECT_TRUE(core.mapLocked());
}

TEST(LcpCore, SquareUsesStableAxisAndAirborneRequestCannotClearMap)
{
	LcpCore core(testConfig(1));
	const MavrosSnapshot mavros = healthyMavros(0.0);
	startAt(core, mavros);
	const auto scan = makeScan(3.0, 3.0, 0.15, -0.10, 0.0);
	const auto locked = core.processScan(scan, 0.1, 0.1, mavros);
	ASSERT_EQ(locked.status, lslidar_driver::lcp::kStatusLocked);
	ASSERT_TRUE(core.mapLocked());

	MavrosSnapshot airborne = mavros;
	airborne.state.armed = true;
	std::string reason;
	EXPECT_FALSE(core.startInitialization(airborne, 0.2, reason));
	EXPECT_TRUE(core.mapLocked());
	EXPECT_TRUE(core.startInitialization(mavros, 0.2, reason));
	EXPECT_FALSE(core.mapLocked());
	EXPECT_EQ(core.status(), lslidar_driver::lcp::kStatusInitializing);

	const auto rotated = makeScan(3.0, 3.0, 0.15, -0.10, 0.35);
	const auto tracked = core.processScan(rotated, 0.1, 0.2, mavros);
	EXPECT_EQ(tracked.status, lslidar_driver::lcp::kStatusLocked);
	EXPECT_TRUE(tracked.pose_valid);
}

} // namespace
