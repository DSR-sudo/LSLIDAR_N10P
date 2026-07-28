#include "lslidar_driver/lcp_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace
{
using lslidar_driver::ScanPoint;
using lslidar_driver::lcp::Config;
using lslidar_driver::lcp::LcpCore;
using lslidar_driver::lcp::MavrosSnapshot;

constexpr double kPi = 3.14159265358979323846;

struct ReplayScan
{
	double angle_min_rad{0.0};
	double angle_increment_rad{0.0};
	double range_min_m{0.0};
	double range_max_m{0.0};
	std::vector<double> ranges;
};

struct ReplayData
{
	std::vector<ReplayScan> scans;
};

std::vector<double> parseCsvLine(const std::string &line)
{
	std::vector<double> values;
	std::stringstream stream(line);
	std::string token;
	while (std::getline(stream, token, ','))
		values.push_back(std::stod(token));
	return values;
}

ReplayData loadReplay(const std::string &path)
{
	std::ifstream stream(path);
	if (!stream.is_open())
		throw std::runtime_error("could not open live-capture replay: " + path);
	ReplayData replay;
	std::string line;
	if (!std::getline(stream, line) || line != "# lcp live-capture replay v1")
		throw std::runtime_error("unrecognized live-capture replay header");
	while (std::getline(stream, line)) {
		if (line.empty()) continue;
		auto values = parseCsvLine(line);
		if (values.size() < 6)
			throw std::runtime_error("missing live-capture replay scan metadata");
		const size_t range_count = static_cast<size_t>(values[4]);
		if (values.size() != range_count + 5)
			throw std::runtime_error("inconsistent range count in live-capture replay");
		ReplayScan scan;
		scan.angle_min_rad = values[0];
		scan.angle_increment_rad = values[1];
		scan.range_min_m = values[2];
		scan.range_max_m = values[3];
		scan.ranges.assign(values.begin() + 5, values.end());
		replay.scans.push_back(std::move(scan));
	}
	return replay;
}

MavrosSnapshot healthyMavros(double now)
{
	MavrosSnapshot snapshot;
	snapshot.state.received = true;
	snapshot.state.connected = true;
	snapshot.state.armed = false;
	snapshot.state.received_at_sec = now;
	snapshot.extended_state.received = true;
	snapshot.extended_state.landed_state = lslidar_driver::lcp::kLandedStateOnGround;
	snapshot.extended_state.received_at_sec = now;
	snapshot.imu.received = true;
	snapshot.imu.roll_rad = 0.0;
	snapshot.imu.pitch_rad = 0.0;
	snapshot.imu.received_at_sec = now;
	return snapshot;
}

std::vector<ScanPoint> makeScan(const ReplayScan &replay)
{
	std::vector<ScanPoint> scan;
	scan.reserve(replay.ranges.size());
	for (size_t index = 0; index < replay.ranges.size(); ++index) {
		const double angle = replay.angle_min_rad + index * replay.angle_increment_rad;
		double degree = -angle * 180.0 / kPi;
		while (degree < 0.0) degree += 360.0;
		while (degree >= 360.0) degree -= 360.0;
		scan.push_back(ScanPoint{degree, replay.ranges[index], 0.0});
	}
	return scan;
}

TEST(LcpLiveCapture, RebuildsFiniteXYFromRecordedScanSequence)
{
	const ReplayData replay = loadReplay(LCP_LIVE_CAPTURE_REPLAY_FILE);
	ASSERT_GE(replay.scans.size(), 20u);
	ASSERT_GT(replay.scans.front().angle_increment_rad, 0.0);

	Config config;
	config.init_scans = 20;
	config.min_range_m = replay.scans.front().range_min_m;
	config.max_range_m = replay.scans.front().range_max_m;
	config.attitude_timeout_sec = 2.0;
	config.flight_state_timeout_sec = 2.0;
	config.scan_timeout_sec = 0.5;
	LcpCore core(config);
	std::string reason;
	ASSERT_TRUE(core.startInitialization(healthyMavros(0.0), 0.0, reason)) << reason;

	bool locked = false;
	unsigned valid_xy_samples = 0;
	for (size_t index = 0; index < replay.scans.size(); ++index) {
		const double now = 0.1 * static_cast<double>(index + 1);
		const auto result = core.processScan(
			makeScan(replay.scans[index]), 0.1, now, healthyMavros(now));
		if (result.status != lslidar_driver::lcp::kStatusLocked || !result.pose_valid)
			continue;
		locked = true;
		ASSERT_TRUE(std::isfinite(result.pose.x_m));
		ASSERT_TRUE(std::isfinite(result.pose.y_m));
		ASSERT_TRUE(std::isfinite(result.pose.yaw_rad));
		++valid_xy_samples;
	}
	EXPECT_TRUE(locked);
	EXPECT_GE(valid_xy_samples, 3u);
}

} // namespace
