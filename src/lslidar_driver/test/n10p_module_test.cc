#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "lslidar_driver/n10p_module.hpp"

namespace
{

void writeU16(unsigned char *data, int offset, unsigned value)
{
	data[offset] = static_cast<unsigned char>((value >> 8) & 0xffu);
	data[offset + 1] = static_cast<unsigned char>(value & 0xffu);
}

std::vector<unsigned char> makePacket(double start_degree, double end_degree,
								  unsigned primary_mm = 3000,
								  unsigned secondary_mm = 4000)
{
	std::vector<unsigned char> packet(lslidar_driver::N10PModule::kPacketSize, 0);
	packet[0] = 0xa5;
	packet[1] = 0x5a;
	writeU16(packet.data(), lslidar_driver::N10PModule::kStartDegreeOffset,
		static_cast<unsigned>(std::fmod(start_degree + 360.0, 360.0) * 100.0));
	writeU16(packet.data(), lslidar_driver::N10PModule::kEndDegreeOffset,
		static_cast<unsigned>(std::fmod(end_degree + 360.0, 360.0) * 100.0));
	for (int point = 0; point < lslidar_driver::N10PModule::kPointsPerPacket; ++point) {
		const int offset = lslidar_driver::N10PModule::kPrimaryDataOffset
			+ point * lslidar_driver::N10PModule::kPointStride;
		writeU16(packet.data(), offset, primary_mm + point);
		packet[offset + 2] = static_cast<unsigned char>(point);
		writeU16(packet.data(), offset + 3, secondary_mm + point);
		packet[offset + 5] = static_cast<unsigned char>(point + 20);
	}
	packet.back() = lslidar_driver::N10PModule::calculateCrc8(packet.data(), packet.size() - 1);
	return packet;
}

} // namespace

TEST(N10PModule, VerifiesChecksumAndDecodesTwoReturns)
{
	lslidar_driver::N10PModule module;
	std::vector<unsigned char> packet = makePacket(10.0, 13.0);
	ASSERT_TRUE(lslidar_driver::N10PModule::verifyPacket(packet.data(), packet.size()));

	lslidar_driver::N10PModule::Sweep sweep;
	EXPECT_FALSE(module.processPacket(packet.data(), packet.size(), sweep));
	EXPECT_EQ(sweep.point_count, 0);
	packet[10] ^= 1;
	EXPECT_FALSE(lslidar_driver::N10PModule::verifyPacket(packet.data(), packet.size()));
}

TEST(N10PModule, EmitsOneSweepAtZeroDegreeWrap)
{
	lslidar_driver::N10PModule module;
	lslidar_driver::N10PModule::Sweep sweep;
	bool emitted = false;
	for (int packet = 0; packet < 100 && !emitted; ++packet) {
		const double start = packet * 3.6;
		const std::vector<unsigned char> bytes = makePacket(start, start + 3.6);
		emitted = module.processPacket(bytes.data(), bytes.size(), sweep);
	}
	ASSERT_TRUE(emitted);
	EXPECT_GT(sweep.point_count, 10);
	ASSERT_EQ(sweep.points.size(), static_cast<size_t>(lslidar_driver::N10PModule::kAccumulatorSize));
	EXPECT_NEAR(sweep.points[0].range, 3.0, 1e-6);
	EXPECT_NEAR(sweep.points[lslidar_driver::N10PModule::kSecondaryPointOffset].range, 4.0, 1e-6);
	EXPECT_EQ(sweep.points[0].intensity, 0.0);
}

TEST(N10PModule, AppliesAngleAndRangeFiltersToBothReturns)
{
	lslidar_driver::N10PModule::Config config;
	config.min_range = 1.0;
	config.max_range = 5.0;
	config.angle_able_min = 20.0;
	config.angle_able_max = 40.0;
	lslidar_driver::N10PModule module(config);
	lslidar_driver::N10PModule::Sweep sweep;

	for (int packet = 0; packet < 100 && sweep.point_count == 0; ++packet) {
		const std::vector<unsigned char> bytes = makePacket(packet * 3.6, packet * 3.6 + 3.6,
			packet == 0 ? 6000 : 3000, 4000);
		module.processPacket(bytes.data(), bytes.size(), sweep);
	}
	ASSERT_GT(sweep.point_count, 10);
	EXPECT_EQ(sweep.points[0].range, 0.0); // 0-degree scan angle is outside [20, 40].
	EXPECT_EQ(sweep.points[lslidar_driver::N10PModule::kSecondaryPointOffset].range, 0.0);
}
