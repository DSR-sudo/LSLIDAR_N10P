// N10P 协议核心：每次输入一个完整 108 字节包，内部累积两个回波区，
// 检测角度从 359° 回到 0° 后输出上一圈。模块不持有 ROS 对象，也不操作锁。
#include "lslidar_driver/n10p_module.hpp"

#include <algorithm>
#include <cmath>

namespace lslidar_driver
{

namespace
{
constexpr double kFullCircle = 360.0;

uint16_t readBigEndian16(const unsigned char *data)
{
	return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

void writeZero(ScanPoint &point)
{
	point.degree = 0.0;
	point.range = 0.0;
	point.intensity = 0.0;
}
} // namespace

N10PModule::N10PModule() : N10PModule(Config{}) {}

N10PModule::N10PModule(const Config &config) : accumulator_(kAccumulatorSize)
{
	configure(config);
}

void N10PModule::configure(const Config &config)
{
	config_ = config;
	config_.min_range = std::max(0.0, config_.min_range);
	config_.max_range = std::max(config_.min_range, config_.max_range);
	reset();
}

void N10PModule::reset()
{
	index_ = 0;
	last_degree_ = 0.0;
	has_last_degree_ = false;
	clearAccumulator();
}

double N10PModule::normalizeDegree(double degree)
{
	while (degree < 0.0)
		degree += kFullCircle;
	while (degree >= kFullCircle)
		degree -= kFullCircle;
	return degree;
}

uint8_t N10PModule::calculateCrc8(const unsigned char *packet_bytes, int len)
{
	if (packet_bytes == nullptr || len <= 0)
		return 0;
	uint32_t sum = 0;
	for (int i = 0; i < len; ++i)
		sum += packet_bytes[i];
	return static_cast<uint8_t>(sum & 0xffu);
}

bool N10PModule::verifyPacket(const unsigned char *packet_bytes, int len)
{
	return packet_bytes != nullptr && len == kPacketSize
		&& calculateCrc8(packet_bytes, kPacketSize - 1) == packet_bytes[kPacketSize - 1];
}

bool N10PModule::decodePacket(const unsigned char *packet_bytes, int len,
						 double &start_degree, double &degree_interval,
						 int &valid_point_count) const
{
	if (!verifyPacket(packet_bytes, len))
		return false;

	// 角度以百分之一度的大端整数存储；包尾角度可能小于包首角度，
	// 此时说明这个小包跨过了 0°，插值跨度要加一整圈。
	start_degree = normalizeDegree(
		static_cast<double>(readBigEndian16(packet_bytes + kStartDegreeOffset)) / 100.0);
	const double end_degree = normalizeDegree(
		static_cast<double>(readBigEndian16(packet_bytes + kEndDegreeOffset)) / 100.0);
	degree_interval = end_degree >= start_degree
		? end_degree - start_degree : end_degree + kFullCircle - start_degree;

	int invalid_count = 0;
	for (int point = 0; point < kPointsPerPacket; ++point) {
		const int offset = kPrimaryDataOffset + point * kPointStride;
		if (readBigEndian16(packet_bytes + offset) == 0xffffu)
			++invalid_count;
	}
	// 协议中的最后一个角度是包尾角，沿用原驱动的 N10P 插值约定。
	// 无效首回波不参与累积，但保留其包内序号，保持与设备角度采样一致。
	valid_point_count = kPointsPerPacket - invalid_count - 1;
	return valid_point_count > 1;
}

bool N10PModule::isAngleAllowed(double degree) const
{
	const double scan_degree = kFullCircle - degree;
	if (config_.angle_able_max > kFullCircle) {
		return !(scan_degree > config_.angle_able_max - kFullCircle
			&& scan_degree < config_.angle_able_min);
	}
	return !(scan_degree > config_.angle_able_max || scan_degree < config_.angle_able_min);
}

void N10PModule::filterSweep(int point_count)
{
	// 过滤必须在整圈边界确认后执行，保证首回波和次回波使用同一角度窗口。
	for (int point = 0; point < point_count; ++point) {
		ScanPoint &primary = accumulator_[point];
		ScanPoint &secondary = accumulator_[point + kSecondaryPointOffset];
		if (!isAngleAllowed(primary.degree)) {
			primary.range = 0.0;
			secondary.range = 0.0;
		}
		if (primary.range < config_.min_range || primary.range > config_.max_range)
			primary.range = 0.0;
		if (secondary.range < config_.min_range || secondary.range > config_.max_range)
			secondary.range = 0.0;
	}
}

void N10PModule::clearAccumulator()
{
	for (ScanPoint &point : accumulator_)
		writeZero(point);
}

bool N10PModule::processPacket(const unsigned char *packet_bytes, int len, Sweep &sweep)
{
	/*
	 * DF-04：一个 108 字节包进入这里；本函数只改变内部 accumulator_。
	 * DF-05：跨零点时把上一圈复制到 sweep，并清空内部缓存。
	 * DF-06：返回 true 后由驱动把 sweep 交给发布线程，当前模块不发布 ROS。
	 */
	sweep.point_count = 0;
	sweep.points.clear();

	double start_degree = 0.0;
	double degree_interval = 0.0;
	int valid_point_count = 0;
	if (!decodePacket(packet_bytes, len, start_degree, degree_interval, valid_point_count))
		return false;

	// 一个包可能同时包含首回波和次回波；两者共享同一个插值角度。
	for (int point = 0; point < kPointsPerPacket; ++point) {
		const int offset = kPrimaryDataOffset + point * kPointStride;
		const uint16_t primary_raw = readBigEndian16(packet_bytes + offset);
		if (primary_raw == 0xffffu)
			continue;
		if (index_ >= kMaxSweepPoints)
			return false;

		const double degree = normalizeDegree(
			start_degree + degree_interval * point / valid_point_count);
		ScanPoint &primary = accumulator_[index_];
		ScanPoint &secondary = accumulator_[index_ + kSecondaryPointOffset];
		primary.degree = degree;
		primary.range = static_cast<double>(primary_raw) / 1000.0;
		primary.intensity = packet_bytes[offset + 2];
		const int secondary_offset = offset + kPointStride / 2;
		secondary.degree = degree;
		secondary.range = static_cast<double>(readBigEndian16(packet_bytes + secondary_offset)) / 1000.0;
		secondary.intensity = packet_bytes[secondary_offset + 2];

		// 当前点属于“下一圈”的第一个点，因此上一圈只发布 index_ 个点，
		// 当前点按旧驱动约定丢弃，下一包重新从累积区 0 位置开始。
		const bool crossed_zero = has_last_degree_ && degree < last_degree_
			&& degree < 5.0 && last_degree_ > 355.0;
		if ((crossed_zero || index_ >= kMaxSweepPoints - 1) && index_ > 10) {
			filterSweep(index_);
			sweep.point_count = index_;
			sweep.points = accumulator_;
			last_degree_ = degree;
			index_ = 0;
			has_last_degree_ = true;
			clearAccumulator();
			return true;
		}

		last_degree_ = degree;
		has_last_degree_ = true;
		++index_;
	}
	return false;
}

} // namespace lslidar_driver
