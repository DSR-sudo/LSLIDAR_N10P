// 通用数据处理模块：保留旧型号的单回波/双回波路径，并提供统一的整圈缓存交接。
// N10P 走最前面的独立分支，收到完整一圈后才唤醒扫描发布线程。
#include "lslidar_driver/lslidar_driver.h"

#include <algorithm>
#include <cmath>
#include <ctime>

namespace lslidar_driver
{

uint64_t LslidarDriver::get_gps_stamp(struct tm t)
{
	return static_cast<uint64_t>(timegm(&t));
}

uint8_t LslidarDriver::N10_CalCRC8(unsigned char *data, int len)
{
	uint32_t sum = 0;
	for (int i = 0; i < len; ++i)
		sum += data[i];
	return static_cast<uint8_t>(sum & 0xffu);
}

void LslidarDriver::difop_processing(unsigned char *packet_bytes)
{
	const int raw = packet_bytes[173];
	degree_compensation = static_cast<double>((raw & 0x7f) * 256 + packet_bytes[174]) / 100.0;
	if (raw & 0x80)
		degree_compensation = -degree_compensation;
	first_compensation = false;
}

void LslidarDriver::publishBufferedSweep(int point_count)
{
	if (point_count <= 0)
		return;
	// 解析线程在未持有发布锁时准备数据；只有下面的短临界区会替换备份缓存。
	// 发布线程随后通过条件变量唤醒，避免在协议解析路径构造 ROS 消息。
	for (int point = 0; point < point_count; ++point) {
		if (scan_points_[point].range < min_range || scan_points_[point].range > max_range)
			scan_points_[point].range = 0.0;
		if (point + N10PModule::kSecondaryPointOffset < static_cast<int>(scan_points_.size())
			&& (scan_points_[point + N10PModule::kSecondaryPointOffset].range < min_range
			|| scan_points_[point + N10PModule::kSecondaryPointOffset].range > max_range))
			scan_points_[point + N10PModule::kSecondaryPointOffset].range = 0.0;
	}
	boost::unique_lock<boost::mutex> lock(mutex_);
	count_num = point_count;
	scan_points_bak_ = scan_points_;
	pre_time_ = time_;
	lock.unlock();
	pubscan_cond_.notify_one();
	time_ = get_clock()->now();
	idx = 0;
	for (ScanPoint &point : scan_points_)
		point = ScanPoint{};
}

void LslidarDriver::publishN10PSweep(const N10PModule::Sweep &sweep)
{
	if (sweep.point_count <= 0)
		return;
	// DF-07：N10PModule 已完成协议解码和整圈判断；这里不再重新解释字节。
	// DF-08：把模块结果放进现有备份缓存，保持 /scan、点云和 LCP 的旧消费接口。
	// DF-09：notify_one() 只唤醒发布线程，解析线程不会直接构造 ROS 消息。
	boost::unique_lock<boost::mutex> lock(mutex_);
	count_num = sweep.point_count;
	scan_points_bak_ = sweep.points;
	pre_time_ = time_;
	lock.unlock();
	pubscan_cond_.notify_one();
	time_ = get_clock()->now();
}

void LslidarDriver::data_processing(unsigned char *packet_bytes, int len)
{
	const double degree = (packet_bytes[degree_bits_start] * 256
		+ packet_bytes[degree_bits_start + 1]) / 100.0 + degree_compensation;
	double start_degree = degree;
	while (start_degree < 0) start_degree += 360;
	while (start_degree >= 360) start_degree -= 360;
	double interval = 15.0;
	if (lidar_name == "N10" || lidar_name == "L10") {
		const double end_degree = (packet_bytes[end_degree_bits_start] * 256
			+ packet_bytes[end_degree_bits_start + 1]) / 100.0;
		interval = end_degree >= start_degree ? end_degree - start_degree
			: end_degree + 360.0 - start_degree;
	}
	if (lidar_name == "M10_PLUS" || lidar_name == "M10_P") {
		PACKET_SIZE = len;
		package_points = (PACKET_SIZE - 20) / 2;
	}
	int point_len = (lidar_name == "N10" || lidar_name == "L10") ? 3 : 2;
	int invalid = 0;
	for (int point = 0; point < package_points; ++point) {
		const int offset = data_bits_start + point * point_len;
		if (packet_bytes[offset] * 256 + packet_bytes[offset + 1] == 0xffff)
			++invalid;
	}
	if (use_gps_ts && lidar_name != "N10") {
		pTime.tm_year = packet_bytes[PACKET_SIZE - 12] + 2000 - 1900;
		pTime.tm_mon = packet_bytes[PACKET_SIZE - 11] - 1;
		pTime.tm_mday = packet_bytes[PACKET_SIZE - 10];
		pTime.tm_hour = packet_bytes[PACKET_SIZE - 9];
		pTime.tm_min = packet_bytes[PACKET_SIZE - 8];
		pTime.tm_sec = packet_bytes[PACKET_SIZE - 7];
		sub_second = (packet_bytes[PACKET_SIZE - 6] * 256 + packet_bytes[PACKET_SIZE - 5]) * 1000000
			+ (packet_bytes[PACKET_SIZE - 4] * 256 + packet_bytes[PACKET_SIZE - 3]) * 1000;
		sweep_end_time_gps = get_gps_stamp(pTime);
		sweep_end_time_hardware = sub_second % 1000000000;
	}
	const int valid = package_points - invalid - ((lidar_name == "N10" || lidar_name == "L10") ? 1 : 0);
	if (valid <= 1)
		return;

	for (int point = 0; point < package_points; ++point) {
		const int offset = data_bits_start + point * point_len;
		const int raw_range = packet_bytes[offset] * 256 + packet_bytes[offset + 1];
		if (raw_range == 0xffff)
			continue;
		if (idx >= points_size_)
			return;
		ScanPoint &scan_point = scan_points_[idx];
		if (lidar_name == "N10" || lidar_name == "L10") {
			scan_point.range = raw_range / 1000.0;
			scan_point.intensity = packet_bytes[offset + 2];
		} else if ((lidar_name == "M10_P" || lidar_name == "M10_PLUS") && !high_reflection) {
			scan_point.range = raw_range / 1000.0;
			scan_point.intensity = 0;
		} else {
			scan_point.range = (packet_bytes[offset] & 0x7f) * 256.0
				+ packet_bytes[offset + 1];
			scan_point.range /= 1000.0;
			scan_point.intensity = packet_bytes[offset] & 0x80 ? 255 : 0;
		}
		scan_point.degree = start_degree + interval * point / valid;
		if (scan_point.degree >= 360) scan_point.degree -= 360;
		if ((scan_point.degree < last_degree && scan_point.degree < 5 && last_degree > 355)
			|| idx >= points_size_ - 1) {
			publishBufferedSweep(idx);
			last_degree = scan_point.degree;
			return;
		}
		last_degree = scan_point.degree;
		++idx;
	}
}

void LslidarDriver::data_processing_2(unsigned char *packet_bytes, int len)
{
	if (lidar_name == "N10_P") {
		// DF-04：网络、串口和文本回放最终都从这里进入 N10PModule。
		// 只有完整一圈返回后才进入 DF-07，半圈数据不会触发 LCP 或 /scan。
		N10PModule::Sweep sweep;
		if (n10p_module_.processPacket(packet_bytes, len, sweep))
			publishN10PSweep(sweep);
		return;
	}

	PACKET_SIZE = len;
	package_points = (PACKET_SIZE - 20) / 4;
	int invalid = 0;
	for (int point = 0; point < package_points; ++point) {
		const int offset = data_bits_start + point * 4;
		if (packet_bytes[offset] * 256 + packet_bytes[offset + 1] == 0xffff)
			++invalid;
	}
	if (use_gps_ts) {
		pTime.tm_year = packet_bytes[PACKET_SIZE - 12] + 2000 - 1900;
		pTime.tm_mon = packet_bytes[PACKET_SIZE - 11] - 1;
		pTime.tm_mday = packet_bytes[PACKET_SIZE - 10];
		pTime.tm_hour = packet_bytes[PACKET_SIZE - 9];
		pTime.tm_min = packet_bytes[PACKET_SIZE - 8];
		pTime.tm_sec = packet_bytes[PACKET_SIZE - 7];
		sub_second = (packet_bytes[PACKET_SIZE - 6] * 256 + packet_bytes[PACKET_SIZE - 5]) * 1000000
			+ (packet_bytes[PACKET_SIZE - 4] * 256 + packet_bytes[PACKET_SIZE - 3]) * 1000;
		sweep_end_time_gps = get_gps_stamp(pTime);
		sweep_end_time_hardware = sub_second % 1000000000;
	}
	const int valid = package_points - invalid;
	if (valid <= 1)
		return;
	const double start_degree = (packet_bytes[degree_bits_start] * 256
		+ packet_bytes[degree_bits_start + 1]) / 100.0;
	for (int point = 0; point < package_points; ++point) {
		const int offset = data_bits_start + point * 4;
		if (packet_bytes[offset] * 256 + packet_bytes[offset + 1] == 0xffff)
			continue;
		if (idx >= points_size_ - 1)
			return;
		ScanPoint &primary = scan_points_[idx];
		primary.range = (packet_bytes[offset] * 256 + packet_bytes[offset + 1]) / 1000.0;
		primary.intensity = 0;
		const int second = offset + 2;
		scan_points_[idx + N10PModule::kSecondaryPointOffset].range =
			(packet_bytes[second] * 256 + packet_bytes[second + 1]) / 1000.0;
		scan_points_[idx + N10PModule::kSecondaryPointOffset].intensity = 0;
		primary.degree = start_degree + 15.0 * point / valid;
		if (primary.degree >= 360) primary.degree -= 360;
		if ((primary.degree < last_degree && primary.degree < 5 && last_degree > 355)
			|| idx >= points_size_ - 1) {
			publishBufferedSweep(idx);
			last_degree = primary.degree;
			return;
		}
		last_degree = primary.degree;
		++idx;
	}
}

} // namespace lslidar_driver
