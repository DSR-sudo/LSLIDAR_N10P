#ifndef LSLIDAR_DRIVER_N10P_MODULE_HPP_
#define LSLIDAR_DRIVER_N10P_MODULE_HPP_

#include <cstdint>
#include <vector>

#include "lslidar_driver/scan_point.hpp"

namespace lslidar_driver
{

// N10P 的协议解析器只处理字节流，不依赖 ROS，也不触碰驱动锁。
// 数据布局：偏移 5/105 是包首/包尾角度；偏移 7 起每 6 字节是一组
// [首回波距离(2)+强度(1)+次回波距离(2)+强度(1)]。驱动线程把完成的
// 一圈交给发布线程，线程边界由 LslidarDriver 负责。
class N10PModule
{
public:
	static constexpr int kPacketSize = 108;
	static constexpr int kPointsPerPacket = 16;
	static constexpr int kStartDegreeOffset = 5;
	static constexpr int kPrimaryDataOffset = 7;
	static constexpr int kEndDegreeOffset = 105;
	static constexpr int kPointStride = 6;
	static constexpr int kSecondaryPointOffset = 3000;
	static constexpr int kMaxSweepPoints = 2000;
	static constexpr int kAccumulatorSize = 6000;

	struct Config
	{
		// 这些值来自驱动参数；协议字段和包长不在运行时修改。
		double min_range{0.3};
		double max_range{100.0};
		double angle_able_min{0.0};
		double angle_able_max{360.0};
	};

	struct Sweep
	{
		// points[0..point_count) 是首回波，points[3000..3000+point_count)
		// 是次回波，保持旧发布线程的数据布局以兼容现有点云输出。
		int point_count{0};
		std::vector<ScanPoint> points;
	};

	N10PModule();
	explicit N10PModule(const Config &config);

	// 配置变化时清空半圈缓存，防止新旧量程/角度规则混用。
	void configure(const Config &config);
	void reset();

	// 返回 true 表示本包结束了上一圈，并在 sweep 中给出可发布数据。
	// packet_bytes 的所有权始终属于调用者，本模块不会 delete 它。
	// 输入必须是已经同步到 0xa5 0x5a 的完整包；不完整/CRC 错误包会被丢弃。
	bool processPacket(const unsigned char *packet_bytes, int len, Sweep &sweep);

	static uint8_t calculateCrc8(const unsigned char *packet_bytes, int len);
	static bool verifyPacket(const unsigned char *packet_bytes, int len);

private:
	static double normalizeDegree(double degree);
	bool isAngleAllowed(double degree) const;
	bool decodePacket(const unsigned char *packet_bytes, int len,
				  double &start_degree, double &degree_interval,
				  int &valid_point_count) const;
	void filterSweep(int point_count);
	void clearAccumulator();

	Config config_{};
	std::vector<ScanPoint> accumulator_;
	int index_{0};
	double last_degree_{0.0};
	bool has_last_degree_{false};
};

} // namespace lslidar_driver

#endif // LSLIDAR_DRIVER_N10P_MODULE_HPP_
