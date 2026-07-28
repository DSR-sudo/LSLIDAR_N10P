#ifndef LSLIDAR_DRIVER_SCAN_POINT_HPP_
#define LSLIDAR_DRIVER_SCAN_POINT_HPP_

namespace lslidar_driver
{

// 驱动统一使用的二维扫描点：角度单位为度，距离单位为米。
// N10P 的正角度沿雷达协议方向增加；发布/建系模块再按各自坐标约定转换。
struct ScanPoint
{
	double degree{0.0};
	double range{0.0};
	double intensity{0.0};
};

} // namespace lslidar_driver

#endif // LSLIDAR_DRIVER_SCAN_POINT_HPP_
