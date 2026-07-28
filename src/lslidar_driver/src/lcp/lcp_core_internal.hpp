#ifndef LSLIDAR_DRIVER_LCP_CORE_INTERNAL_HPP_
#define LSLIDAR_DRIVER_LCP_CORE_INTERNAL_HPP_

#include <array>
#include <cmath>
#include <limits>

namespace lslidar_driver
{
namespace lcp
{
namespace detail
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 0.5 * kPi;
constexpr double kTwoPi = 2.0 * kPi;
constexpr unsigned kSidePositiveX = 0;
constexpr unsigned kSidePositiveY = 1;
constexpr unsigned kSideNegativeX = 2;
constexpr unsigned kSideNegativeY = 3;

inline double clampAngle(double angle)
{
	while (angle < 0.0) angle += kTwoPi;
	while (angle >= kTwoPi) angle -= kTwoPi;
	return angle;
}

template<typename Point>
double sideCoordinate(const Point &point, unsigned side)
{
	return side == kSidePositiveX || side == kSideNegativeX ? point.x : point.y;
}

template<typename Point>
unsigned nearestSide(const Point &point, const std::array<double, 4> &lines,
	double &residual)
{
	unsigned best = 0;
	residual = std::numeric_limits<double>::max();
	for (unsigned side = 0; side < 4; ++side) {
		const double value = std::fabs(sideCoordinate(point, side) - lines[side]);
		if (value < residual) {
			residual = value;
			best = side;
		}
	}
	return best;
}
} // namespace detail
} // namespace lcp
} // namespace lslidar_driver

#endif // LSLIDAR_DRIVER_LCP_CORE_INTERNAL_HPP_
