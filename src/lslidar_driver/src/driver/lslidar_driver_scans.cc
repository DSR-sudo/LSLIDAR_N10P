// 扫描发布模块：把已完成的一圈转换为 LaserScan 和 PointCloud2。
// 雷达协议角度为“顺时针正”，这里统一转换为 ROS 的角度索引和点云坐标。
#include "lslidar_driver/lslidar_driver.h"

#include <cmath>
#include <limits>
#include <utility>

int truncated_mode_ = 0; // 设为 1 可启用下面的固定角度屏蔽区。
int scan_crop_min[] = {0, 180};
int scan_crop_max[] = {90, 270};

namespace lslidar_driver
{
namespace
{
int scanIndex(double degree, int count)
{
	if (count <= 0) return 0;
	int index = static_cast<int>(std::lround((360.0 - degree) * count / 360.0));
	index %= count;
	return index < 0 ? index + count : index;
}

bool angleAllowed(double scan_degree, double min_degree, double max_degree)
{
	if (max_degree > 360.0)
		return scan_degree >= min_degree || scan_degree <= max_degree - 360.0;
	return scan_degree >= min_degree && scan_degree <= max_degree;
}

bool isCropped(int index, int count)
{
	if (!truncated_mode_) return false;
	const int size = sizeof(scan_crop_max) / sizeof(scan_crop_max[0]);
	for (int i = 0; i < size; ++i) {
		if (index >= scan_crop_min[i] * count / 360 && index <= scan_crop_max[i] * count / 360)
			return true;
	}
	return false;
}

void appendPoint(const ScanPoint &scan_point, double degree, double timestamp,
	int index, double scan_time, VPointCloud &cloud)
{
	VPoint point;
	point.timestamp = timestamp - index * scan_time;
	point.x = scan_point.range * std::cos(M_PI / 180.0 * scan_point.degree);
	point.y = -scan_point.range * std::sin(M_PI / 180.0 * scan_point.degree);
	point.z = 0.0;
	point.intensity = scan_point.intensity;
	cloud.points.push_back(point);
	++cloud.width;
	(void)degree;
}
} // namespace

int LslidarDriver::getScan(std::vector<ScanPoint> &points, rclcpp::Time &scan_time,
	float &scan_duration)
{
	boost::unique_lock<boost::mutex> lock(mutex_);
	points = scan_points_bak_;
	scan_time = pre_time_;
	scan_duration = static_cast<float>(time_.seconds() - pre_time_.seconds());
	return 1;
}

void LslidarDriver::pubScanThread()
{
	/*
	 * DF-10：等待 DF-09 的条件变量通知。
	 * DF-11：getScan() 复制一圈，之后本地 vector 与解析线程完全解耦。
	 * DF-12：同一份 vector 依次送入 LCP、LaserScan 和 PointCloud2。
	 */
	bool wait_for_wake = true;
	boost::unique_lock<boost::mutex> lock(pubscan_mutex_);
	while (rclcpp::ok() && !shutting_down_.load()) {
		while (wait_for_wake && !shutting_down_.load()) {
			pubscan_cond_.wait(lock);
			wait_for_wake = false;
		}
		if (shutting_down_.load()) break;

		std::vector<ScanPoint> points;
		rclcpp::Time start_time;
		float scan_duration = 0.0f;
		getScan(points, start_time, scan_duration);
		// DF-11：此处 points 是本发布周期的只读快照，后续不会访问 scan_points_。
		const int count = std::min(count_num, static_cast<int>(points.size()));
		if (count <= 1) {
			wait_for_wake = true;
			continue;
		}
		// LCP 与 LaserScan 使用同一份已完成一圈缓存，确保建系时序稳定。
		// 此时已经离开串口协议线程，满足“完整扫描发布线程中建系”的时序要求。
		// DF-12a：LCP 必须在构造对应 /scan 前处理同一圈完整数据。
		if (lidar_name == "N10_P")
			processLcpScan(points, scan_duration, now());

		if (lidar_name == "N10_P" || lidar_name == "M10_DOUBLE") {
			if (pubScan) {
				// DF-12b：首回波按 degree 映射到 LaserScan；次回波不写入 /scan。
				// LaserScan 只有首回波；次回波继续保留在点云输出中，兼容旧接口。
				sensor_msgs::msg::LaserScan scan;
				scan.header.frame_id = frame_id;
				scan.header.stamp = use_gps_ts ? rclcpp::Time(sweep_end_time_gps, sweep_end_time_hardware) : now();
				scan.angle_min = 0.0;
				scan.angle_max = 2.0 * M_PI;
				scan.angle_increment = 2.0 * M_PI / count;
				scan.range_min = min_range;
				scan.range_max = max_range;
				scan.ranges.assign(count, std::numeric_limits<float>::infinity());
				scan.intensities.assign(count, 0.0f);
				for (int i = 0; i < count; ++i) {
					const int index = scanIndex(points[i].degree, count);
					if (!isCropped(index, count) && points[i].range > 0.0) {
						scan.ranges[index] = static_cast<float>(points[i].range);
						scan.intensities[index] = static_cast<float>(points[i].intensity);
					}
				}
				scan_pub->publish(scan);
			}
			if (pubPointCloud2) {
				// DF-12c：点云同时消费首回波和次回波，保持双回波信息。
				// 点云坐标：x 向前，y 向左；协议角度正方向与 ROS 角度方向相反。
				VPointCloud::Ptr cloud(new VPointCloud());
				const double timestamp = start_time.seconds();
				cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
				cloud->header.frame_id = frame_id;
				cloud->height = 1;
				for (int i = 0; i < count; ++i) {
					const double scan_degree = 360.0 - points[i].degree;
					const int index = scanIndex(points[i].degree, count);
					const double time_increment = scan_duration / count;
					if (points[i].range > 0.001 && angleAllowed(scan_degree, angle_able_min, angle_able_max))
						appendPoint(points[i], scan_degree, timestamp, index, time_increment, *cloud);
					const ScanPoint &secondary = points[i + N10PModule::kSecondaryPointOffset];
					if (secondary.range > 0.001 && angleAllowed(scan_degree, angle_able_min, angle_able_max))
						appendPoint(secondary, scan_degree, timestamp, index, time_increment, *cloud);
				}
				sensor_msgs::msg::PointCloud2 message;
				pcl::toROSMsg(*cloud, message);
				point_cloud_pub->publish(message);
			}
		} else {
			publishGenericScan(points, count, start_time, scan_duration);
		}
		count_num = 0;
		wait_for_wake = true;
		if (first_compensation && compensation)
			lidar_difop();
	}
}

void LslidarDriver::publishGenericScan(const std::vector<ScanPoint> &points, int count,
	const rclcpp::Time &start_time, float scan_duration)
{
	const int scan_count = std::max(1, static_cast<int>(std::ceil(
		(angle_able_max - angle_able_min) / 360.0 * count)) + 1);
	const int start_index = static_cast<int>(std::floor(angle_able_min * count / 360.0));
	const int end_index = static_cast<int>(std::floor(angle_able_max * count / 360.0));
	if (pubScan) {
		sensor_msgs::msg::LaserScan scan;
		scan.header.frame_id = frame_id;
		scan.header.stamp = use_gps_ts ? rclcpp::Time(sweep_end_time_gps, sweep_end_time_hardware) : now();
		scan.angle_min = angle_able_max > 360 ? 2 * M_PI * (angle_able_min - 360) / 360 : 2 * M_PI * angle_able_min / 360;
		scan.angle_max = angle_able_max > 360 ? 2 * M_PI * (angle_able_max - 360) / 360 : 2 * M_PI * angle_able_max / 360;
		scan.angle_increment = 2 * M_PI / std::max(1, count - 1);
		scan.range_min = min_range;
		scan.range_max = max_range;
		scan.scan_time = scan_duration;
		scan.time_increment = scan_duration / std::max(1, count - 1);
		scan.ranges.assign(scan_count, std::numeric_limits<float>::infinity());
		scan.intensities.assign(scan_count, 0.0f);
		for (int i = 0; i < count; ++i) {
			int index = scanIndex(points[i].degree, count);
			if (index < end_index - count) index += count;
			index -= start_index;
			if (index < 0 || index >= scan_count || isCropped(index, count)) continue;
			if (points[i].range > 0.0) scan.ranges[index] = points[i].range;
			scan.intensities[index] = points[i].intensity;
		}
		scan_pub->publish(scan);
	}
	if (!pubPointCloud2) return;
	VPointCloud::Ptr cloud(new VPointCloud());
	const double timestamp = start_time.seconds();
	cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
	cloud->header.frame_id = frame_id;
	cloud->height = 1;
	for (int i = 0; i < count; ++i) {
		const double scan_degree = 360.0 - points[i].degree;
		if (points[i].range > 0.001 && angleAllowed(scan_degree, angle_able_min, angle_able_max))
			appendPoint(points[i], scan_degree, timestamp, scanIndex(points[i].degree, count),
				scan_duration / count, *cloud);
	}
	sensor_msgs::msg::PointCloud2 message;
	pcl::toROSMsg(*cloud, message);
	point_cloud_pub->publish(message);
}

} // namespace lslidar_driver
