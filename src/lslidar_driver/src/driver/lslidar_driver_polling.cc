// 输入轮询模块：统一处理网络、串口和文本回放三种来源。
// 该层只负责帧同步、长度识别、CRC/设备包分流；真正的 N10P 解码在 n10p/ 中完成。
#include "lslidar_driver/lslidar_driver.h"

#include <cmath>
#include <fstream>
#include <string>
#include <unistd.h>

namespace lslidar_driver
{

void LslidarDriver::recvThread_crc(int &count, int &link_time)
{
	link_time = count <= 0 ? link_time + 1 : 0;
	if (link_time <= 150) return;
	serial_->close();
	if (serial_->init() < 0) {
		RCLCPP_ERROR(get_logger(), "serial open fail");
		usleep(200000);
	}
	link_time = 0;
}

int LslidarDriver::receive_data(unsigned char *packet_bytes)
{
	// 串口帧同步顺序：A5、5A、长度字段/固定长度、剩余字节、CRC。
	// N10P 使用固定 108 字节，不读取通用型号的动态长度字段。
	int link_time = 0;
	int count = 0;
	int read_count = 0;
	while (count <= 0) {
		count = serial_->read(packet_bytes, 1);
		recvThread_crc(count, link_time);
	}
	if (packet_bytes[0] != 0xa5) return 0;
	while (read_count <= 0) {
		read_count = serial_->read(packet_bytes + count, 1);
		if (read_count > 0) count += read_count;
		recvThread_crc(read_count, link_time);
	}
	read_count = 0;
	if (packet_bytes[1] != 0x5a) return 0;
	while (read_count <= 0) {
		read_count = serial_->read(packet_bytes + count, 2);
		if (read_count > 0) count += read_count;
		recvThread_crc(read_count, link_time);
	}

	int len = 0;
	if (lidar_name == "M10") len = 92;
	else if (lidar_name == "M10_GPS") len = 102;
	else if (lidar_name == "N10_P") len = N10PModule::kPacketSize;
	else if (lidar_name == "N10" || lidar_name == "L10") len = packet_bytes[2];
	else len = packet_bytes[2] * 256 + packet_bytes[3];
	if (lidar_name == "M10" || lidar_name == "M10_DOUBLE" || lidar_name == "M10_GPS"
		|| lidar_name == "M10_P" || lidar_name == "M10_PLUS")
		if (packet_bytes[2] == 0x55 && packet_bytes[3] == 0x00) len = 188;
	while (count < len) {
		read_count = serial_->read(packet_bytes + count, len - count);
		if (read_count > 0) count += read_count;
		recvThread_crc(read_count, link_time);
	}
	if (lidar_name == "N10_P")
		return N10PModule::verifyPacket(packet_bytes, len) ? len : 0;
	if (lidar_name == "N10" || lidar_name == "L10")
		if (packet_bytes[PACKET_SIZE - 1] != N10_CalCRC8(packet_bytes, PACKET_SIZE - 1)) return 0;
	return len;
}

namespace
{
int fixedPacketLength(const std::string &name, const unsigned char *data)
{
	if (name == "N10_P") return N10PModule::kPacketSize;
	if (name == "N10" || name == "L10") return 58;
	if (name == "M10") return 92;
	if (name == "M10_GPS") return 102;
	return data[1] * 256 + data[2];
}
}

bool LslidarDriver::polling()
{
	if (!is_start) return true;
	/*
	 * 数据流 DF-01：InputSocket/InputPCAP/LSIOSR 只提供原始字节。
	 * 数据流 DF-02：本函数把三种来源统一成 packet_bytes + len。
	 * 数据流 DF-03：通过 CRC 后再进入 data_processing*，失败包不会污染扫描缓存。
	 */
	unsigned char *packet_bytes = new unsigned char[500];
	int len = 0;
	bool difop = false;
	// 网络输入的某些设备包以 5A 开始，先右移一字节补回 A5 帧头。
	// 之后所有来源都复制到 packet_bytes，再统一进入协议分流。
	if (interface_selection == "net") {
		auto packet = lslidar_msgs::msg::LslidarPacket::UniquePtr(new lslidar_msgs::msg::LslidarPacket());
		while (true) {
			len = msop_input_->getPacket(packet);
			if (packet->data[0] == 0x5a) {
				const int packet_len = fixedPacketLength(lidar_name, packet->data.data());
				for (int i = packet_len - 1; i > 0; --i) packet->data[i] = packet->data[i - 1];
				packet->data[0] = 0xa5;
			}
			if (lidar_name == "N10_P") len = N10PModule::kPacketSize;
			else len = fixedPacketLength(lidar_name, packet->data.data());
			if (compensation && (lidar_name == "M10" || lidar_name == "M10_DOUBLE"
				|| lidar_name == "M10_GPS" || lidar_name == "M10_P" || lidar_name == "M10_PLUS")
				&& packet->data[2] == 0x55 && packet->data[3] == 0x00
				&& packet->data[186] == 0xfa && packet->data[187] == 0xfb) {
				len = 188; difop = true;
			}
			if (len <= 0 || len >= 500 || packet->data[0] != 0xa5 || packet->data[1] != 0x5a)
				continue;
			// DF-02：复制到驱动自己的临时缓冲区，后续协议处理不再依赖 ROS 消息对象。
			for (int i = 0; i < len; ++i) packet_bytes[i] = packet->data[i];
			if (lidar_name == "N10_P") {
				if (!N10PModule::verifyPacket(packet_bytes, len)) continue;
			} else if ((lidar_name == "N10" || lidar_name == "L10")
				&& packet_bytes[len - 1] != N10_CalCRC8(packet_bytes, len - 1)) {
				continue;
			}
			break;
		}
	} else if (!in_file_name.empty()) {
		// 文本回放直接调用处理器，不经过 ROS 输入对象，但仍复用 N10P CRC/解码逻辑。
		// 文本回放保留原格式：每行是一个十六进制数据包。
		const int pause_us = static_cast<int>(std::round(1000000.0 / 10.0 / 24.0)) - 135;
		while (true) {
			std::ifstream reader(in_file_name);
			std::string line;
			while (std::getline(reader, line)) {
				for (size_t i = 0; i + 1 < line.size(); ++i) {
					line[i] = line[i] - '0';
					if (line[i] > 9) line[i] = line[i] - 39;
				}
				for (size_t i = 0; i < (line.size() - 1) / 2; ++i)
					packet_bytes[i] = line[2 * i] * 16 + line[2 * i + 1];
				len = lidar_name == "N10_P" ? N10PModule::kPacketSize : fixedPacketLength(lidar_name, packet_bytes);
				if (lidar_name == "N10_P" || lidar_name == "M10_DOUBLE") data_processing_2(packet_bytes, len);
				else data_processing(packet_bytes, len);
				usleep(pause_us);
			}
		}
	} else {
		// 串口 receive_data() 已完成帧同步和校验；这里仅识别 DIFOP 与 MSOP。
		while ((len = receive_data(packet_bytes)) == 0) {}
		if (compensation && (lidar_name == "M10" || lidar_name == "M10_DOUBLE"
			|| lidar_name == "M10_GPS" || lidar_name == "M10_P" || lidar_name == "M10_PLUS")
			&& packet_bytes[2] == 0x55 && packet_bytes[3] == 0x00
			&& packet_bytes[186] == 0xfa && packet_bytes[187] == 0xfb)
			difop = true;
	}
	// DF-03：DIFOP 只更新设备补偿；N10P 进入独立协议模块，其他型号走兼容路径。
	if (difop) difop_processing(packet_bytes);
	else if (lidar_name == "N10_P" || lidar_name == "M10_DOUBLE") data_processing_2(packet_bytes, len);
	else data_processing(packet_bytes, len);
	delete[] packet_bytes;
	return true;
}

} // namespace lslidar_driver
