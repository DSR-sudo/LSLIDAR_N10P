// 串口 IO 模块：封装可选超时的读写和可读/可写等待。
// 该层只返回字节数，不理解 N10P 帧头、长度和 CRC。
#include "lslidar_driver/lsiosr.h"

#include <cstdio>
#include <cstring>

namespace lslidar_driver
{

int LSIOSR::waitReadable(int millis)
{
	if (fd_ < 0) return -1;
	fd_set set;
	timeval timeout{};
	while (millis > 0) {
		timeout.tv_sec = millis / 1000;
		timeout.tv_usec = millis % 1000 * 1000;
		millis = 0;
		FD_ZERO(&set); FD_SET(fd_, &set);
		const int result = select(fd_ + 1, &set, nullptr, nullptr, &timeout);
		if (result > 0) return FD_ISSET(fd_, &set) ? 1 : -1;
		if (result < 0) return -1;
	}
	return 0;
}

int LSIOSR::waitWritable(int millis)
{
	if (fd_ < 0) return -1;
	fd_set set;
	timeval timeout{};
	while (millis > 0) {
		timeout.tv_sec = millis / 1000;
		timeout.tv_usec = millis % 1000 * 1000;
		millis = 0;
		FD_ZERO(&set); FD_SET(fd_, &set);
		const int result = select(fd_ + 1, nullptr, &set, nullptr, &timeout);
		if (result > 0) return FD_ISSET(fd_, &set) ? 1 : -1;
		if (result < 0) return -1;
	}
	return 0;
}

int LSIOSR::read(unsigned char *buffer, int length, int timeout)
{
	// timeout>0 时先等待首字节，再尽量读完当前请求；timeout<=0 保持非阻塞语义。
	if (buffer == nullptr || length <= 0) return -1;
	std::memset(buffer, 0, length);
	if (timeout > 0) {
		int result = waitReadable(timeout);
		if (result <= 0) return result == 0 ? 0 : -1;
	}
	int total = 0;
	unsigned char *cursor = buffer;
	while (length > 0) {
		const int result = ::read(fd_, cursor, static_cast<size_t>(length));
		if (result > 0) {
			total += result; length -= result; cursor += result;
			if (length == 0 || timeout <= 0) break;
		} else if (result < 0 && errno != EINTR && errno != EAGAIN) {
			perror("read error"); return -1;
		} else if (timeout <= 0) {
			break;
		}
		if (timeout > 0 && waitReadable(20) <= 0) break;
	}
	return total;
}

int LSIOSR::send(const char *buffer, int length, int timeout)
{
	if (fd_ < 0 || buffer == nullptr || length <= 0) return -1;
	if (timeout > 0) {
		const int result = waitWritable(timeout);
		if (result <= 0) return result == 0 ? 0 : -1;
	}
	int total = 0;
	const char *cursor = buffer;
	while (length > 0) {
		const int result = ::write(fd_, cursor, static_cast<size_t>(length));
		if (result > 0) {
			total += result; length -= result; cursor += result;
			if (length == 0 || timeout <= 0) break;
		} else if (result < 0 && errno != EINTR && errno != EAGAIN) {
			return -1;
		} else if (timeout <= 0) {
			break;
		}
		if (timeout > 0 && waitWritable(50) <= 0) break;
	}
	return total;
}

} // namespace lslidar_driver
