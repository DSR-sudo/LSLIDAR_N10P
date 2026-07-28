// 串口生命周期和端口参数模块：打开设备、设置波特率/校验位并提供关闭接口。
// 实际 read/write 和 select 等待逻辑放在 lsiosr_io.cpp。
#include "lslidar_driver/lsiosr.h"

namespace lslidar_driver
{

LSIOSR *LSIOSR::instance(std::string name, int speed, int fd)
{
	static LSIOSR serial(name, speed, fd);
	return &serial;
}

LSIOSR::LSIOSR(std::string port, int baud_rate, int fd)
	: port_(port), baud_rate_(baud_rate), fd_(fd)
{
	printf("port = %s, baud_rate = %d\n", port.c_str(), baud_rate);
}

LSIOSR::~LSIOSR()
{
	close();
}

int LSIOSR::setOpt(int bits, uint8_t event, int stops)
{
	// 所有支持型号使用 8 数据位、无校验、1 停止位；波特率由型号参数选择。
	// 这里不处理数据内容，避免串口配置和雷达协议耦合。
	termios new_settings{}, old_settings{};
	if (tcgetattr(fd_, &old_settings) != 0) { perror("SetupSerial 1"); return -1; }
	new_settings.c_cflag |= CLOCAL | CREAD;
	if (bits == 7) new_settings.c_cflag |= CS7;
	else if (bits == 8) new_settings.c_cflag |= CS8;
	if (event == 'O') {
		new_settings.c_iflag |= INPCK | ISTRIP;
		new_settings.c_cflag |= PARENB | PARODD;
	} else if (event == 'E') {
		new_settings.c_iflag |= INPCK | ISTRIP;
		new_settings.c_cflag |= PARENB;
		new_settings.c_cflag &= ~PARODD;
	} else {
		new_settings.c_cflag &= ~PARENB;
	}
	speed_t baud = B460800;
	if (baud_rate_ == 230400) baud = B230400;
	else if (baud_rate_ == 500000) baud = B500000;
	else if (baud_rate_ == 921600) baud = B921600;
	else if (baud_rate_ == 460800) baud = B460800;
	cfsetispeed(&new_settings, baud);
	cfsetospeed(&new_settings, baud);
	if (stops == 1) new_settings.c_cflag &= ~CSTOPB;
	else if (stops == 2) new_settings.c_cflag |= CSTOPB;
	new_settings.c_cc[VTIME] = 0;
	new_settings.c_cc[VMIN] = 0;
	tcflush(fd_, TCIFLUSH);
	if (tcsetattr(fd_, TCSANOW, &new_settings) != 0) { perror("serial set error"); return -1; }
	return 0;
}

void LSIOSR::flushinput()
{
	tcflush(fd_, TCIFLUSH);
}

int LSIOSR::init()
{
	fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd_ <= 0) return -1;
	return setOpt(DATA_BIT_8, PARITY_NONE, STOP_BIT_1);
}

int LSIOSR::close()
{
	if (fd_ >= 0) {
		::close(fd_);
		fd_ = -1;
	}
	return 0;
}

std::string LSIOSR::getPort()
{
	return port_;
}

int LSIOSR::setPortName(std::string name)
{
	port_ = name;
	return 0;
}

} // namespace lslidar_driver
