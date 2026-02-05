#include "wheeltec_driver/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <stdexcept>

namespace wheeltec_driver
{

SerialPort::SerialPort()
: fd_(-1), is_open_(false)
{
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string& port, int baudrate)
{
  // 打开串口
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // 获取当前配置
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // 设置波特率
  speed_t baud;
  switch (baudrate) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 921600: baud = B921600; break;
    default:     baud = B115200; break;
  }
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  // 8N1模式（8数据位，无校验，1停止位）
  tty.c_cflag &= ~PARENB;   // 无校验
  tty.c_cflag &= ~CSTOPB;   // 1停止位
  tty.c_cflag &= ~CSIZE;    // 清除数据位掩码
  tty.c_cflag |= CS8;       // 8数据位
  tty.c_cflag &= ~CRTSCTS;  // 无硬件流控
  tty.c_cflag |= CREAD | CLOCAL;  // 使能接收，忽略调制解调器状态

  // 原始模式
  tty.c_lflag &= ~ICANON;   // 非规范模式
  tty.c_lflag &= ~ECHO;     // 关闭回显
  tty.c_lflag &= ~ECHOE;    // 关闭擦除回显
  tty.c_lflag &= ~ECHONL;   // 关闭换行回显
  tty.c_lflag &= ~ISIG;     // 关闭信号处理

  // 输入模式
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // 关闭软件流控
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // 输出模式
  tty.c_oflag &= ~OPOST;    // 原始输出
  tty.c_oflag &= ~ONLCR;    // 关闭换行符转换

  // 控制字符
  tty.c_cc[VMIN] = 0;       // 非阻塞读取
  tty.c_cc[VTIME] = 1;      // 100ms超时

  // 应用配置
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // 清空缓冲区
  tcflush(fd_, TCIOFLUSH);

  is_open_ = true;
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  is_open_ = false;
}

bool SerialPort::is_open() const
{
  return is_open_;
}

ssize_t SerialPort::write(const uint8_t* data, size_t size)
{
  if (!is_open_) {
    return -1;
  }
  return ::write(fd_, data, size);
}

ssize_t SerialPort::read(uint8_t* buffer, size_t size)
{
  if (!is_open_) {
    return -1;
  }
  return ::read(fd_, buffer, size);
}

int SerialPort::available() const
{
  if (!is_open_) {
    return 0;
  }
  int bytes_available = 0;
  ioctl(fd_, FIONREAD, &bytes_available);
  return bytes_available;
}

void SerialPort::flush_input()
{
  if (is_open_) {
    tcflush(fd_, TCIFLUSH);
  }
}

void SerialPort::flush_output()
{
  if (is_open_) {
    tcflush(fd_, TCOFLUSH);
  }
}

}  // namespace wheeltec_driver
