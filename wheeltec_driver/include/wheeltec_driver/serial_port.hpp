#ifndef WHEELTEC_DRIVER__SERIAL_PORT_HPP_
#define WHEELTEC_DRIVER__SERIAL_PORT_HPP_

#include <string>
#include <vector>
#include <cstdint>

namespace wheeltec_driver
{

/**
 * @brief 串口通信类
 */
class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  /**
   * @brief 打开串口
   * @param port 串口设备路径
   * @param baudrate 波特率
   * @return 是否成功
   */
  bool open(const std::string& port, int baudrate);

  /**
   * @brief 关闭串口
   */
  void close();

  /**
   * @brief 检查串口是否打开
   * @return 是否打开
   */
  bool is_open() const;

  /**
   * @brief 写入数据
   * @param data 数据指针
   * @param size 数据大小
   * @return 实际写入的字节数，-1表示错误
   */
  ssize_t write(const uint8_t* data, size_t size);

  /**
   * @brief 读取数据
   * @param buffer 缓冲区指针
   * @param size 最大读取大小
   * @return 实际读取的字节数，-1表示错误
   */
  ssize_t read(uint8_t* buffer, size_t size);

  /**
   * @brief 获取可读取的字节数
   * @return 可读字节数
   */
  int available() const;

  /**
   * @brief 清空接收缓冲区
   */
  void flush_input();

  /**
   * @brief 清空发送缓冲区
   */
  void flush_output();

private:
  int fd_;  // 文件描述符
  bool is_open_;
};

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__SERIAL_PORT_HPP_
