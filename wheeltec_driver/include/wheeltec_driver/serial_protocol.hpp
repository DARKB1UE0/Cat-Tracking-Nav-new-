#ifndef WHEELTEC_DRIVER__SERIAL_PROTOCOL_HPP_
#define WHEELTEC_DRIVER__SERIAL_PROTOCOL_HPP_

#include <cstdint>
#include <array>

namespace wheeltec_driver
{

// ============================================================================
// 帧格式定义
// ============================================================================

constexpr uint8_t FRAME_HEADER = 0x7B;  // 帧头
constexpr uint8_t FRAME_TAIL = 0x7D;    // 帧尾

// 发送帧（ROS -> STM32）长度：11字节
constexpr size_t TX_FRAME_LENGTH = 11;

// 接收帧（STM32 -> ROS）长度：24字节
constexpr size_t RX_FRAME_LENGTH = 24;

// ============================================================================
// 控制模式定义
// ============================================================================

enum class ControlMode : uint8_t
{
  NORMAL_MOTION = 0x00,      // 正常运动控制
  AUTO_CHARGE_NAV = 0x01,    // 自动回充导航
  ACKERMANN_NAV = 0x02,      // 阿克曼导航
  IR_DOCKING_SPEED = 0x03,   // 红外对接速度
  IP_CONFIG = 0xFF           // IP地址帧
};

// ============================================================================
// 发送数据帧结构（ROS -> STM32）
// ============================================================================

#pragma pack(push, 1)
struct TxFrame
{
  uint8_t header;       // 帧头 0x7B
  uint8_t mode;         // 控制模式
  uint8_t vx_high;      // X轴速度高字节 (mm/s)
  uint8_t vx_low;       // X轴速度低字节
  uint8_t vy_high;      // Y轴速度高字节 (mm/s)
  uint8_t vy_low;       // Y轴速度低字节
  uint8_t vz_high;      // Z轴角速度高字节 (mrad/s)
  uint8_t vz_low;       // Z轴角速度低字节
  uint8_t reserved;     // 保留字节
  uint8_t checksum;     // BCC校验和
  uint8_t tail;         // 帧尾 0x7D
};
#pragma pack(pop)

// ============================================================================
// 接收数据帧结构（STM32 -> ROS）
// ============================================================================

#pragma pack(push, 1)
struct RxFrame
{
  uint8_t header;           // 帧头 0x7B
  uint8_t status;           // 状态标志 (0=正常, 1=失能)
  int16_t vx;               // X轴实际速度 (mm/s)
  int16_t vy;               // Y轴实际速度 (mm/s)
  int16_t vz;               // Z轴实际角速度 (mrad/s)
  int16_t accel_x;          // 加速度X
  int16_t accel_y;          // 加速度Y
  int16_t accel_z;          // 加速度Z
  int16_t gyro_x;           // 角速度X
  int16_t gyro_y;           // 角速度Y
  int16_t gyro_z;           // 角速度Z
  uint16_t voltage;         // 电池电压 (mV)
  uint8_t checksum;         // BCC校验和
  uint8_t tail;             // 帧尾 0x7D
};
#pragma pack(pop)

// ============================================================================
// 解析后的机器人状态
// ============================================================================

struct RobotState
{
  // 速度 (m/s, rad/s)
  double vx;
  double vy;
  double vz;
  
  // IMU加速度 (m/s^2)
  double accel_x;
  double accel_y;
  double accel_z;
  
  // IMU角速度 (rad/s)
  double gyro_x;
  double gyro_y;
  double gyro_z;
  
  // 电池电压 (V)
  double voltage;
  
  // 电机状态
  bool motor_enabled;
};

// ============================================================================
// 工具函数
// ============================================================================

/**
 * @brief 计算BCC校验和（异或校验）
 * @param data 数据指针
 * @param length 数据长度
 * @return 校验和
 */
inline uint8_t calculate_bcc(const uint8_t* data, size_t length)
{
  uint8_t bcc = 0;
  for (size_t i = 0; i < length; ++i) {
    bcc ^= data[i];
  }
  return bcc;
}

/**
 * @brief 构建发送帧
 * @param vx X轴速度 (m/s)
 * @param vy Y轴速度 (m/s)
 * @param vz Z轴角速度 (rad/s)
 * @param mode 控制模式
 * @return 发送帧数据
 */
inline std::array<uint8_t, TX_FRAME_LENGTH> build_tx_frame(
  double vx, double vy, double vz,
  ControlMode mode = ControlMode::NORMAL_MOTION)
{
  std::array<uint8_t, TX_FRAME_LENGTH> frame;
  
  // 转换为mm/s和mrad/s
  int16_t vx_mm = static_cast<int16_t>(vx * 1000.0);
  int16_t vy_mm = static_cast<int16_t>(vy * 1000.0);
  int16_t vz_mrad = static_cast<int16_t>(vz * 1000.0);
  
  frame[0] = FRAME_HEADER;
  frame[1] = static_cast<uint8_t>(mode);
  frame[2] = static_cast<uint8_t>((vx_mm >> 8) & 0xFF);
  frame[3] = static_cast<uint8_t>(vx_mm & 0xFF);
  frame[4] = static_cast<uint8_t>((vy_mm >> 8) & 0xFF);
  frame[5] = static_cast<uint8_t>(vy_mm & 0xFF);
  frame[6] = static_cast<uint8_t>((vz_mrad >> 8) & 0xFF);
  frame[7] = static_cast<uint8_t>(vz_mrad & 0xFF);
  frame[8] = 0x00;  // 保留
  frame[9] = calculate_bcc(frame.data(), 9);
  frame[10] = FRAME_TAIL;
  
  return frame;
}

/**
 * @brief 解析接收帧
 * @param data 接收的原始数据
 * @param state 输出的机器人状态
 * @return 解析是否成功
 */
inline bool parse_rx_frame(const uint8_t* data, RobotState& state)
{
  // 检查帧头帧尾
  if (data[0] != FRAME_HEADER || data[RX_FRAME_LENGTH - 1] != FRAME_TAIL) {
    return false;
  }
  
  // 校验
  uint8_t checksum = calculate_bcc(data, RX_FRAME_LENGTH - 2);
  if (checksum != data[RX_FRAME_LENGTH - 2]) {
    return false;
  }
  
  // 解析速度（大端序）
  int16_t vx_raw = static_cast<int16_t>((data[2] << 8) | data[3]);
  int16_t vy_raw = static_cast<int16_t>((data[4] << 8) | data[5]);
  int16_t vz_raw = static_cast<int16_t>((data[6] << 8) | data[7]);
  
  state.vx = vx_raw / 1000.0;
  state.vy = vy_raw / 1000.0;
  state.vz = vz_raw / 1000.0;
  
  // 解析IMU加速度
  int16_t accel_x_raw = static_cast<int16_t>((data[8] << 8) | data[9]);
  int16_t accel_y_raw = static_cast<int16_t>((data[10] << 8) | data[11]);
  int16_t accel_z_raw = static_cast<int16_t>((data[12] << 8) | data[13]);
  
  // TODO: 根据实际IMU量程进行转换，这里假设单位为mg
  state.accel_x = accel_x_raw * 9.8 / 1000.0;
  state.accel_y = accel_y_raw * 9.8 / 1000.0;
  state.accel_z = accel_z_raw * 9.8 / 1000.0;
  
  // 解析IMU角速度
  int16_t gyro_x_raw = static_cast<int16_t>((data[14] << 8) | data[15]);
  int16_t gyro_y_raw = static_cast<int16_t>((data[16] << 8) | data[17]);
  int16_t gyro_z_raw = static_cast<int16_t>((data[18] << 8) | data[19]);
  
  // TODO: 根据实际IMU量程进行转换，这里假设单位为0.01 deg/s
  constexpr double DEG_TO_RAD = 0.017453292519943295;
  state.gyro_x = gyro_x_raw * 0.01 * DEG_TO_RAD;
  state.gyro_y = gyro_y_raw * 0.01 * DEG_TO_RAD;
  state.gyro_z = gyro_z_raw * 0.01 * DEG_TO_RAD;
  
  // 解析电压
  uint16_t voltage_raw = static_cast<uint16_t>((data[20] << 8) | data[21]);
  state.voltage = voltage_raw / 1000.0;
  
  // 电机状态
  state.motor_enabled = (data[1] == 0);
  
  return true;
}

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__SERIAL_PROTOCOL_HPP_
