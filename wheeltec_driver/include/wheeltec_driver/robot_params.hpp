#ifndef WHEELTEC_DRIVER__ROBOT_PARAMS_HPP_
#define WHEELTEC_DRIVER__ROBOT_PARAMS_HPP_

namespace wheeltec_driver
{

// ============================================================================
// 机器人参数配置
// ============================================================================
// 请根据实际机器人参数修改以下值

// ----------------------------------------------------------------------------
// 机器人尺寸参数
// ----------------------------------------------------------------------------

// 轮距（左右轮之间的距离）单位：米
// TODO: 请根据实际机器人测量并修改此值
#ifndef ROBOT_WHEEL_SEPARATION
#define ROBOT_WHEEL_SEPARATION 0.50
#endif

// 轴距（前后轮之间的距离）单位：米
// TODO: 请根据实际机器人测量并修改此值
#ifndef ROBOT_WHEEL_BASE
#define ROBOT_WHEEL_BASE 0.43
#endif

// 轮子直径，单位：米
// TODO: 请根据实际机器人测量并修改此值
#ifndef ROBOT_WHEEL_DIAMETER
#define ROBOT_WHEEL_DIAMETER 0.15
#endif

// ----------------------------------------------------------------------------
// 串口参数
// ----------------------------------------------------------------------------

// 串口设备路径
// TODO: 请根据实际连接修改
#ifndef SERIAL_PORT_DEFAULT
#define SERIAL_PORT_DEFAULT "/dev/ttyUSB0"
#endif

// 波特率
#ifndef SERIAL_BAUDRATE_DEFAULT
#define SERIAL_BAUDRATE_DEFAULT 115200
#endif

// ----------------------------------------------------------------------------
// 坐标系参数
// ----------------------------------------------------------------------------

// 里程计坐标系名称
#ifndef ODOM_FRAME_DEFAULT
#define ODOM_FRAME_DEFAULT "odom"
#endif

// 机器人基座坐标系名称
#ifndef BASE_FRAME_DEFAULT
#define BASE_FRAME_DEFAULT "base_link"
#endif

// IMU坐标系名称
#ifndef IMU_FRAME_DEFAULT
#define IMU_FRAME_DEFAULT "imu_link"
#endif

// ----------------------------------------------------------------------------
// 速度限制
// ----------------------------------------------------------------------------

// 最大线速度 (m/s)
#ifndef MAX_LINEAR_VELOCITY
#define MAX_LINEAR_VELOCITY 1.0
#endif

// 最大角速度 (rad/s)
#ifndef MAX_ANGULAR_VELOCITY
#define MAX_ANGULAR_VELOCITY 3.14
#endif

// ----------------------------------------------------------------------------
// 控制参数
// ----------------------------------------------------------------------------

// 控制频率 (Hz)
#ifndef CONTROL_RATE_DEFAULT
#define CONTROL_RATE_DEFAULT 50
#endif

// 速度指令超时时间 (秒)
#ifndef CMD_VEL_TIMEOUT_DEFAULT
#define CMD_VEL_TIMEOUT_DEFAULT 0.5
#endif

// ----------------------------------------------------------------------------
// IMU参数
// ----------------------------------------------------------------------------

// 加速度计量程 (g)
// 根据实际IMU型号设置，常见值：2, 4, 8, 16
#ifndef IMU_ACCEL_RANGE
#define IMU_ACCEL_RANGE 8
#endif

// 陀螺仪量程 (deg/s)
// 根据实际IMU型号设置，常见值：250, 500, 1000, 2000
#ifndef IMU_GYRO_RANGE
#define IMU_GYRO_RANGE 2000
#endif

// IMU加速度计比例因子
// 原始值 * IMU_ACCEL_SCALE = m/s^2
#ifndef IMU_ACCEL_SCALE
#define IMU_ACCEL_SCALE (9.8 * IMU_ACCEL_RANGE / 32768.0)
#endif

// IMU陀螺仪比例因子
// 原始值 * IMU_GYRO_SCALE = rad/s
#ifndef IMU_GYRO_SCALE
#define IMU_GYRO_SCALE (IMU_GYRO_RANGE * 0.017453292519943295 / 32768.0)
#endif

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__ROBOT_PARAMS_HPP_
