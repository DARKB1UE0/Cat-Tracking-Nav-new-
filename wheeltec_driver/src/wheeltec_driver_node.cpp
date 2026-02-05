/**
 * @file wheeltec_driver_node.cpp
 * @brief WHEELTEC麦克纳姆轮机器人ROS2驱动节点
 * 
 * 功能：
 * - 串口通信与底盘控制
 * - 发布里程计(odom)
 * - 发布IMU数据
 * - 订阅cmd_vel速度指令
 * - 发布TF变换
 */

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "wheeltec_driver/serial_port.hpp"
#include "wheeltec_driver/serial_protocol.hpp"
#include "wheeltec_driver/robot_params.hpp"

using namespace std::chrono_literals;

namespace wheeltec_driver
{

class WheeltecDriverNode : public rclcpp::Node
{
public:
  WheeltecDriverNode()
  : Node("wheeltec_driver_node"),
    x_(0.0), y_(0.0), theta_(0.0),
    last_cmd_time_(this->now())
  {
    // 声明参数
    this->declare_parameter<std::string>("serial_port", SERIAL_PORT_DEFAULT);
    this->declare_parameter<int>("baudrate", SERIAL_BAUDRATE_DEFAULT);
    this->declare_parameter<std::string>("odom_frame", ODOM_FRAME_DEFAULT);
    this->declare_parameter<std::string>("base_frame", BASE_FRAME_DEFAULT);
    this->declare_parameter<std::string>("imu_frame", IMU_FRAME_DEFAULT);
    this->declare_parameter<double>("control_rate", static_cast<double>(CONTROL_RATE_DEFAULT));
    this->declare_parameter<double>("cmd_vel_timeout", CMD_VEL_TIMEOUT_DEFAULT);
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<double>("wheel_separation", ROBOT_WHEEL_SEPARATION);
    this->declare_parameter<double>("wheel_base", ROBOT_WHEEL_BASE);
    this->declare_parameter<double>("wheel_diameter", ROBOT_WHEEL_DIAMETER);
    this->declare_parameter<double>("max_linear_velocity", MAX_LINEAR_VELOCITY);
    this->declare_parameter<double>("max_angular_velocity", MAX_ANGULAR_VELOCITY);
    this->declare_parameter<double>("imu_accel_scale", IMU_ACCEL_SCALE);
    this->declare_parameter<double>("imu_gyro_scale", IMU_GYRO_SCALE);

    // 获取参数
    serial_port_name_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    imu_frame_ = this->get_parameter("imu_frame").as_string();
    control_rate_ = this->get_parameter("control_rate").as_double();
    cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    imu_accel_scale_ = this->get_parameter("imu_accel_scale").as_double();
    imu_gyro_scale_ = this->get_parameter("imu_gyro_scale").as_double();

    // 初始化串口
    if (!serial_.open(serial_port_name_, baudrate_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_name_.c_str());
      throw std::runtime_error("Failed to open serial port");
    }
    RCLCPP_INFO(this->get_logger(), "Serial port opened: %s @ %d baud", 
                serial_port_name_.c_str(), baudrate_);

    // 创建发布者
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_voltage", 10);

    // 创建订阅者
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&WheeltecDriverNode::cmd_vel_callback, this, std::placeholders::_1));

    // 创建TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 创建定时器
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WheeltecDriverNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Wheeltec driver node started");
  }

  ~WheeltecDriverNode()
  {
    // 停止机器人
    send_velocity(0.0, 0.0, 0.0);
    serial_.close();
  }

private:
  /**
   * @brief cmd_vel回调函数
   */
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 限制速度
    target_vx_ = std::clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    target_vy_ = std::clamp(msg->linear.y, -max_linear_velocity_, max_linear_velocity_);
    target_vz_ = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
    
    last_cmd_time_ = this->now();
  }

  /**
   * @brief 定时器回调函数
   */
  void timer_callback()
  {
    // 检查速度指令超时
    auto now = this->now();
    if ((now - last_cmd_time_).seconds() > cmd_vel_timeout_) {
      target_vx_ = 0.0;
      target_vy_ = 0.0;
      target_vz_ = 0.0;
    }

    // 发送速度指令
    send_velocity(target_vx_, target_vy_, target_vz_);

    // 读取底盘反馈
    read_feedback();
  }

  /**
   * @brief 发送速度指令
   */
  void send_velocity(double vx, double vy, double vz)
  {
    auto frame = build_tx_frame(vx, vy, vz);
    serial_.write(frame.data(), frame.size());
  }

  /**
   * @brief 读取底盘反馈数据
   */
  void read_feedback()
  {
    // 查找帧头
    uint8_t buffer[256];
    int available = serial_.available();
    
    if (available < static_cast<int>(RX_FRAME_LENGTH)) {
      return;
    }

    // 读取数据
    ssize_t bytes_read = serial_.read(buffer, std::min(available, 256));
    if (bytes_read < 0) {
      RCLCPP_WARN(this->get_logger(), "Serial read error");
      return;
    }

    // 在缓冲区中查找有效帧
    for (size_t i = 0; i <= static_cast<size_t>(bytes_read) - RX_FRAME_LENGTH; ++i) {
      if (buffer[i] == FRAME_HEADER && buffer[i + RX_FRAME_LENGTH - 1] == FRAME_TAIL) {
        RobotState state;
        if (parse_rx_frame(&buffer[i], state)) {
          process_feedback(state);
          break;
        }
      }
    }
  }

  /**
   * @brief 处理反馈数据
   */
  void process_feedback(const RobotState& state)
  {
    auto now = this->now();
    
    // 计算时间差
    if (last_feedback_time_.nanoseconds() == 0) {
      last_feedback_time_ = now;
      return;
    }
    
    double dt = (now - last_feedback_time_).seconds();
    last_feedback_time_ = now;

    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    // 更新里程计
    // 麦克纳姆轮运动学：可以全向移动
    double delta_x = (state.vx * std::cos(theta_) - state.vy * std::sin(theta_)) * dt;
    double delta_y = (state.vx * std::sin(theta_) + state.vy * std::cos(theta_)) * dt;
    double delta_theta = state.vz * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // 归一化角度到 [-π, π]
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    // 发布里程计
    publish_odometry(state, now);

    // 发布IMU
    publish_imu(state, now);

    // 发布电压
    publish_voltage(state.voltage);

    // 发布TF
    if (publish_tf_) {
      publish_transform(now);
    }
  }

  /**
   * @brief 发布里程计消息
   */
  void publish_odometry(const RobotState& state, const rclcpp::Time& stamp)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // 位置
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // 姿态（四元数）
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // 速度（在base_link坐标系下）
    odom_msg.twist.twist.linear.x = state.vx;
    odom_msg.twist.twist.linear.y = state.vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = state.vz;

    // 协方差 (粗略估计)
    odom_msg.pose.covariance[0] = 0.01;   // x
    odom_msg.pose.covariance[7] = 0.01;   // y
    odom_msg.pose.covariance[35] = 0.01;  // yaw
    odom_msg.twist.covariance[0] = 0.01;  // vx
    odom_msg.twist.covariance[7] = 0.01;  // vy
    odom_msg.twist.covariance[35] = 0.01; // vz

    odom_pub_->publish(odom_msg);
  }

  /**
   * @brief 发布IMU消息
   */
  void publish_imu(const RobotState& state, const rclcpp::Time& stamp)
  {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = imu_frame_;

    // 角速度
    imu_msg.angular_velocity.x = state.gyro_x;
    imu_msg.angular_velocity.y = state.gyro_y;
    imu_msg.angular_velocity.z = state.gyro_z;

    // 线加速度
    imu_msg.linear_acceleration.x = state.accel_x;
    imu_msg.linear_acceleration.y = state.accel_y;
    imu_msg.linear_acceleration.z = state.accel_z;

    // 我们不提供姿态估计，设置协方差为-1表示无效
    imu_msg.orientation_covariance[0] = -1;

    // 角速度协方差
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;

    // 线加速度协方差
    imu_msg.linear_acceleration_covariance[0] = 0.1;
    imu_msg.linear_acceleration_covariance[4] = 0.1;
    imu_msg.linear_acceleration_covariance[8] = 0.1;

    imu_pub_->publish(imu_msg);
  }

  /**
   * @brief 发布电池电压
   */
  void publish_voltage(double voltage)
  {
    std_msgs::msg::Float32 voltage_msg;
    voltage_msg.data = static_cast<float>(voltage);
    voltage_pub_->publish(voltage_msg);
  }

  /**
   * @brief 发布TF变换
   */
  void publish_transform(const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);
  }

  // 串口
  SerialPort serial_;
  std::string serial_port_name_;
  int baudrate_;

  // 坐标系
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;

  // 控制参数
  double control_rate_;
  double cmd_vel_timeout_;
  bool publish_tf_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double imu_accel_scale_;
  double imu_gyro_scale_;

  // 目标速度
  double target_vx_ = 0.0;
  double target_vy_ = 0.0;
  double target_vz_ = 0.0;

  // 里程计状态
  double x_, y_, theta_;

  // 时间戳
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_feedback_time_;

  // ROS接口
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wheeltec_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<wheeltec_driver::WheeltecDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("wheeltec_driver"), "Failed to start node: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
