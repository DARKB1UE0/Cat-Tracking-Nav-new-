# 猫车项目工作日志

## 2026-09-15：上位机 ROS 2 与下位机 USB 云台控制

| 字段 | 内容 |
|------|------|
| 日期 | 2026-09-15（北京时间） |
| 记录人 | Codex |
| 类型 | 功能 |
| 模块 | ROS 2、USB CDC、云台控制 |
| 状态 | 待验证 |
| 关联记录 | 下位机 `BasicFramework_F4/WORK_LOG.md` |

#### 目标与背景

通过大疆 C 板 USB CDC 虚拟串口，由 ROS 2 上位机发送 yaw、pitch 角度指令，并接收云台状态反馈。

#### 工作内容

- 固件端新增 USB 云台协议，使用 `AA 55` 帧头、版本、消息类型、序号、长度和 CRC16/IBM 校验。
- 支持设定点、使能、急停、Ping 和状态反馈；100 ms 未收到有效指令时自动停止。
- 上位机新增 `Cat-Tracking-RealSense/ros2_usb_gimbal.py` ROS 2 桥接节点。
- 节点订阅 `/gimbal/command`，消息为 `[yaw_deg, pitch_deg, enable]`。
- 节点发布 `/gimbal/status`，包含当前角度、使能状态、故障码和指令年龄。
- 新增 `tools/gimbal_usb.py`、`docs/gimbal_usb_protocol.md`，并在 `requirements.txt` 增加 `pyserial`。

#### 验证与结果

- 已完成代码和接口静态检查。
- 尚未在 ROS 2 Humble、真实 USB 串口和云台硬件上联调。

#### 问题与限制

- 需要确认 Linux 下实际 USB 设备名和串口权限。
- 下位机 pitch 轴仍需配置真实 DM4310P 的 CAN ID、控制模式和机械限位。

#### 下一步

- [ ] 在 ROS 2 环境安装依赖并启动 USB 桥接节点。
- [ ] 使用 `ros2 topic pub` 验证角度指令、状态回传和急停。
- [ ] 完成 yaw、pitch 单轴低功率测试及整机联调。

### 20260915-02：车体与云台空间坐标转换接口

| 字段 | 内容 |
|------|------|
| 日期 | 2026-09-15（北京时间） |
| 记录人 | Codex |
| 类型 | 功能 |
| 模块 | ROS 2、空间坐标、云台 USB |
| 状态 | 待验证 |
| 关联记录 | 下位机 `BasicFramework_F4/WORK_LOG.md` |

#### 目标与背景

统一上位机车体坐标和下位机云台角度定义，支持目标点到 yaw/pitch 的转换，并提供云台初始安装姿态与最终车身关系配置。

#### 工作内容

- 新增 `tools/gimbal_geometry.py`，约定车体坐标为 x 前、y 左、z 上。
- 实现车体向量到云台 yaw/pitch、云台角度反算车体视线向量、世界坐标目标转换。
- 增加 yaw/pitch 初始零位和方向符号配置，解决安装偏角和电机正方向差异。
- 下位机新增 `gimbal_geometry.h/.c`，并提供 `GimbalUsbSetInitialPose()` 接口。
- 下位机接口将安装初始姿态作为角度转换基准，最终输出仍使用 USB 协议中的角度单位（度）。

#### 验证与结果

- 已完成静态代码检查和接口对照；未连接真实 ROS、IMU、USB 和电机硬件。

#### 问题与限制

- 当前变换假设车体无横滚/俯仰；若车体姿态变化，需要接入完整 TF/quaternion 变换。
- 初始零位和方向符号必须通过实车低速点动标定。

#### 下一步

- [ ] 用实车测量 yaw/pitch 初始零位并写入配置。
- [ ] 接入 ROS TF，验证世界坐标目标转换。
- [ ] 完成目标点、云台角度和电机反馈的闭环联调。

### 20260915-03：双目目标位置在 RViz 地图标注

| 字段 | 内容 |
|------|------|
| 日期 | 2026-09-15（北京时间） |
| 记录人 | Codex |
| 类型 | 功能 |
| 模块 | RealSense 双目、ROS 2 TF、RViz |
| 状态 | 待验证 |
| 关联记录 | 20260915-02 |

#### 目标与背景

利用双目相机得到目标距离和相机坐标，在 RViz 的 `map` 坐标系中标注目标位置。

#### 工作内容

- 新增 `Cat-Tracking-RealSense/ros2_target_marker.py`。
- 节点订阅 `/cat_target_camera`（`geometry_msgs/msg/PointStamped`），点坐标应为相机光学坐标系，单位米。
- 通过 TF 查找相机坐标系到 `map` 的变换，并发布 `/target_marker`（`visualization_msgs/msg/Marker`）供 RViz 显示。
- 使用 18 cm 红色球体标注目标，1 秒无更新自动消失。

#### 验证与结果

- 已完成静态接口检查；未连接 RealSense、TF 树和实际 RViz。

#### 问题与限制

- 必须存在 `map -> ... -> camera_color_optical_frame` 的 TF 链。
- 双目节点需要发布带距离的三维点，不能只发布二维像素。

#### 下一步

- [ ] 将检测节点输出的像素中心和深度转换为相机三维点并发布 `/cat_target_camera`。
- [ ] 在 ROS 2 中启动节点并检查 RViz 标记位置。

### 20260915-04：整合云台姿态与双目目标地图坐标

| 字段 | 内容 |
|------|------|
| 日期 | 2026-09-15（北京时间） |
| 记录人 | Codex |
| 类型 | 功能 |
| 模块 | 云台 USB 状态、相机 TF、目标标注 |
| 状态 | 待验证 |
| 关联记录 | 20260915-03 |

#### 目标与背景

摄像头安装在云台上，视线方向随云台 yaw/pitch 改变，需要把下位机 USB 回传的云台姿态接入相机 TF，再将双目目标位置标注到 RViz 的 `map`。

#### 工作内容

- 修改 `ros2_target_marker.py`，订阅 `/gimbal/status` 获取 yaw/pitch。
- 动态发布 `base_link -> camera_link` TF，使摄像头朝向跟随云台角度。
- 保留 `/cat_target_camera` 三维点到 `map` 的 TF 转换和 `/target_marker` 发布。
- 相机安装平移默认使用 x=0.20 m、z=0.35 m，可通过参数调整。

#### 验证与结果

- 已完成代码级链路整合；尚未连接真实 USB、TF、RealSense 和 RViz 验证姿态方向。

#### 问题与限制

- 必须继续提供 `camera_link -> camera_color_optical_frame` 的固定光学坐标变换。
- yaw/pitch 正负方向和相机安装零位需要实车标定；当前四元数计算采用 yaw 绕 z、pitch 绕 y 的约定。

#### 下一步

- [ ] 发布并检查相机光学坐标系静态 TF。
- [ ] 用 `tf2_echo` 验证云台转动时 camera TF 方向。
- [ ] 由 RealSense 深度生成 `/cat_target_camera` 后检查 RViz 标记位置。

### 20260915-05：代码审查与坐标链路修正

| 字段 | 内容 |
|------|------|
| 日期 | 2026-09-15（北京时间） |
| 记录人 | Codex |
| 类型 | 修复 |
| 模块 | ROS 2 TF、USB 云台协议 |
| 状态 | 待验证 |
| 关联记录 | 20260915-04 |

#### 目标与背景

审查摄像头随云台转动的 TF 和 USB 状态链路，修复会导致 RViz 标记失败或状态帧解析失败的问题。

#### 工作内容

- ROS 节点显式声明相机平移、yaw/pitch 安装偏置参数。
- 增加 `camera_link -> camera_color_optical_frame` 固定光学坐标变换。
- 云台角度更新 `base_link -> camera_link` 动态 TF，并保留 `map` 目标点转换。
- 修复下位机 Ping 回复错误使用设定点结构的问题，统一回复状态帧长度。

#### 验证与结果

- 已完成静态审查；未执行 ROS 2 运行和实车验证。

#### 问题与限制

- 需要用实际安装方向标定 yaw/pitch 偏置及正负号。
- 如果系统已有其他节点发布同名 TF，应避免重复发布。
