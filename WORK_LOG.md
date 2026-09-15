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
