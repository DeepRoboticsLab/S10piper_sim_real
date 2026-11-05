# S10 硬件接口

`s10_piper_hardware_interface.hpp` 通过 S10 原生 ROS 2 / DDS 读取腿轮和 IMU，并通过 TCP 与 Orin 上的 Piper 服务交互。机械臂电机命令由 Orin 发送，不混入机器狗 16 轴腿轮报文。

控制侧的 22 维关节顺序：

```text
[FL hipx, hipy, knee, wheel,
 FR hipx, hipy, knee, wheel,
 HL hipx, hipy, knee, wheel,
 HR hipx, hipy, knee, wheel,
 Piper joint1 ... joint6]
```

SDK 申请失败、反馈失效和故障处理按代码中的保护流程执行。请勿用其他机器人型号的报文定义替代 S10 的 `drdds`。
