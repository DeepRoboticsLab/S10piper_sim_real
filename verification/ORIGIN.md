# 来源、命名与发布边界

本包提供 S10 + Piper 分层控制代码和模型。项目为 `S10piper_sim_real`，控制程序 `rl_deploy`，模型 `policy.onnx`；仿真从 `sim/build.sh` 编译，实物从 `103/rl_deploy/build.sh` 编译。模型字节未修改。

模型 SHA256：`ab5a7b7ff15f84ab0d0d8fa028c4c556912c1b1eab0ee6f22231eb09ba8ea81d`。

重命名覆盖源码目录、编译/运行入口、默认模型路径、显示名称、Piper服务模块和相互引用。已编译的旧 ARM64 程序不能靠改文件名完成迁移，旧程序、源文件快照和历史日志已原样移到整理前的本地备份，不作为当前构建产物分发；新的103程序必须在目标设备重新编译。历史日志未改写成新测试记录。

仿真和实物共用控制源码及模型，但仿真使用本机 UDP/TCP 与模拟执行器。仅电脑离线/仿真验证不会证明实物连续稳定、故障恢复可靠或允许取消保护。实物曾进入策略，随后出现过机械臂重连与角度超限，仍需单独验证。未验证可用的电脑 Wi-Fi DDS 实验入口已移出，当前仅保留仿真、103键盘与VR路线。

整理方式参考 [Lite3_rl_deploy](https://github.com/DeepRoboticsLab/Lite3_rl_deploy)。`103/rl_deploy/vendor/drdds` 为 DEEPRobotics 官方 `deep-robotics-msg` 1.1.0 接口包，来源为 <https://github.com/DeepRoboticsLab/deep-robotics-msg>，保留其 BSD-3-Clause 许可证；它仅编译到项目本地 overlay，不安装到宿主机 `/opt/ros`。发布包不包含 Piper SDK 源码或 PXREA RoboticsService 程序、SDK及动态库；使用者须从官方来源另行取得并遵守其授权。S10网格、模型和其他依赖的授权条件仍需分别审核，不能把目录整理当作完整开源许可。

发布前确认第三方再分发条件，补齐自有源码/模型许可和装配供电资料，移除个人环境、密钥、密码、设备授权和非通用校准。此任务未上传公开仓库，也没有修改或运行在线103/Orin程序。

发布目录按实际运行需求精简，不包含单元测试、诊断工具、自检脚本或历史日志。保留的启动前置检查和故障保护属于正式运行逻辑，不属于可选测试工具。本次未在线103或Orin上重新做实物验收。
