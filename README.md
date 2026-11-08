# S10 + Piper Reinforcement-Learning Deployment

[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.11.0-silver)](https://github.com/google-deepmind/mujoco)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![PICO XR](https://img.shields.io/badge/XR-PICO-00AEEF)](https://www.picoxr.com/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)](https://en.cppreference.com/w/cpp/17)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3.10/)
[![x86_64](https://img.shields.io/badge/platform-x86__64-lightgrey)](README_EN.md#1-sim-to-sim)
[![ARM64](https://img.shields.io/badge/platform-ARM64-lightgrey)](README_EN.md#2-sim-to-real)
[![Discord](https://img.shields.io/badge/Discord-5865F2?logo=discord&logoColor=white)](https://discord.gg/gdM9mQutC8)

English | [简体中文](README.md)

<table>
  <tr>
    <td align="center"><strong>Sim-to-Sim</strong><br><img src="docs/images/s10_piper_demo_1_2x.gif" alt="S10 and Piper coordinated simulation demo" width="420"></td>
    <td align="center"><strong>Real-robot demo 1</strong><br><a href="docs/videos/s10_piper_real_demo_2.mp4"><img src="docs/videos/s10_piper_real_demo_2.gif" alt="S10 and Piper real-robot demo 2" width="420"></a></td>
    <td align="center"><strong>Real-robot demo 2</strong><br><a href="docs/videos/s10_piper_real_demo_1.mp4"><img src="docs/videos/s10_piper_real_demo_1.gif" alt="S10 and Piper real-robot demo 1" width="420"></a></td>
  </tr>
</table>

<p align="center">S10 and Piper coordinated-control demo (click a real-robot GIF to open the MP4)</p>

Mount a Piper robotic arm on an S10 quadruped and run a coordinated whole-body policy. This repository includes PC simulation, deployment on the S10 controller (referred to as **103** below), and Pico VR control. Start with simulation to learn the controls, then use the same controls on the real robot.

The project directory is `S10piper_sim_real`, the policy is `103/rl_deploy/policy/ppo/policy.onnx`, and the deployment program is `rl_deploy`. The commands below assume the repository is cloned under the PC user's home directory. If you clone it elsewhere, adjust the project path accordingly.

## Get the Source

```bash
cd "$HOME"
git clone https://github.com/li269150/S10piper_sim_real.git
cd S10piper_sim_real
```

## 1. Sim-to-Sim

### Set up and run

Use an Ubuntu 22.04 x86_64 PC with Conda installed and initialized, a working graphical desktop, and permission to install system packages with `sudo`. Simulation does not require ROS 2, Piper SDK, or a robot connection. Run these commands once:

```bash
cd "$HOME/S10piper_sim_real"  # Enter the cloned project directory
sudo apt update  # Refresh the system package index
sudo apt install -y build-essential cmake python3-dev \
  libgl1 libgl1-mesa-dri libglfw3 libx11-6 libxrandr2 libxinerama1 libxcursor1 libxi6  # Install build and graphics dependencies
conda create -n s10 python=3.10 pip -y  # Create the Python 3.10 simulation environment; run once
conda activate s10  # Activate the simulation environment
unset PYTHONPATH PYTHONHOME  # Keep system/ROS Python packages out of this simulation terminal
python -m pip install -r sim/requirements-lock.txt  # Install the pinned simulation dependencies
python -m pip check  # Check for Python dependency conflicts
cd sim  # Enter the simulation directory
./build.sh  # Build the simulation with -j20
./run.sh  # Start the simulation and accept keyboard input in this terminal
```

For later sessions, only run:

```bash
conda activate s10  # Activate the existing environment
unset PYTHONPATH PYTHONHOME  # Keep system/ROS Python packages out of this simulation terminal
cd "$HOME/S10piper_sim_real/sim"  # Enter the simulation directory
./run.sh  # Start simulation; no real hardware is required
```

For a machine without a graphical desktop, use `./run.sh --headless` from the same `sim` directory. This still requires an interactive terminal: enter `z`, then `c`, and press `Ctrl+C` to finish. It does not start the policy or exit automatically. If `conda activate` is unavailable, initialize Conda for Bash and reopen the terminal first. If an `s10` environment already exists, reuse it or create a differently named environment and use that name in every activation command.

### Check that startup succeeded

1. Wait for `[SIM] Arm bridge ready.` and the keyboard instructions; the robot starts crouched.
2. With focus in the **launch terminal**, tap lowercase `z` without Enter. Allow the standing trajectory and arm movement to finish; the terminal shows `idle_state -> standup_state`.
3. Tap lowercase `c`. Wait for `standup_state -> rl_control` before sending arm or base commands. If that transition does not appear, keep the base stopped and inspect the full log.
4. On exit, wait for `[SIM] Stopped.` before starting another session.

### Keyboard controls

Keep focus on the terminal that launched the program and use **lowercase letters**. Press `z` first; after the robot is standing, press `c`. Simulation and real-hardware modes use the same keys.

| Key | Action |
| --- | --- |
| `z` → `c` | `z`: stand up. After the robot is stable and the arm reaches its training home pose, press `c` to enter policy control. |
| `i` / `k` | Move the arm end-effector target forward / backward. |
| `j` / `l` | Move the arm end-effector target left / right. |
| `u` / `o` | Move the arm end-effector target up / down. |
| `h` | Return to the training home pose and clear the base command. |
| `n` / `m` | Open / close the gripper. |
| `w` / `s` | Increase forward / backward base speed. Releasing the key does not stop the base. |
| `q` / `e` | Adjust left / right yaw speed. |
| Space | Clear the base velocity command. This is not a hardware emergency stop. |
| `Ctrl+C` | Exit normally. See the real-hardware shutdown sequence below. |

For a basic demonstration, tap `o` to lower the end effector and body, then tap `u` to raise them. Try `i/k` and `j/l` for forward/backward and left/right arm motion. Next, apply small `w/s` and `q/e` inputs to observe base motion at different body heights. Press Space to stop the base, then press `Ctrl+C` to exit.

Simulation uses local loopback communication only and does not connect to the real robot. The full log is stored under `sim/logs/<current-run>/run.log`; `state.json` contains the latest state and `telemetry.json` is written on shutdown. Startup and arm diagnostics may also appear in Chinese. An occasional IK slew-limit message means the target is approached over several frames; repeated feedback, connection, or joint-limit faults require investigation.

Arm directions are relative to the robot: +X forward, +Y left, and +Z up. This controller adjusts the end-effector position through J1–J3; J4–J6 remain at the training home orientation. It does not expose independent wrist/orientation control.

Each `w/s` tap changes the forward command by 0.05; each `q/e` tap changes yaw by 0.20. Selecting forward/reverse clears yaw, and selecting yaw clears forward/reverse; these commands do not combine. Sideways base driving is disabled. Arm taps change the requested target by 0.02 m, subject to workspace limits. Small vertical commands may change posture without a clearly visible body-height change. Space clears the requested base velocity; the current policy can still drift with a zero command and does not provide position hold.

If no window opens, verify the desktop/display connection or use `--headless`. If `pip check` reports unrelated ROS packages, make sure `PYTHONPATH` and `PYTHONHOME` are unset in this simulation terminal. If a local port is already in use, stop your previous session cleanly or run `./run.sh --port-base 12880`. The default local ports are UDP 11880/11882 and TCP 11999.

## 2. Sim-to-Real

### Which device runs each part?

| Device | Required environment | Files / role |
| --- | --- | --- |
| PC | Simulation: Ubuntu 22.04 x86_64, Python 3.10; upload: SSH/SCP | Clone the complete repo; run `sim/` locally; upload the subdirectories below. |
| S10 controller (103) | ARM64, preinstalled ROS 2 Jazzy, C++17 compiler, CMake, `colcon`, and working manufacturer SDK/DDS access | `/home/user/rl_deploy`: policy and S10 controller. |
| Orin | Python 3.10, NumPy, Piper SDK 0.6.1, CAN tools and permissions; PXREA service/SDK for VR | `/home/user/piper`: arm service; `/home/user/pico_bridge`: VR bridge. |

`103` is the name used here for the onboard controller, not a fourth computer or a fixed IP suffix. `user` and `/home/user` below are example remote credentials and paths: replace both with the account and home directory on each device. The PC simulation setup does not install the controller or Orin prerequisites.

### Hardware installation and network

Secure the Piper arm, Orin, and all cables before deployment. Follow the manufacturers' documentation to prepare the S10 SDK, communication environment, and control permission. Install the Piper SDK and its dependencies on Orin.

On the reference robot, the three side aviation connectors are two integrated Ethernet/24 V connectors and one 72 V connector. Piper uses 24 V power, while Orin is powered through a 72 V-to-19 V regulator. Keep the robot safely supported, clear the full arm workspace, and keep the hardware emergency stop accessible during testing.

![Two 24 V plus network aviation connectors and one 72 V power aviation connector on the reference S10](docs/images/hardware_installation.png)

*Figure 1. Two 24 V plus network connectors and one 72 V power connector on the reference robot. Users must follow the pinout of their own hardware.*

The 3D-printed structural parts used in this project are also included in the open-source package under `docs/3d_print/`. The `leige3Ddayin.SLDPRT` file is the SolidWorks source for a part that helps mount the Orin, Piper arm, and related components on the S10 platform. Check the dimensions, fasteners, clearances, and load capacity against your own hardware before printing or operating the robot.

This guide uses the Wi-Fi arrangement from the demonstration: the PC and Orin connect to the robot hotspot, 103 runs the policy, and Orin controls Piper. Piper communicates with Orin through USB-CAN / SocketCAN.

```text
PC ──SSH / SCP──> 103: policy ──ROS 2 / drdds──> S10 legs and wheels
                         └──TCP 9999──> Orin ──CAN──> Piper
Pico ──robot Wi-Fi──> Orin: VR bridge ──TCP 8089──> 103
```

Connect the PC and Orin to the hotspot in their respective terminals. Skip this step if already connected. Replace the SSID and password with those of your robot:

```bash
sudo nmcli radio wifi on  # Enable Wi-Fi
sudo nmcli --ask device wifi connect "YOUR_ROBOT_HOTSPOT"  # Connect and enter the password when prompted
```

The reference addresses below are `10.21.33.103` for 103 and `10.21.41.12` for Orin. **`10.21.41.12` is the author's current reference-Orin address, not a universal default. Replace it with the actual address of your own Orin on the robot network everywhere below, including SSH, SCP, XRoboToolkit, and configuration files.** The examples use the default SSH port 22 and prompt for each device's account password; preconfigured SSH keys are not required. Verify the host fingerprint on the first connection. For another robot, update the configuration files listed in Section 4.

### Copy files to 103 and Orin

Open a terminal on the PC, enter the complete project directory, and upload the source. If a directory with the same name already exists remotely, back it up first instead of overwriting a running environment.

```bash
cd "$HOME/S10piper_sim_real"  # Enter the project root on the PC
scp -r 103/rl_deploy user@10.21.33.103:/home/user/  # Upload the deployment source to 103
scp -r orin/piper user@10.21.41.12:/home/user/  # Upload the arm service to Orin
```

Before launching anything, update `103/rl_deploy/deploy.conf`, `orin/piper/deploy.conf`, and, for VR, `DEFAULT_ROBOT_IP` in `orin/pico_bridge/s10_piper_vr_bridge.py` as listed in Section 4. Edit before uploading or edit the corresponding files on each remote device afterwards. Verify that 103 can reach Orin on TCP 9999, that Orin's allowlist contains the source IP used by 103, and that Orin can reach 103 on TCP 8089 for VR. `PIPER_CAN_PORT=can2` is a sample CAN interface, not an automatically detected universal default.

Then open two PC terminals and log in to each device. Run all later build and launch commands in the corresponding remote terminal.

```bash
# PC terminal 1
ssh user@10.21.33.103  # Log in to 103 for policy, keyboard, or VR control
```

```bash
# PC terminal 2
ssh user@10.21.41.12  # Log in to Orin for the arm service
```

### Build on 103

103 uses its existing ROS 2 Jazzy and compiler environment. The vendor `drdds` interface source is stored in `vendor/drdds`; the build script installs it inside the project directory and does not modify `/opt/ros`.

```bash
cd /home/user/rl_deploy  # Enter the deployment directory uploaded to 103
./build.sh  # Build drdds, keyboard mode, and VR mode with -j20
```

The build outputs are stored in `install_drdds`, `install`, and `install_vr`. The ARM64 program must be built on 103; the x86_64 simulation executable from the PC cannot be reused on 103. To build only VR, run `./build.sh vr`; to build only keyboard mode, run `./build.sh realtime`.

### Start the arm service on Orin

On your own Orin, prepare Python 3.10, NumPy, Piper SDK (this package targets `piper_sdk==0.6.1`), CAN utilities, and a valid CAN configuration. Obtain Piper SDK from the [official AgileX Robotics Piper SDK repository](https://github.com/agilexrobotics/piper_sdk), follow its installation instructions, and confirm that the arm firmware and SDK interface version are compatible. The Piper SDK source is not bundled in this repository.

```bash
cd /home/user/piper  # Enter the arm-service directory on Orin
./start_s10_piper_layered_server_realtime.sh  # Start the service and perform a low-speed homing sequence after valid feedback is received
```

Wait for J1/J2 to return to the mechanical zero, followed by J3-J6. Leave this terminal running after homing completes. The service receives targets from 103 and returns arm state to 103. If feedback is abnormal, stop and check CAN, the SDK, and wiring before standing the robot or entering policy control.

### Run keyboard control on 103

```bash
cd /home/user/rl_deploy  # Enter the deployment directory on 103
./run.sh  # Start keyboard mode and request the real-robot SDK control lease
```

After the SDK and feedback are healthy, press `z` to stand. Once the robot is stable and the arm reaches its training home pose, press `c` to enter policy control. The controls are identical to simulation: `i/k` forward/backward, `j/l` left/right, `u/o` up/down, `h` training home, `n/m` gripper open/close, `w/s` base forward/backward speed, `q/e` yaw, and Space to clear the base command.

To stop normally, **press `Ctrl+C` in the 103 control terminal first**. Wait for the robot to crouch, the arm to return, and the SDK to exit before stopping the Orin arm service. Pressing `Ctrl+C` directly in the Orin terminal does not home the arm. In an emergency, use the hardware emergency stop instead of the normal shutdown sequence.

## 3. Pico VR Control

### First-time setup and direction calibration

Install and authorize XRoboToolkit on Pico and PXREA RoboticsService plus its ARM64 SDK on Orin **before** starting the bridge. The launcher expects `/opt/apps/roboticsservice/runService.sh`; the bridge searches `/opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so` (and the x64 SDK path). For a different layout, configure `PXREA_SERVICE_ROOT` in the launch script and `PXREA_SDK_LIB` to point to the matching library.

The bundled `vr_axes.json` is a reference calibration. Calibrate for your own operator orientation before the first session and after Pico recentering or an orientation change. Keep the S10 controller and Piper arm service stopped, leave the arm disabled, connect Pico to XRoboToolkit, enable `Controller` and `Send`, then run on Orin:

```bash
cd /home/user/pico_bridge
./run_vr_bridge.sh --self-test  # Offline JSON, axis, and protocol checks; no robot connection
./run_vr_bridge.sh --calibrate-right  # Read controller tracking; never connects to 103
```

Follow the prompts: face the intended operating direction, release right Grip, then hold Grip and move horizontally to your right by 10–20 cm, hold for one second, and release. Return with Grip released and repeat the same rightward stroke. Wait for the calibration-success message and saved `vr_axes.json`. Then start the arm service as described in Section 2 and follow the normal sequence below.

### Startup sequence

Exit the 103 keyboard program normally and keep the Orin arm service running. Keyboard and VR modes are mutually exclusive. On 103, `./build.sh` builds both modes by default; use `./build.sh vr` or `./build.sh realtime` to build either mode separately.

For first-time use, upload the bridge from the PC project root:

```bash
cd "$HOME/S10piper_sim_real"  # Enter the complete project directory on the PC
scp -r orin/pico_bridge user@10.21.41.12:/home/user/  # Upload the VR bridge to Orin
```

Open another PC terminal, log in to Orin, and start the VR bridge:

```bash
ssh user@10.21.41.12  # Open a second Orin session; keep the arm-service terminal running
cd /home/user/pico_bridge  # Enter the VR bridge directory
./run_vr_bridge.sh  # Receive controller tracking and forward commands to 103; default scale is 1.0
```

Connect Pico to the robot Wi-Fi, open XRoboToolkit, and connect to the actual IP of your own Orin (`10.21.41.12` is only the reference address shown in the figure). Confirm that Network shows green `WORKING`, then enable `Controller` and `Send`. Start VR mode on 103 once those Pico settings are ready. The bridge connects to 103 only when fresh controller data is available; it can wait for the 103 listener during startup:

![XRoboToolkit connected to Orin and sending Pico controller data](docs/images/xrobotoolkit_connection.png)

*Figure 2. XRoboToolkit connection example. Replace the PC Service address with the IP of your own Orin.*

```bash
cd /home/user/rl_deploy  # Enter the deployment directory on 103
./run_vr.sh  # Start VR control
```

Confirm `[VR] listening TCP 0.0.0.0:8089` on 103 and `[PXREA Bridge] connected to 103 at ...:8089` on Orin. Press `z` in the 103 terminal, wait for the robot to stand, then press `c` to enter policy control. Center the joysticks and test the right controller with small movements before a full demonstration. For XRoboToolkit installation and operation, refer to the [official website](https://xr-robotics.github.io/). Orin also requires the officially licensed PXREA RoboticsService under `/opt/apps/roboticsservice`. This repository does not include that service, its SDK or shared libraries, or a Pico APK.

### VR controls

| Input | Action |
| --- | --- |
| Left joystick forward/backward | Drive the base forward/backward. |
| Left joystick left/right | Turn the base left/right. |
| Hold right Grip and move the controller | Move the arm target relative to the current reference; releasing Grip holds the target. |
| Press/release right Trigger | Open/close the gripper; closed is the default. |
| Right B | Return to the training home pose, clear the base command, and set the current controller position as the new position origin. Direction calibration is unchanged. |

Arm lateral motion and forward/vertical motion are separate control groups: while moving laterally, X/Z targets are held; while moving in X/Z, Y is held. Release and re-engage Grip to set a new relative-motion reference. If VR data is not updated for more than 300 ms, the bridge stops the base command and freezes the arm target. This is not a hardware emergency stop. To finish, press `Ctrl+C` in the 103 terminal first and wait for a clean shutdown before stopping the Orin services.

### Change VR scale and coordinate mapping

Edit `/home/user/pico_bridge/s10_piper_vr_bridge.py` on Orin near the top of the file:

```python
DEFAULT_ARM_SCALE = 1.0  # 10 cm of hand motion produces about 10 cm of arm-target motion
```

![Location of the VR displacement-scale setting](docs/images/vr_scale_setting.png)

*Figure 3. Location of `DEFAULT_ARM_SCALE` in the bridge script.*

For example, change it to `2.0` for approximately 20 cm of arm-target motion from 10 cm of hand motion. Save the file, stop the bridge, and restart `./run_vr_bridge.sh`. No compilation or command-line suffix is required.

The scale changes **target displacement, not motor speed**. Workspace and rate limits still apply. Coordinate conversion is implemented in `vr_delta_to_robot()` in the same file, while direction calibration is stored in `vr_axes.json` in the same directory. If an axis is reversed, fix the mapping or calibration instead of increasing the scale.

## 4. Configuration for Another Robot

Normal operation uses the short launch commands shown above. Do not add manual `export` statements, addresses, or scale arguments to every launch command. Change the configuration once, then restart the corresponding program.

| File | Setting |
| --- | --- |
| `103/rl_deploy/deploy.conf` | `PIPER_SERVER_IP`: Orin address reachable from 103. The default arm-service port is 9999. |
| `orin/piper/deploy.conf` | `PIPER_CAN_PORT`: actual CAN interface; service port; `PIPER_ALLOWED_IPS`: permitted 103 addresses. |
| Top of `orin/pico_bridge/s10_piper_vr_bridge.py` | `DEFAULT_ROBOT_IP`: 103 address (`10.21.41.1` on the reference robot); `DEFAULT_ARM_SCALE`: motion scale; `DEFAULT_VR_BASE`: joystick base-control switch. |
| `orin/pico_bridge/vr_axes.json` | Controller-axis calibration. |

`10.21.33.103` and `10.21.41.1` are addresses of the same 103 controller on different subnets on the reference robot. Always use addresses that match the actual robot network. The examples use Wi-Fi, but the software communicates by IP and does not require a specific network-interface type; a wired setup only needs mutually reachable addresses.

## Project Structure

```text
S10piper_sim_real/
├── 103/                         # Deployment files for the onboard S10 controller
│   └── rl_deploy/               # Policy, control source, build scripts, and launch scripts
├── orin/                        # Programs running on Orin
│   ├── piper/                   # Piper CAN communication, homing, and real-time control service
│   └── pico_bridge/             # Pico tracking receiver and VR command bridge
├── sim/                         # PC MuJoCo simulation, dependencies, and launch scripts
├── docs/                        # Documentation assets used by the README files
│   ├── images/                  # Demo animation, hardware photo, and operation screenshots
│   ├── videos/                  # Real-robot demo GIFs and MP4 files
│   └── 3d_print/                # 3D-printed structural-part source files
├── verification/                # Provenance notes and offline release-integrity checks
├── README.md                    # Chinese deployment guide
└── README_EN.md                 # English deployment guide
```


## Training reference

This repository contains deployment, simulation, and runtime code; it does not include the training source. For a high-level introduction to whole-body leg-arm reinforcement learning, see [Deep Whole-Body Control](https://github.com/MarkFzp/Deep-Whole-Body-Control) and its [CoRL paper](https://proceedings.mlr.press/v205/fu23a.html).
