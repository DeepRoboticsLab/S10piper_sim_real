# AGENTS.md

This file is for AI coding assistants and maintainers. Read it together with
`README.md` before changing this repository. The README is the user-facing
deployment guide; this file records project topology and practical debugging
knowledge that is easy to miss when reading code alone.

## Project purpose

This repository deploys a whole-body leg-arm policy for an S10 quadruped with a
Piper arm. It contains PC simulation, the onboard 103 deployment, the Orin arm
service, and the Pico/XRoboToolkit VR bridge. The policy artifact is
`103/rl_deploy/policy/ppo/policy.onnx`; training source is not included.

## Runtime topology

```text
PC --SSH/SCP--> 103 --ROS 2 / drdds--> S10 legs and wheels
                         \--TCP 9999--> Orin --CAN/SocketCAN--> Piper
Pico/XRoboToolkit --robot Wi-Fi--> Orin --UDP bridge--> 103
```

The IP addresses shown in the README are reference-machine examples. Treat all
addresses as user configuration, never as hard-coded universal values. Never
commit passwords, private keys, tokens, or vendor SDK binaries.

## Source of truth

- `103/rl_deploy/`: policy, observation/action wiring, keyboard and VR control,
  build scripts, and runtime launch scripts.
- `orin/piper/`: Piper CAN communication, homing, feedback recovery, and the
  TCP arm service.
- `orin/pico_bridge/`: Pico tracking receiver, coordinate mapping, gripper
  mapping, and the default VR scale.
- `sim/`: MuJoCo simulation, Python environment files, build, and launch scripts.
- `docs/`: README images and 3D-printed mounting-part sources.

Keep generated binaries, run logs, temporary captures, and local credentials
out of commits. Preserve the existing `policy.onnx` path unless a release
layout is intentionally changed in both README files and all launch scripts.

## Controls and expected behavior

Simulation and 103 keyboard mode share the same controls: `z` stand, `c` enter
policy, `i/k` arm forward/backward, `j/l` arm left/right, `u/o` arm up/down,
`h` arm home, `n/m` gripper open/close, `w/s` base forward/backward, `q/e`
yaw, Space clear base velocity, and `Ctrl+C` cleanly exits the policy. VR uses
the left joystick for base motion, the right Grip plus controller motion for
the arm, the Trigger for the gripper, and B for arm Home plus a new position
origin.

The arm and base are coordinated around the end-effector target. Keep the
workspace and rate limits conservative until simulation and a supported real
robot have both been checked.

## Build and deployment

- Simulation uses Conda with Python 3.10. From `sim/`, `./build.sh` builds and
  `./run.sh` starts the local simulation.
- On 103, `build.sh` can build the keyboard and VR deployment targets. Do not
  run keyboard and VR control at the same time.
- On Orin, the Piper service and Pico bridge are independent processes. The
  bridge normally starts with `./run_vr_bridge.sh`; keep network addresses and
  the arm scale in the checked-in configuration rather than adding command-line
  suffixes to every launch command.
- The Piper SDK, vendor RoboticsService, and Pico application are external
  prerequisites. Do not copy them into this repository.

## Practical debugging notes

1. A temporary TCP `EAGAIN`/resource-busy result is network back-pressure, not
   proof that the arm failed. Buffer or skip that status frame and continue on
   the next cycle. Treat `Broken pipe`, `Connection reset`, and a confirmed
   disconnect differently from a short send congestion event.
2. Piper feedback loss must not stop the independent S10 policy. Preserve the
   last arm target and keep the TCP service alive; only resume motion after
   stable feedback. Do not blindly re-home to a stale target after recovery.
3. An invalid encoder angle or missing CAN feedback is a hardware/zero/CAN
   problem. Inspect the actual feedback and CAN wiring before changing limits;
   never force-enable an arm solely to silence a warning.
4. If the VR stream is stale, stop the base command and freeze the arm target.
   On strategy exit or `Ctrl+C`, clear bridge state so the next run does not
   replay an old arm target. This is separate from hardware emergency stop.
5. A large one-frame command can make the arm appear slow because it is being
   rate-limited. Prefer gradual target updates and test axis signs and scale in
   simulation before real hardware.
6. A robot tilt/roll-pitch protection event belongs to the S10 policy safety
   layer, not automatically to Piper. Do not remove such a guard without a
   replacement test and an explicit hardware safety review.

## AI editing rules

- Read the relevant README section and the current `git diff` before editing.
- Make small, reviewable commits; never force-push or rewrite another
  contributor's branch.
- Keep Chinese and English user documentation behaviorally aligned.
- Do not change IPs, gains, limits, homing order, or safety behavior just to
  make a log quieter. Explain the reason and test in simulation first.
- Avoid running real-hardware control commands unless the user explicitly asks
  for that operation and confirms the robot is safe.

## Training context

The training implementation is outside this deployment repository. Describe it
at a high level as whole-body reinforcement learning for stable leg-arm
coordination, with proprioceptive observations, end-effector/task commands,
action/workspace constraints, and simulation randomization to reduce the
sim-to-real gap. For background, see [Deep Whole-Body Control](https://github.com/MarkFzp/Deep-Whole-Body-Control)
and its [CoRL paper](https://proceedings.mlr.press/v205/fu23a.html). These are
conceptual references; this repository's policy architecture and parameters
are not claimed to be identical.

## Future media

When real footage is available, the opening GIF can be replaced or accompanied
by a side-by-side simulation/real image. Do not invent or commit a placeholder
image while preparing the documentation.
