#!/usr/bin/env python3
"""S10/Piper LOCAL ONLY: MuJoCo body + real Orin IK over a simulated Piper driver."""
import argparse
import json
import math
import os
from pathlib import Path
import socket
import struct
import threading
import time
from types import SimpleNamespace

import mujoco
import numpy as np

from bridge_loader import bridge

ROOT = Path(__file__).resolve().parents[1]
DT = .005
DOG_JOINTS = [f'{leg}_{joint}_joint' for leg in ('fl','fr','hl','hr')
              for joint in ('hipx','hipy','knee','wheel')]
NAMES = DOG_JOINTS + [f'joint{i}' for i in range(1,7)]
ARM_KP = np.array([50.,60.,80.,30.,30.,20.])
ARM_KD = np.array([6.,6.,8.,4.,4.,2.])
LIMIT = np.array([50.,50.,50.,14.]*4+[100.]*6)
DOG_IDLE = [.08,-1.2,2.4,0.,-.08,-1.2,2.4,0.,
            .08,1.2,-2.4,0.,-.08,1.2,-2.4,0.]


class SimPiper:
    """SDK-shaped simulated driver. No SDK interface, device or CAN socket."""
    def __init__(self, max_speed=3.):
        self.lock = threading.RLock()
        self.q = np.zeros(6)
        self.target = self.q.copy()
        self.servo_target = self.q.copy()
        self.stamp = time.time()
        # Idealized initial brake/servo hold at mechanical zero. Explicitly
        # documented simulation assumption, not measured firmware behavior.
        self.enabled = True
        self.speed = 15
        self.max_speed = max_speed
        self.gripper = 0.
        self.feedback_missing = False
        self.motion_count = 0

    def measure(self, q):
        with self.lock:
            self.q = np.array(q)
            self.stamp = time.time()

    def GetArmJointMsgs(self):
        with self.lock:
            return SimpleNamespace(Hz=200., time_stamp=self.stamp-(1. if self.feedback_missing else 0.),
                joint_state=SimpleNamespace(**{f'joint_{i+1}': round(float(v)*bridge.RAD_TO_MILLI_DEG)
                                               for i,v in enumerate(self.q)}))

    def GetArmStatus(self):
        with self.lock:
            return SimpleNamespace(Hz=100.,time_stamp=self.stamp,
                arm_status=SimpleNamespace(ctrl_mode=1,arm_status=0,err_code=0))

    def GetArmLowSpdInfoMsgs(self):
        with self.lock:
            flags = dict(voltage_too_low=False,motor_overheating=False,
                         driver_overcurrent=False,driver_overheating=False,
                         collision_status=False,driver_error_status=False,stall_status=False,
                         driver_enable_status=self.enabled)
            return SimpleNamespace(Hz=100.,time_stamp=self.stamp,**{
                f'motor_{i}': SimpleNamespace(can_id=0x260+i,foc_status=SimpleNamespace(**flags))
                for i in range(1,7)})

    def MotionCtrl_2(self, *args):
        with self.lock:
            if len(args)<3 or args[:2] != (1,1):
                raise RuntimeError('simulation supports only layered MOVE_J')
            self.speed = int(args[2])

    def JointCtrl(self, *target):
        q = np.asarray(target,dtype=float)/bridge.RAD_TO_MILLI_DEG
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('invalid simulated Piper target')
        with self.lock:
            self.target = q
            self.motion_count += 1

    def EnablePiper(self):
        with self.lock:
            self.enabled = True
        return True

    def DisablePiper(self):
        with self.lock:
            self.enabled = False
        return True

    def GripperCtrl(self, opening, *args):
        with self.lock:
            self.gripper = min(.08,max(0.,opening/1.e6))

    def ConnectPort(self, **kwargs):
        pass  # Simulated receive reopen; never opens a device.

    def DisconnectPort(self, **kwargs):
        pass

    def servo(self, dt):
        with self.lock:
            step = self.max_speed*self.speed/100.*dt
            if self.enabled:
                self.servo_target += np.clip(self.target-self.servo_target,-step,step)
            else:
                self.servo_target = self.q.copy()
            return self.servo_target.copy(),self.enabled,self.gripper


class LocalOrin:
    def __init__(self, arm, port, home_delay):
        self.arm = arm
        self.recovery = bridge.ArmRecovery(arm)
        self.zero = bridge.StartupZero(arm,self.recovery,15,30.,1.,home_delay,True)
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.sock.bind(('127.0.0.1',port))
        self.sock.listen(1)
        self.sock.settimeout(.05)
        self.running = True
        self.conn = None
        self.thread = threading.Thread(target=self.run,daemon=True)
        self.thread.start()
        print(f'[SIM Orin] 127.0.0.1:{port}; real IK/server source, simulated driver only',flush=True)

    def run(self):
        while self.running:
            try:
                self.conn,_ = self.sock.accept()
            except socket.timeout:
                if self.zero.active:
                    self.zero.step()
                else:
                    self.recovery.observe()
                continue
            except OSError:
                break
            try:
                bridge.serve_client(self.conn,self.arm,100.,50,1.,.08,50,.08,
                                    link6_rate_hz=100.,recovery=self.recovery,
                                    startup_zero=self.zero,
                                    initially_enabled=self.arm.enabled)
            except (OSError,RuntimeError,ValueError) as exc:
                if self.running:
                    print(f'[SIM Orin] connection ended: {exc}',flush=True)
                self.arm.DisablePiper()
            finally:
                self.conn.close()
                self.conn = None

    def stop(self):
        self.running = False
        self.sock.close()
        if self.conn is not None:
            try:
                self.conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.thread.join(timeout=1.)


def rpy(quat):
    w,x,y,z = quat
    return np.array([math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
                     math.asin(float(np.clip(2*(w*y-z*x),-1,1))),
                     math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))])


class Simulation:
    def __init__(self, args):
        self.args = args
        self.model = mujoco.MjModel.from_xml_path(str(ROOT/'S10_description/S10_Piper/S10_Piper.xml'))
        self.model.opt.timestep = DT/args.physics_substeps
        self.data = mujoco.MjData(self.model)
        ids = [self.model.joint(n).id for n in NAMES]
        self.qadr = self.model.jnt_qposadr[ids]
        self.vadr = self.model.jnt_dofadr[ids]
        self.act = np.array([self.model.actuator(n).id for n in NAMES])
        finger = [self.model.joint('joint7').id,self.model.joint('joint8').id]
        self.fq = self.model.jnt_qposadr[finger]
        self.fv = self.model.jnt_dofadr[finger]
        self.fa = np.array([self.model.actuator('joint7').id,self.model.actuator('joint8').id])
        self.acc = self.model.sensor('linear-acceleration').adr[0]
        self.gyro = self.model.sensor('angular-velocity').adr[0]
        self.data.qpos[:3] = [0.,0.,.225]
        self.data.qpos[3:7] = [1.,0.,0.,0.]
        self.data.qpos[self.qadr[:16]] = DOG_IDLE
        self.data.qpos[self.qadr[16:]] = 0.
        mujoco.mj_forward(self.model,self.data)
        self.arm = SimPiper(args.arm_max_speed)
        self.arm.measure(self.data.qpos[self.qadr[16:]])
        self.orin = LocalOrin(self.arm,args.piper_port,args.home_delay)
        self.command = np.zeros((5,16))
        self.command[1] = DOG_IDLE
        self.body_stamp = None
        self.torque = np.zeros(22)
        self.arm_integral = np.zeros(6)
        self.recv = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.recv.bind(('127.0.0.1',args.command_port))
        self.recv.setblocking(False)
        self.send = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.viewer = None
        if not args.headless:
            from mujoco import viewer
            self.viewer = viewer.launch_passive(self.model,self.data)
            self.viewer.cam.distance=2.8
            self.viewer.cam.azimuth=135
            self.viewer.cam.elevation=-18
        self.samples = []

    def receive(self):
        while True:
            try:
                packet,_ = self.recv.recvfrom(4096)
            except BlockingIOError:
                break
            if len(packet)!=96*4:
                continue
            values = np.frombuffer(packet,dtype='<f4').reshape(6,16)
            if not np.isfinite(values).all():
                raise ValueError('non-finite local body command')
            if np.all(values[5]==4):
                self.command = values[:5].copy()
                self.body_stamp = time.monotonic()

    def step(self):
        self.receive()
        stale = self.body_stamp is None or time.monotonic()-self.body_stamp>.2
        kp,qdes,kd,vdes,tff = self.command
        for _ in range(self.args.physics_substeps):
            self.arm.measure(self.data.qpos[self.qadr[16:]])
            target,enabled,opening = self.arm.servo(self.model.opt.timestep)
            q,dq = self.data.qpos[self.qadr],self.data.qvel[self.vadr]
            body = kp*(qdes-q[:16])+kd*(vdes-dq[:16])+tff
            if stale:
                body[:] = 0.
            error = target-q[16:]
            if enabled:
                # MOVE_J is a firmware position servo, NOT actor arm PD.
                # A bounded integral approximates its zero steady-state error.
                # This is an explicit simulator-only approximation; no qfrc_bias
                # or gravity feed-forward is used, and firmware gains are unknown.
                self.arm_integral = np.clip(self.arm_integral+40.*error*self.model.opt.timestep,-8.,8.)
                arm = ARM_KP*error-ARM_KD*dq[16:]+self.arm_integral
            else:
                self.arm_integral[:] = 0.
                arm = np.zeros(6)
            self.torque = np.clip(np.r_[body,arm],-LIMIT,LIMIT)
            self.data.ctrl[self.act] = self.torque
            grip = min(opening/2.,.035)
            fingers = np.array([grip,-grip])
            self.data.ctrl[self.fa] = np.clip(100*(fingers-self.data.qpos[self.fq])-2*self.data.qvel[self.fv],-10,10)
            mujoco.mj_step(self.model,self.data)
        self.arm.measure(self.data.qpos[self.qadr[16:]])
        angles = rpy(self.data.qpos[3:7])
        payload = np.r_[self.data.time,angles,self.data.sensordata[self.acc:self.acc+3],
                        self.data.sensordata[self.gyro:self.gyro+3],self.data.qpos[self.qadr],
                        self.data.qvel[self.vadr],self.torque].astype('<f4')
        self.send.sendto(payload.tobytes(),('127.0.0.1',self.args.state_port))
        return angles

    def run(self):
        next_tick = time.monotonic()
        count = 0
        print('[SIM_READY] loopback body + simulated Orin ready; no hardware connections',flush=True)
        try:
            while self.viewer is None or self.viewer.is_running():
                delay = next_tick-time.monotonic()
                if delay>0:
                    time.sleep(delay)
                next_tick += DT
                angles = self.step()
                count += 1
                if count%100==0:
                    sample = dict(t=float(self.data.time),root_z=float(self.data.qpos[2]),
                                  rpy=angles.tolist(),q=self.data.qpos[self.qadr].tolist(),
                                  arm_target=self.arm.target.tolist(),arm_enabled=self.arm.enabled,
                                  arm_motion_count=self.arm.motion_count,
                                  startup_zero_complete=self.orin.zero.complete,
                                  body_command=self.command.tolist(),torque=self.torque.tolist())
                    self.samples.append(sample)
                    if count%400==0:
                        print(f'[SIM] t={self.data.time:.1f} z={self.data.qpos[2]:.3f} '
                              f'roll={angles[0]:.3f} pitch={angles[1]:.3f}',flush=True)
                    if self.args.state_file:
                        self.args.state_file.write_text(json.dumps(sample,indent=2))
                    if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                        raise RuntimeError('non-finite MuJoCo state')
                if self.viewer is not None and count%4==0:
                    self.viewer.cam.lookat[:] = self.data.qpos[:3]
                    self.viewer.sync()
        except KeyboardInterrupt:
            pass
        finally:
            if self.args.telemetry:
                self.args.telemetry.write_text(json.dumps(self.samples,indent=2))
            self.orin.stop()
            self.recv.close();self.send.close()
            if self.viewer is not None:
                self.viewer.close()


def main():
    import signal
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--state-port',type=int,default=11880)
    parser.add_argument('--command-port',type=int,default=11882)
    parser.add_argument('--piper-port',type=int,default=11999)
    parser.add_argument('--home-delay',type=float,default=3.)
    parser.add_argument('--physics-substeps',type=int,default=2)
    parser.add_argument('--arm-max-speed',type=float,default=3.)
    parser.add_argument('--state-file',type=Path)
    parser.add_argument('--telemetry',type=Path)
    args = parser.parse_args()
    if (any(not 1024<=p<=65535 for p in (args.state_port,args.command_port,args.piper_port)) or
        len({args.state_port,args.command_port,args.piper_port})!=3 or
        args.physics_substeps<1 or not math.isfinite(args.arm_max_speed) or args.arm_max_speed<=0 or
        not math.isfinite(args.home_delay) or args.home_delay<0):
        parser.error('invalid simulation settings')
    Simulation(args).run()


if __name__=='__main__':
    main()
