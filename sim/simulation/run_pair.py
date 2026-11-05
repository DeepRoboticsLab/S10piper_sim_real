#!/usr/bin/env python3
"""Start and stop only this local simulation pair; never SSH or request SDK."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import termios
import time
from console_output import SimConsole

ROOT = Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--port-base',type=int,default=11880)
    parser.add_argument('--physics-substeps',type=int,default=2)
    args=parser.parse_args()
    if not 1024<=args.port_base<=65400 or args.physics_substeps<1:
        parser.error('invalid simulation options')
    model=ROOT.parent/'103/rl_deploy/policy/ppo/policy.onnx'
    binary=ROOT/'bin/rl_deploy'
    if not binary.is_file():
        raise SystemExit('Run ./build.sh first')
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    logdir=ROOT/'logs'/stamp
    logdir.mkdir(parents=True)
    env=os.environ.copy()
    env.update(S10_SIM_STATE_PORT=str(args.port_base),S10_SIM_COMMAND_PORT=str(args.port_base+2),
               PIPER_SERVER_PORT=str(args.port_base+119),PIPER_SERVER_IP='127.0.0.1',
               S10_PIPER_MODE='real',S10_PIPER_HOLD_ON_EXIT='0',
               S10_POLICY_PATH=str(model),PYTHONUNBUFFERED='1',PYTHONDONTWRITEBYTECODE='1')
    # Keep all simulation traffic loopback and independent of the real VR 8089
    # and Orin 9999. The binary also unconditionally overrides its Piper IP.
    simcmd=[sys.executable,'-u',str(ROOT/'simulation/layered_mujoco.py'),
            '--state-port',str(args.port_base),'--command-port',str(args.port_base+2),
            '--piper-port',str(args.port_base+119),'--physics-substeps',str(args.physics_substeps),
            '--state-file',str(logdir/'state.json'),'--telemetry',str(logdir/'telemetry.json')]
    if args.headless:simcmd.append('--headless')
    master,slave=pty.openpty()
    controller=sim=None
    old_terminal=None
    tail=b''
    start=time.monotonic()
    log=(logdir/'run.log').open('wb')
    def console_write(data):
        sys.stdout.buffer.write(data);sys.stdout.buffer.flush()
    console=SimConsole(console_write)
    def output(data,source='runner'):
        log.write(data);log.flush()
        console.feed(data,source)
    force_stop=False
    def request_force_stop(_signum,_frame):
        nonlocal force_stop
        force_stop=True
    def pump_output(streams,timeout):
        if not streams:
            if timeout:time.sleep(timeout)
            return
        try:ready,_,_=select.select(list(streams),[],[],timeout)
        except (OSError,ValueError):return
        for fd in ready:
            try:data=os.read(fd,8192)
            except OSError:data=b''
            if data:output(data,streams[fd])
            else:streams.pop(fd,None)
    def stop_child(child,timeout,streams):
        if child is None or child.poll() is not None:return
        try:os.killpg(child.pid,signal.SIGTERM)
        except ProcessLookupError:return
        deadline=time.monotonic()+timeout
        while child.poll() is None and not force_stop:
            remaining=deadline-time.monotonic()
            if remaining<=0:break
            pump_output(streams,min(.05,remaining))
        if child.poll() is None:
            try:os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError:pass
        child.wait()
        # A PTY can remain readable briefly after its slave process exits.
        pump_output(streams,0.)
    try:
        sim=subprocess.Popen(simcmd,cwd=ROOT,env=env,stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
        # Start the unmodified adapter only after simulator binds its endpoints.
        while b'[SIM_READY]' not in tail:
            if sim.poll() is not None:raise RuntimeError('simulator initialization failed')
            if time.monotonic()-start>15:raise RuntimeError('simulator startup timeout')
            ready,_,_=select.select([sim.stdout],[],[],.1)
            if ready:
                chunk=os.read(sim.stdout.fileno(),8192)
                output(chunk,'sim');tail=(tail+chunk)[-65536:]
        controller=subprocess.Popen([str(binary)],cwd=ROOT,env=env,stdin=slave,
                                    stdout=slave,stderr=slave,start_new_session=True)
        os.close(slave);slave=-1
        keyboard=sys.stdin.fileno() if sys.stdin.isatty() else None
        if keyboard is not None:
            old_terminal=termios.tcgetattr(keyboard)
            raw=termios.tcgetattr(keyboard);raw[3]&=~(termios.ICANON|termios.ECHO)
            termios.tcsetattr(keyboard,termios.TCSANOW,raw)
        output(('[SIM] Local simulation only. Z: stand; C: policy.\n'
                'Arm: I/K forward/back, J/L left/right, U/O up/down; H: Home; N/M: gripper.\n'
                'Base: W/S forward/back, Q/E turn; Space: stop base command.\n'
                'Full log: sim/logs/'+stamp+'/run.log\n').encode())
        while True:
            if sim.poll()==0:break
            if sim.poll() is not None or controller.poll() is not None:
                raise RuntimeError(f'simulation/controller exited: {sim.poll()}/{controller.poll()}')
            streams=[master,sim.stdout.fileno()]
            if keyboard is not None:streams.append(keyboard)
            ready,_,_=select.select(streams,[],[],.05)
            for fd in ready:
                try:data=os.read(fd,8192)
                except OSError:data=b''
                if fd==keyboard:
                    if data:os.write(master,data)
                elif data:
                    output(data,'controller' if fd==master else 'sim');tail=(tail+data)[-65536:]
    except KeyboardInterrupt:
        pass
    finally:
        # The first Ctrl+C reaches the main loop and starts a safe shutdown.
        # During cleanup, a second Ctrl+C requests an immediate kill instead
        # of interrupting Popen.wait() and leaking a Python traceback.
        previous_sigint=signal.signal(signal.SIGINT,request_force_stop)
        if old_terminal is not None:
            termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,old_terminal)
        output(b'\n[SIM] Stopping safely; press Ctrl+C again to force stop.\n')
        # Only terminate the two children created here. No broad process-name
        # kill, remote control, mode request or service stop is ever performed.
        streams={master:'controller'}
        if sim is not None and sim.stdout is not None:
            streams[sim.stdout.fileno()]='sim'
        stop_child(controller,12.,streams)
        stop_child(sim,3.,streams)
        while streams:
            before=len(streams);pump_output(streams,0.)
            if len(streams)==before:break
        output(b'[SIM] Stopped.\n')
        console.flush()
        log.close();os.close(master)
        if slave!=-1:os.close(slave)
        signal.signal(signal.SIGINT,previous_sigint)


if __name__=='__main__':
    main()
