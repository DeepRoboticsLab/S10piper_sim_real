"""Load the real Orin control code with a local math shim, NEVER SocketCAN."""
import importlib.util
from pathlib import Path
import sys
import types

PIPER = Path(__file__).resolve().parents[2] / 'orin/piper'


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def install_sdk_shim():
    fk = load_file('sim_piper_fk', Path(__file__).with_name('piper_fk_shim.py'))
    sdk = types.ModuleType('piper_sdk')
    sdk.C_PiperForwardKinematics = fk.C_PiperForwardKinematics

    class NoPhysicalPiper:
        def __init__(self, *args, **kwargs):
            raise RuntimeError('SIM ONLY: physical Piper/CAN creation is forbidden')

    sdk.C_PiperInterface_V2 = NoPhysicalPiper
    sys.modules['piper_sdk'] = sdk


install_sdk_shim()
bridge = load_file('sim_orin_bridge', PIPER / 'app/piper_arm_server.py')
