"""Filter simulation console noise while run.log keeps unfiltered output."""
import codecs
import re

ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
CJK = re.compile(r'[\u3400-\u9fff]')
PROBLEM = re.compile(
    r'\[ERROR\]|\[WARN(?:ING)?\]|\[SIM FATAL\]|Traceback|Exception|'
    r'failed|failure|timeout|unavailable|lose control|tilted|disconnected|'
    r'异常|错误|失败|不可用|中断|超限|超时|拒绝|未更新|⚠|🔄',
    re.I,
)
QUIET = (
    'policy:', '[Layered]', '[StandUp]', 'Loaded ONNX',
    '[SIM_READY]', '[SIM Orin]', 'Enter RL Control State',
    'stand up success', '[Keyboard]', '[SIM ONLY]', '[S10/Piper]',
    '[Timing]', '[Idle]', 'Robot interface started',
    '--- Main Loop', '>>> Loading', '>>> Heartbeat',
)
MENU = re.compile(
    r'^\s*\[\s*(?:W/S|A/D|Q/E|SPACE|I/K|J/L|U/O|J1-J3|N/M|[HZC])\s*\]'
)


class SimConsole:
    def __init__(self, write):
        self.write = write
        self.decoders = {}
        self.pending = {}

    def feed(self, data, source='runner'):
        decoder = self.decoders.setdefault(
            source, codecs.getincrementaldecoder('utf-8')('replace')
        )
        value = self.pending.get(source, '') + decoder.decode(data)
        # Controller progress messages may be terminated by CR instead of LF.
        value = value.replace('\r\n', '\n').replace('\r', '\n')
        lines = value.split('\n')
        self.pending[source] = lines.pop()
        for line in lines:
            self.line(line)

    def flush(self):
        for source, decoder in self.decoders.items():
            value = self.pending.get(source, '') + decoder.decode(b'', final=True)
            if value:
                self.line(value)
        self.pending.clear()
        self.decoders.clear()

    def line(self, line):
        clean = ANSI.sub('', line).strip()
        if not clean:
            return
        if clean == '⚠️ Piper 已失能: Ctrl+C回零完成':
            self.write(b'[SIM] Arm returned to zero; disabled.\n')
            return
        # Diagnose before suppressing chatter, including warnings mixed with
        # state/arm messages. Full original wording remains in run.log.
        if PROBLEM.search(clean):
            if CJK.search(clean):
                is_error = re.search(
                    r'\[ERROR\]|\[SIM FATAL\]|Traceback|Exception|错误|失败',
                    clean,
                    re.I,
                )
                level = 'ERROR' if is_error else 'WARNING'
                clean = f'[SIM {level}] Arm/control diagnostic; see full run.log.'
            self.write((clean + '\n').encode('utf-8'))
            return
        if CJK.search(clean):
            return
        if clean.startswith(QUIET) or MENU.match(clean):
            return
        if re.match(r'^\[SIM\]\s+t=', clean):
            return
        if re.fullmatch(r'[=\-_* ]{3,}', clean):
            return
        if clean == 'S10+Piper State-Machine Control':
            return
        if clean.startswith('[Piper] connected to '):
            clean = '[SIM] Arm bridge ready.'
        if '------------>' in clean:
            clean = '[State] ' + clean.replace('------------>', '->')
        if clean.startswith('[SIM_RUN_PASS]'):
            clean = '[SIM] Finished.'
        self.write((clean + '\n').encode('utf-8'))
