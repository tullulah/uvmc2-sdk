#!/usr/bin/env python3
"""uvmc2.py — the UVMC2 multicart over USB, from the command line (no IDE needed).

    uvmc2.py deploy build_uvm2/game.um2     put the game on the SD card and run it
    uvmc2.py upload FILE [NAME]             put any file on the SD card (NAME: its path there)
    uvmc2.py run build_uvm2/pico/game.elf   run a game's ELF from RAM (nothing written to the card)
    uvmc2.py log                            the running game's DEBUG_PRINT / vpy_log text
    uvmc2.py gdb [game.elf]                 the port arm-none-eabi-gdb attaches to (and the command)
    uvmc2.py ports                          what is on USB

The cartridge must be on USB, either in the multicart's menu or running a game built with the
USB debug monitor (make uvm2 UVM2_USB_MONITOR=1). Two programs answer, with one protocol:

  - the debug monitor, MONITOR.BIN on the card, loaded into the PSRAM by a monitor build;
  - the SD bridge, a RAM-only program for when the cartridge sits in its menu: the menu's
    firmware has the pico-sdk's USB Reset interface, so `picotool reboot -u -f` sends the chip
    to BOOTSEL and `picotool load -x` runs the bridge (the flash is never written).

Both come built in uvm2-sdk/prebuilt/ (sources: uvm2-sdk/monitor, uvm2-sdk/sdbridge). `deploy`
and `upload` also put MONITOR.BIN on the card, so a monitor build always finds the current one.
`run` ends with the game started the way the multicart starts it: from SRAM.

Needs Python 3.8+ and picotool (macOS: brew install picotool). macOS and Linux.

The protocol (text lines; answers end in CR LF; lines sent unasked start with '!'):
  v                  -> "UVMC2-MON <n>" / "UVMC2-BRIDGE <n>"
  u <size> <path>    -> "OK", <size> raw bytes, then "DONE <size> <crc32 hex>" (or "ERR ...")
  x                  -> "OK", then the chip goes to BOOTSEL (to load a game into SRAM)
  b                  -> "OK"; the monitor resets into the multicart's firmware, the bridge
                        shows DONE and waits for RESET on the console
  "!T <hex>"           the game's text; "!S <why> <pc>" a debugger stop (monitor/debug.c)
"""
import os
import select
import shutil
import subprocess
import sys
import termios
import time
import zlib

VID, PID = 0x2E8A, 0x10DB          # the monitor and the bridge
MENU_PID, BOOTSEL_PID = 0x0009, 0x000F
HERE = os.path.dirname(os.path.abspath(__file__))
PREBUILT = os.path.join(HERE, '..', 'uvm2-sdk', 'prebuilt')


def die(msg):
    print(f'uvmc2: {msg}', file=sys.stderr)
    sys.exit(1)


# ── finding the ports ─────────────────────────────────────────────────────────────────────

def usb_serial_ports(vid, pid):
    """The serial devices of a USB device, its interface 0 first."""
    out = []
    if sys.platform == 'darwin':
        r = subprocess.run(['ioreg', '-l', '-w0', '-r', '-c', 'IOUSBHostDevice'], capture_output=True, text=True)
        v = p = 0
        for line in r.stdout.splitlines():
            if '"idVendor" = ' in line:
                v = int(line.split('= ')[1])
            elif '"idProduct" = ' in line:
                p = int(line.split('= ')[1])
            elif '"IOCalloutDevice" = ' in line and v == vid and p == pid:
                out.append(line.split('"')[3])
    elif sys.platform.startswith('linux'):
        base = '/sys/class/tty'
        for n in sorted(os.listdir(base)) if os.path.isdir(base) else []:
            if not n.startswith('ttyACM'):
                continue
            try:
                dev = os.path.join(base, n, 'device', '..')
                v = int(open(os.path.join(dev, 'idVendor')).read(), 16)
                p = int(open(os.path.join(dev, 'idProduct')).read(), 16)
            except OSError:
                continue
            if v == vid and p == pid:
                out.append('/dev/' + n)
    else:
        die('macOS and Linux only for now')
    return sorted(out)


class Port:
    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = 0                                   # iflag: no CR/LF rewriting, no flow control
        a[1] = 0                                   # oflag: raw out (uploads are binary)
        a[2] |= termios.CLOCAL | termios.CREAD
        a[3] = 0                                   # lflag: no echo, no line editing
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        self.buf = b''

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

    def write(self, data):
        if isinstance(data, str):
            data = data.encode()
        view, sent, idle = memoryview(data), 0, time.time()
        while sent < len(view):
            _, w, _ = select.select([], [self.fd], [], 1)
            try:
                n = os.write(self.fd, view[sent:sent + 4096]) if w else 0
            except BlockingIOError:
                n = 0
            if n:
                sent += n
                idle = time.time()
            elif time.time() - idle > 30:
                raise IOError('the cartridge stopped taking data')

    def _fill(self, timeout):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if r:
            try:
                self.buf += os.read(self.fd, 4096)
            except BlockingIOError:
                pass
            except OSError:
                raise IOError('the cartridge went off USB')

    def line(self, timeout, on_event=None):
        """The next answer line (events, '!' lines, go to on_event), or None after timeout."""
        end = time.time() + timeout
        while True:
            while b'\n' in self.buf:
                l, self.buf = self.buf.split(b'\n', 1)
                l = l.decode('latin1').strip()
                if not l:
                    continue
                if l.startswith('!'):
                    if on_event:
                        on_event(l)
                    continue
                return l
            left = end - time.time()
            if left <= 0:
                return None
            self._fill(min(left, 0.05))


def connect(tries=1):
    """The monitor's or the bridge's port that answers `v`: (port, version) or None."""
    for i in range(tries):
        for path in usb_serial_ports(VID, PID):
            try:
                p = Port(path)
            except OSError:
                continue
            p.write('v\n')
            ver = p.line(2)
            if ver and ver.startswith('UVMC2-'):
                return p, ver
            p.close()
        if i + 1 < tries:
            time.sleep(0.25)
    return None


# ── picotool ──────────────────────────────────────────────────────────────────────────────

def picotool(args, timeout=20):
    """Run picotool; a command that reboots the chip is done when it says so (on macOS its
    libusb can deadlock at exit while the device it reset detaches)."""
    exe = shutil.which('picotool') or next((p for p in ('/opt/homebrew/bin/picotool', '/usr/local/bin/picotool') if os.path.exists(p)), None)
    if not exe:
        die('picotool not found (macOS: brew install picotool)')
    proc = subprocess.Popen([exe] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    out, end = b'', time.time() + timeout
    os.set_blocking(proc.stdout.fileno(), False)
    while time.time() < end:
        r, _, _ = select.select([proc.stdout], [], [], 0.1)
        if r:
            chunk = proc.stdout.read() or b''
            out += chunk
            if b'asked to reboot' in out or b'rebooted to start' in out:
                time.sleep(0.2)
                proc.kill()
                return 0, out.decode(errors='replace')
        if proc.poll() is not None:
            out += proc.stdout.read() or b''
            return proc.returncode, out.decode(errors='replace')
    proc.kill()
    return -2, out.decode(errors='replace')


def load_and_run(elf):
    """BOOTSEL is up (or coming): the ELF into SRAM and started."""
    last = ''
    for _ in range(20):
        time.sleep(0.3)
        code, out = picotool(['load', '-x', elf, '--vid', hex(VID), '--pid', hex(BOOTSEL_PID)])
        if code == 0:
            return True
        last = out.strip().splitlines()[-1] if out.strip() else ''
    die(f'could not load {elf} in BOOTSEL: {last}')


def menu_to_bootsel():
    """The multicart's menu to BOOTSEL; also fine if the chip is in BOOTSEL already."""
    code, out = picotool(['reboot', '-u', '-f', '--vid', hex(VID), '--pid', hex(MENU_PID)])
    return code == 0


def reach_card():
    """A port that can write to the card: a running game's monitor, or the bridge started
    from the menu."""
    c = connect()
    if c:
        return c
    bridge = os.path.join(PREBUILT, 'sdbridge.elf')
    if not os.path.exists(bridge):
        die(f'no SD bridge at {bridge}')
    print('no game with the monitor on USB: starting the SD bridge from the multicart menu')
    if not menu_to_bootsel() and not _bootsel_present():
        die("the cartridge did not answer: put it in the multicart's menu, USB plugged in")
    load_and_run(bridge)
    c = connect(32)
    if not c:
        die('the SD bridge started but did not answer on USB')
    return c


def _bootsel_present():
    code, _ = picotool(['info', '--vid', hex(VID), '--pid', hex(BOOTSEL_PID)], timeout=5)
    return code == 0


# ── the commands ──────────────────────────────────────────────────────────────────────────

def send_file(port, data, name):
    port.write(f'u {len(data)} {name}\n')   # LF alone: an old monitor took a CR's LF for data
    ans = port.line(5)
    if ans != 'OK':
        die(f'the card did not open {name}: {ans}')
    t0 = time.time()
    for off in range(0, len(data), 65536):
        port.write(data[off:off + 65536])
        print(f'\r  {name}: {min(off + 65536, len(data)) * 100 // len(data)}%', end='', flush=True)
    done = port.line(60)
    want = f'DONE {len(data)} {zlib.crc32(data):08x}'
    if done != want:
        print()
        die(f'upload of {name} failed: {done} (expected {want})')
    dt = max(time.time() - t0, 0.001)
    print(f'\r  {name}: {len(data)} bytes on the card, CRC checked ({len(data) / 1024 / dt:.0f} KB/s)')


def with_monitor(port):
    mon = os.path.join(PREBUILT, 'MONITOR.BIN')
    if os.path.exists(mon):
        send_file(port, open(mon, 'rb').read(), 'MONITOR.BIN')


def cmd_upload(path, name=None):
    port, ver = reach_card()
    print(ver)
    send_file(port, open(path, 'rb').read(), name or os.path.basename(path))
    with_monitor(port)
    port.write('b\n')
    port.line(2)
    port.close()
    print('done: press RESET on the console for the multicart menu')


def game_elf(um2):
    d, stem = os.path.dirname(um2), os.path.splitext(os.path.basename(um2))[0]
    for p in (os.path.join(d, stem + '.elf'), os.path.join(d, 'pico', stem + '.elf'), os.path.join(d, 'uvm2', stem + '.elf')):
        if os.path.exists(p):
            return p
    return None


def cmd_deploy(um2):
    elf = game_elf(um2)
    if not elf:
        die(f'no ELF beside {um2} (it is what runs from RAM)')
    port, ver = reach_card()
    print(ver)
    send_file(port, open(um2, 'rb').read(), os.path.basename(um2))
    with_monitor(port)
    to_bootsel(port)
    load_and_run(elf)
    print(f'{os.path.basename(um2)} is on the card and running')


def to_bootsel(port):
    port.write('x\n')
    ok = port.line(2) == 'OK'
    port.close()
    if not ok:
        die('the monitor has no `x` (an old one): run deploy again, or press RESET and pick the game')


def cmd_run(elf):
    c = connect()
    if c:
        to_bootsel(c[0])
    elif not menu_to_bootsel() and not _bootsel_present():
        die("the cartridge did not answer: put it in the multicart's menu, or run a monitor build")
    load_and_run(elf)
    print(f'{os.path.basename(elf)} is running')


def cmd_log():
    c = connect(20)
    if not c:
        die('no game with the monitor on USB')
    port, ver = c
    print(f'{ver}: the game\'s text (Ctrl-C to stop)', file=sys.stderr)
    pending = ''

    def event(l):
        nonlocal pending
        if l.startswith('!T '):
            pending += bytes.fromhex(l[3:]).decode('latin1')
            *lines, pending = pending.split('\n')
            for x in lines:
                print(x.rstrip('\r'), flush=True)
    try:
        while True:
            port.line(1, on_event=event)
    except KeyboardInterrupt:
        pass
    finally:
        port.close()


def cmd_gdb(elf=None):
    ports = usb_serial_ports(VID, PID)
    if len(ports) < 2:
        die('no game with the monitor on USB (its gdb port is the second of its two)')
    print(ports[1])
    print(f'  arm-none-eabi-gdb {elf or "<game>.elf"} -ex "target remote {ports[1]}"', file=sys.stderr)


def cmd_ports():
    for label, vid, pid in (('monitor / bridge', VID, PID), ("multicart menu", VID, MENU_PID)):
        for p in usb_serial_ports(vid, pid):
            print(f'{p}  {label}')
    if _bootsel_present():
        print('(a chip in BOOTSEL)')


def main():
    a = sys.argv[1:]
    if not a or a[0] in ('-h', '--help', 'help'):
        print(__doc__.split('\n\nThe cartridge')[0])
        return
    cmd, args = a[0], a[1:]
    table = {'deploy': (cmd_deploy, 1, 1), 'upload': (cmd_upload, 1, 2), 'run': (cmd_run, 1, 1),
             'log': (cmd_log, 0, 0), 'gdb': (cmd_gdb, 0, 1), 'ports': (cmd_ports, 0, 0)}
    if cmd not in table:
        die(f'unknown command {cmd} (see uvmc2.py --help)')
    fn, lo, hi = table[cmd]
    if not lo <= len(args) <= hi:
        die(f'{cmd}: wrong arguments (see uvmc2.py --help)')
    try:
        fn(*args)
    except IOError as e:
        die(str(e))


if __name__ == '__main__':
    main()
