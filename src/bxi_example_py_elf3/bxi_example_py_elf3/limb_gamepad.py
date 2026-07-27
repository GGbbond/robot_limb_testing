"""Optional Linux joystick reader for the limb-inspection debug UI."""

import ctypes
import errno
import fcntl
import os
from pathlib import Path
from queue import SimpleQueue
import select
import struct
from threading import Event, Thread


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
XBOX_DPAD_X_AXIS = 6
XBOX_DPAD_THRESHOLD = 16000
UNSUPPORTED_RUMBLE_USB_IDS = {(0x20D6, 0x4013)}

# Linux joydev order used by the Xbox-compatible controller shipped with the
# Elf3 setup.  It matches the repository's original remote_controller node;
# this device reports X/Y as 3/4 instead of the common 2/3 ordering.
XBOX_BUTTON_A = 0
XBOX_BUTTON_B = 1
XBOX_BUTTON_X = 3
XBOX_BUTTON_Y = 4
XBOX_BUTTON_ACTIONS = {
    XBOX_BUTTON_A: "initialize",
    XBOX_BUTTON_B: "stop",
    XBOX_BUTTON_X: "start",
    XBOX_BUTTON_Y: "emergency_stop",
}

EV_FF = 0x15
FF_RUMBLE = 0x50


class _FFTrigger(ctypes.Structure):
    _fields_ = [("button", ctypes.c_ushort),
                ("interval", ctypes.c_ushort)]


class _FFReplay(ctypes.Structure):
    _fields_ = [("length", ctypes.c_ushort),
                ("delay", ctypes.c_ushort)]


class _FFEnvelope(ctypes.Structure):
    _fields_ = [("attack_length", ctypes.c_ushort),
                ("attack_level", ctypes.c_ushort),
                ("fade_length", ctypes.c_ushort),
                ("fade_level", ctypes.c_ushort)]


class _FFPeriodicEffect(ctypes.Structure):
    _fields_ = [("waveform", ctypes.c_ushort),
                ("period", ctypes.c_ushort),
                ("magnitude", ctypes.c_short),
                ("offset", ctypes.c_short),
                ("phase", ctypes.c_ushort),
                ("envelope", _FFEnvelope),
                ("custom_len", ctypes.c_uint),
                ("custom_data", ctypes.POINTER(ctypes.c_short))]


class _FFRumbleEffect(ctypes.Structure):
    _fields_ = [("strong_magnitude", ctypes.c_ushort),
                ("weak_magnitude", ctypes.c_ushort)]


class _FFEffectData(ctypes.Union):
    _fields_ = [("periodic", _FFPeriodicEffect),
                ("rumble", _FFRumbleEffect),
                ("padding", ctypes.c_ubyte * 32)]


class _FFEffect(ctypes.Structure):
    _fields_ = [("type", ctypes.c_ushort),
                ("id", ctypes.c_short),
                ("direction", ctypes.c_ushort),
                ("trigger", _FFTrigger),
                ("replay", _FFReplay),
                ("data", _FFEffectData)]


def _write_ioctl(request_number, size):
    return (1 << 30) | (int(size) << 16) | (ord("E") << 8) | request_number


EVIOCSFF = _write_ioctl(0x80, ctypes.sizeof(_FFEffect))
EVIOCRMFF = _write_ioctl(0x81, ctypes.sizeof(ctypes.c_int))


def find_force_feedback_device(joystick_device, input_class="/sys/class/input"):
    """Find the event device paired with a joydev device via input sysfs."""
    override = os.environ.get("BXI_GAMEPAD_EVENT_DEVICE", "")
    if override:
        return override
    device_dir = Path(input_class) / Path(joystick_device).name / "device"
    try:
        candidates = sorted(device_dir.glob("event*"))
    except OSError:
        candidates = []
    if not candidates:
        return None
    return str(Path(joystick_device).parent / candidates[0].name)


def joystick_usb_id(joystick_device, input_class="/sys/class/input"):
    """Return the USB vendor/product pair exposed by the joydev input node."""
    device_dir = Path(input_class) / Path(joystick_device).name / "device" / "id"
    try:
        vendor = int((device_dir / "vendor").read_text().strip(), 16)
        product = int((device_dir / "product").read_text().strip(), 16)
    except (OSError, ValueError):
        return None
    return vendor, product


def play_rumble(event_device, duration_ms=300, stop_event=None):
    """Play one Linux EV_FF rumble effect without requiring python-evdev."""
    duration_ms = max(1, min(int(duration_ms), 5000))
    descriptor = os.open(event_device, os.O_RDWR | os.O_NONBLOCK)
    effect_id = None
    try:
        effect = _FFEffect()
        effect.type = FF_RUMBLE
        effect.id = -1
        effect.replay.length = duration_ms
        effect.data.rumble.strong_magnitude = 0x8000
        effect.data.rumble.weak_magnitude = 0x4000
        payload = bytearray(ctypes.string_at(
            ctypes.byref(effect), ctypes.sizeof(effect)))
        fcntl.ioctl(descriptor, EVIOCSFF, payload, True)
        effect_id = _FFEffect.from_buffer(payload).id
        os.write(descriptor, struct.pack(
            "llHHi", 0, 0, EV_FF, effect_id, 1))
        if stop_event is None:
            Event().wait(duration_ms / 1000.0 + 0.05)
        else:
            stop_event.wait(duration_ms / 1000.0 + 0.05)
    finally:
        if effect_id is not None:
            try:
                os.write(descriptor, struct.pack(
                    "llHHi", 0, 0, EV_FF, effect_id, 0))
                fcntl.ioctl(
                    descriptor, EVIOCRMFF, struct.pack("i", effect_id))
            except OSError:
                pass
        os.close(descriptor)


def decode_joystick_event(payload):
    """Decode one joydev button or axis record, ignoring initial state."""
    if len(payload) != JS_EVENT_SIZE:
        return None
    _timestamp, value, event_type, number = struct.unpack(
        JS_EVENT_FORMAT, payload)
    if event_type & JS_EVENT_INIT:
        return None
    if event_type & JS_EVENT_BUTTON:
        return "button", int(number), bool(value)
    if event_type & JS_EVENT_AXIS:
        return "axis", int(number), int(value)
    return None


class XboxGamepadReader:
    """Read button edges without adding a dependency on evdev or SDL."""

    def __init__(self, device="/dev/input/js0"):
        self.device = str(device)
        self.events = SimpleQueue()
        self._stopped = Event()
        self._thread = None
        self._rumble_thread = None

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stopped.clear()
        self._thread = Thread(
            target=self._run, name="limb-inspection-gamepad", daemon=True)
        self._thread.start()

    def stop(self):
        self._stopped.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        if self._rumble_thread is not None:
            self._rumble_thread.join(timeout=1.0)
        self._rumble_thread = None

    def rumble(self, duration_ms=300):
        """Request a non-blocking reminder vibration."""
        if (self._rumble_thread is not None and
                self._rumble_thread.is_alive()):
            return
        self._rumble_thread = Thread(
            target=self._run_rumble, args=(duration_ms,),
            name="limb-inspection-gamepad-rumble", daemon=True)
        self._rumble_thread.start()

    def _run_rumble(self, duration_ms):
        if joystick_usb_id(self.device) in UNSUPPORTED_RUMBLE_USB_IDS:
            return
        event_device = find_force_feedback_device(self.device)
        if not event_device:
            return
        try:
            play_rumble(event_device, duration_ms, self._stopped)
        except OSError:
            return
        self.events.put(("rumble", True, event_device))

    def drain_events(self):
        result = []
        while not self.events.empty():
            result.append(self.events.get())
        return result

    def _wait_retry(self):
        return self._stopped.wait(1.0)

    def _run(self):
        last_error = None
        while not self._stopped.is_set():
            try:
                descriptor = os.open(
                    self.device, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as exc:
                message = os.strerror(exc.errno) if exc.errno else str(exc)
                if message != last_error:
                    self.events.put(("status", False, message))
                    last_error = message
                if self._wait_retry():
                    return
                continue
            self.events.put(("status", True, self.device))
            last_error = None
            try:
                while not self._stopped.is_set():
                    readable, _, _ = select.select([descriptor], [], [], 0.2)
                    if not readable:
                        continue
                    try:
                        payload = os.read(descriptor, JS_EVENT_SIZE)
                    except OSError as exc:
                        if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                            continue
                        raise
                    if not payload:
                        raise OSError(errno.ENODEV, "gamepad disconnected")
                    event = decode_joystick_event(payload)
                    if event is not None:
                        self.events.put(event)
            except (OSError, ValueError) as exc:
                message = os.strerror(exc.errno) if (
                    isinstance(exc, OSError) and exc.errno) else str(exc)
                self.events.put(("status", False, message))
            finally:
                os.close(descriptor)
            if self._wait_retry():
                return
