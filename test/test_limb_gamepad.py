import struct

import bxi_example_py_elf3.limb_gamepad as limb_gamepad
from bxi_example_py_elf3.limb_gamepad import (
    JS_EVENT_AXIS, JS_EVENT_BUTTON, JS_EVENT_FORMAT, JS_EVENT_INIT,
    XBOX_BUTTON_ACTIONS, XBOX_DPAD_X_AXIS, XboxGamepadReader,
    decode_joystick_event,
    find_force_feedback_device, joystick_usb_id,
)


def joystick_event(value, event_type, number):
    return struct.pack(JS_EVENT_FORMAT, 123, value, event_type, number)


def test_decodes_button_press_and_release():
    assert decode_joystick_event(
        joystick_event(1, JS_EVENT_BUTTON, 2)) == ("button", 2, True)
    assert decode_joystick_event(
        joystick_event(0, JS_EVENT_BUTTON, 2)) == ("button", 2, False)


def test_decodes_dpad_axis_and_ignores_initial_state_and_short_records():
    assert decode_joystick_event(
        joystick_event(1, JS_EVENT_BUTTON | JS_EVENT_INIT, 0)) is None
    assert decode_joystick_event(
        joystick_event(-32767, JS_EVENT_AXIS, XBOX_DPAD_X_AXIS)) == (
            "axis", XBOX_DPAD_X_AXIS, -32767)
    assert decode_joystick_event(b"short") is None


def test_elf3_xbox_compatible_buttons_map_to_workflow_actions():
    assert XBOX_BUTTON_ACTIONS == {
        0: "initialize", 1: "stop", 3: "start", 4: "emergency_stop",
    }


def test_finds_event_device_paired_with_joystick(tmp_path, monkeypatch):
    monkeypatch.delenv("BXI_GAMEPAD_EVENT_DEVICE", raising=False)
    device_dir = tmp_path / "js0" / "device"
    device_dir.mkdir(parents=True)
    (device_dir / "event7").touch()
    assert find_force_feedback_device(
        "/dev/input/js0", input_class=tmp_path) == "/dev/input/event7"


def test_force_feedback_device_can_be_overridden(monkeypatch):
    monkeypatch.setenv("BXI_GAMEPAD_EVENT_DEVICE", "/dev/input/event12")
    assert find_force_feedback_device("/dev/input/js0") == "/dev/input/event12"


def test_reads_gamepad_usb_id(tmp_path):
    input_device = tmp_path / "class" / "js0" / "device"
    (input_device / "id").mkdir(parents=True)
    (input_device / "id" / "vendor").write_text("20d6\n")
    (input_device / "id" / "product").write_text("4013\n")
    assert joystick_usb_id(
        "/dev/input/js0", input_class=tmp_path / "class") == (
            0x20D6, 0x4013)


def test_known_unsupported_powera_skips_rumble_silently(monkeypatch):
    event_device_lookups = []
    monkeypatch.setattr(
        limb_gamepad, "joystick_usb_id", lambda _device: (0x20D6, 0x4013))
    monkeypatch.setattr(
        limb_gamepad, "find_force_feedback_device",
        lambda _device: event_device_lookups.append(_device))
    reader = XboxGamepadReader("/dev/input/js0")
    reader._run_rumble(300)
    assert event_device_lookups == []
    assert reader.drain_events() == []
