"""Tests for the CDC console driver.

Covers the firmware menu contract (digits + newline navigates forward,
ESC backs out one level), the task-level helpers' keystroke sequences,
the reader thread's line capture and per-modem log timestamping, and
udev TTY resolution by USB serial.
"""
from __future__ import annotations

import os
import re
import time
from pathlib import Path

import pytest

from experiments.lib.cdc_console import (
    CdcConsole,
    FakeSerial,
    find_tty_by_serial,
)


def _read_lines(p: Path) -> list[str]:
    return [ln for ln in p.read_text().splitlines() if ln.strip()]


# ---------------------------------------------------------------------------
# Reader thread: incoming line capture + log file shape
# ---------------------------------------------------------------------------

def test_reader_captures_appended_lines(tmp_path: Path) -> None:
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        fake.inject("Range: 487.20m\r\n")
        fake.inject("Received: PROBE 0042\r\n")
        line1 = console.expect_line(r"Range:\s+([0-9.]+)m", timeout=2.0)
        line2 = console.expect_line(r"Received:\s+PROBE", timeout=2.0)
        assert "Range: 487.20m" in line1
        assert "PROBE 0042" in line2
    finally:
        console.detach()

    log_lines = _read_lines(tmp_path / "cdc.log")
    assert len(log_lines) == 2
    # Each line prefixed with "[<ns> ns] <payload>".
    for ln in log_lines:
        assert re.match(r"^\[\d+ ns\] ", ln), f"unexpected log shape: {ln!r}"
    assert "Range: 487.20m" in log_lines[0]
    assert "PROBE 0042"     in log_lines[1]


def test_log_timestamps_monotonic(tmp_path: Path) -> None:
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        fake.inject("line1\r\n")
        console.expect_line("line1", timeout=2.0)
        fake.inject("line2\r\n")
        console.expect_line("line2", timeout=2.0)
        fake.inject("line3\r\n")
        console.expect_line("line3", timeout=2.0)
    finally:
        console.detach()

    log_lines = _read_lines(tmp_path / "cdc.log")
    ts = [int(re.match(r"^\[(\d+) ns\]", ln).group(1)) for ln in log_lines]
    assert ts == sorted(ts)


def test_expect_line_times_out_when_no_match(tmp_path: Path) -> None:
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        fake.inject("hello\r\n")
        with pytest.raises(TimeoutError):
            console.expect_line(r"never_matches", timeout=0.5)
    finally:
        console.detach()


# ---------------------------------------------------------------------------
# Task-level helpers: keystroke sequence is the contract
# ---------------------------------------------------------------------------

def test_send_back_writes_esc(tmp_path: Path) -> None:
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        console.send_back(count=3)
    finally:
        console.detach()
    assert bytes(fake.written) == b"\x1b\x1b\x1b"


def test_send_line_appends_terminator(tmp_path: Path) -> None:
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        console.send_line("42")
    finally:
        console.detach()
    assert bytes(fake.written) == b"42\r"


def test_send_ranging_request_sequence(tmp_path: Path) -> None:
    """ESC*4 + "4" + "9" triggers a ranging request."""
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        console.send_ranging_request()
    finally:
        console.detach()
    # 4 ESCs (reset_to_main is generous), then "4\r", then "9\r".
    assert bytes(fake.written) == b"\x1b\x1b\x1b\x1b" + b"4\r" + b"9\r"


def test_send_ranging_request_with_target(tmp_path: Path) -> None:
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        console.send_ranging_request(target_modem_id=2)
    finally:
        console.detach()
    assert bytes(fake.written).endswith(b"4\r" + b"9\r" + b"2\r")


def test_send_chirp_tx_sequence(tmp_path: Path) -> None:
    """ROOT -> DBG (2) -> CHIRP_TX (15) -> handler-trigger empty line."""
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        console.send_chirp_tx()
    finally:
        console.detach()
    assert bytes(fake.written) == (
        b"\x1b\x1b\x1b\x1b"   # reset_to_main
        + b"2\r"              # DBG
        + b"15\r"             # CHIRP_TX
        + b"\r"               # trigger handler
    )


def test_send_text_message_sequence(tmp_path: Path) -> None:
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        console.send_text_message("HELLO", target_modem_id=7)
    finally:
        console.detach()
    assert bytes(fake.written) == (
        b"\x1b\x1b\x1b\x1b"
        + b"4\r"
        + b"3\r"
        + b"7\r"
        + b"HELLO\r"
    )


def test_send_janus_011_01_tx_sequence(tmp_path: Path) -> None:
    """ROOT -> JANUS (5) -> SEND (2) -> 011_01_OUT (1) -> [text]."""
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        console.send_janus_011_01_tx("PROBE 042")
    finally:
        console.detach()
    assert bytes(fake.written) == (
        b"\x1b\x1b\x1b\x1b"
        + b"5\r"
        + b"2\r"
        + b"1\r"
        + b"PROBE 042\r"
    )


def test_send_eval_tx_sequence(tmp_path: Path) -> None:
    """ROOT -> EVAL (6) -> TRANSDUCER (3); no further prompt."""
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        console.send_eval_tx()
    finally:
        console.detach()
    assert bytes(fake.written) == (
        b"\x1b\x1b\x1b\x1b"
        + b"6\r"
        + b"3\r"
    )


def test_set_eval_message_len_sequence(tmp_path: Path) -> None:
    """ROOT -> EVAL (6) -> SETLEN (1) -> [bytes]."""
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_a", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        console.set_eval_message_len(100)
    finally:
        console.detach()
    assert bytes(fake.written) == (
        b"\x1b\x1b\x1b\x1b"
        + b"6\r"
        + b"1\r"
        + b"100\r"
    )


# ---------------------------------------------------------------------------
# verify_main_menu — modem responsiveness check
# ---------------------------------------------------------------------------

def test_verify_main_menu_returns_true_when_modem_echoes(tmp_path: Path) -> None:
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_b", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        # Inject the canonical menu print sequence the firmware emits on
        # ESC.
        fake.inject("\r\nMain Menu\r\n1: CFG\r\n2: DBG\r\n")
        assert console.verify_main_menu(timeout_s=2.0) is True
    finally:
        console.detach()
    # Verify 4 ESCs were written to the modem.
    assert bytes(fake.written) == b"\x1b\x1b\x1b\x1b"


def test_verify_main_menu_returns_false_on_silent_modem(tmp_path: Path) -> None:
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_b", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        # No injection => no bytes => expect_line times out.
        assert console.verify_main_menu(timeout_s=0.3) is False
    finally:
        console.detach()


def test_verify_main_menu_ignores_stale_pre_esc_lines(tmp_path: Path) -> None:
    """``verify_main_menu`` is a fresh ping, not a history scan: a 'Main
    Menu' line that arrived before the call must not satisfy the check."""
    fake = FakeSerial()
    console = CdcConsole.attach_backend("modem_b", fake,
                                        log_path=tmp_path / "cdc.log")
    try:
        fake.inject("\r\nMain Menu\r\n1: CFG\r\n")
        time.sleep(0.1)        # let reader consume it
        assert console.verify_main_menu(timeout_s=0.3) is False
    finally:
        console.detach()


# ---------------------------------------------------------------------------
# udev TTY resolution
# ---------------------------------------------------------------------------

def _make_fake_sysfs(tmp_path: Path, *,
                     tty_name: str, usb_serial: str) -> tuple[Path, Path]:
    """Build a /sys/class/tty/<tty>/device/<...>/serial tree under tmp."""
    sysfs = tmp_path / "sys" / "class" / "tty"
    sysfs.mkdir(parents=True, exist_ok=True)
    devroot = tmp_path / "sys" / "devices" / "usb" / "1-1"
    devroot.mkdir(parents=True, exist_ok=True)
    (devroot / "serial").write_text(usb_serial + "\n")

    # ttyACMx -> device -> ../../1-1
    tty_dir = sysfs / tty_name
    tty_dir.mkdir()
    # Place a deeper directory and link "device" to the parent that holds serial.
    deeper = devroot / f"{tty_name}_iface"
    deeper.mkdir()
    os.symlink(deeper, tty_dir / "device")

    dev_root = tmp_path / "dev"
    dev_root.mkdir(parents=True, exist_ok=True)
    (dev_root / tty_name).write_bytes(b"")
    return sysfs, dev_root


def test_find_tty_by_serial_matches(tmp_path: Path) -> None:
    sysfs, devroot = _make_fake_sysfs(tmp_path,
                                      tty_name="ttyACM0",
                                      usb_serial="OA-2-1")
    found = find_tty_by_serial("OA-2-1",
                               sys_class_tty=sysfs,
                               dev_root=devroot)
    assert found == devroot / "ttyACM0"


def test_find_tty_by_serial_returns_none_when_absent(tmp_path: Path) -> None:
    sysfs, devroot = _make_fake_sysfs(tmp_path,
                                      tty_name="ttyACM0",
                                      usb_serial="OA-2-1")
    assert find_tty_by_serial("OA-2-2",
                              sys_class_tty=sysfs,
                              dev_root=devroot) is None


def test_find_tty_by_serial_picks_correct_modem(tmp_path: Path) -> None:
    """With two TTYs and two different USB serials, return the matching one."""
    sysfs = tmp_path / "sys" / "class" / "tty"
    devroot = tmp_path / "dev"
    devroot.mkdir(parents=True)
    sysfs.mkdir(parents=True)

    for tty, serial in [("ttyACM0", "OA-2-1"), ("ttyACM1", "OA-2-2")]:
        dev_node = tmp_path / "sys" / "devices" / serial
        dev_node.mkdir(parents=True)
        (dev_node / "serial").write_text(serial + "\n")
        leaf = dev_node / "iface"
        leaf.mkdir()
        os.symlink(leaf, sysfs / tty)
        # find_tty_by_serial expects (tty_dir / 'device'); rebuild the
        # scaffold so the lookup walks through the serial-bearing dir.
        (sysfs / tty).rename(sysfs / (tty + ".tmp"))
        actual_tty = sysfs / tty
        actual_tty.mkdir()
        os.symlink(leaf, actual_tty / "device")
        (sysfs / (tty + ".tmp")).unlink()
        (devroot / tty).write_bytes(b"")

    found1 = find_tty_by_serial("OA-2-1", sys_class_tty=sysfs, dev_root=devroot)
    found2 = find_tty_by_serial("OA-2-2", sys_class_tty=sysfs, dev_root=devroot)
    assert found1 == devroot / "ttyACM0"
    assert found2 == devroot / "ttyACM1"
