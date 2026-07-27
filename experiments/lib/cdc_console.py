"""USB CDC console driver for the OpenAquatix HMI.

Each modem exposes a USB CDC interface that prints decoded payloads,
ranging results, and status to a text terminal alongside the bulk HIL
data path. This driver tails the terminal into a per-modem log file and
sends menu-driven commands (chirp, ranging request, text/JANUS transmit).

The simulator's USB transport claims the bulk endpoints; this driver opens
the modem's CDC TTY (e.g. ``/dev/ttyACM0``) directly using pyserial. The
two share the device but address disjoint endpoints.

Menu navigation: digits ``1..N`` (followed by a newline) select a child
menu by index; ``\\e`` (ESC) backs out one level. Task-level helpers
(``send_chirp_tx``, ``send_ranging_request``, ``send_text_message``,
``send_janus_011_01_tx``) encode the known firmware menu order.
"""
from __future__ import annotations

import os
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, IO, Optional, Protocol

try:
    import serial as pyserial      # pyserial
except ImportError:                # pragma: no cover
    pyserial = None                # type: ignore[assignment]


# ---------------------------------------------------------------------------
# udev TTY resolution
# ---------------------------------------------------------------------------

def find_tty_by_serial(usb_serial: str,
                       sys_class_tty: str | os.PathLike = "/sys/class/tty",
                       dev_root: str | os.PathLike = "/dev") -> Optional[Path]:
    """Return ``/dev/ttyACMN`` for the modem with the given USB serial, or
    ``None`` if no matching device is enumerated. ``sys_class_tty`` and
    ``dev_root`` are overridable so tests can point at a fake layout.
    """
    base = Path(sys_class_tty)
    dev  = Path(dev_root)
    if not base.is_dir():
        return None
    for tty_dir in sorted(base.glob("ttyACM*")):
        device_link = tty_dir / "device"
        if not device_link.exists():
            continue
        try:
            node = device_link.resolve()
        except OSError:
            continue
        for _ in range(8):
            serial_path = node / "serial"
            try:
                if serial_path.is_file():
                    if serial_path.read_text().strip() == usb_serial:
                        return dev / tty_dir.name
            except OSError:
                pass
            parent = node.parent
            if parent == node:
                break
            node = parent
    return None


# ---------------------------------------------------------------------------
# Backend abstraction (so tests don't need a real /dev/ttyACM*)
# ---------------------------------------------------------------------------

class SerialBackend(Protocol):
    """Minimal duck-typed interface for ``pyserial.Serial``."""
    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def close(self) -> None: ...


@dataclass
class FakeSerial:
    """In-memory ``SerialBackend`` used by the tests.

    Caller writes are appended to ``written``. Bytes the fake "receives
    from the modem" are fed via :meth:`inject` and yielded one byte at a
    time from :meth:`read`.
    """
    written: bytearray = None
    _rx: queue.Queue = None
    _closed: bool = False

    def __post_init__(self) -> None:
        if self.written is None:
            self.written = bytearray()
        if self._rx is None:
            self._rx = queue.Queue()

    def inject(self, data: bytes | str) -> None:
        if isinstance(data, str):
            data = data.encode("utf-8")
        for b in data:
            self._rx.put(bytes([b]))

    def read(self, size: int = 1) -> bytes:
        if self._closed:
            return b""
        try:
            return self._rx.get(timeout=0.1)
        except queue.Empty:
            return b""

    def write(self, data: bytes) -> int:
        self.written.extend(data)
        return len(data)

    def close(self) -> None:
        self._closed = True


# ---------------------------------------------------------------------------
# CdcConsole
# ---------------------------------------------------------------------------

class CdcConsole:
    """Tail a modem's CDC terminal and issue menu commands.

    Typical usage::

        console = CdcConsole.attach(
            modem_id   = "modem_a",
            usb_serial = "OA-2-1",
            log_path   = cell_dir / "modem_a_cdc.log",
        )
        console.send_ranging_request()
        line = console.expect_line(r"Range:\\s+([0-9.]+)m", timeout=5.0)
        console.detach()
    """

    # Firmware menu indices (1-based, see comm_*_menu.c).
    MAIN_TXRX  = "4"
    MAIN_DBG   = "2"
    MAIN_JANUS = "5"
    MAIN_EVAL  = "6"
    TXRX_STROUT     = "3"      # "Send string through transducer"
    TXRX_RANGEOUT   = "9"      # "Send ranging request through transducer"
    DBG_CHIRP_TX    = "15"     # "Send LFM chirp through transducer"
    JANUS_SEND               = "2"  # "Send JANUS message"
    JANUS_SEND_011_01_OUT    = "1"  # "Send JANUS 011 01 (SMS) through transducer"
    EVAL_SETLEN       = "1"    # "Set evaluation message length"
    EVAL_TRANSDUCER   = "3"    # "Send evaluation message through transducer"

    LINE_TERMINATOR = "\r"
    BACK_KEY        = "\x1b"   # WITHDRAW_CHAR

    # Inter-keystroke pause so the firmware's input loop can process each line
    # before the next arrives. Well below the minimum acoustic propagation
    # delay so it doesn't matter for experiment timing.
    _KEYSTROKE_PAUSE_S = 0.05

    def __init__(self,
                 modem_id: str,
                 backend: SerialBackend,
                 log_path: Path,
                 *,
                 clock: Callable[[], int] = time.monotonic_ns,
                 owns_backend: bool = True) -> None:
        self.modem_id   = modem_id
        self._serial    = backend
        self._log_path  = Path(log_path)
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log: IO[str] = self._log_path.open("a", buffering=1)
        self._clock     = clock
        self._owns      = owns_backend

        self._lines: queue.Queue[str] = queue.Queue()
        self._stop_evt = threading.Event()
        self._reader_error: Optional[Exception] = None
        self._buf      = bytearray()
        self._reader   = threading.Thread(target=self._read_loop,
                                          name=f"cdc-{modem_id}",
                                          daemon=True)
        self._reader.start()

    # --- factories -------------------------------------------------------

    @classmethod
    def attach(cls,
               modem_id: str,
               usb_serial: str,
               log_path: str | Path,
               *,
               baud: int = 115200,
               sys_class_tty: str | os.PathLike = "/sys/class/tty",
               dev_root: str | os.PathLike = "/dev") -> "CdcConsole":
        """Look up the CDC TTY for ``usb_serial`` via udev and open it.

        Raises ``FileNotFoundError`` if no matching TTY is enumerated.
        """
        if pyserial is None:
            raise RuntimeError(
                "pyserial is required for CdcConsole.attach; install via "
                "experiments/requirements.txt")
        tty = find_tty_by_serial(usb_serial, sys_class_tty, dev_root)
        if tty is None:
            raise FileNotFoundError(
                f"no /dev/ttyACM* with USB serial '{usb_serial}'")
        ser = pyserial.Serial(str(tty), baudrate=baud, timeout=0.1)
        return cls(modem_id, ser, Path(log_path))

    @classmethod
    def attach_backend(cls,
                       modem_id: str,
                       backend: SerialBackend,
                       log_path: str | Path) -> "CdcConsole":
        """Wrap an arbitrary ``SerialBackend`` (tests, or hosts that have
        already opened the TTY)."""
        return cls(modem_id, backend, Path(log_path), owns_backend=False)

    # --- I/O -------------------------------------------------------------

    def detach(self) -> None:
        self._stop_evt.set()
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if self._owns:
            try:
                self._serial.close()
            except Exception:
                pass
        self._log.close()

    def __enter__(self) -> "CdcConsole":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.detach()

    @property
    def log_path(self) -> Path:
        return self._log_path

    def is_reader_alive(self) -> bool:
        """True iff the background read thread is still running. Returns False
        after :meth:`detach` and after a pyserial-level failure (USB hot-
        unplug, kernel I/O error)."""
        return self._reader.is_alive() and not self._stop_evt.is_set()

    @property
    def reader_error(self) -> Optional[Exception]:
        """The exception that killed the reader thread, or None. Lets a
        caller distinguish a genuine I/O death from a clean detach."""
        return self._reader_error

    def _read_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                chunk = self._serial.read(64)
            except Exception as exc:
                # Surface the death instead of dying silently. A reader that
                # stops here makes the caller's TX loop bail before it logs
                # any request, which upstream reads as a bare "no usable
                # cell" with no hint why (USB I/O error, hot-unplug, or a
                # leaked sim still holding the port).
                self._reader_error = exc
                if not self._stop_evt.is_set():
                    line = (f"[cdc] {self.modem_id} reader thread died: "
                            f"{type(exc).__name__}: {exc}")
                    try:
                        self._log.write(line + "\n")
                    except Exception:
                        pass
                    sys.stderr.write(line + "\n")
                break
            if not chunk:
                continue
            self._buf.extend(chunk)
            while True:
                # Firmware terminates lines with "\r\n"; bare "\n" tolerated
                # for test fixtures.
                idx = self._buf.find(b"\n")
                if idx < 0:
                    break
                raw = bytes(self._buf[:idx])
                del self._buf[:idx + 1]
                if raw.endswith(b"\r"):
                    raw = raw[:-1]
                line = raw.decode("utf-8", errors="replace")
                self._log.write(f"[{self._clock()} ns] {line}\n")
                self._lines.put(line)

    # --- low-level keystrokes -------------------------------------------

    def send_keys(self, keys: str) -> None:
        """Raw passthrough. ``keys`` may include ``\\e`` for back navigation
        and ``\\r`` to commit a selection or payload."""
        self._serial.write(keys.encode("utf-8"))

    def send_line(self, text: str) -> None:
        """Send ``text`` followed by the firmware's line terminator and pause
        briefly so the firmware can ingest it."""
        self._serial.write((text + self.LINE_TERMINATOR).encode("utf-8"))
        time.sleep(self._KEYSTROKE_PAUSE_S)

    def send_back(self, count: int = 1) -> None:
        """Issue ``count`` ESC keystrokes (each backs out one menu level)."""
        for _ in range(count):
            self._serial.write(self.BACK_KEY.encode("utf-8"))
            time.sleep(self._KEYSTROKE_PAUSE_S)

    def reset_to_main(self) -> None:
        """Back out to the root menu. ESC at root is a no-op in the firmware,
        so 4 ESCs is safe and idempotent for the 2-deep menu tree."""
        self.send_back(4)

    # --- responsiveness check -------------------------------------------

    def verify_main_menu(self,
                         timeout_s: float = 2.0,
                         n_esc:     int   = 4) -> bool:
        """Ping the modem and confirm it echoes the main menu within
        ``timeout_s``. Distinguishes a silent/wedged modem from one that
        responds but later produces no decode output.
        """
        # Drain stale lines so they don't satisfy the regex accidentally.
        self.drain_lines()
        for _ in range(n_esc):
            self._serial.write(self.BACK_KEY.encode("utf-8"))
            time.sleep(self._KEYSTROKE_PAUSE_S)
        try:
            self.expect_line(r"Main Menu", timeout=timeout_s)
            return True
        except TimeoutError:
            return False

    # --- task-level helpers ---------------------------------------------

    def send_chirp_tx(self) -> None:
        """Trigger the firmware LFM chirp on the transducer.

        Menu path: ROOT -> ``DBG`` (2) -> ``CHIRP_TX`` (15) -> empty-line
        handler trigger. After execution the firmware auto-returns to DBG.
        """
        self.reset_to_main()
        self.send_line(self.MAIN_DBG)
        self.send_line(self.DBG_CHIRP_TX)
        # The chirp leaf has no parameters; an empty line submits the call.
        self.send_line("")

    def send_ranging_request(self, target_modem_id: int | None = None) -> None:
        """Trigger a TX-through-transducer ranging request.

        Menu path: ROOT -> ``TXRX`` (4) -> ``RANGEOUT`` (9) -> [address].
        If ``target_modem_id`` is omitted the caller handles the address
        prompt separately (e.g. via :meth:`send_line`).
        """
        self.reset_to_main()
        self.send_line(self.MAIN_TXRX)
        self.send_line(self.TXRX_RANGEOUT)
        if target_modem_id is not None:
            self.send_line(str(int(target_modem_id)))

    def send_text_message(self, text: str,
                          target_modem_id: int | None = None) -> None:
        """Transmit ``text`` through the transducer.

        Menu path: ROOT -> ``TXRX`` (4) -> ``STROUT`` (3) -> [address] -> [text]
        """
        self.reset_to_main()
        self.send_line(self.MAIN_TXRX)
        self.send_line(self.TXRX_STROUT)
        if target_modem_id is not None:
            self.send_line(str(int(target_modem_id)))
        self.send_line(text)

    def send_janus_011_01_tx(self, text: str) -> None:
        """Transmit ``text`` as a JANUS 011_01 SMS through the transducer.

        Menu path: ROOT -> ``JANUS`` (5) -> ``SEND`` (2) -> ``011_01_OUT`` (1) -> [text]

        The firmware only accepts the message if the modem is already in
        JANUS protocol mode; configure it out of band before invoking.
        """
        self.reset_to_main()
        self.send_line(self.MAIN_JANUS)
        self.send_line(self.JANUS_SEND)
        self.send_line(self.JANUS_SEND_011_01_OUT)
        self.send_line(text)

    def send_eval_tx(self) -> None:
        """Transmit a PRBS evaluation message through the transducer (the
        HIL path when the modem is in HIL mode). The receiving modem
        scores the cargo against the shared PRBS and prints
        ``Uncoded BER: e/n, x%`` / ``Coded BER: e/n, x%``.

        Menu path: ROOT -> ``EVAL`` (6) -> ``TRANSDUCER`` (3). Cargo
        length comes from the firmware's eval-message-length parameter
        (:meth:`set_eval_message_len`).
        """
        self.reset_to_main()
        self.send_line(self.MAIN_EVAL)
        self.send_line(self.EVAL_TRANSDUCER)

    def set_eval_message_len(self, n_bytes: int) -> None:
        """Set the PRBS evaluation cargo length in bytes (firmware range
        1..480, default 100).

        Menu path: ROOT -> ``EVAL`` (6) -> ``SETLEN`` (1) -> [value].
        """
        self.reset_to_main()
        self.send_line(self.MAIN_EVAL)
        self.send_line(self.EVAL_SETLEN)
        self.send_line(str(int(n_bytes)))

    # --- line consumption -----------------------------------------------

    def drain_lines(self) -> list[str]:
        """Return all lines that have been received but not yet consumed."""
        out: list[str] = []
        while True:
            try:
                out.append(self._lines.get_nowait())
            except queue.Empty:
                return out

    def expect_line(self, pattern: str, timeout: float = 5.0) -> str:
        """Block until a received line matches ``pattern`` (regex search)
        or ``timeout`` seconds elapse. Returns the matched line. Raises
        ``TimeoutError`` if nothing matches in time.
        """
        rx = re.compile(pattern)
        deadline = time.monotonic() + timeout
        while True:
            try:
                line = self._lines.get(timeout=max(deadline - time.monotonic(),
                                                   0.001))
            except queue.Empty:
                raise TimeoutError(
                    f"expect_line: no match for /{pattern}/ within "
                    f"{timeout:.1f}s") from None
            if rx.search(line):
                return line
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"expect_line: no match for /{pattern}/ within "
                    f"{timeout:.1f}s")
