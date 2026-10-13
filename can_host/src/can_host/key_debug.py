"""Debug key listener using only the standard library.

Provides :class:`KeyListener` which prints individual keystrokes to the log
in a daemon thread.  Uses ``select`` + ``termios`` so it works on headless
Raspberry Pi without external dependencies (no ``pynput`` or ``keyboard``
needed).
"""

from __future__ import annotations

import logging
import os
import select
import sys
import termios
import threading
import tty
from typing import Any

logger = logging.getLogger(__name__)


class KeyListener:
    """Print keystrokes to the log in a daemon thread.

    When enabled the terminal is switched to cbreak mode so individual key
    presses (not just whole lines) are captured immediately.

    Only activates when *stdin is a TTY*; in non-interactive contexts it is
    silently skipped.
    """

    def __init__(self, enabled: bool = False) -> None:
        self._enabled = enabled
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._old_termios: Any | None = None

    def start(self) -> None:
        """Start the background listener if enabled and stdin is a TTY."""
        if not self._enabled:
            return
        fd = sys.stdin.fileno()
        if not os.isatty(fd):
            logger.warning(
                "Key debug enabled but stdin is not a TTY; disabling."
            )
            return
        try:
            self._old_termios = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except termios.error as exc:
            logger.warning("Cannot set cbreak mode on stdin: %s", exc)
            self._enabled = False
            return
        self._thread = threading.Thread(
            target=self._run, name="key-listener", daemon=True
        )
        self._thread.start()
        logger.info("Key debug listener started (press keys to see them logged).")

    def stop(self) -> None:
        """Stop the listener and restore the original terminal settings."""
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._old_termios is not None:
            try:
                termios.tcsetattr(
                    sys.stdin.fileno(), termios.TCSADRAIN, self._old_termios
                )
            except termios.error:
                pass
            self._old_termios = None

    def _run(self) -> None:
        """Poll stdin via *select* and log each keystroke."""
        fd = sys.stdin.fileno()
        while not self._stop_event.is_set():
            try:
                ready, _, _ = select.select([fd], [], [], 0.1)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            try:
                raw = os.read(fd, 1)
            except OSError:
                break
            if not raw:
                break
            ch = raw.decode("utf-8", errors="replace")
            logger.info("[key] 0x%02X %r", raw[0], ch)
