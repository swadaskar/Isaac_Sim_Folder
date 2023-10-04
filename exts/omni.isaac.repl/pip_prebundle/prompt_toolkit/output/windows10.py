from ctypes import byref, windll
from ctypes.wintypes import DWORD, HANDLE
from typing import Any, Optional, TextIO

from prompt_toolkit.data_structures import Size
from prompt_toolkit.utils import is_windows
from prompt_toolkit.win32_types import STD_OUTPUT_HANDLE

from .base import Output
from .color_depth import ColorDepth
from .vt100 import Vt100_Output
from .win32 import Win32Output

__all__ = [
    "Windows10_Output",
]

# See: https://msdn.microsoft.com/pl-pl/library/windows/desktop/ms686033(v=vs.85).aspx
ENABLE_PROCESSED_INPUT = 0x0001
ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004


class Windows10_Output:
    """
    Windows 10 output abstraction. This enables and uses vt100 escape sequences.
    """

    def __init__(
        self, stdout: TextIO, default_color_depth: Optional[ColorDepth] = None
    ) -> None:
        self.win32_output = Win32Output(stdout, default_color_depth=default_color_depth)
        self.vt100_output = Vt100_Output(
            stdout, lambda: Size(0, 0), default_color_depth=default_color_depth
        )
        self._hconsole = HANDLE(windll.kernel32.GetStdHandle(STD_OUTPUT_HANDLE))

    def flush(self) -> None:
        """
        Write to output stream and flush.
        """
        original_mode = DWORD(0)

        # Remember the previous console mode.
        windll.kernel32.GetConsoleMode(self._hconsole, byref(original_mode))

        # Enable processing of vt100 sequences.
        windll.kernel32.SetConsoleMode(
            self._hconsole,
            DWORD(ENABLE_PROCESSED_INPUT | ENABLE_VIRTUAL_TERMINAL_PROCESSING),
        )

        try:
            self.vt100_output.flush()
        finally:
            # Restore console mode.
            windll.kernel32.SetConsoleMode(self._hconsole, original_mode)

    @property
    def responds_to_cpr(self) -> bool:
        return False  # We don't need this on Windows.

    def __getattr__(self, name: str) -> Any:
        if name in (
            "get_size",
            "get_rows_below_cursor_position",
            "enable_mouse_support",
            "disable_mouse_support",
            "scroll_buffer_to_prompt",
            "get_win32_screen_buffer_info",
            "enable_bracketed_paste",
            "disable_bracketed_paste",
            "get_default_color_depth",
        ):
            return getattr(self.win32_output, name)
        else:
            return getattr(self.vt100_output, name)


Output.register(Windows10_Output)


def is_win_vt100_enabled() -> bool:
    """
    Returns True when we're running Windows and VT100 escape sequences are
    supported.
    """
    if not is_windows():
        return False

    hconsole = HANDLE(windll.kernel32.GetStdHandle(STD_OUTPUT_HANDLE))

    # Get original console mode.
    original_mode = DWORD(0)
    windll.kernel32.GetConsoleMode(hconsole, byref(original_mode))

    try:
        # Try to enable VT100 sequences.
        result: int = windll.kernel32.SetConsoleMode(
            hconsole, DWORD(ENABLE_PROCESSED_INPUT | ENABLE_VIRTUAL_TERMINAL_PROCESSING)
        )

        return result == 1
    finally:
        windll.kernel32.SetConsoleMode(hconsole, original_mode)
