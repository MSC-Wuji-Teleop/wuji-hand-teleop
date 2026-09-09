"""Put scripts/ on sys.path so the session and its terminal import by name.

Same shape as tools/tests/conftest.py. scripts/ is host-side code and not a
colcon package, so these run under plain pytest:

    python3 -m pytest scripts/tests/
"""

from __future__ import annotations

import signal
import sys
from pathlib import Path

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))


@pytest.fixture(autouse=True)
def _signal_handlers_restored():
    """main() ignores the fatal signals for its teardown and then restores what
    it installed over. A test that stubs the install would otherwise leave
    SIG_IGN on SIGINT in this process, and Ctrl-C would stop working in pytest."""
    from interactive.terminal import FATAL_SIGNALS

    before = {number: signal.getsignal(number) for number in FATAL_SIGNALS}
    yield
    for number, handler in before.items():
        signal.signal(number, handler)
