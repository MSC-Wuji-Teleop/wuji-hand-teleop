"""Put scripts/ on sys.path so the session and its terminal import by name.

Same shape as tools/tests/conftest.py. scripts/ is host-side code and not a
colcon package, so these run under plain pytest:

    python3 -m pytest scripts/tests/
"""

from __future__ import annotations

import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
