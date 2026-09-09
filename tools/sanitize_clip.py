#!/usr/bin/env python3
"""Entry point for the drop-in sanitizer (docs/spec/RoboSTAR_dropin_fix.md).

    python3 tools/sanitize_clip.py IN.npz -o OUT.npz
    python3 tools/sanitize_clip.py clips/safe/<clip> -o clips/candidate/<clip>
    python3 tools/sanitize_clip.py IN.npz --check

The work is in tools/sanitize/; see that package's README.md for what each
stage does and tools/sanitize/cli.py --help for the options. This file only
puts tools/ on sys.path, the same way prepare_clip.py and clip_audit.py are
imported by each other and by the tests.
"""

import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sanitize.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
