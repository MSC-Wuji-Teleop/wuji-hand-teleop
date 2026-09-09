"""`python3 -m sanitize` with tools/ on sys.path. Normal entry point is
tools/sanitize_clip.py, which puts tools/ there for you."""

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
