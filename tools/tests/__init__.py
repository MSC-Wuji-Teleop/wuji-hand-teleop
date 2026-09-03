"""Tests for tools/ (prepare_clip.py, clip_audit.py). Run from the repo root:

    PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider tools/tests

tools/ is mounted read-only in the teleop container, so nothing here writes
next to the sources; all output goes to tmp_path.
"""
