"""Shared launch-time helpers for wuji_teleop_bringup."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def resolve_config_path(package: str, config_file: str) -> str:
    """Return the ament share-dir path to a config yaml.

    Repo tracks only `<config_file>.template` (placeholders); the operator's
    live `<config_file>` (real SNs / IPs) is gitignored and seeded from the
    template by `docker/entrypoint.sh` on container start. By the time any
    launch helper calls this, the live yaml exists at the returned path.
    Single resolution path — no fallback / no special-case override layer.
    """
    return str(Path(get_package_share_directory(package)) / "config" / config_file)
