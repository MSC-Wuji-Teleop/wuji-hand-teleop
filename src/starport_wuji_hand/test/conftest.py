"""Session-scoped rclpy context and hardware interlock for this package's node tests.

Identical in purpose to ros2/starport_ws/src/conftest.py: cycling global rclpy state per test
intermittently segfaults rmw/DDS when many node tests share one pytest process. This package's
tests are sometimes run alone (a single test path), so it carries its own guarded copy rather
than relying on the parent conftest being collected.
"""

import os
import sys

import pytest
import rclpy

#: The same test-only domain `src/conftest.py` uses, and it MUST be the same: the guard below means
#: whichever conftest runs first owns the session's context, so this one initialising without a
#: domain would put a whole-workspace run back on the cell's -- where these suites stand up nodes
#: under the real names and their mode requests reach the real manager. The environment variable is
#: the shared definition; an import from the parent conftest is exactly what this twin exists to
#: avoid. Same hazard as `_no_hand_sdk` below, one layer out: a test that quietly reaches hardware.
TEST_ROS_DOMAIN_ID = int(os.environ.get("STARPORT_TEST_ROS_DOMAIN_ID", "88"))


@pytest.fixture(scope="session", autouse=True)
def _rclpy_session():
    started_here = not rclpy.ok()
    if started_here:
        rclpy.init(domain_id=TEST_ROS_DOMAIN_ID)
    yield
    if started_here and rclpy.ok():
        rclpy.shutdown()


@pytest.fixture(autouse=True)
def _no_hand_sdk(monkeypatch):
    """Make ``import wuji_sdk`` fail for every test in this package.

    wuji_sdk IS installed in the starport-deploy environment, and the hand is an ETHERNET device:
    it answers to whoever is on its subnet, with no cable to this machine and nothing physical to
    notice. A test that reached the driver's connect path would scan, find the real hand, set its
    effort ceiling and ENABLE it. A None entry in sys.modules is what the import system treats as
    unimportable, so the driver takes its no-SDK branch instead.

    Both names are blocked. wujihandpy is the USB SDK this package no longer uses; keeping it here
    costs nothing and means a stray import cannot quietly reach a device either.
    """
    monkeypatch.setitem(sys.modules, "wuji_sdk", None)
    monkeypatch.setitem(sys.modules, "wujihandpy", None)
