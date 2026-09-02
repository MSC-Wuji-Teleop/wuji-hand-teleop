import glob
import os

from setuptools import find_packages, setup

package_name = "starport_wuji_hand"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob.glob("launch/*.py")),
        (f"share/{package_name}/config", glob.glob("config/*.yaml")),
        # Files only: running the tests drops a __pycache__/ into scripts/, and a bare glob
        # then hands setup.py a directory it cannot copy, failing every rebuild after a run.
        (f"share/{package_name}/scripts", [p for p in glob.glob("scripts/*") if os.path.isfile(p)]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Multiply Labs",
    maintainer_email="solvin@multiplylabs.com",
    description="Wuji hand2 (beta1) driver node for Starport.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "hand_node = starport_wuji_hand.hand_node:main",
            "wave_check = starport_wuji_hand.wave_check:main",
            "replay_clip = starport_wuji_hand.replay_clip:main",
        ],
    },
)
