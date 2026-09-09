from glob import glob
from setuptools import find_packages, setup

package_name = "redrhex_lowlevel_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="RedRhex Team",
    maintainer_email="redrhex@example.com",
    description="Replaceable low-level board bridge for RedRhex.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "lowlevel_bridge_node = redrhex_lowlevel_bridge.lowlevel_bridge_node:main",
            "rinbo_bringup_check = redrhex_lowlevel_bridge.rinbo_bringup_check:main",
            "rinbo_bringup_plan = redrhex_lowlevel_bridge.biorola_bringup_plan:main",
            "rinbo_power_tool = redrhex_lowlevel_bridge.rinbo_power_tool:main",
            "biorola_fault_diag = redrhex_lowlevel_bridge.biorola_fault_diag:main",
            "biorola_servo_probe = redrhex_lowlevel_bridge.biorola_servo_probe:main",
            "biorola_bringup_check = redrhex_lowlevel_bridge.rinbo_bringup_check:main",
            "biorola_bringup_plan = redrhex_lowlevel_bridge.biorola_bringup_plan:main",
            "biorola_power_tool = redrhex_lowlevel_bridge.rinbo_power_tool:main",
        ],
    },
)
