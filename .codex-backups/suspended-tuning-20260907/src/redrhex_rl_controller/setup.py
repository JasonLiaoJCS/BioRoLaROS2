from glob import glob
from setuptools import find_packages, setup

package_name = "redrhex_rl_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="RedRhex Team",
    maintainer_email="redrhex@example.com",
    description="Jetson ROS2 RL policy deployment controller for RedRhex.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "rl_controller_node = redrhex_rl_controller.rl_controller_node:main",
            "fake_sensor_node = redrhex_rl_controller.fake_sensor_node:main",
            "motor_command_tool = redrhex_rl_controller.motor_command_tool:main",
            "estop_tool = redrhex_rl_controller.estop_tool:main",
            "preflight_check = redrhex_rl_controller.preflight_check:main",
            "bridge_config_hash = redrhex_rl_controller.bridge_config_hash:main",
            "imu_alignment_tool = redrhex_rl_controller.imu_alignment_tool:main",
            "rinbo_leg_mask = redrhex_rl_controller.rinbo_leg_mask:main",
        ],
    },
)
