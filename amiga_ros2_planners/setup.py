import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amiga_ros2_planners"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "schemas"),
            glob(os.path.join("schemas", "*.xsd")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="appuser",
    maintainer_email="appuser@todo.todo",
    description="Path planning and BT-condition backends ported from problog_project onto this simulation's orchard/tf2 state",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plan_service = amiga_ros2_planners.plan_service_node:main",
            "condition_service = amiga_ros2_planners.condition_service_node:main",
            "battery_sim = amiga_ros2_planners.battery_sim_node:main",
            "orchard_map = amiga_ros2_planners.orchard_map_node:main",
        ],
    },
)
