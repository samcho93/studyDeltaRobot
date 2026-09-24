from glob import glob

from setuptools import find_packages, setup

package_name = "delta_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/config", glob("config/*.json")),
    ],
    # `deltarobot` is installed separately: pip install -e <repo>/python
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="studyDeltaRobot",
    maintainer_email="samdori93@gmail.com",
    description="Rotary delta robot driver, commander and web bridge (studyDeltaRobot course)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "delta_driver = delta_robot.driver_node:main",
            "delta_commander = delta_robot.commander_node:main",
            "delta_web_bridge = delta_robot.web_bridge_node:main",
            "delta_pick_place_demo = delta_robot.examples.pick_place_demo:main",
        ],
    },
)
