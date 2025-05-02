from glob import glob

from setuptools import find_packages, setup

package_name = "oc_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bemunin",
    maintainer_email="tw.munin@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_control_node = oc_teleop.joy_control_node:main",
        ],
    },
)
