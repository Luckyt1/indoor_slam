from glob import glob
import os

from setuptools import find_packages, setup


package_name = "bxi_slam_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BXI Robotics",
    maintainer_email="dev@bxirobotics.cn",
    description="Quiet-boot runtime supervisor for indoor SLAM modes",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["bxi_slam_manager = bxi_slam_manager.node:main"]
    },
)
