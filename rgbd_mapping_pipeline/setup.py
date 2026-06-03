from glob import glob
from setuptools import find_packages, setup

package_name = "rgbd_mapping_pipeline"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sudha",
    maintainer_email="sudha@todo.todo",
    description="ROS 2 RGB-D mapping pipeline for indoor 3D reconstruction.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rgbd_dataset_publisher = rgbd_mapping_pipeline.rgbd_dataset_publisher_node:main",
            "depth_to_cloud = rgbd_mapping_pipeline.depth_to_cloud_node:main",
            "cloud_filter = rgbd_mapping_pipeline.cloud_filter_node:main",
        ],
    },
)
