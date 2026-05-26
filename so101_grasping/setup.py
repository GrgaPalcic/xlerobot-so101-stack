from setuptools import find_packages, setup

package_name = "so101_grasping"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "msgpack>=1.0",
        "numpy>=1.24",
        "pyzmq>=25.0",
        "robokin[placo]",
    ],
    zip_safe=True,
    maintainer="Dmitri Manajev",
    maintainer_email="dmitri@manajev.com",
    description="ROS 2 grasp perception client for SO-101 with remote GPU inference",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "compressed_camera_node = so101_grasping.compressed_camera_node:main",
            "grasp_request_node = so101_grasping.grasp_request_node:main",
            "grasp_planner_node = so101_grasping.grasp_planner_node:main",
        ],
    },
)
