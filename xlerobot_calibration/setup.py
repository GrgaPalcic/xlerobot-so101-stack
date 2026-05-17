from setuptools import find_packages, setup

package_name = "xlerobot_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="GrgaPalcic",
    maintainer_email="26028466+GrgaPalcic@users.noreply.github.com",
    description="Interactive dual-arm XLeRobot calibration runner for the SO-101 stack.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "xlerobot-calib = xlerobot_calibration.cli:main",
        ],
    },
)
