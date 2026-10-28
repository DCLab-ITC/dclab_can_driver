from setuptools import setup

package_name = "can_driver"

setup(
    name=package_name,
    version="2.6.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DCLab",
    maintainer_email="soubungit@gmail.com",
    description="ROS 2 CAN driver for DCLab boards (Smart Driver, Sensor, Controller, "
                "Absolute Encoder, IMU) and VESC and RoboMaster motors.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "can_driver_node = can_driver.can_driver:main",
            "dclab_link = can_driver.dclab_link:main",
            "test_can_driver = can_driver.test_can_driver:main",
        ],
    },
)
