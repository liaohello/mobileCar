from glob import glob
from setuptools import find_packages, setup


package_name = "mobilecar_navigation_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mobileCar",
    maintainer_email="noreply@example.invalid",
    description="G3A navigation IPC to ROS 2 Humble bridge",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "navigation_bridge = mobilecar_navigation_bridge.bridge_node:main",
        ],
    },
)
