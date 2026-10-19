from glob import glob
from setuptools import find_packages, setup


package_name = "mobilecar_nav_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/behavior_trees", glob("behavior_trees/*.xml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    zip_safe=True,
    maintainer="mobileCar",
    maintainer_email="noreply@example.invalid",
    description="G3B Nav2 bringup",
    license="Proprietary",
)
