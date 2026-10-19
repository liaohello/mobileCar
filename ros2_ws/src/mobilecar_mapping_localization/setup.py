from glob import glob
from setuptools import find_packages, setup


package_name = "mobilecar_mapping_localization"

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
    description="G3B mapping/localization support",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "mapping_manager = mobilecar_mapping_localization.mapping_manager:main",
            "localization_monitor = mobilecar_mapping_localization.localization_monitor:main",
        ],
    },
)
