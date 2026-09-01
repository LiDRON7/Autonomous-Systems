from setuptools import find_packages, setup

package_name = "perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="LiDRON",
    maintainer_email="lidron@uprm.edu",
    description="OAK-D obstacle perception and LiDAR landing assessment.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "perception_node = perception.node:main",
    ]},
)
