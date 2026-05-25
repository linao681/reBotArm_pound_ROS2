from glob import glob

from setuptools import setup

package_name = "rebotarm_moveit_demos"

setup(
    name=package_name,
    version="0.2.3",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="MoveIt 2 application demos for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pick_place = rebotarm_moveit_demos.pick_place:main",
            "draw_square = rebotarm_moveit_demos.draw_square:main",
            "pound = rebotarm_moveit_demos.pound:main",
            "load_controllers = rebotarm_moveit_demos.load_controllers:main",
            "solve_ik = rebotarm_moveit_demos.solve_ik:main",
        ],
    },
)
