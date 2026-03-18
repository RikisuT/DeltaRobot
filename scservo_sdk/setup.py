from setuptools import find_packages, setup

package_name = "scservo_sdk"

setup(
    name=package_name,
    version="0.0.0",
    packages=["scservo_sdk"],
    package_dir={"scservo_sdk": "external/FTServo_Python/scservo_sdk"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RikisuT",
    maintainer_email="likhiacharya@gmail.com",
    description="Feetech SCServo Python SDK for Waveshare STS motors",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "see_motors = see_motors:main",
        ],
    },
)
