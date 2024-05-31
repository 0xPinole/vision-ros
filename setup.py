"""Initialize ros service."""

from setuptools import find_packages, setup

package_name = "srv_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pinole",
    maintainer_email="pinole@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'node_service = srv_vision.vision_service:main',
            'node_analize = srv_vision.analize_node:main',
            'node_procedure = srv_vision.process_node:main',
            'node_evaluation = srv_vision.evaluation_node:main',
            'node_data = srv_vision.data_node:main',
            'service_compose = srv_vision.compose:main',
            'testing = srv_vision.camera_raw_topic_sub:main'
        ],
    },
)
