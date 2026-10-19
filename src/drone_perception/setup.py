from setuptools import setup

package_name = 'drone_perception'
setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harshavardhan Coimbatore Sekar',
    maintainer_email='harshasekar.09@gmail.com',
    description='Depth processing, YOLO victim-candidate localization and safety monitoring.',
    license='MIT',
    entry_points={'console_scripts': [
        'depth_to_cloud = drone_perception.depth_to_cloud:main',
        'victim_detector = drone_perception.victim_detector:main',
        'safety_monitor = drone_perception.safety_monitor:main',
    ]},
)
