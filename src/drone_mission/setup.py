from setuptools import find_packages, setup
package_name = 'drone_mission'
setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Harshavardhan Coimbatore Sekar',
    maintainer_email='harshasekar.09@gmail.com',
    description='Mission orchestration state machine for autonomous drone SAR.',
    license='MIT',
    entry_points={'console_scripts': ['mission_manager = drone_mission.mission_manager:main']},
)
