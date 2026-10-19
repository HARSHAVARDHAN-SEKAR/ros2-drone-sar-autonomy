from setuptools import find_packages, setup

package_name = 'drone_exploration'
setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harshavardhan Coimbatore Sekar',
    maintainer_email='harshasekar.09@gmail.com',
    description='Fixed-altitude frontier exploration and A* planning for the SAR drone.',
    license='MIT',
    entry_points={'console_scripts': [
        'frontier_explorer = drone_exploration.frontier_explorer:main',
    ]},
)
