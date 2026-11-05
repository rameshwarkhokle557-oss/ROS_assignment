from setuptools import find_packages, setup

package_name = 'rb_assignment'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Starter package for the assignment.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'example_driver = rb_assignment.example_driver:main',
            'nearest_obstacle = rb_assignment.nearest_obstacle:main',
            'emergency_stop = rb_assignment.emergency_stop:main',
        ],
    },
)
