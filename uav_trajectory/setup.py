from setuptools import find_packages, setup

package_name = 'uav_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SCHWEITZER',
    maintainer_email='thibault.schweitzer@estaca.eu',
    description='TODO: Package description',
    license='CECILL-2.1',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_trajectories = uav_trajectory.generate_trajectories:main',
            'talker = uav_trajectory.publisher_member_function:main',
            'listener = uav_trajectory.subscriber_member_function:main',
        ],
    },
)
