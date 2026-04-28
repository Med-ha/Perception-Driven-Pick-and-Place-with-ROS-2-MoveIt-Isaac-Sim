from setuptools import find_packages, setup

package_name = 'so101_state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/so101_pipeline.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test',
    maintainer_email='test@fireloop.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bt_node = so101_state_machine.bt_node:main',
            'isaac_state_relay = so101_state_machine.isaac_state_relay:main',
            'trajectory_to_isaac = so101_state_machine.trajectory_to_isaac:main',
            'test_traj_pub = so101_state_machine.test_traj_pub:main',
            'moveit_executor = so101_state_machine.moveit_executor:main',
            'movegroup_executor = so101_state_machine.movegroup_executor:main',
            'perception_node = so101_state_machine.perception_node:main',
        ],
    },
)
