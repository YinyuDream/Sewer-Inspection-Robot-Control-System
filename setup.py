from setuptools import setup

package_name = 'four_wheel_bot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/four_wheel_bot.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/four_wheel_bot.urdf.xacro']),
        ('share/' + package_name + '/worlds', ['worlds/four_wheel_bot.world']),
        ('share/' + package_name + '/config', ['config/four_wheel_bot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Four wheel robot simulation package for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = four_wheel_bot.scripts.teleop_keyboard:main',
        ],
    },
)