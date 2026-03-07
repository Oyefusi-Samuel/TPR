from setuptools import find_packages, setup

package_name = 'jackal_ar4_goals'

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
    maintainer='ROS2 Developer',
    maintainer_email='maintainer@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'send_nav_goal = jackal_ar4_goals.send_nav_goal:main',
            'send_arm_goal = jackal_ar4_goals.send_arm_goal:main',
        ],
    },
)
