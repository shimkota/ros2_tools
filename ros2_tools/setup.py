from setuptools import setup, find_packages

package_name = 'ros2_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kota shimizu',
    maintainer_email='your_email@example.com',
    description='ROS2 tools for rosbag2 analysis',
    entry_points={
        'console_scripts': [
            'extract_pcl_pose = scripts.extract_pcl_pose:main',
        ],
    },
)