from setuptools import setup, find_packages

package_name = 'tf_platform'

setup(
    name=package_name,
    version='0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/cartographer_launch.py']),
        (f'share/{package_name}/launch', ['launch/cartographer_launch_cloud.py']),
        (f'share/{package_name}/config', ['config/cartographer_config.lua']),
        (f'share/{package_name}/config', ['config/cartographer_config_cloud.lua']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_publisher = tf_platform.tf_publisher:main',
            'tf_publisher_cloud = tf_platform.tf_publisher_cloud:main',
            'tf = tf_platform.tf_publisher_test:main',
            'odom = tf_platform.odom_publisher:main',
        ],
    },
)

