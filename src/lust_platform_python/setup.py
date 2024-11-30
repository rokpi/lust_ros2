from setuptools import find_packages, setup

package_name = 'lust_platform_python'

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
    maintainer='rok',
    maintainer_email='rok@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listen_from_python = lust_platform_python.python_subscriber_data_axis:main',
            'talk_from_python = lust_platform_python.python_publisher_set_motion:main',
        ],
    },
)
