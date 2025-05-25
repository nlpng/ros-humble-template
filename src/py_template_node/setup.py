from setuptools import find_packages, setup

package_name = 'py_template_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/py_template.launch.py']),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='A reusable ROS 2 humble Python package template with publisher, subscriber, and timer functionality',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_template_node = py_template_node.py_template_node:main',
        ],
    },
)