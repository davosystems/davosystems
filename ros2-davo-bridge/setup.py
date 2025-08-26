from setuptools import setup

package_name = 'davo_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=[
        'setuptools',
        'web3>=6.0.0',
        'eth-account>=0.9.0',
        'eth-utils>=2.0.0',
        'pyyaml>=6.0',
        'sqlite3',
    ],
    zip_safe=True,
    maintainer='Davo Systems',
    maintainer_email='dev@davo-systems.com',
    description='ROS 2 bridge for Davo Systems blockchain integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = davo_bridge.bridge_node:main',
            'signer = davo_bridge.signer:main',
        ],
    },
)
