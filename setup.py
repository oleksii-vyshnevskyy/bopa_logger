from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'bopa_logger'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/bopa_logger', ['package.xml']),
        ('share/bopa_logger/launch', ['launch/logger.launch.py']),
    ],
    install_requires=['setuptools', 'paramiko',],
    zip_safe=True,
    maintainer='Lex',
    maintainer_email='oleksii.vyshnevskyi@jetsoftpro.com',
    description='Logger collector node for ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run = nodes.run:main',
        ],
    },
)
