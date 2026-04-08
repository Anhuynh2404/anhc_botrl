import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'anhc_benchmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anhuynh',
    maintainer_email='huynhcongan2442004@gmail.com',
    description='Algorithm benchmarking and comparison framework for the anhc autonomous vehicle',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'anhc_benchmark_runner_node = anhc_benchmark.anhc_benchmark_runner_node:main',
            'anhc_live_metrics_node = anhc_benchmark.anhc_live_metrics_node:main',
            'anhc_results_analyzer_node = anhc_benchmark.anhc_results_analyzer_node:main',
        ],
    },
)
