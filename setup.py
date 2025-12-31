from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mypkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryoichi Sakamaki',
    maintainer_email='gerggms14@gmail.com',
    description='ros2でチューナーを再現したシステム',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mic_freq_pub = mypkg.mic_freq_pub:main',
            'tuner_node = mypkg.tuner_node:main',
            'draw_piano = mypkg.draw_piano:main',
        ],
    },
)
