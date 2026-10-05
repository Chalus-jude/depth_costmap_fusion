from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'depth_costmap_fusion'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='cj-007',
    maintainer_email='jude.chalus2005@gmail.com',

    description='Depth and costmap fusion package',
    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'fusion_node = depth_costmap_fusion.fusion_node:main',
            'odom_tf_broadcaster = depth_costmap_fusion.odom_tf_broadcaster:main',
        ],
    },
)
