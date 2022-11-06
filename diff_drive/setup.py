import os
from glob import glob
from setuptools import setup


package_name = 'diff_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world'))
    ],
    install_requires=['setuptools', 'gazebo', 'rviz', 'urdf'],
    zip_safe=True,
    maintainer='alyssa',
    maintainer_email='alyssachen2022@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['turtle_robot = diff_drive.flip:main'
        ],
    },
)
