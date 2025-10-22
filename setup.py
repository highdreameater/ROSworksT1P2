import os
from glob import glob
from setuptools import setup

package_name = 'rover_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install all .launch.py files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Install our .rviz config file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),

        # Install all .urdf files from the 'urdf' directory
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),

        # Install all .STL files from the 'meshes' directory
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vitthal',
    maintainer_email='highdreameater@gmail.com',
    description='Rover description and control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_kinematics.py = rover_description.forward_kinematics:main',#for python nodes
        ],
    },
)