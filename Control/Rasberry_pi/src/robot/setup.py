import os

from glob import glob

from setuptools import find_packages, setup

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include URDF (.urdf) files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf')) + glob(os.path.join('urdf', '*.xacro'))),

        # Include mesh (.stl) files

        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),

        # Include launch files

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mscrobotic2324laptop7',
    maintainer_email='mscrobotic2324laptop7@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
