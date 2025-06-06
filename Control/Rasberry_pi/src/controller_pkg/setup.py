from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'controller_pkg'

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
        # Include world (.sdf or .dae) files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '**/**.[sd][da][fe]'), recursive=True)),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        #Yaml install
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mscrobotic2324laptop7',
    maintainer_email='mscrobotic2324laptop7@todo.todo',
    description='Package for Rasberry Pi',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ps4_to_uart = controller_pkg.ps4_to_uart:main",
            "visualisation = controller_pkg.visualisation:main"
        ],
    },
)
