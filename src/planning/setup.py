import os
from glob import glob
from setuptools import find_packages, setup
from setuptools import find_packages, setup

package_name = 'planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'pddl'), glob('pddl/*')),
    (os.path.join('share', package_name, 'behaviour_trees'), glob('behaviour_trees/*.xml')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aksc',
    maintainer_email='camposanakarol@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planning_controller = planning.planning_controller:main',
            'move_action = planning.move_action:main',
            'pick_from_location_action = planning.pick_from_location_action:main',
            'pick_from_container_action = planning.pick_from_container_action:main',
            'place_at_location_action = planning.place_at_location_action:main',
            'place_in_container_action = planning.place_in_container_action:main',
            'stack_action = planning.stack_action:main',
            'unstack_action = planning.unstack_action:main',
        ],
    },
)
