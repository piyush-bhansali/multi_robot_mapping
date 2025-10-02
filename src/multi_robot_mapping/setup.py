from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'multi_robot_mapping'

def files_only(pattern):
    """Helper to filter out directories from glob results"""
    return [p for p in glob(pattern, recursive=True) if os.path.isfile(p)]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package marker file
        ('share/ament_index/resource_index/packages', 
            [f'resource/{package_name}']),
        
        # Package.xml
        (f'share/{package_name}', ['package.xml']),
        
        # Launch files (must end with .launch.py)
        (os.path.join('share', package_name, 'launch'), 
            files_only('launch/*.launch.py')),
        
        # World files (.sdf for Gazebo)
        (os.path.join('share', package_name, 'worlds'), 
            files_only('worlds/*.sdf')),

        # Add to data_files:
        (os.path.join('share', package_name, 'models/turtlebot3_waffle_pi'),    
            files_only('models/turtlebot3_waffle_pi/*')),
        
        # Config files (if you add YAML parameter files later)
        # (os.path.join('share', package_name, 'config'), 
        #     files_only('config/*.yaml')),
        
        # RViz configs (if you create custom RViz views)
        # (os.path.join('share', package_name, 'rviz'), 
        #     files_only('rviz/*.rviz')),
        
        # Models directory (if you add custom Gazebo models)
        # Note: This would need special handling for nested directories
        # (os.path.join('share', package_name, 'models'), 
        #     files_only('models/**/*')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    
    maintainer='piyush',
    maintainer_email='piyush@todo.todo',  # TODO: Update this
    
    description='Multi-robot SLAM system with TurtleBot3 in Gazebo Harmonic',
    license='Apache-2.0',
    
    tests_require=['pytest'],
    extras_require={'test': ['pytest']},
    
    entry_points={
        'console_scripts': [
        ],
    },
)