from setuptools import find_packages, setup
import os # For path joining if you add data_files later, keep for consistency
# from glob import glob # Uncomment if you need to use glob for data_files later

package_name = 'husky_nav_ros2'

setup(
    name=package_name, # This must match the name of your package
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # This is for non-Python code files that need to be installed.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add any other data files here. For example, if you want waypoints.csv to be installed:
        # (os.path.join('share', package_name), ['husky_nav_ros2/waypoints.csv']),
        # Note: This assumes waypoints.csv is inside the inner husky_nav_ros2/ directory.
    ],
    install_requires=['setuptools'], # setuptools itself is a dependency
    zip_safe=True,
    maintainer='danielg',
    maintainer_email='dgranda2022@fau.edu',
    description='ROS 2 package for Husky robot navigation scripts (Navigator and Waypoint Recorder).',
    license='Apache-2.0',
    tests_require=['pytest'], # Standard for ament_python, ignore UserWarning during build
    entry_points={
        'console_scripts': [
            # Format: 'ros2_run_command_name = python_module_name.script_file_name_without_py:main_function_name'
            # These names are what you'll use with 'ros2 run husky_nav_ros2 <NODE_NAME>'
            'simple_navigator_node = husky_nav_ros2.navigation_simple:main',
            'simple_collector_node = husky_nav_ros2.collection_simple:main',
        ],
    },
)
