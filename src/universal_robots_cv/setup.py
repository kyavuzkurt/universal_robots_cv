from setuptools import find_packages, setup

package_name = 'universal_robots_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kadir Yavuz Kurt',
    maintainer_email='k.yavuzkurt1@gmail.com',
    description='Object recognition and motion planning for Universal Robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = universal_robots_cv.camera_node:main',
            'object_recognition_node = universal_robots_cv.object_recognition_node:main',
            'tracked_image_node = universal_robots_cv.tracked_image_node:main',
            'motion_planning_node = universal_robots_cv.motion_planning_node:main',
            'object_filtering_node = universal_robots_cv.object_filtering_node:main',
            '3d_mapping_node = universal_robots_cv.3d_mapping_node:main',
        ],
    },
)
