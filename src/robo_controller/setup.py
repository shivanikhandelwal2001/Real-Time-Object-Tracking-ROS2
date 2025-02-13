from setuptools import find_packages, setup

package_name = 'robo_controller'

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
    maintainer='shivani',
    maintainer_email='shivani@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = robo_controller.hello_node:main",
            "draw_circle = robo_controller.draw_circle:main",
            "pose_subs = robo_controller.pose_subs:main",
            "turtle_controller = robo_controller.turtle_controller:main",
        ],
    },
)
