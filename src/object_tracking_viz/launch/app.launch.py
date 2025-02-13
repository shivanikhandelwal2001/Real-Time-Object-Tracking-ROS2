import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory("object_tracking_viz"), "config", "params.yaml")

    image_publisher = Node(package="object_tracking_viz", 
                           executable="image_publisher", 
                           name="image_publisher", 
                           parameters=[config],
                           remappings=[("/camera/image_raw", "/camera/image_raw")],)
    
    image_subscriber = Node(package="object_tracking_viz", 
                            executable="image_subscriber.py",
                            name="image_subscriber",
                            parameters=[config],
                            remappings=[("/camera/image_raw", "/camera/image_raw")],)
    
    object_tracking = Node(package="object_tracking_viz", 
                            executable="object_tracking",
                            name="object_tracking",
                            parameters=[config],)
    
    object_visualization = Node(package="object_tracking_viz", 
                                executable="object_visualization.py",
                                name="object_visualization",
                                parameters=[config],)

    ld.add_action(image_publisher)
    ld.add_action(image_subscriber)
    ld.add_action(object_tracking)
    ld.add_action(object_visualization)

    return ld