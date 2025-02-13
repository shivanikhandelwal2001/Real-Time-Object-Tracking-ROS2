from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    image_publisher = Node(package="object_tracking_viz", 
                           executable="image_publisher", 
                           name="image_publisher", 
                           remappings=[("/camera/image_raw", "/camera/image_raw")],)
    
    image_subscriber = Node(package="object_tracking_viz", 
                            executable="image_subscriber.py",
                            name="image_subscriber",
                            remappings=[("/camera/image_raw", "/camera/image_raw")],)
    
    object_tracking = Node(package="object_tracking_viz", 
                            executable="object_tracking",
                            name="object_tracking",)
    
    object_visualization = Node(package="object_tracking_viz", 
                                executable="object_visualization.py",
                                name="object_visualization",)

    ld.add_action(image_publisher)
    ld.add_action(image_subscriber)
    ld.add_action(object_tracking)
    ld.add_action(object_visualization)

    return ld