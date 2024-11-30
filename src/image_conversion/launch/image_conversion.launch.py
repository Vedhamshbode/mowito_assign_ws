import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    camera_arg = DeclareLaunchArgument('camera_topic', default_value="/usb_cam_0")
    input_image_topic_arg = DeclareLaunchArgument('input_topic', default_value="/usb_cam_0/image_raw")
    output_image_topic_arg = DeclareLaunchArgument('output_topic', default_value="/converted_image")

    Image_Converter_Node = Node(
        package="image_conversion",
        executable="image_converter_node",
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic')
        }]
    )
    
    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        remappings=[('__ns', LaunchConfiguration('camera_topic'))]
    )

    return LaunchDescription([
        camera_arg,
        input_image_topic_arg,
        output_image_topic_arg,
        usb_cam_node,
        Image_Converter_Node,
    ])
    