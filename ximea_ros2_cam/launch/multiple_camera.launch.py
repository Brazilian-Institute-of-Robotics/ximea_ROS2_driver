import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate description for camera nodes"""
    package_name ="ximea_ros2_cam"

    cam1_config_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "xi_cam1_config.yaml")

    cam2_config_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "xi_cam2_config.yaml")

    camera1_node = Node(
        package=package_name,
        executable='ximea_ros2_cam_node',
        name='ximea_cam1_node',
        parameters=[cam1_config_params],
        remappings=[
                ('/ximea/camera_info', '/ximea/camera_info_1'),
                ('/ximea/image_raw', '/ximea/image_raw_1')
            ],
        output="screen",
    )


    camera2_node = Node(
        package=package_name,
        executable='ximea_ros2_cam_node',
        name='ximea_cam2_node',
        parameters=[cam2_config_params],
        remappings=[
                ('/ximea/camera_info', '/ximea/camera_info_2'),
                ('/ximea/image_raw', '/ximea/image_raw_2')
            ],
        output="screen",
    )

    # Define 5 seconds delay to run the second node
    delayed_camera2_node = TimerAction(
        period=5.0,  # seconds
        actions=[camera2_node]
    )

    return LaunchDescription([
        camera1_node,       # Run first node
        delayed_camera2_node  # Run second node after delay
    ])
