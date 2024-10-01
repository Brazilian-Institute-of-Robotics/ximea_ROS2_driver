import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name ="ximea_ros2_cam"

    cam1_config_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "xiCam1_config.yaml")

    cam2_config_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "xiCam2_config.yaml")

    first_node = Node(
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


    second_node = Node(
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

    # Definir um atraso de 10 segundos antes de iniciar o segundo nó
    delayed_second_node = TimerAction(
        period=5.0,  # seconds
        actions=[second_node]
    )

    return LaunchDescription([
        first_node,       # Executa o primeiro nó imediatamente
        delayed_second_node  # Inicia o segundo nó após 10 segundos
    ])
