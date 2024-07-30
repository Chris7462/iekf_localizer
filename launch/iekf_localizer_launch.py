from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = join(
        get_package_share_directory('iekf_localizer'), 'params', 'iekf_localizer.yaml'
    )

    iekf_localizer_node = Node(
        package='iekf_localizer',
        executable='iekf_localizer_node',
        name='iekf_localizer_node',
        parameters=[params]
    )

    return LaunchDescription([
        iekf_localizer_node
    ])
