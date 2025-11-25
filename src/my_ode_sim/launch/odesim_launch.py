from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # use_3d_lidar をコマンドライン引数から指定できるように
    use_3d_lidar_arg = DeclareLaunchArgument(
        'use_3d_lidar',
        default_value='true',
        description='Use 3D LiDAR (true) or 2D LiDAR (false)'
    )

    use_3d_lidar = LaunchConfiguration('use_3d_lidar')

    # YAML パラメータファイルのパス
    pkg_share = get_package_share_directory('my_ode_sim')
    params_file = os.path.join(pkg_share, 'config', 'odesim_params.yaml')

    odesim_node = Node(
        package='my_ode_sim',
        executable='odesim_node',
        name='ode_diffdrive_sim',
        output='screen',
        parameters=[
            params_file,
            {'use_3d_lidar': ParameterValue(use_3d_lidar, value_type=bool)}
        ]
    )

    return LaunchDescription([
        use_3d_lidar_arg,
        odesim_node
    ])
