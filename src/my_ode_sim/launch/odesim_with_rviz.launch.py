from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # LiDAR モードを指定する引数（true: 3D, false: 2D）
    use_3d_lidar_arg = DeclareLaunchArgument(
        'use_3d_lidar',
        default_value='true',
        description='Use 3D LiDAR (true) or 2D LiDAR (false)'
    )

    use_3d_lidar = LaunchConfiguration('use_3d_lidar')

    # パッケージの share ディレクトリ
    pkg_share = get_package_share_directory('my_ode_sim')

    # パラメータ YAML
    params_file = os.path.join(pkg_share, 'config', 'odesim_params.yaml')

    # RViz 設定ファイル
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'odesim.rviz')

    # ODE シミュレータノード
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

    # RViz2 ノード
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        use_3d_lidar_arg,
        odesim_node,
        rviz_node
    ])
