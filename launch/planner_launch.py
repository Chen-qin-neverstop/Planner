import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    启动 MPC Planner 节点
    """
    
    # 获取配置文件路径
    planner_share_dir = get_package_share_directory('planner')
    config_file = os.path.join(planner_share_dir, 'config', 'planner_config.yaml')
    
    # 声明参数
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='配置文件路径'
    )
    
    # Planner 节点
    planner_node = Node(
        package='planner',
        executable='planner_node',
        name='planner',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # 可以在这里重映射话题
            # ('/detector/armors', '/your/custom/topic'),
        ]
    )
    
    return LaunchDescription([
        config_arg,
        planner_node,
    ])
