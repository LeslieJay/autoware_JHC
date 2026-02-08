from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取激光包的 launch 文件路径
    first_launch_file = os.path.join(
        get_package_share_directory('rslidar_sdk'),  
        'launch',
        'start.py'  # 替换为你的 launch 文件名
    )

    # 获取第二个包的 launch 文件路径
    second_launch_file = os.path.join(
        get_package_share_directory('yesense_std_ros2'),  # 替换为你的第二个包名
        'launch',
        'yesense_node.launch.py'  # 替换为你的 launch 文件名
    )

    return LaunchDescription([
        # 包含第一个 launch 文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(first_launch_file),
            # 可选：传递参数到子 launch 文件
            # launch_arguments={'arg1': 'value1'}.items(),
        ),

        # 包含第二个 launch 文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(second_launch_file),
            # 可选：传递参数
            # launch_arguments={'arg2': 'value2'}.items(),
        ),
    ])