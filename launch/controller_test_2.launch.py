from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()


    node1 = Node(
        package='uwb_control',      # 节点的包名
        executable='controller',     # 节点的可执行文件名
        name='uwb_controller_3',                # 节点的名称
        output='screen',              # 输出日志到屏幕
    )
    ld.add_action(node1)


    node2 = Node(
        package='uwb_control',      # 节点的包名
        executable='controller',     # 节点的可执行文件名
        name='uwb_controller_4',                # 节点的名称
        output='screen'              # 输出日志到屏幕
    )
    ld.add_action(node2)
    

    return ld