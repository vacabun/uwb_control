from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    node1 = Node(
        package='uwb_ros_bridge_sim',
        executable='bridge',
        name='bridge_1',
	    output='screen',
        parameters=[{'label_name': "x500_1"}]
    )
    ld.add_action(node1)

    node2 = Node(
        package='uwb_ros_bridge_sim',
        executable='bridge',
        name='bridge_2',
	    output='screen',
        parameters=[{'label_name': "x500_2"}]
    )
    ld.add_action(node2)
    
    node3 = Node(
        package='uwb_ros_bridge_sim',
        executable='bridge',
        name='bridge_3',
	    output='screen',
        parameters=[{'label_name': "x500_3"}]
    )
    ld.add_action(node3)

    anchors_str = '11,12,13,14'
    brigde_service = '/x500_2/uwb_bridge'
    node5 = Node(
        package='uwb_control',      # 节点的包名
        executable='controller_with_anchor',     # 节点的可执行文件名
        name='uwb_controller_2',                # 节点的名称
        output='screen',              # 输出日志到屏幕
        parameters=[{'anchor_list': anchors_str, 'brigde_service': brigde_service}]
    )
    ld.add_action(node5)
    brigde_service = '/x500_1/uwb_bridge'
    node4 = Node(
        package='uwb_control',      # 节点的包名
        executable='controller_with_anchor',     # 节点的可执行文件名
        name='uwb_controller_1',                # 节点的名称
        output='screen',              # 输出日志到屏幕
        parameters=[{'anchor_list': anchors_str, 'brigde_service': brigde_service}]
    )
    ld.add_action(node4)





    return ld
