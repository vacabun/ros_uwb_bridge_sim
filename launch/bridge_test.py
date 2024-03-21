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


    node4 = Node(
        package='uwb_ros_bridge_sim',
        executable='bridge',
        name='bridge_1',
	    output='screen',
        parameters=[{'label_name': "x500_4"}]
    )
    ld.add_action(node4)

    node5 = Node(
        package='uwb_ros_bridge_sim',
        executable='bridge',
        name='bridge_5',
	    output='screen',
        parameters=[{'label_name': "x500_5"}]
    )
    ld.add_action(node5)

    return ld