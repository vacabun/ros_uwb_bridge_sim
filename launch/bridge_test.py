from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    node1 = Node(
        package='uwb_ros_bridge_sim',
        executable='bridge',
        name='bridge_0',
	    output='screen',
        parameters=[{'label_name': "x500_0"}]
    )
    ld.add_action(node1)

    node2 = Node(
        package='uwb_ros_bridge_sim',
        executable='bridge',
        name='bridge_1',
	    output='screen',
        parameters=[{'label_name': "x500_1"}]
    )
    ld.add_action(node2)
    return ld