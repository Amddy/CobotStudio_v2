import os

from ament_index_python import get_package_share_path
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    res = []

    model_launch_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_path("mycobot_description"),
            "urdf/mycobot_320_pi_2022/mycobot_320_pi_2022.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(
            get_package_share_path("mycobot_320pi"),
            "config/mycobot_320_pi.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    gui_launch_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true"
    )
    res.append(gui_launch_arg)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    res.append(robot_state_publisher_node)

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    res.append(joint_state_publisher_node)

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    res.append(joint_state_publisher_gui_node)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)
    
    controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=["~/colcon_ws/build/mycobot_description/urdf/mycobot_320_pi_2022/mycobot_320_pi_2022.urdf", "~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/ros2_controllers.yaml"],
        output="screen"
    )
    res.append(controller_manager)
        
    joint_state_broadcaster = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"]
    )
    res.append(joint_state_broadcaster)
    
    arm_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["arm_controller"]
    )
    res.append(arm_controller)

    return LaunchDescription(res)
