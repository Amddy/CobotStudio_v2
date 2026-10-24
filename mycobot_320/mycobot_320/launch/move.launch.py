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
            "urdf/mycobot_320_pi_2022/mycobot_320_pi_2022DH.urdf.xacro"
        )
    )
    res.append(model_launch_arg)

    align_arg = DeclareLaunchArgument(
        name="gripper_align",
        default_value="false",
        choices=["true", "false"],
        description="Flag para configuración de la herramienta. Si es true, la pinza se monta alineada con la brida. Si es false, a 90 grados."
    )
    res.append(align_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(
            get_package_share_path("mycobot_320"),
            "config/mycobot_pro_320.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    # gui_launch_arg = DeclareLaunchArgument(
    #     name="gui",
    #     default_value="true"
    # )
    # res.append(gui_launch_arg)

    # robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
    #                                    value_type=str)

    robot_description_content = Command([
        'xacro ', LaunchConfiguration('model'),
        ' gripper_align:=', LaunchConfiguration('gripper_align')
    ])
    
    robot_description = ParameterValue(robot_description_content, value_type=str)

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'source_list': ['/joint_states']
        }]
    )

    res.append(joint_state_publisher_node)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    res.append(robot_state_publisher_node)


    # Joint_state_publisher con sliders interactivos
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )
    # res.append(joint_state_publisher_gui_node)


    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)

    return LaunchDescription(res)
