import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch



def patch_and_launch(context, *args, **kwargs):
    """
    Lee el argumento de la consola, modifica el .xacro para la pinza
    y configura MoveIt con soporte para OctoMap (sensors_3d).
    """
    align_value = LaunchConfiguration("gripper_align").perform(context)

    if align_value.lower() not in ['true', 'false']:
        print(f"[Launch] ⚠ Valor '{align_value}' no válido. Forzando 'false'.")
        align_value = 'false'

    print(f"[Launch] Configurando pinza: gripper_align = {align_value}")

    # Modificar el xacro según el argumento
    try:
        pkg_share = get_package_share_directory("mycobot_description")
        xacro_path = os.path.join(pkg_share, "urdf/mycobot_320_pi_2022/mycobot_320_pi_2022DH.urdf.xacro")

        if os.path.exists(xacro_path):
            with open(xacro_path, 'r') as f:
                content = f.read()

            pattern = r'(<xacro:arg\s+name="gripper_align"\s+default=")(true|false)("\s*/>)'
            match = re.search(pattern, content)
            current_val = match.group(2) if match else None

            if current_val != align_value:
                new_content = re.sub(pattern, f'\\g<1>{align_value}\\g<3>', content)
                with open(xacro_path, 'w') as f:
                    f.write(new_content)
                print(f"[Launch] Archivo .xacro actualizado (src modificado via Symlink).")
            else:
                print(f"[Launch] El archivo ya estaba configurado correctamente.")
        else:
            print(f"[Launch] Error: No se encontró el archivo en {xacro_path}")

    except Exception as e:
        print(f"[Launch] Error crítico parcheando URDF: {e}")

    # Construcción de MoveItConfigs incluyendo sensors_3d.yaml
    moveit_config = (
        MoveItConfigsBuilder("firefighter", package_name="mycobot_moveit")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    # Inyectar parámetros globales del OctoMap en move_group
    moveit_config.move_group_capabilities["capabilities"] = ""
    moveit_config.trajectory_execution["octomap_frame"] = "base"
    moveit_config.trajectory_execution["octomap_resolution"] = 0.025
    moveit_config.trajectory_execution["max_range"] = 1.8

    return [generate_demo_launch(moveit_config)]


def generate_launch_description():

    # Nodo encargado de cargar la mesa en RViz y en el Planning Scene de MoveIt
    # Espera 5 segundos a que move_group inicie el servicio /apply_planning_scene
    mesa_scene_node = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    "CobotLauncher.py",
                    "scripts/Pruebas_amddy/mesa_scene.py",
                    "ros"
                ],
                cwd="/home/amddy/CobotStudio/mycobot_320/mycobot_320",
                output="screen"
            )
        ]
    )

    # Argumento de orientación de pinza
    gripper_arg = DeclareLaunchArgument(
        "gripper_align",
        default_value="false",
        description="Orientación de la pinza: true=0deg, false=90deg"
    )

    # Argumento para activar/desactivar la cámara Asus Xtion
    use_camera_arg = DeclareLaunchArgument(
        "use_camera",
        default_value="true",
        description="Lanzar driver Asus Xtion y OctoMap: true o false"
    )

    # Argumento para declara el frame_id de la camara
    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame",
        default_value="camera_link",
        description="Frame raíz reportado por openni2_camera"
    )

    # --- NODOS DE LA CÁMARA ASUS XTION PRO LIVE ---
    # Driver openni2_camera
    openni2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("openni2_camera"),
                "launch",
                "camera_only.launch.py"
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_camera"))
    )

    # Nodo para generar la nube de puntos con color (XYZRGB)
    point_cloud_xyzrgb_node = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        name="point_cloud_xyzrgb_node",
        output="screen",
        parameters=[{
            "qos_overrides./points.publisher.reliability": "reliable",
            "qos_overrides./point_cloud/points.publisher.reliability": "reliable",
        }],
        remappings=[
            ("rgb/image_rect_color", "/camera/rgb/image_raw"),
            ("rgb/camera_info", "/camera/rgb/camera_info"),
            ("depth_registered/image_rect", "/camera/depth/image_raw"),
            ("points", "/point_cloud/points"),
        ],
        condition=IfCondition(LaunchConfiguration("use_camera"))
    )

    # Transformación estática (TF2) entre la 'base' del MyCobot y la cámara
    # Ajustar x, y, z (metros) y roll, pitch, yaw (radianes) según donde este la cámara
    camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_base_tf_publisher",
        arguments=[
            "--x", "0.59",
            "--y", "-0.12",
            "--z", "0.32",
            "--roll", "0.0",
            "--pitch", "0.2",
            "--yaw", "3.0 ",
            "--frame-id", "base",
            "--child-frame-id", LaunchConfiguration("camera_frame")
        ],
        condition=IfCondition(LaunchConfiguration("use_camera"))
    )

    return LaunchDescription([
        gripper_arg,
        use_camera_arg,
        camera_frame_arg,
        openni2_launch,
        point_cloud_xyzrgb_node,
        camera_tf_node,
        mesa_scene_node,
        OpaqueFunction(function=patch_and_launch)
    ])