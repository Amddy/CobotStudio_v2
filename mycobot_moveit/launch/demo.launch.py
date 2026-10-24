import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

def patch_and_launch(context, *args, **kwargs):
    """
    Lee el argumento de la consola y modifica el .xacro para lanzar la configuración de pinza elegida.
    """
    
    # Obtener el argumento booleano de la terminal
    align_value = LaunchConfiguration("gripper_align").perform(context)
    
    # Validación simple
    if align_value.lower() not in ['true', 'false']:
        print(f"[Launch] ⚠Valor '{align_value}' no válido. Forzando 'false'.")
        align_value = 'false'

    print(f"[Launch] Configurando pinza: gripper_align = {align_value}")

    # Modificar el xacro según el argumento
    try:
        # Buscar la ruta instalada (que es un symlink al src)
        pkg_share = get_package_share_directory("mycobot_description")
        xacro_path = os.path.join(pkg_share, "urdf/mycobot_320_pi_2022/mycobot_320_pi_2022DH.urdf.xacro")
        
        if os.path.exists(xacro_path):
            with open(xacro_path, 'r') as f:
                content = f.read()

            # Regex para cambiar default="true/false" en gripper_align_0
            pattern = r'(<xacro:arg\s+name="gripper_align"\s+default=")(true|false)("\s*/>)'
            
            # Verificar si ya tiene el valor correcto para no reescribir sin sentido
            match = re.search(pattern, content)
            current_val = match.group(2) if match else None

            if current_val != align_value:
                # Reemplazar si es necesario
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
        # Se intenta lanzar igual

    # Ejecución original según lo generado en el SetupAssistant   
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mycobot_moveit").to_moveit_configs()
    
    return [generate_demo_launch(moveit_config)]


def generate_launch_description():
    
    # Declarar el argumento
    gripper_arg = DeclareLaunchArgument(
        "gripper_align",
        default_value="false",
        description="Orientación de la pinza: true=0deg, false=90deg"
    )

    # Declaramos el argumento mode
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="ros",
        description="Modo de ejecucion: 'ros' (simulador) o 'real' (robot fisico)"
    )

    # nodo de hardware_inferface
    
    hardware_interface_node = Node(
        package="mycobot_320",
        executable="mycobot_hardware_interface",
        name = "mycobot_hardware_interface",
        output ="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("mode"), "' == 'real'"]))
        )

    # Envolver la lógica
    return LaunchDescription([
        gripper_arg,
        mode_arg,
        hardware_interface_node,
        OpaqueFunction(function=patch_and_launch)
    ])