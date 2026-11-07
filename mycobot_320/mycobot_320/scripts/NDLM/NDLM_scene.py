from pathlib import Path
import time
import numpy as np
from Cobot_sdk import *

def setup_scene(robot_manager: ROSManager):
    """
    Configura la escena de trabajo agregando objetos a una instancia de ROSManager.
    """
    print("--- Configurando la escena de trabajo en RViz ---")
    brida_base = 175e-3
    rotz_piezas = -np.pi / 6
    offset_x = 12e-3
    offset_y = 0.0

    # Obtiene automáticamente la ruta de los STL en la misma carpeta del script
    carpeta_actual = Path(__file__).resolve().parent
    stl_porta = (carpeta_actual / "Porta_llavero.STL").as_uri()
    stl_llavero = (carpeta_actual / "Llavero.STL").as_uri()     
    stl_mesa = (carpeta_actual / "Mesa_matera10.STL").as_uri()

    # Verificar rápido si MoveIt (/apply_planning_scene) está activo en ROS 2.
    moveit_backup = getattr(robot_manager, "moveit_manager", None)
    if moveit_backup is not None and hasattr(moveit_backup, "_apply_cli"):
        if not moveit_backup._apply_cli.service_is_ready():
            print("[Escena] MoveIt no detectado activo. Cargando objetos solo en RViz (MarkerManager)...")
            robot_manager.moveit_manager = None

    try:
        robot_manager.add_scene_object(
            name="listón",
            pose_init=(offset_x + 273.0e-3, offset_y - 125.0e-3, brida_base),
            size=(30.0e-3, 30.0e-3, 325.0e-3),  # escala en metros
            color=(212/255, 212/255, 212/255, 1.0),
            shape=Shapes.CUBE,
            movable=False,
            rot_euler=(np.pi/2, 0, rotz_piezas)
        )

        robot_manager.add_scene_object(
            name="porta_llavero1",
            pose_init=(offset_x + 235.0e-3 - 12e-3 + 3e-3, offset_y - 171.0e-3 + 70e-3 + 4e-3, brida_base + 20e-3),
            size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
            color=(0.0, 1.0, 0.0, 0.5),
            shape=Shapes.MESH,
            movable=False,
            mesh=stl_porta,
            rot_euler=(np.pi/2, 0, rotz_piezas)
        )

        robot_manager.add_scene_object(
            name="llavero1",
            pose_init=(offset_x + 247.0e-3 - 18e-3 + 3e-3, offset_y - 165.721e-3 + 52e-3 + 4e-3, brida_base + 40e-3),
            size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
            color=(243/255, 133/255, 64/255, 1.0),
            shape=Shapes.MESH,
            movable=True,
            mesh=stl_llavero,
            rot_euler=(np.pi/2, 0, rotz_piezas)
        )

        robot_manager.add_scene_object(
            name="porta_llavero2",
            pose_init=(offset_x + 278.0e-3 - 12e-3 + 15e-3, offset_y - 62.0e-3 + 70e-3 - 6e-3, brida_base + 20e-3),
            size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
            color=(0.0, 1.0, 0.0, 0.5),
            shape=Shapes.MESH,
            movable=False,
            mesh=stl_porta,
            rot_euler=(np.pi/2, 0, rotz_piezas)
        )

        robot_manager.add_scene_object(
            name="llavero2",
            pose_init=(offset_x + 290.0e-3 - 18e-3 + 15e-3, offset_y - 56.721e-3 + 52e-3 - 6e-3, brida_base + 40e-3),
            size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
            color=(143/255, 33/255, 64/255, 1.0),
            shape=Shapes.MESH,
            movable=True,
            mesh=stl_llavero,
            rot_euler=(np.pi/2, 0, rotz_piezas)
        )

        robot_manager.add_scene_object(
            name="porta_llavero3",
            pose_init=(offset_x + 188.0e-3 - 12e-3 + 5e-3, offset_y - 266.0e-3 + 70e-3 + 5e-3, brida_base + 20e-3),
            size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
            color=(0.0, 1.0, 0.0, 0.5),
            shape=Shapes.MESH,
            movable=False,
            mesh=stl_porta,
            rot_euler=(np.pi/2, 0, rotz_piezas)
        )

        robot_manager.add_scene_object(
            name="llavero3",
            pose_init=(offset_x + 200.0e-3 - 18e-3 + 5e-3, offset_y - 260.721e-3 + 52e-3 + 5e-3, brida_base + 40e-3),
            size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
            color=(43/255, 33/255, 164/255, 1.0),
            shape=Shapes.MESH,
            movable=True,
            mesh=stl_llavero,
            rot_euler=(np.pi/2, 0, rotz_piezas)
        )

        robot_manager.add_scene_object(
            name="mesa",
            pose_init=(0.546, -0.257, 0.0),
            size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
            color=(128/255, 81/255, 46/255, 1.0),
            shape=Shapes.MESH,
            movable=False,
            mesh=stl_mesa,
            rot_euler=(np.pi/2, 3.14, np.pi)
        )
    finally:
        # Restaurar la referencia de moveit_manager
        robot_manager.moveit_manager = moveit_backup

    time.sleep(1.0)

def run(robot, **kwargs):
    setup_scene(robot)