from pathlib import Path
import time
import numpy as np
from Cobot_sdk import *

# Obtiene la carpeta donde está guardado este archivo .py y entra a "Mallas"
MALLAS_DIR = (Path(__file__).parent / "Mallas").resolve()

def setup_scene(robot: ROSManager):
    """
    Configura la escena de trabajo agregando el archivo STL existente a RViz y al PlanningScene de MoveIt.
    """
    print("--- Configurando la escena de trabajo en RViz y MoveIt ---")

    robot.add_scene_object(
        name="mesa",
        pose_init=(0.546, -0.257, -0.072),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(128/255, 81/255, 46/255, 1.0),
        shape=Shapes.MESH,
        movable=False,
        mesh=(MALLAS_DIR / "Mesa_matera10.STL").as_uri(),
        rot_euler=(np.pi/2, 3.14, np.pi),
        touch_links=["base_mesa", "base_flange"]
    )

    time.sleep(1.0)
    print("--- Escena configurada con colisiones en MoveIt ---")

def run(robot, **kwargs):
    setup_scene(robot)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass