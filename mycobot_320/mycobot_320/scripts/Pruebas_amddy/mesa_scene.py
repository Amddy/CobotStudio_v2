from pathlib import Path
from Cobot_sdk import *

# Obtiene la carpeta donde está guardado este archivo .py y entra a "Mallas"
# .resolve() obtiene la ruta absoluta real en la PC del usuario actual
MALLAS_DIR = (Path(__file__).parent / "Mallas").resolve()

def setup_scene(robot: ROSManager):
        """
        Configura la escena de trabajo agregando objetos a una instancia de SimManager.

        Args:
        robot: La instancia de SimManager a la que se agregarán los objetos.
        """
        print("--- Configurando la escena de trabajo en RViz ---")

        robot.add_scene_object(name="mesa",
        pose_init=(0.0, 0.0, 0.0),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(128/255, 81/255, 46/255, 1.0),
        shape=Shapes.MESH,
        movable=False,
        mesh=(MALLAS_DIR / "Mesa_matera9.STL").as_uri(),
        rot_euler=(np.pi/2, 3.14, 0.0))

        
        time.sleep(1.0)
        print("--- Escena configurada ---")

def run(robot, **kwargs):
    setup_scene(robot)