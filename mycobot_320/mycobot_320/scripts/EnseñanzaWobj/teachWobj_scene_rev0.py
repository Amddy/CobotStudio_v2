from Cobot_sdk import *

def setup_scene(robot: ROSManager):
        """
        Configura la escena de trabajo agregando objetos a una instancia de SimManager.

        Args:
        robot_manager: La instancia de SimManager a la que se agregarán los objetos.
        """
        print("--- Configurando la escena de trabajo en RViz ---")
        altura_base = 0.025 # Base de aluminio del cobot
        altura_mesa = 0.09 # Altura de los bloques de madera a apilar
        brida_base = 0.02

        # Agregamos los objetos según sus dimensiones
        robot.add_scene_object("robot_base", pose_init=(0.0, 0.0, -altura_base/2 - brida_base),
                        size=(0.51, 0.51, altura_base),
                        color=(1.0, 1.0, 1.0, 0.9),
                        shape=Shapes.CYLINDER, movable=False)
        
        robot.add_scene_object("mesa", pose_init=(-20e-3, -445.00e-3, altura_mesa/2- brida_base),
                        size=(300e-3, 300e-3, altura_mesa),
                        color=(0.38, 0.38, 0.38, 0.8),
                        shape=Shapes.CUBE, movable=True, rot_euler=np.deg2rad((0, 0, -6)))
        
        robot.add_scene_object(name="pieza_aux",
        pose_init=(0.0, -181.721e-3, 402.32e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(1.0, 0.0, 0.0, 0.5),
        shape=Shapes.MESH,
        movable=True,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/EnseñanzaWobj/Palpador_pinza4.STL",
        rot_euler=(np.pi/2 + 0*np.pi/15, 0, 0))