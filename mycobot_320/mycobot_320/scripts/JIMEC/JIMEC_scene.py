from Cobot_sdk import *

def setup_scene(robot: ROSManager):
        """
        Configura la escena de trabajo agregando objetos a una instancia de ROSManager.

        Args:
        robot: La instancia de ROSManager a la que se agregarán los objetos.
        """
        print("--- Configurando la escena de trabajo en RViz ---")
        altura_caja = 0.07 # Altura de los bloques de madera a apilar
        brida_base = 0.02

        # Agregamos los objetos según sus dimensiones
        robot.add_scene_object("caja1", pose_init=(0.1, -0.2, altura_caja/2- brida_base),
                        size=(0.04, 0.04, altura_caja),
                        color=(1.0, 0.0, 0.0, 0.9),
                        shape=Shapes.CUBE, movable=True)
        
        robot.add_scene_object("caja2", pose_init=(0.1, -0.265, altura_caja/2 - 0.015- brida_base),
                size=(0.04, 0.04, altura_caja),
                color=(0.0, 1.0, 0.0, 0.9),
                shape=Shapes.CUBE, movable=True)
        
        robot.add_scene_object("caja3", pose_init=(0.1, -0.33, altura_caja/2 - 0.015- brida_base),
                size=(0.04, 0.04, altura_caja),
                color=(0.0, 0.0, 1.0, 0.9),
                shape=Shapes.CUBE, movable=True)
        
def run(robot, **kwargs):
    setup_scene(robot)