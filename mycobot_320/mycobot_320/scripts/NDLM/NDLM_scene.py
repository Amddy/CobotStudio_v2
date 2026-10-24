from Cobot_sdk import *

def setup_scene(robot_manager: ROSManager):
        """
        Configura la escena de trabajo agregando objetos a una instancia de SimManager.

        Args:
        robot_manager: La instancia de SimManager a la que se agregarán los objetos.
        """
        print("--- Configurando la escena de trabajo en RViz ---")
        brida_base = 0.0
        rotz_piezas = -np.pi/6.6 

        robot_manager.add_scene_object(name="listón",
        pose_init=(273.0e-3, -125.0e-3,- brida_base),
        size=(30.0e-3, 30.0e-3, 325.0e-3),  # escala en metros
        color=(212/255, 212/255, 212/255, 1.0),
        shape=Shapes.CUBE,
        movable=False,
        rot_euler=(np.pi/2, 0, rotz_piezas))

        robot_manager.add_scene_object(name="porta_llavero1",
        pose_init=(235.0e-3 - 12e-3 + 3e-3, -171.0e-3 + 70e-3 + 4e-3, - brida_base + 20e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(0.0, 1.0, 0.0, 0.5),
        shape=Shapes.MESH,
        movable=False,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Porta_llavero.STL",
        rot_euler=(np.pi/2, 0, rotz_piezas))

        robot_manager.add_scene_object(name="llavero1",
        pose_init=(247.0e-3 - 18e-3 + 3e-3, -165.721e-3 + 52e-3 + 4e-3, - brida_base + 40e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(243/255, 133/255, 64/255, 1.0),
        shape=Shapes.MESH,
        movable=True,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Llavero.STL",
        rot_euler=(np.pi/2, 0, rotz_piezas))

        robot_manager.add_scene_object(name="porta_llavero2",
        pose_init=(278.0e-3 - 12e-3 + 15e-3, -62.0e-3 + 70e-3 - 6e-3, - brida_base + 20e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(0.0, 1.0, 0.0, 0.5),
        shape=Shapes.MESH,
        movable=False,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Porta_llavero.STL",
        rot_euler=(np.pi/2, 0, rotz_piezas))

        robot_manager.add_scene_object(name="llavero2",
        pose_init=(290.0e-3 - 18e-3 + 15e-3, -56.721e-3 + 52e-3 - 6e-3, - brida_base + 40e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(143/255, 33/255, 64/255, 1.0),
        shape=Shapes.MESH,
        movable=True,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Llavero.STL",
        rot_euler=(np.pi/2, 0, rotz_piezas))

        robot_manager.add_scene_object(name="porta_llavero3",
        pose_init=(188.0e-3 - 12e-3 + 5e-3, -266.0e-3 + 70e-3 + 5e-3, - brida_base + 20e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(0.0, 1.0, 0.0, 0.5),
        shape=Shapes.MESH,
        movable=False,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Porta_llavero.STL",
        rot_euler=(np.pi/2, 0, rotz_piezas))

        robot_manager.add_scene_object(name="llavero3",
        pose_init=(200.0e-3 - 18e-3 + 5e-3, -260.721e-3 + 52e-3 + 5e-3, - brida_base + 40e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(43/255, 33/255, 164/255, 1.0),
        shape=Shapes.MESH,
        movable=True,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Llavero.STL",
        rot_euler=(np.pi/2, 0, rotz_piezas))


        time.sleep(1.0)

def run(robot, **kwargs):
    setup_scene(robot)