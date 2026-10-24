from Cobot_sdk import *

def run(robot: ROSManager, **kwargs):
    #  Definir una pose objetivo (o vector q en radianes)
    robot.load_scene("mesa_scene")

  
    q_objetivo = [np.deg2rad(0), np.deg2rad(-60), np.deg2rad(-10), np.deg2rad(70), np.deg2rad(55), np.deg2rad(0)]

    #  Planificar y ejecutar en MoveIt/RViz
    robot.moveit_manager.plan_and_execute(target=q_objetivo, execute=True)

    #  Guardar la trayectoria interpolada en Trayectorias/mi_archivo.py
    ok = robot.moveit_manager.save_trajectory("mi_archivo", "TRAJ_1")
    print("¿Se guardó en la carpeta Trayectorias?:", ok)