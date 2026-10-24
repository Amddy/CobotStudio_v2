from CobotStudio import ROSManager

# Inicia nodo MoveItManager que queda escuchando /display_planned_path

def run(robot: ROSManager, **kwargs):
    robot.load_scene("mesa_scene")

    input("\n>>> 1. Andá a RViz, mové el robot y dale a 'Plan' o 'Plan & Execute'.\n>>> 2. Cuando termine, presioná [ENTER] acá para guardarla...")

    if robot.moveit_manager.save_trajectory("trayectoria_1", "TRAJ_POS_1"):
        print("¡Listo! Guardada en Trayectorias/")
    else:
        print("No se detectó ninguna trayectoria.")

    robot.shutdown()