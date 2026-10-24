from Cobot_sdk import *
import time

def run(robot: BaseRobotController, **kwargs):
    # 1. Definir herramienta (ejemplo: pinza o palpador) y workobject nulo (base)
    pinza = SE3(0, 0, 100)  
    wobj = SE3()

    print(">>> Paso 1: Llevando el robot a Home con MoveJ_q...")
    robot.GoHome(speed=30)
    time.sleep(1.0)

    print(">>> Paso 2: Ejecutando trayectoria sincronizada de MoveIt...")
    # Ejecuta la trayectoria guardada y recupera el RobTarget exacto donde terminó
    robt_fin_moveit = robot.traj_moveit(
        archivo="mi_archivo",
        traj_name="TRAJ",
        tool=pinza
    )

    if robt_fin_moveit is None:
        print("[!] Falló la carga de la trayectoria.")
        return

    print(f">>> Pose final de MoveIt alcanzada:\n{robt_fin_moveit.pose}")
    time.sleep(1.0)

    print(">>> Paso 3: Probando encadenamiento con MoveL (bajada vertical de 20 mm)...")
    # Usamos el método offset() de RobTarget sobre la pose que devolvió traj_moveit
    robt_bajada = robt_fin_moveit.offset(dz=-20.0)
    robot.MoveL(robt_bajada, speed=20, tool=pinza, wobj=wobj)
    time.sleep(1.0)

    print(">>> Paso 4: Subiendo de regreso con MoveJ y volviendo a Home...")
    robot.MoveJ(robt_fin_moveit, speed=25, tool=pinza, wobj=wobj)
    time.sleep(0.5)
    robot.GoHome(speed=30)
    print(">>> ¡Prueba híbrida completada con éxito!")