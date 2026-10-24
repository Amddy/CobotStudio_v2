from Cobot_sdk import *
import time

def run(robot: BaseRobotController, **kwargs):
    # Definición del TCP de la pinza relevado en el proyecto
    pinza = SE3(-1.71, 106.91, 28.33)
    wobj_nulo = SE3()

    print(">>> 1. Llevando el robot a Home...")
    robot.GoHome(speed=20)
    time.sleep(3.0)

    print(">>> 2. Ejecutando trayectoria de MoveIt (traj_moveit)...")
    ultimo_robt = robot.traj_moveit(
        archivo="mi_archivo",
        traj_name="TRAJ_1",
        tool=pinza
    )

    if ultimo_robt is not None:
        print(f"\n>>> Trayectoria completada con éxito.")
        print(f">>> RobTarget final alcanzado:\n{ultimo_robt.pose}")
        print(f">>> Configuración final: {ultimo_robt.config}")

    
    time.sleep(1.0)

    # # Probar encadenamiento continuo con MoveL y GripperState
    # print("\n>>> [3/4] Probando encadenamiento: descenso relativo MoveL (dz = -25 mm) y pinza...")
    # robt_aprox = ultimo_robt.offset(dz=-25.0)
    # robot.MoveL(robt_aprox, speed=20, tool=pinza, wobj=wobj_nulo)
    # time.sleep(0.5)

    # robot.GripperState(100, 30)  # Abrir pinza
    # time.sleep(1.5)
    # robot.GripperState(0, 30)    # Cerrar pinza
    # time.sleep(1.5)

    # # 4Retorno a la pose de salida de MoveIt y vuelta a Home
    # print("\n>>> [4/4] Subiendo con MoveJ y regresando a Home...")
    # robot.MoveJ(ultimo_robt, speed=25, tool=pinza, wobj=wobj_nulo)
    # time.sleep(0.5)
    # robot.GoHome(speed=30)

    print("\n>>> ¡Protocolo finalizado con éxito!")



