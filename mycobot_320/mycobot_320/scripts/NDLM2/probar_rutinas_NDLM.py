from Cobot_sdk import *
import random
import time
import numpy as np

ARCHIVO_TRAJ = "Rutina_NDLM"

ESPERA_ENTRE_TRAMOS = 1.0
ESPERA_AGARRE_LLAVE = 1.5
ESPERA_ENTREGA_PUBLICO = 4.0


def run(robot: BaseRobotController, **kwargs):
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    robot.load_scene("NDLM_scene_2")


    catalogo = robot.load_data("Trayectorias", ARCHIVO_TRAJ, "CATALOGO_RUTAS")
    if not catalogo:
        print(f"[!] No se encontró CATALOGO_RUTAS en Trayectorias/{ARCHIVO_TRAJ}.py")
        return

    def saludar():
        saludo1 = RobTarget(SE3(215, 0.5, 425) * SE3.Ry(np.pi/2) * SE3.Rz(np.pi/2), [-1, 1, 1])
        saludo2 = RobTarget(SE3(250, 0.5, 350) * SE3.Ry(np.pi/2) * SE3.Rz(np.pi/2) * SE3.Rx(np.pi/4), [-1, 1, 1])
        robot.MoveJ(saludo1, 50, pinza, SE3())
        robot.MoveJ(saludo2, 40, pinza, SE3())
        robot.MoveJ(saludo1, 30, pinza, SE3())
        robot.GoHome(30)


    def ejecutar_llave(num_llave: int):
        variantes = catalogo.get(num_llave) or []
        if not variantes:
            print(f"[!] No hay variantes válidas para la Llave {num_llave}.")   
            return

        ruta = random.choice(variantes)
        print(f"\n>>> Ejecutando Llave {num_llave} (Variante {ruta['variante']}) "
              f"| Config Llave: {ruta['conf_llave']} -> Entrega: {ruta['conf_entrega']}")
         
        # saludar()

        robot.GripperState(60, 30)
        time.sleep(0.5)

        print(f"  [1/3] Home -> Llave {num_llave}...")
        robt_llave = robot.traj_moveit(ARCHIVO_TRAJ, ruta["ida"], tool=pinza)
        time.sleep(ESPERA_ENTRE_TRAMOS)

        print("  [*] Acercando en línea recta con MoveL...")
        robot.MoveL(robt_llave.offset(0, 0, -35), speed=10, tool=pinza, wobj=SE3())
        time.sleep(1.5)
        print("  [*] Cerrando pinza...")
        robot.GripperState(0, 30)
        time.sleep(ESPERA_AGARRE_LLAVE)


        print("  [*] Retirando llave con MoveL...")
        robot.MoveL(robt_llave, speed=10, tool=pinza, wobj=SE3())
        time.sleep(0.5)

        print("  [2/3] Llave -> Entrega...")
        robot.traj_moveit(ARCHIVO_TRAJ, ruta["entrega"], tool=pinza)

        
        # print(f"  [*] Esperando {ESPERA_ENTREGA_PUBLICO}s para entregar...")
        input("  [*] Presione [Enter] para realizar la entrega...")
        # time.sleep(ESPERA_ENTREGA_PUBLICO)
        robot.GripperState(100, 30)
        time.sleep(1.0)

        print("  [3/3] Entrega -> Home...")
        robot.traj_moveit(ARCHIVO_TRAJ, ruta["home"], tool=pinza)
        time.sleep(ESPERA_ENTRE_TRAMOS)

    while True:
        print("\n=== MENÚ NOCHE DE LOS MUSEOS ===")
        print(f"  [1] Llave 1 ({len(catalogo.get(1, []))} variantes limpias)")
        print(f"  [2] Llave 2 ({len(catalogo.get(2, []))} variantes limpias)")
        print(f"  [3] Llave 3 ({len(catalogo.get(3, []))} variantes limpias)")
        print("  [a] Aleatorio (Llave 1, 2 o 3)")
        print("  [s] Solo saludar")
        print("  [q] Salir")

        op = input("\nOpción: ").strip().lower()
        if op == "q":
            break
        elif op == "s":
            saludar()
        elif op == "a":
            ejecutar_llave(random.choice([1, 2, 3]))
        elif op in ("1", "2", "3"):
            ejecutar_llave(int(op))