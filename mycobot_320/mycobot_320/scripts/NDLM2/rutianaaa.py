#!/usr/bin/env python3
"""
Script Maestro para La Noche de los Museos (demo_museos_master.py)
Integra:
  1. Rutina de Llaveros Multiconfiguración (Offline/Híbrida con MoveL + Saludo)
  2. Desafío en Vivo: Evasión Dinámica de Obstáculos con Cámara RGB-D y OctoMap
  3. Demo Didáctica: Movimiento en el Núcleo de la Singularidad de Muñeca (q5 = 0°)
  4. Demo Didáctica: Múltiples Configuraciones Cinemáticas para una Misma Pose
"""

from Cobot_sdk import *
import os
import random
import subprocess
import time
import importlib.util
from pathlib import Path
import numpy as np

# Nombre del archivo de trayectorias pre-calculadas para los llaveros
ARCHIVO_TRAJ = "Rutina_NDLM"

# Tiempos y parámetros operativos
ESPERA_ENTRE_TRAMOS = 1.0
ESPERA_AGARRE_LLAVE = 1.5
ESPERA_ENTREGA_PUBLICO = 4.0
DESCENSO_MOVEL_MM = -25.0

# Offset global de tu escena
dx, dy, dz = 0.0, 0.0, 175.0
offset_xy = SE3(dx, dy, dz)


def limpiar_octomap():
    """Llama al servicio /clear_octomap de MoveIt 2 para borrar vóxeles fantasma."""
    try:
        subprocess.run(
            ["ros2", "service", "call", "/clear_octomap", "std_srvs/srv/Empty"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2.0
        )
    except Exception:
        pass


def cargar_catalogo_robusto(robot, nombre_archivo: str):
    """Carga CATALOGO_RUTAS usando load_data o búsqueda directa en Trayectorias/."""
    try:
        cat = robot.load_data("Trayectorias", nombre_archivo, "CATALOGO_RUTAS")
        if cat:
            return cat
    except Exception:
        pass

    posibles_bases = []
    if hasattr(robot, "project_root") and robot.project_root:
        posibles_bases.append(Path(robot.project_root))
    posibles_bases.append(Path(os.path.dirname(os.path.abspath(__file__))))

    for base in posibles_bases:
        for nombre in (nombre_archivo, "mi_archivo", "reconfiguraciones"):
            ruta = base / "Trayectorias" / f"{nombre}.py"
            if ruta.exists():
                try:
                    spec = importlib.util.spec_from_file_location("mod_rutas", ruta)
                    modulo = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(modulo)
                    if hasattr(modulo, "CATALOGO_RUTAS"):
                        print(f"[*] Catálogo cargado desde: {ruta}")
                        return getattr(modulo, "CATALOGO_RUTAS")
                except Exception as e:
                    print(f"[!] Error leyendo {ruta}: {e}")
    return None


def ir_a_home_seguro(robot, speed=15):
    """Lleva el robot a Home [0,0,0,0,0,0] de manera compatible con ROS y Real."""
    q_home_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if hasattr(robot, "GoHome"):
        try:
            robot.GoHome(speed)
        except TypeError:
            robot.GoHome()
    elif hasattr(robot, "MoveJ_q"):
        robot.MoveJ_q(q_home_deg, speed, unit="deg")
    elif hasattr(robot, "MoveJAngles"):
        robot.MoveJAngles(np.zeros(6), speed, unit="rad")
    elif hasattr(robot, "mc"):
        robot.mc.send_angles(q_home_deg, speed)
        time.sleep(2.5)


def mover_articular_grados(robot, q_deg_list, speed=30, pausa=2.0):
    """Envía ángulos articulares en grados tanto en simulación como en el robot físico."""
    q_deg = np.asarray(q_deg_list, dtype=float)
    if hasattr(robot, "MoveJ_q"):
        # En ROSManager la firma es: MoveJ_q(self, q, spd=30, unit='deg')
        robot.MoveJ_q(q_deg, speed, unit="deg")
    elif hasattr(robot, "MoveJAngles"):
        robot.MoveJAngles(np.deg2rad(q_deg), speed, unit="rad")
    elif hasattr(robot, "mc"):
        robot.mc.send_angles(list(q_deg), speed)
        time.sleep(pausa)

def run(robot: BaseRobotController, **kwargs):
    # TCP calibrado de la pinza
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi / 2) * SE3.Rz(np.pi)
    wobj_nulo = SE3()

    if hasattr(robot, "load_scene"):
        try:
            robot.load_scene("NDLM_scene_2")
        except Exception:
            pass

    catalogo = cargar_catalogo_robusto(robot, ARCHIVO_TRAJ)

    # =========================================================================
    # MÓDULO 1: RUTINA DE LLAVEROS + SALUDO + MOVEL (OFFLINE / HÍBRIDA)
    # =========================================================================
    # def saludar():
    #     print("\n>>> Ejecutando Saludo al Público...")
    #     try:
    #         robot.traj_moveit(ARCHIVO_TRAJ, "SALUDO_1", tool=pinza)
    #         robot.traj_moveit(ARCHIVO_TRAJ, "SALUDO_2", tool=pinza)
    #         robot.traj_moveit(ARCHIVO_TRAJ, "SALUDO_3", tool=pinza)
    #         robot.traj_moveit(ARCHIVO_TRAJ, "SALUDO_HOME", tool=pinza)
    #         time.sleep(ESPERA_ENTRE_TRAMOS)
    #     except Exception:
    #         # Fallback con MoveJ directo si no están grabados los tramos de saludo
    #         if hasattr(robot, "toggle_ros_connection"):
    #             robot.toggle_ros_connection(False)
    #         saludo1 = RobTarget(offset_xy * SE3(215, 0.5, 425) * SE3.Ry(np.pi/2) * SE3.Rz(np.pi/2), [-1, 1, 1])
    #         saludo2 = RobTarget(offset_xy * SE3(250, 0.5, 350) * SE3.Ry(np.pi/2) * SE3.Rz(np.pi/2) * SE3.Rx(np.pi/4), [-1, 1, 1])
    #         robot.MoveJ(saludo1, 45, pinza, wobj_nulo)
    #         robot.MoveJ(saludo2, 40, pinza, wobj_nulo)
    #         robot.MoveJ(saludo1, 35, pinza, wobj_nulo)
    #         ir_a_home_seguro(robot, speed=30)

    def ejecutar_llave(num_llave: int, con_saludo: bool = True):
        if not catalogo:
            print(f"[!] Catálogo no disponible en Trayectorias/{ARCHIVO_TRAJ}.py.")
            return

        variantes = catalogo.get(num_llave) or catalogo.get(str(num_llave)) or []
        if not variantes:
            print(f"[!] No hay variantes válidas guardadas para la Llave {num_llave}.")
            return

        ruta = random.choice(variantes)
        print("\n" + "-" * 68)
        print(f" >>> ENTREGANDO LLAVERO {num_llave} (Variante Aleatoria #{ruta['variante']})")
        print(f"     Configuración Toma    : {ruta['conf_llave']}")
        print(f"     Configuración Entrega : {ruta['conf_entrega']}")
        print("-" * 68)

        try:
            # if con_saludo:
            #     saludar()

            # robot.GripperState(60, 30)
            # time.sleep(0.5)

            # print(f"  [1/3] Yendo de Home a Llavero {num_llave}...")
            # robt_llave = robot.traj_moveit(ARCHIVO_TRAJ, ruta["ida"], tool=pinza)
            # time.sleep(ESPERA_ENTRE_TRAMOS)

            # # Desactivar broadcaster temporalmente en simulación para que MoveL no parpadee
            # if hasattr(robot, "toggle_ros_connection"):
            #     robot.toggle_ros_connection(False)

            print(f"  [*] Descenso lineal MoveL ({DESCENSO_MOVEL_MM} mm)...")
            robot.MoveL(robt_llave.offset(0, 0, DESCENSO_MOVEL_MM), speed=12, tool=pinza, wobj=wobj_nulo)
            time.sleep(0.8)

            print("  [*] Cerrando pinza para sujetar llavero...")
            robot.GripperState(0, 30)
            time.sleep(ESPERA_AGARRE_LLAVE)

            print("  [*] Ascenso lineal MoveL de retorno...")
            robot.MoveL(robt_llave, speed=12, tool=pinza, wobj=wobj_nulo)
            time.sleep(0.5)

            print("  [2/3] Llevando llavero a posición de Entrega...")
            robot.traj_moveit(ARCHIVO_TRAJ, ruta["entrega"], tool=pinza)

            print(f"  [*] Esperando {ESPERA_ENTREGA_PUBLICO}s para que el visitante reciba el llavero...")
            time.sleep(ESPERA_ENTREGA_PUBLICO)
            robot.GripperState(100, 30)
            time.sleep(1.0)

            print("  [3/3] Regresando a Home...")
            robot.traj_moveit(ARCHIVO_TRAJ, ruta["home"], tool=pinza)
            robot.GripperState(0, 30)  # Dejar pinza cerrada en Home para el Self-Filtering
            time.sleep(ESPERA_ENTRE_TRAMOS)
            print(f"[OK] ¡Secuencia de Llavero {num_llave} completada!")

        except Exception as e:
            print(f"[!] Error controlado en rutina de llavero: {e}")
            ir_a_home_seguro(robot, speed=25)

    # =========================================================================
    # MÓDULO 2: DESAFÍO EN VIVO - EVASIÓN DINÁMICA CON OCTOMAP (CÁMARA RGB-D)
    # =========================================================================
    def demo_evasion_octomap():
        if not getattr(robot, "moveit_manager", None):
            print("[!] Esta demo requiere estar conectado a MoveIt (modo 'ros').")
            return

        if hasattr(robot, "toggle_ros_connection"):
            robot.toggle_ros_connection(True)

        # Puntos A (Izquierda) y B (Derecha) sobre la mesa para cruzar esquivando el obstáculo central
        robt_punto_A = RobTarget(SE3(135.95, -292.23, 200.18) * SE3.RPY([179.32, -2.93, 59.39], unit='deg'), [1, -1, 1])
        robt_punto_B = RobTarget(SE3(325.31, 27.88, 219.26) * SE3.RPY([179.32, -2.95, 59.39], unit='deg'), [1, -1, 1])


        q_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        robt_home = RobTarget.from_q(q_home, pinza)

        print("\n" + "=" * 68)
        print(" DESAFÍO INTERACTIVO: EVASIÓN DE OBSTÁCULOS EN TIEMPO REAL")
        print("=" * 68)
        print(" 1. El robot se posicionará en el Lado Izquierdo (Punto A).")
        print(" 2. Pídele al público que coloque una caja en el centro de la mesa.")
        print(" 3. MoveIt actualizará los vóxeles y calculará una ruta de evasión hacia B.")

        robot.GripperState(0, 30)  # Pinza cerrada para coincidir con el filtro de vóxeles
        limpiar_octomap()
        time.sleep(1)

        print("\n>>> Moviendo a Punto de Partida A (Izquierda)...")
        q_actual = robot.moveit_manager.plan_and_execute(
            target=robt_punto_A, start=q_home, tool=pinza, execute=True, update_rviz=True
        )
        if not q_actual:
            print("[!] No se pudo llegar al Punto A. Revisa que la mesa esté despejada.")
            return

        flag=False

        while flag ==False:
            input("\n[PAUSA] Coloca el obstáculo en el centro de la mesa y presiona ENTER para esquivarlo...")

            print(">>> Escaneando escena con cámara Xtion y limpiando rastro previo...")
            limpiar_octomap()
            time.sleep(1)  # Espera medio segundo para integrar los vóxeles frescos a 10 Hz

            print(">>> Calculando ruta libre de colisiones de A -> B...")
            q_en_B = robot.moveit_manager.plan_and_execute(
                target=robt_punto_B, start=q_actual[-1], tool=pinza, execute=True, update_rviz=True
            )

            if q_en_B:
                print("[OK] ¡Obstáculo esquivado con éxito!")
                input("\n[PAUSA] Retira o cambia de lugar el obstáculo y presiona ENTER para regresar a posicion A...")
                limpiar_octomap()
                time.sleep(0.5)
                q_en_A =robot.moveit_manager.plan_and_execute(
                    target=robt_punto_A, start=q_en_B[-1], tool=pinza, execute=True, update_rviz=True
                )

            entrada = input("\nPresiona T para terminar\n").strip().lower()

            if entrada == "t":
                print("Rutina Terminada")
                robot.moveit_manager.plan_and_execute(
                    target=q_home, start=q_en_A[-1], tool=pinza, execute=True, update_rviz=True
                )
                flag = True
                


    # =========================================================================
    # MÓDULO 4: DEMO MÚLTIPLES CONFIGURACIONES PARA UNA MISMA POSE (PCI)
    # =========================================================================
    def demo_cambio_configuraciones():
        print("\n" + "=" * 68)
        print(" DEMO DIDÁCTICA: MÚLTIPLES CONFIGURACIONES PARA UNA MISMA POSE")
        print("=" * 68)

        if hasattr(robot, "toggle_ros_connection"):
            robot.toggle_ros_connection(False)

        # Usamos la pose de Entrega como punto testigo espacial
        pose_testigo = SE3(314, -163, 382) * SE3.RPY([-53, -8, -121], unit="deg")
        robt_semilla = RobTarget(pose_testigo, [-1, 1, -1])

        if hasattr(robot, "MostrarTerna"):
            robot.MostrarTerna(pose_testigo, "Punto_Testigo_Espacial")

        try:
            configs_validas = robt_semilla.find_valid_configs(pinza, wobj_nulo)
            configs_validas = [list(c) for c in configs_validas]
        except Exception:
            configs_validas = [[-1, 1, -1], [-1, 1, 1]]

        print(f"[*] Se encontraron {len(configs_validas)} soluciones cinemáticas para el mismo punto:")
        for idx, c in enumerate(configs_validas[:3], start=1):
            print(f"\n>>> [Solución #{idx}] Llegando al punto testigo con Configuración {c}...")
            ir_a_home_seguro(robot, speed=35)
            time.sleep(0.6)
            try:
                robot.MoveJ(RobTarget(pose_testigo, c), speed=30, tool=pinza, wobj=wobj_nulo)
                time.sleep(2.5)
            except Exception as e:
                print(f"    [!] Configuración {c} omitida ({e})")

        print("\n>>> Regresando a Home...")
        ir_a_home_seguro(robot, speed=30)

    # =========================================================================
    # BUCLE PRINCIPAL DEL MENÚ DE LA NOCHE DE LOS MUSEOS
    # =========================================================================
    robot.GripperState(0, 30)  # Inicia con pinza cerrada (0% = -0.7 rad)
    ir_a_home_seguro(robot, speed=30)

    while True:
        n1 = len(catalogo.get(1) or catalogo.get("1") or []) if catalogo else 0
        n2 = len(catalogo.get(2) or catalogo.get("2") or []) if catalogo else 0
        n3 = len(catalogo.get(3) or catalogo.get("3") or []) if catalogo else 0

        print("\n" + "=" * 68)
        print("   PANEL DE CONTROL MAESTRO - LA NOCHE DE LOS MUSEOS (MyCobot 320)")
        print("=" * 68)
        print(" --- SHOW DE LLAVEROS (MULTICONFIGURACIÓN + MOVEL) ---")
        print(f"  [1] Saludar + Entregar Llavero 1 ({n1} variantes)")
        print(f"  [2] Saludar + Entregar Llavero 2 ({n2} variantes)")
        print(f"  [3] Saludar + Entregar Llavero 3 ({n3} variantes)")
        print("  [a] Modo Azar: Sortear Llavero y Configuración")
        print("  [s] Solo Saludar al público")
        print("\n --- SHOW DE VISIÓN 3D Y CONCEPTOS DE INGENIERÍA ---")
        print("  [v] Desafío en Vivo: Esquivar Obstáculo Real con Cámara (OctoMap)")
        print("  [c] Demo Configuraciones: Llegar al mismo punto con distintas posturas")
        print("  [o] Limpiar vóxeles del OctoMap (/clear_octomap)")
        print("  [h] Ir a HOME seguro (con pinza cerrada)")
        print("  [q] Salir")

        op = input("\nSeleccione una demostración: ").strip().lower()

        if op == "q":
            print("Cerrando panel maestro...")
            break
        elif op == "h":
            robot.GripperState(0, 30)
            ir_a_home_seguro(robot, speed=30)
        elif op == "o":
            limpiar_octomap()
            print("[OK] OctoMap limpiado.")
        elif op == "s":
            saludar()
        elif op == "a":
            disp = [k for k in (1, 2, 3) if (catalogo and (catalogo.get(k) or catalogo.get(str(k))))]
            ejecutar_llave(random.choice(disp) if disp else 1, con_saludo=True)
        elif op in ("1", "2", "3"):
            ejecutar_llave(int(op), con_saludo=True)
        elif op == "v":
            demo_evasion_octomap()
        elif op == "c":
            demo_cambio_configuraciones()
        else:
            print("[!] Opción no reconocida.")