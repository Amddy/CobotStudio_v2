#!/usr/bin/env python3
"""
Script Maestro para La Noche de los Museos (demo_museos_master.py)
Integra:
  1. Desafío en Vivo: Evasión Dinámica de Obstáculos con Cámara RGB-D y OctoMap
  2. Demo Didáctica: Movimiento en el Núcleo de la Singularidad de Muñeca (q5 = 0°)
  3. Demo Didáctica: Múltiples Configuraciones Cinemáticas para una Misma Pose
"""

from Cobot_sdk import *
import os
import random
import subprocess
import time
import importlib.util
from pathlib import Path
import numpy as np

def obtener_q_actual(robot: BaseRobotController) -> np.ndarray:
    """Obtiene el vector articular actual (en rad) en ROS."""
    #  Solo consultar robot.mc si estamos en el controlador físico real
    if robot.__class__.__name__ == "MyCobotController" and hasattr(robot, "mc"):
        try:
            q_deg = robot.mc.get_angles()
            if q_deg is not None and len(q_droboteg) == 6:
                return np.deg2rad(np.array(q_deg, dtype=float))
        except Exception:
            pass

    #  En modo ROS, leer directamente del suscriptor de /joint_states
    if hasattr(robot, "get_current_q"):
        try:
            q = robot.get_current_q(prefer_gripper=False)
            if q is not None and len(q) >= 6:
                return np.array(q[:6], dtype=float)
        except Exception:
            pass

    if hasattr(robot, "q_current") and robot.q_current is not None:
        return np.array(robot.q_current[:6], dtype=float)

    return np.zeros(6, dtype=float)

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


    # =========================================================================
    # MÓDULO 1: DESAFÍO EN VIVO - EVASIÓN DINÁMICA CON OCTOMAP (CÁMARA RGB-D)
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
    # MÓDULO 2: DEMO MÚLTIPLES CONFIGURACIONES PARA UNA MISMA POSE (PCI)
    # =========================================================================
    def demo_cambio_configuraciones():
        print("\n" + "=" * 68)
        print(" DEMO DIDÁCTICA: MÚLTIPLES CONFIGURACIONES PARA UNA MISMA POSE")
        print("=" * 68)

        # Definir herramienta (TCP) y Workobject
        tool = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
        wobj = SE3()

        # Usamos la pose de Entrega como punto testigo espacial
        pose_objetivo = SE3(267.95, -119.19, 245.12) * SE3.RPY([-178.93, 3.58, 58.22], unit='deg')
        robt_base = RobTarget(pose_objetivo)

        # Mostrar la terna objetivo en RViz si estamos en ROS
        if hasattr(robot, "show_tf"):
            robot.show_tf(wobj * pose_objetivo, "Pose_Objetivo")

        # Buscar todas las configuraciones analíticas válidas con el modelo del robot
        configs_validas = robt_base.find_valid_configs(tool, wobj)
        if not configs_validas:
            print("[!] La pose elegida está fuera del alcance para cualquier configuración.")
            return

        # Calcular para cada configuración sus ángulos exactos, manipulabilidad y colisión
        opciones = []
        pose_brida = wobj * pose_objetivo * tool.inv()

        for conf in configs_validas:
            robt_cand = RobTarget(pose_objetivo, list(conf))
            q_sol = robot.cobot_tb.ikine(pose_brida, list(conf))[0]
            manip = robot._manipulability(q_sol, wobj, tool, q_unit='rad', debug=False)

            libre_colision = True
            if hasattr(robot, "moveit_manager") and robot.moveit_manager is not None:
                try:
                    libre_colision = bool(robot.moveit_manager._check_collision(robt_cand, tool, wobj))
                except Exception:
                    libre_colision = True

            opciones.append({
                'conf': list(conf),
                'robt': robt_cand,
                'q': np.array(q_sol).flatten(),
                'manip': float(manip),
                'ok': libre_colision
            })

        # Home definido ligeramente fuera del 0 exacto para que el IK de RobTarget no falle al planificar con MoveIt
        q_home_ik =  np.deg2rad([0.0, -1.0, 2.0, -1.0, -1.0, 0.0]).tolist()
        robt_home = RobTarget.from_q(q_home_ik, pinza)

        # Estado articular actual para encadenar los cambios de configuración desde donde está el robot
        q_now = np.array(obtener_q_actual(robot), dtype=float).flatten()
        ultimo_idx = 0

        while True:
            conf_actual = robot.cobot_tb.calc_conf(q_now)

            print("\n" + "=" * 72)
            print(f" POSE OBJETIVO: X={pose_objetivo.t[0]:.1f}, Y={pose_objetivo.t[1]:.1f}, Z={pose_objetivo.t[2]:.1f} mm")
            print(f" Estado ACTUAL: Conf={conf_actual} | q(°)= {np.round(np.rad2deg(q_now), 1)}")
            print("=" * 72)

            for i, opt in enumerate(opciones, start=1):
                estado = "OK" if opt['ok'] else "COLISIÓN"
                q_deg = np.round(np.rad2deg(opt['q']), 1)
                marca = " <-- [Seleccionada]" if (i - 1) == ultimo_idx else ""
                print(f"  [{i}] Conf: {str(opt['conf']):<13} | {estado:<8} | W={opt['manip']:.2f} | q(°)= {q_deg}{marca}")

            print("-" * 72)
            print(" Comandos disponibles:")
            print("  - Número (1..N) : Previsualizar configuración en RViz (Goal naranja)")
            print("  - 'p' + Número  : Ir con el planificador MoveIt desde la configuración ACTUAL (ej: p1, p2)")
            print("  - 'h'           : Ir a posición HOME con el planificador MoveIt")
            print("  - 'q'           : Salir")

            entrada = input("\nSeleccione acción: ").strip().lower()

            if entrada == 'q':
                break

            elif entrada == 'h':
                if hasattr(robot, "moveit_manager") and robot.moveit_manager is not None:
                    print(">>> Planificando y ejecutando ida a HOME con MoveIt desde la posición actual...")
                    q_plan = robot.moveit_manager.plan_and_execute(
                        target=robt_home,
                        start=q_now.tolist(),
                        tool=tool,
                        wobj=wobj,
                        execute=True,
                        update_rviz=True,
                        planner_id = "BiTRRTkConfigDefault"
                    )
                    if q_plan is not None and len(q_plan) > 0:
                        q_now = np.array(q_plan[-1][:6], dtype=float).flatten()
                else:
                    print("[!] MoveIt no está activo.")
                continue

            # Detectar únicamente los comandos 'v' (1..N) o 'p' (p1..pN)
            if entrada.isdigit():
                modo = 'v'
                idx = int(entrada) - 1
            elif entrada.startswith('p') and entrada[1:].isdigit():
                modo = 'p'
                idx = int(entrada[1:]) - 1
            else:
                print("[!] Entrada no válida. Use 1..N, p1..pN, 'h' o 'q'.")
                continue

            if not (0 <= idx < len(opciones)):
                print(f"[!] Elija un número entre 1 y {len(opciones)}.")
                continue

            ultimo_idx = idx
            elegida = opciones[idx]
            robt_elegido = elegida['robt']

            if modo == 'v':
                print(f">>> Previsualizando configuración #{idx+1}: {elegida['conf']}...")
                if hasattr(robot, "moveit_manager") and robot.moveit_manager is not None:
                    robot.moveit_manager.move_goal(robt_elegido, tool=tool, wobj=wobj, publish_tf=True)
                elif hasattr(robot, "view_pose"):
                    robot.view_pose(robt_elegido, tool=tool, wobj=wobj)

            elif modo == 'p':
                if not elegida['ok']:
                    print("[!] Esa configuración está en colisión. MoveIt rechazará el plan.")
                    continue

                if hasattr(robot, "moveit_manager") and robot.moveit_manager is not None:
                    print(f">>> Planificando en MoveIt desde {conf_actual} hacia Conf #{idx+1}: {elegida['conf']}...")
                    q_plan = robot.moveit_manager.plan_and_execute(
                        target=robt_elegido,
                        start=q_now.tolist(),  # Parte de la configuración actual en la que está
                        tool=tool,
                        wobj=wobj,
                        execute=True,
                        update_rviz=True,
                        planner_id = "BiTRRTkConfigDefault"
                    )
                    if q_plan is not None and len(q_plan) > 0:
                        q_now = np.array(q_plan[-1][:6], dtype=float).flatten()
    
    
    # =========================================================================
    # BUCLE PRINCIPAL DEL MENÚ DE LA NOCHE DE LOS MUSEOS
    # =========================================================================
    robot.GripperState(0, 30)  # Inicia con pinza cerrada (0% = -0.7 rad)
    ir_a_home_seguro(robot, speed=30)

    while True:

        print("\n" + "=" * 68)
        print("   PANEL DE CONTROL MAESTRO - LA NOCHE DE LOS MUSEOS (MyCobot 320)")
        print("=" * 68)
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
        elif op == "v":
            demo_evasion_octomap()
        elif op == "c":
            demo_cambio_configuraciones()
        else:
            print("[!] Opción no reconocida.")