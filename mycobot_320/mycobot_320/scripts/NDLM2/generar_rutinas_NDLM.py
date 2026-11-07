from Cobot_sdk import *
import os
import random
import time
import numpy as np
from pathlib import Path

ARCHIVO_SALIDA = "Rutina_NDLM"
NUM_VARIANTES_POR_LLAVE = 4


DESCENSO_MOVEL_MM = -35.0

dx, dy, dz = 0.0, 0.0, 103.0
offset_xy = SE3(dx, dy, dz)


def es_estado_valido(robot, target_o_q, tool=SE3(), wobj=SE3()) -> bool:
    """
    Llama al método real _check_collision de tu MoveItManager.py (línea 518)
    usando el grupo 'arm_with_gripper' para que evalúe brazo + pinza contra la mesa.
    Devuelve True solo si NO hay colisión.
    """
    return bool(
        robot.moveit_manager._check_collision(
            positions=target_o_q,
            tool=tool,
            wobj=wobj,
            group_name="arm_with_gripper"
        )
    )


def obtener_configs_sin_colision(
    robot,
    robt_base: RobTarget,
    tool: SE3,
    wobj: SE3 = SE3(),
    verificar_descenso_z: float = 0.0
) -> list:
    """
    1. Halla las configuraciones alcanzables de robt_base.
    2. Verifica que NO choquen en MoveIt en la posición superior.
    3. Si verificar_descenso_z != 0 (ej. -35 mm), verifica además que al bajar con MoveL
       en esa misma configuración siga siendo alcanzable por IK y NO choque en MoveIt.
    """
    confs = robt_base.find_valid_configs(tool, wobj)
    confs = [list(c) for c in confs]

    confs_limpias = []
    for conf in confs:
        target_arriba = RobTarget(robt_base.pose, conf)

        # A) ¿Es válida y sin colisión arriba?
        if not es_estado_valido(robot, target_arriba, tool, wobj):
            print(f"     [-] Config {conf} descartada (colisión en posición superior).")
            continue

        # B) Si lleva MoveL después (ej. -35 mm), ¿es válida y sin colisión durante y al final del descenso?
        if verificar_descenso_z != 0.0:
            target_mitad = target_arriba.offset(0, 0, verificar_descenso_z * 0.5)
            target_abajo = target_arriba.offset(0, 0, verificar_descenso_z)

            # Verificar que el IK analítico exista en esa misma config y no choque en MoveIt
            if not (es_estado_valido(robot, target_mitad, tool, wobj) and
                    es_estado_valido(robot, target_abajo, tool, wobj)):
                print(f"     [-] Config {conf} descartada (chocaría o saldría de alcance al bajar {verificar_descenso_z} mm).")
                continue

        confs_limpias.append(conf)

    random.shuffle(confs_limpias)
    return confs_limpias


def planificar_tramo(
    robot,
    robt_base: RobTarget,
    start_q,
    tool: SE3,
    nombre_var: str,
    evitar: list = None,
    verificar_descenso_z: float = 0.0
):
    candidatas = obtener_configs_sin_colision(
        robot, robt_base, tool, verificar_descenso_z=verificar_descenso_z
    )
    if evitar:
        candidatas = [c for c in candidatas if c not in evitar] + [c for c in candidatas if c in evitar]

    if not candidatas:
        raise RuntimeError(f"Ninguna configuración de '{nombre_var}' superó el chequeo de colisión/descenso.")

    for conf in candidatas:
        robt_target = RobTarget(robt_base.pose, conf)
        print(f"  -> Planificando '{nombre_var}' con config {conf}...")

        q_traj = robot.moveit_manager.plan_and_execute(
            target=robt_target,
            start=start_q,
            tool=tool,
            execute=True,
            update_rviz=False
        )

        if not q_traj:
            print(f"     [!] MoveIt rechazó el camino para {conf}. Probando siguiente...")
            continue

        # Verificar que todos los waypoints del camino estén libres de colisión
        if any(not es_estado_valido(robot, list(q_wp[:6])) for q_wp in q_traj):
            print(f"     [!] Colisión detectada en el trayecto de {conf}. Descartando...")
            continue

        robot.moveit_manager.save_trajectory(ARCHIVO_SALIDA, nombre_var)
        print(f"     [OK] '{nombre_var}' guardada limpia (incluyendo margen para bajar {verificar_descenso_z} mm).")
        return list(q_traj[-1][:6]), conf

    raise RuntimeError(f"No se encontró trayectoria sin colisión para '{nombre_var}'.")

def run(robot: ROSManager, **kwargs):
    if not getattr(robot, "moveit_manager", None):
        print("[!] Ejecuta este script en modo 'ros' con demo.launch.py abierto.")
        return

    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)

    # Home ligeramente fuera del 0 exacto para que el IK analítico de RobTarget no dé discriminante negativo
    q_home = np.deg2rad([0.0, -1.0, 2.0, -1.0, -1.0, 0.0]).tolist()
    Home = RobTarget.from_q(q_home, pinza)

    # saludo1 = RobTarget(SE3(205.52, -156.04, 387.39) * SE3.RPY([151.19, -0.01, 58.25], unit='deg'), [1, -1, 1])
    # saludo2 = RobTarget(SE3(141.52, -116.41, 356.81) * SE3.RPY([-165.70, -0.01, 58.26], unit='deg'), [1, -1, 1])

    llaves = {
        1: RobTarget(SE3(338.51, -35.45, 200.05) * SE3.RPY([-179.77, -1.45, 64.10], unit='deg'), [-1, 1, -1]),
        2: RobTarget(SE3(289.78, -135.80, 197.21) * SE3.RPY([-179.76, -1.44, 64.11], unit='deg'), [-1, 1, -1]),
        3: RobTarget(SE3(227.27, -219.77, 199.88) * SE3.RPY([-179.21, -0.00, 64.19], unit='deg'), [1, -1, 1])
    }
    Robt_entrega = RobTarget(SE3(307.81, -150.79, 392.58) * SE3.RPY([-61.64, -0.50, -119.64], unit='deg'), [-1, -1, -1])

    carpeta_tray = Path(robot.project_root) / "Trayectorias"
    carpeta_tray.mkdir(parents=True, exist_ok=True)
    ruta_archivo = carpeta_tray / f"{ARCHIVO_SALIDA}.py"
    ruta_archivo.write_text("# Formato: [ ([q1..q6 en deg], speed_1_100, t_sec), ... ]\n\n", encoding="utf-8")

    # # 1. Generar Saludo
    # print("\n=== GENERANDO SALUDO ===")
    # q_s1, _ = planificar_tramo(robot, saludo1, q_home, pinza, "SALUDO_1")
    # q_s2, _ = planificar_tramo(robot, saludo2, q_s1, pinza, "SALUDO_2")
    # q_s3, _ = planificar_tramo(robot, saludo1, q_s2, pinza, "SALUDO_3")
    # planificar_tramo(robot, Home, q_s3, pinza, "SALUDO_HOME")

    # 2. Generar Variantes de Llave 1 y Llave 2
    catalogo_rutas = {1: [], 2: [], 3: []}

    for num_llave, robt_llave in llaves.items():
        usadas_llave = []
        usadas_entrega = []

        for v in range(1, NUM_VARIANTES_POR_LLAVE + 1):
            print(f"\n=== LLAVE {num_llave} - VARIANTE {v}/{NUM_VARIANTES_POR_LLAVE} ===")
            var_ida = f"LLAVE_{num_llave}_V{v}_IDA"
            var_ent = f"LLAVE_{num_llave}_V{v}_ENTREGA"
            var_hom = f"LLAVE_{num_llave}_V{v}_HOME"

            try:
                q_llave, conf_l = planificar_tramo(
                    robot, robt_llave, q_home, pinza, var_ida, evitar=usadas_llave, verificar_descenso_z=DESCENSO_MOVEL_MM
                )
                usadas_llave.append(conf_l)

                q_ent, conf_e = planificar_tramo(
                    robot, Robt_entrega, q_llave, pinza, var_ent, evitar=usadas_entrega
                )
                usadas_entrega.append(conf_e)

                planificar_tramo(
                    robot, Home, q_ent, pinza, var_hom
                )

                catalogo_rutas[num_llave].append({
                    "variante": v,
                    "ida": var_ida,
                    "entrega": var_ent,
                    "home": var_hom,
                    "conf_llave": conf_l,
                    "conf_entrega": conf_e
                })
            except Exception as e:
                print(f"  [!] Variante {v} descartada: {e}")

    with open(ruta_archivo, "a", encoding="utf-8") as f:
        f.write(f"\nCATALOGO_RUTAS = {repr(catalogo_rutas)}\n")

    print(f"\n[OK] Rutinas guardadas en: {ruta_archivo}")
    print(f"     Variantes limpias -> Llave 1: {len(catalogo_rutas[1])} | Llave 2: {len(catalogo_rutas[2])} | Llave 3: {len(catalogo_rutas[3])}")