import time
import numpy as np
from spatialmath import SE3, UnitQuaternion
from Cobot_sdk import *

def run(robot: BaseRobotController, **kwargs):
    # =========================================================================
    # 1. DEFINICIÓN EXACTA DE POSES DE SINGULARIDAD (de singCart_rev0 y SingMuñeca)
    # =========================================================================
    TA_hombro = SE3([
        [-1,  0,  0,  103.6],
        [ 0, -1,  0,  -89.1],
        [ 0,  0,  1,  331.4],
        [ 0,  0,  0,    1.0]
    ])
    TB_hombro = SE3([
        [-1,  0,  0, -103.6],
        [ 0, -1,  0,  -89.1],
        [ 0,  0,  1,  331.4],
        [ 0,  0,  0,    1.0]
    ])

    TA_codo = SE3([
        [ 0, -1,  0,  300.0],
        [-1,  0,  0,  -90.2],
        [ 0,  0, -1,   86.0],
        [ 0,  0,  0,    1.0]
    ])
    TB_codo = SE3([
        [ 0, -1,  0,  348.0],
        [-1,  0,  0,  -90.2],
        [ 0,  0, -1,   86.0],
        [ 0,  0,  0,    1.0]
    ])

    # Carga directa desde SingMuñeca (con fallback por si se corre desde otra carpeta)
    try:
        TA_muneca = robot.load_data('Poses', 'SingMuñeca', 'PuntoA')
        TB_muneca = robot.load_data('Poses', 'SingMuñeca', 'PuntoB')
    except Exception:
        q_mun = [2.504995e-01, 6.872045e-01, 2.329163e-01, 6.408978e-01]
        TA_muneca = SE3([114.12, 127.59, 296.19]) * UnitQuaternion(q_mun).SE3()
        TB_muneca = SE3([194.12, 127.59, 296.19]) * UnitQuaternion(q_mun).SE3()

    # =========================================================================
    # 2. CONFIGURACIONES ARTICULARES PARA MISMA POSE (de checkConf_rev0.py)
    # =========================================================================
    dar = RobTarget(SE3(340, -135.721, 200) * SE3.Ry(np.pi/2) * SE3.Rz(np.pi), [-1, 1, 1])
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)

    configs_q_rad = {
        "Configuración 1 (q1)": [-1.27559587, -1.32895753,  0.44925002,  0.35574428,  0.08017588, -0.12023292],
        "Configuración 2 (q2)": [ 1.27950439, -0.39132917, -0.31007228, -1.04270240,  2.72883256,  0.02386266],
        "Configuración 3 (q3)": [-1.00486581,  1.16202445,  0.25907755,  1.20117569, -1.14663500, -0.14946468],
    }

    # =========================================================================
    # FUNCIONES AUXILIARES DE EJECUCIÓN
    # =========================================================================
    def traj_sing(spd, pose1, pose2, config):
        robt1 = RobTarget(pose1, config=config)
        robt2 = RobTarget(pose2, config=config)
        # Subir primero a Home evita barridos peligrosos al cambiar de rama cinemática
        robot.GoHome()
        time.sleep(1.0)
        robot.MoveJ(robt1, spd)
        time.sleep(2.0)
        robot.MoveL(robt2, spd)
        time.sleep(1.5)
        robot.logger.info("Trayectoria cartesiana por singularidad completada.")

    def baile_nucleo_muneca(spd=25):
        """Muestra cómo rotan los ejes 4 y 6 en sentidos opuestos con q5=0° sin mover el TCP."""
        robot.GoHome()
        # Llevamos al robot a PuntoA de SingMuñeca y forzamos q5 = 0° exacto
        robt_a = RobTarget(TA_muneca, config=[1, -1, 1])
        robot.MoveJ(robt_a, spd)
        time.sleep(1.5)
        
        q_base_deg = np.array(robot.get_current_q(), dtype=float)
        q_base_deg[4] = 0.0  # Forzamos q5 = 0° (singularidad pura de muñeca)
        robot.MoveJ_q(q_base_deg, spd, unit="deg")
        time.sleep(1.0)

        # Movimiento en el núcleo Null(J) = [0, 0, 0, -1, 0, 1]^T
        for delta in [40.0, -40.0, 0.0]:
            q_step = q_base_deg.copy()
            q_step[3] += delta
            q_step[5] -= delta
            robot.MoveJ_q(q_step, spd, unit="deg")
            time.sleep(0.8)

    def mostrar_misma_pose_distintas_configs(spd=25):
        """Recorre q1, q2 y q3 de checkConf_rev0 mostrando la misma pose con 3 posturas."""
        for nombre_cfg, q_rad in configs_q_rad.items():
            print(f"\n>>> Mostrando {nombre_cfg}")
            if hasattr(robot, "view_pose"):
                # Si estamos en ROSManager, previsualiza o mueve con seguridad
                robot.GoHome()
                q_deg = np.rad2deg(q_rad)
                robot.MoveJ_q(q_deg, spd, unit="deg")
            else:
                robot.GoHome()
                robot.MoveJ_q(np.rad2deg(q_rad), spd, unit="deg")
            time.sleep(2.5)
        robot.GoHome()

    # =========================================================================
    # MENÚ INTERACTIVO PARA DEMOSTRACIÓN EN VIVO
    # =========================================================================
    menu_options = {
        1: {"name": "Singularidad de Hombro (MoveL tangente a d4=88.78mm)", "configs": [[1, 1, 1], [-1, -1, -1], [-1, 1, 1], [1, -1, -1]], "poses": (TA_hombro, TB_hombro)},
        2: {"name": "Singularidad de Codo (MoveL a extensión máxima)",      "configs": [[1, 1, 1], [1, -1, 1], [-1, 1, -1]],             "poses": (TA_codo, TB_codo)},
        3: {"name": "Singularidad de Muñeca (MoveL PuntoA -> PuntoB)",      "configs": [[1, -1, 1]],                                     "poses": (TA_muneca, TB_muneca)},
        4: {"name": "Baile en el Núcleo de Muñeca (dq4 = -dq6 con TCP fijo)"},
        5: {"name": "Misma Pose con 3 Configuraciones Articulares (q1, q2, q3 de checkConf)"}
    }

    while True:
        print("\n=== DEMO SINGULARIDADES Y CONFIGURACIONES (NDLM) ===")
        for key, data in menu_options.items():
            print(f"  {key}: {data['name']}")
        
        try:
            sel = int(input("Seleccione demostración (0 para salir): "))
        except ValueError:
            continue

        if sel == 0:
            break
        elif sel in (1, 2, 3):
            data = menu_options[sel]
            configs = data["configs"]
            for i, c in enumerate(configs, 1):
                print(f"  {i}: {c}")
            try:
                sel_c = int(input("Seleccione configuración (0 para volver): "))
            except ValueError:
                continue
            if 1 <= sel_c <= len(configs):
                traj_sing(30, data["poses"][0], data["poses"][1], configs[sel_c - 1])
        elif sel == 4:
            baile_nucleo_muneca(spd=25)
        elif sel == 5:
            mostrar_misma_pose_distintas_configs(spd=25)

    robot.GoHome()