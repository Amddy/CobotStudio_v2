from Cobot_sdk import *
import time
import datetime
import numpy as np
import pandas as pd
from importlib import import_module
from spatialmath import SE3


def leer_q_seguro_deg(robot: BaseRobotController, intentos: int = 3) -> np.ndarray:
    """
    Lee los ángulos actuales en grados sin disparar warnings de GhostDriver en ROS
    y manejando posibles lecturas nulas puntuales del puerto serie en el robot real.
    """
    if hasattr(robot, 'mc') and robot.__class__.__name__ == 'MyCobotController':
        for _ in range(intentos):
            angles = robot.mc.get_angles()
            if angles is not None and isinstance(angles, (list, tuple)) and len(angles) == 6:
                return np.array(angles, dtype=float)
            time.sleep(0.05)
        raise RuntimeError("No se pudieron leer los ángulos del cobot físico vía serie.")

    # En modo ROS, leer del estado articular actual
    if hasattr(robot, 'get_current_q'):
        try:
            q_rad = robot.get_current_q(prefer_gripper=False)
            if q_rad is not None and len(q_rad) >= 6:
                return np.rad2deg(np.array(q_rad[:6], dtype=float))
        except TypeError:
            q_rad = robot.get_current_q()
            if q_rad is not None and len(q_rad) >= 6:
                return np.rad2deg(np.array(q_rad[:6], dtype=float))

    if hasattr(robot, 'q_current') and robot.q_current is not None:
        return np.rad2deg(np.array(robot.q_current[:6], dtype=float))

    return np.zeros(6, dtype=float)


def sendWobj(robot: BaseRobotController, wobj, pinza: SE3, conf=None):
    """Enviar el cobot directamente al wobj calculado (sin pre-aproximación ni compensaciones)."""
    if conf is None:
        conf = [1, 1, 1]
    robt0 = RobTarget(SE3(wobj), conf)
    print(f"RobTarget del Wobj:\n{robt0.pose}")
    robot.MoveJ(robt0, 30, pinza, SE3())


def send_wobj_to_cobot(robot: BaseRobotController, wobj, pinza_aux: SE3, conf=None):
    """Chequear el wobj directamente en el robot físico."""
    if conf is None:
        conf = [1, -1, -1]  # [-1, 1, 1] para enseñanza en vertical
    sendWobj(robot, wobj, pinza_aux, conf)


def cheqWobj(robot: BaseRobotController, wobj, pinza: SE3, conf=None):
    """Envía el wobj a RViz verificando sus configuraciones válidas."""
    if conf is None:
        conf = [1, 1, 1]
    robt0 = RobTarget(SE3(wobj), conf)
    print("Configuraciones válidas:", robt0.find_valid_configs(pinza, SE3()))
    robot.MoveJ(robt0, 30, pinza, SE3(), robt_name='wobj')
    time.sleep(2)


def revisarPoses(robot: BaseRobotController, poses_list: list, tool: SE3 = SE3(), func: str = 'MoveJ'):
    """
    Envía al robot físico a las poses almacenadas en una lista.
    Mediante un menú se permite ir al home entre poses o moverse directamente entre ellas.

    Args:
        poses_list (list): Lista de poses a recorrer [angles, coords].
        tool (SE3): Herramienta (por defecto nula en la brida).
        func (str): 'MoveJ', 'sendAngles' o 'sendCoords'.
    """
    es_real = hasattr(robot, 'mc') and robot.__class__.__name__ == 'MyCobotController'

    def pose_alcanzada():
        time.sleep(5)
        q_alc = leer_q_seguro_deg(robot)
        print(f'Ángulos alcanzados:\n{np.round(q_alc, 2)}')
        if es_real:
            print(f'Coordenadas alcanzadas:\n{robot.mc.get_coords()}')
            print(f'Ángulos y coords (check):\n{robot.mc.get_angles_coords()}')

    def ejecutar_movimiento(metodo, robt, angles_deg, coords):
        if metodo == 'MoveJ':
            robot.MoveJ(robt, 30, tool, SE3())
        elif metodo == 'sendAngles':
            robot.mc.sync_send_angles(angles_deg, 30, 0)
        elif metodo == 'sendCoords':
            robot.mc.sync_send_coords(coords, 30, 0)
        else:
            print(f"El método seleccionado '{metodo}' no es admisible.")
            return False
        pose_alcanzada()
        return True

    for idx, pose in enumerate(poses_list, 1):
        print(f'\n------------------------- Pose {idx} -------------------------')
        angles_deg = pose[:6]
        coords = pose[6:]
        print(f'Coords leídas:\n{coords}')
        print(f'Ángulos leídos:\n{angles_deg}')

        q_rad = np.deg2rad(angles_deg)
        config = robot.cobot_tb.calc_conf(q_rad).tolist()
        pose_SE3 = robot.pose_to_matrix(coords)
        robt = RobTarget(pose_SE3, config)
        print(f'Robtarget generado (conversión de coords del cobot):\n{robt.pose}')

        POSE_calc = robot.cobot_tb.fkine(q_rad)
        print(f'POSE_calc - PCD_toolbox(q_cobot):\n{POSE_calc}')

        q_ikine = robot.cobot_tb.ikine(pose_SE3, config)[0]
        print(f'Según la tb se alcanza con los ángulos:\n{np.round(np.rad2deg(q_ikine), 2)}')

        while True:
            print("\n¿Qué desea hacer a continuación?")
            print(f"  1. Ir a Home, esperar 3s, y luego ir a la Pose {idx}.")
            print(f"  2. Ir directamente a la Pose {idx} desde la posición actual.")
            print(f"  3. Ir a Home y cerrar.")

            choice = input("Ingrese su opción (1, 2 o 3): ").strip()

            if choice == '1':
                print("\n[Opción 1] Moviendo el robot a la posición Home...")
                robot.MoveJ_q(np.zeros(6), 30, 'deg')
                print("Llegó a Home. Esperando 3 segundos...")
                time.sleep(3)

                print(f"Moviendo a la Pose {idx} con el método {func}...")
                if not ejecutar_movimiento(func, robt, angles_deg, coords):
                    return

                while True:
                    print("\n¿Con qué método analizamos el punto?")
                    print("  1. Armamos el robt y vamos con MoveJ.")
                    print("  2. Vamos con los ángulos que leyó el cobot al grabarlo (sendAngles).")
                    print("  3. Vamos con las coordenadas que leyó el cobot al grabarlo (sendCoords).")
                    print("  q. Salir del submenú y pasar a la siguiente pose.")
                    choice_method = input("Ingrese su opción (1, 2, 3 o q): ").strip().lower()

                    if choice_method == 'q':
                        break
                    elif choice_method == '1':
                        print("\n[Opción 1] MoveJ...")
                        ejecutar_movimiento('MoveJ', robt, angles_deg, coords)
                    elif choice_method == '2':
                        print("\n[Opción 2] sendAngles...")
                        ejecutar_movimiento('sendAngles', robt, angles_deg, coords)
                    elif choice_method == '3':
                        print("\n[Opción 3] sendCoords...")
                        ejecutar_movimiento('sendCoords', robt, angles_deg, coords)
                    else:
                        print("Opción no válida. Ingrese 1, 2, 3 o q.")
                break

            elif choice == '2':
                print(f"\n[Opción 2] Moviendo directamente a la Pose {idx} con el método {func}...")
                if not ejecutar_movimiento(func, robt, angles_deg, coords):
                    return
                break

            elif choice == '3':
                print("\n[Opción 3] Enviando a Home y finalizando...")
                robot.MoveJ_q(np.zeros(6), 30, 'deg')
                print("Llegó a Home. Terminando prueba...")
                return

            else:
                print("\n*** Opción no válida. Por favor, ingrese 1, 2 o 3. ***")


def analizarPoses(robot: BaseRobotController, poses_list: list) -> pd.DataFrame:
    """
    Análisis cinemático de poses de enseñanza de wobj, devolviendo un DataFrame.
    """
    datos_para_tabla = []
    opciones_formato = {'precision': 2, 'floatmode': 'fixed', 'suppress_small': True}

    for idx, pose in enumerate(poses_list, 1):
        POSE_robot = pose[6:]
        q_robot = pose[:6]
        q_robot_rad = np.deg2rad(q_robot)

        config = robot.cobot_tb.calc_conf(q_robot_rad)
        POSE_robot_SE3 = robot.pose_to_matrix(POSE_robot)
        robt = RobTarget(POSE_robot_SE3, config)
        POSE_calc = robot.cobot_tb.fkine(q_robot_rad)
        q_calc = np.rad2deg(robot.cobot_tb.ikine(POSE_calc, config)[0])
        q_robot_recalc = np.rad2deg(robot.cobot_tb.ikine(POSE_robot_SE3, config)[0])

        resultado_pose = {
            "Pose #": idx,
            "Coords Leídas": np.array2string(np.array(POSE_robot), **opciones_formato),
            "Ángulos Leídas": np.array2string(np.array(q_robot), **opciones_formato),
            "Config": str(config),
            "RobTarget Generado": np.array2string(robt.pose.A.flatten(), **opciones_formato),
            "POSE_calc (FKine)": np.array2string(POSE_calc.A.flatten(), **opciones_formato),
            "q_calc (IKine)": np.array2string(q_calc, **opciones_formato),
            "q_recalc (IKine)": np.array2string(q_robot_recalc, **opciones_formato)
        }
        datos_para_tabla.append(resultado_pose)

    return pd.DataFrame(datos_para_tabla)


def armar_csv(robot: BaseRobotController, nombre_archivo: str):
    """Importa los lotes de poses grabadas, los analiza y exporta un archivo CSV."""
    mod_qcoords = import_module("scripts.EnseñanzaWobj.Datos.Workobjects_qcoordsv8")

    casos_map = {
        "Caso 20": 'q_20',
        "Caso 21": 'q_21',
        "Caso 22": 'q_22',
        "Caso 23": 'q_23',
        "Caso 24": 'q_24',
        "Caso 25": 'q_25',
        "Caso 26": 'q_26',
        "Caso 27": 'q_30',
    }

    lista_de_dataframes = []
    for nombre_grupo, attr_name in casos_map.items():
        if not hasattr(mod_qcoords, attr_name):
            print(f"[Advertencia] No se encontró '{attr_name}' en el módulo. Saltando...")
            continue
        poses_list = getattr(mod_qcoords, attr_name)
        print(f"Analizando el grupo: '{nombre_grupo}'...")
        df_parcial = analizarPoses(robot, poses_list)
        df_parcial.insert(0, 'Grupo', nombre_grupo)
        lista_de_dataframes.append(df_parcial)

    if not lista_de_dataframes:
        print("No se encontraron datos para exportar.")
        return

    df_final = pd.concat(lista_de_dataframes, ignore_index=True)
    print("\n----------- DataFrame Final Combinado -----------")
    print(df_final.to_string())

    df_final.to_csv(nombre_archivo, index=False, encoding='utf-8')
    print(f"\n¡Exportación exitosa! El DataFrame combinado se ha guardado en '{nombre_archivo}'.")


def registrar_datos(robot: BaseRobotController, q_list: list):
    """Envía al robot a Home y luego a cada vector articular de q_list para comparar cinemática."""
    es_real = hasattr(robot, 'mc') and robot.__class__.__name__ == 'MyCobotController'
    for i in range(len(q_list)):
        robot.MoveJ_q(np.zeros(6), 40, 'deg')
        robot.MoveJ_q(np.array(q_list[i]), 30, 'deg')
        time.sleep(5)
        print(f'Ángulos pedidos:\n{q_list[i]}')
        print(f'Ángulos leídos:\n{leer_q_seguro_deg(robot)}')
        print(f'Pose según toolbox:\n{robot.cobot_tb.fkine(np.radians(q_list[i]))}')
        if es_real:
            print(f'Cobot pose (get_coords):\n{robot.mc.get_coords()}')


def calibrar_servos(robot: BaseRobotController):
    """Calibrar los servos del cobot real en su posición home."""
    robot.MoveJ_q(np.zeros(6), 20, 'deg')
    time.sleep(2)
    print(f'Lectura de los encoders:\n{robot.mc.get_encoders()}')
    for i in range(6):
        robot.mc.set_servo_calibration(i + 1)
        time.sleep(2)
    print(f'Encoders calibrados:\n{robot.mc.get_encoders()}')


def corregir_offset(robot: BaseRobotController, robt: RobTarget, tool: SE3):
    """
    Mueve el robot directamente a 'robt' (sin compensaciones previas), registra los ángulos
    a los que llegó, permite ajustar con el joystick hasta la esquina real y calcula el offset.
    """
    # 1. Movimiento puro hacia el RobTarget
    robot.MoveJ(robt, 30, tool, SE3())
    time.sleep(1.0)

    # 2. Guarda los ángulos a los que llegó físicamente
    q_incorrecto = np.deg2rad(leer_q_seguro_deg(robot))

    print("\n--- Ajuste Manual ---")
    print("Usa el joystick para mover la punta a la esquina REAL.")

    # 3. Ajuste manual con la misma herramienta
    robt_corregido = robot.joystick_adjust(
        q_incorrecto,
        mover_callback=lambda r: robot.MoveJ(r, 20, tool, SE3())
    )

    q_ajustado = np.deg2rad(leer_q_seguro_deg(robot))

    # Pose de la brida donde el robot creía estar vs. donde está físicamente bien
    T_brida_incorrecta = robot.cobot_tb.fkine(q_incorrecto)
    T_brida_correcta = robot.cobot_tb.fkine(q_ajustado)

    # Transformación de calibración en la brida (fórmula original)
    T_calibracion = T_brida_correcta * T_brida_incorrecta.inv()

    # Diferencia directa en la punta de la herramienta (TCP)
    T_tcp_incorrecta = T_brida_incorrecta * tool
    T_tcp_correcta = T_brida_correcta * tool
    offset_tcp_puro = (T_tcp_correcta.t - T_tcp_incorrecta.t).flatten()

    print(f"\nTransformación de Calibración Calculada (Brida):\n{T_calibracion}")
    print(f"Offset en T_calibracion [x, y, z]: {T_calibracion.t.round(3)} mm")
    print(f"Offset directo en la punta del TCP [dx, dy, dz]: {offset_tcp_puro.round(3)} mm")
    return T_calibracion, offset_tcp_puro


def leer_wobj(
    robot: BaseRobotController,
    it_wobj: int,
    archivo: str,
    version_archivo: int,
    pinza: SE3,
    forzar_ensenanza: bool = False
):
    """
    Carga el Wobj o lo enseña apagando los motores si no existe (auto_teach=True)
    o si se indica forzar_ensenanza=True.
    """
    filename = f'{archivo}_{version_archivo}'
    filename_q = f'{archivo}_{version_archivo}_q'
    wobj_name = f'wobj{it_wobj}'

    if forzar_ensenanza:
        wobj_teach = robot.teach_and_save_wobj(filename, wobj_name, pinza, save_q=True)
    else:
        wobj_teach = robot.load_wobj(filename, wobj_name, pinza, True)

    q_wobj = None
    for f_cand, v_cand in [(filename_q, f'{wobj_name}_q'), (f'{archivo}_qv{version_archivo}', f'q_{it_wobj}')]:
        try:
            q_wobj = robot.load_data('Workobjects', f_cand, v_cand)
            break
        except Exception:
            continue

    return wobj_teach, q_wobj


def cheq_wobj_qs(
    robot: BaseRobotController,
    it_wobj: int,
    archivo: str,
    version_archivo: int,
    pinza: SE3,
    pinza_aux: SE3
):
    """Anima en RViz las 6 poses articulares grabadas durante la enseñanza del Wobj."""
    if not hasattr(robot, 'view_pose') or not hasattr(robot, 'show_tf'):
        print("[Aviso] 'cheq_wobj_qs' requiere visualización en RViz (modo ros).")
        return

    robot.view_pose(np.zeros(6), pinza)
    robot.GripperState(0)
    q_load = robot.load_data('Workobjects', f'{archivo}v{version_archivo}', f'q_{it_wobj}')
    q_rads = np.deg2rad(q_load)

    for idx, q in enumerate(q_rads, start=1):
        if idx <= 3:
            robot.show_tf(robot.cobot_tb.fkine(q) * pinza, f'x{idx}')
        else:
            robot.show_tf(robot.cobot_tb.fkine(q) * pinza, f'y{idx-3}')
        time.sleep(1)
        robot.view_pose(q, pinza_aux)
        time.sleep(1)


def run(robot: BaseRobotController, **kwargs):
    # Definición de herramientas exactamente igual a rev5
    pinza_palp = SE3(0, 148, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza_aux = pinza * SE3(0, 0, 25)

    if hasattr(robot, 'load_scene'):
        robot.load_scene('teachWobj_scene_rev0')

    version_archivo = 9
    iteracion_wobj = 51
    conf_wobj = [1, -1, -1]  # [-1, 1, 1] para enseñanza en vertical

    while True:
        print("\n" + "=" * 65)
        print(f" PRUEBAS PURAS DE WOBJ (Sin compensaciones) | v{version_archivo} - wobj{iteracion_wobj}")
        print("=" * 65)
        print("  1. Chequear puntos q en RViz (cheq_wobj_qs)")
        print("  2. Cargar Wobj (o enseñar apagando motores si no existe)")
        print("  3. Forzar enseñanza de nuevo Wobj (apaga motores + joystick)")
        print("  4. Ir a Home [0,0,0,0,0,0]")
        print("  5. Ir a pose intermedia de tus notas [135, 25, 30, 20, -30, 0]")
        print("  6. Enviar cobot directo al Wobj (send_wobj_to_cobot puro)")
        print("  7. Medir error / corregir_offset desde la posición actual")
        print("  8. Revisar lista de poses grabadas (revisarPoses)")
        print("  9. Exportar análisis cinemático a CSV (armar_csv)")
        print("  0. Cambiar iteracion_wobj / version_archivo")
        print("  q. Salir")

        op = input("\nSeleccione una opción: ").strip().lower()

        if op == 'q':
            break
        elif op == '1':
            cheq_wobj_qs(robot, iteracion_wobj, 'Workobjects_q', version_archivo, pinza, pinza_aux)
        elif op == '2':
            wobj, q_wobj = leer_wobj(robot, iteracion_wobj, 'Workobjects', version_archivo, pinza, forzar_ensenanza=False)
            print(f"Wobj cargado:\n{wobj}")
        elif op == '3':
            wobj, q_wobj = leer_wobj(robot, iteracion_wobj, 'Workobjects', version_archivo, pinza, forzar_ensenanza=True)
            print(f"Wobj enseñado:\n{wobj}")
        elif op == '4':
            robot.MoveJ_q(np.zeros(6), 30, 'deg')
        elif op == '5':
            robot.MoveJ_q(np.array([135, 25, 30, 20, -30, 0]), 30, 'deg')
        elif op == '6':
            wobj, _ = leer_wobj(robot, iteracion_wobj, 'Workobjects', version_archivo, pinza, forzar_ensenanza=False)
            send_wobj_to_cobot(robot, wobj, pinza_aux, conf_wobj)
        elif op == '7':
            wobj, _ = leer_wobj(robot, iteracion_wobj, 'Workobjects', version_archivo, pinza, forzar_ensenanza=False)
            robt_wobj = RobTarget(SE3(wobj), conf_wobj)
            corregir_offset(robot, robt_wobj, pinza_aux)
        elif op == '8':
            _, q_wobj = leer_wobj(robot, iteracion_wobj, 'Workobjects', version_archivo, pinza, forzar_ensenanza=False)
            if q_wobj is not None:
                revisarPoses(robot, q_wobj, pinza)
            else:
                print("[!] No se encontraron poses grabadas para este Wobj.")
        elif op == '9':
            nombre_csv = input("Nombre del archivo CSV (ej. analisis_poses.csv): ").strip() or "analisis_poses.csv"
            armar_csv(robot, nombre_csv)
        elif op == '0':
            try:
                version_archivo = int(input(f"Versión de archivo (actual {version_archivo}): ").strip())
                iteracion_wobj = int(input(f"Número de iteracion_wobj (actual {iteracion_wobj}): ").strip())
            except ValueError:
                print("[!] Ingrese números enteros válidos.")
        else:
            print("[!] Opción no válida.")