from Cobot_sdk import *
import os
import time
import importlib.util
import numpy as np


def cargar_modulo_trayectorias(nombre_archivo: str = "reconfiguraciones"):
    """Carga dinámicamente el archivo Trayectorias/reconfiguraciones.py y devuelve sus trayectorias."""
    carpeta_actual = os.path.dirname(os.path.abspath(__file__))
    ruta = os.path.join(carpeta_actual, "Trayectorias", f"{nombre_archivo}.py")

    if not os.path.exists(ruta):
        print(f"[!] No se encontró el archivo: {ruta}")
        print("    Primero corré 'explorar_cofiguraciones.py' en modo 'ros' y presioná 'a'.")
        return None, {}, None

    spec = importlib.util.spec_from_file_location(nombre_archivo, ruta)
    modulo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modulo)

    # Extraer todas las variables TRAJ_CONF_X ordenadas
    trajs = [attr for attr in dir(modulo) if attr.startswith("TRAJ_CONF_")]
    trajs.sort(key=lambda x: int(x.split("_")[-1]) if x.split("_")[-1].isdigit() else 999)

    metadata = getattr(modulo, "METADATA_CONFIGS", {})
    pose_xyz = getattr(modulo, "POSE_OBJETIVO_XYZ", [250.0, -150.0, 300.0])

    return trajs, metadata, np.array(pose_xyz, dtype=float)


def leer_q_actual_deg(robot: BaseRobotController) -> np.ndarray:
    """Devuelve los 6 ángulos actuales en grados sin disparar warnings en modo ROS."""
    if robot.__class__.__name__ == "MyCobotController" and hasattr(robot, "mc"):
        try:
            ang = robot.mc.get_angles()
            if ang is not None and len(ang) == 6:
                return np.array(ang, dtype=float)
        except Exception:
            pass

    if hasattr(robot, "get_current_q"):
        q_rad = robot.get_current_q(prefer_gripper=False)
        if q_rad is not None and len(q_rad) >= 6:
            return np.rad2deg(np.array(q_rad[:6], dtype=float))

    return np.zeros(6, dtype=float)


def run(robot: BaseRobotController, **kwargs):
    # Definir exactamente el mismo TCP usado al generar las trayectorias
    tool = SE3(-1.71, 106.91, 28.33)
    archivo = "reconfiguraciones"

    if hasattr(robot, "load_scene"):
        robot.load_scene('mesa_scene')

    # Cargar automáticamente todas las trayectorias guardadas en Trayectorias/reconfiguraciones.py
    lista_trajs, metadata, xyz_objetivo = cargar_modulo_trayectorias(archivo)
    if not lista_trajs:
        return

    print("\n" + "=" * 75)
    print(" PRUEBA DE USO: EJECUCIÓN DE CONFIGURACIONES MÚLTIPLES (MOVEIT -> COBOT)")
    print("=" * 75)
    print(f" Pose Objetivo Nominal: X={xyz_objetivo[0]:.1f} mm, Y={xyz_objetivo[1]:.1f} mm, Z={xyz_objetivo[2]:.1f} mm")
    print(f" Trayectorias encontradas en 'Trayectorias/{archivo}.py': {len(lista_trajs)}\n")

    for idx, nombre_traj in enumerate(lista_trajs, start=1):
        info = metadata.get(nombre_traj, {})
        conf_str = str(info.get("conf", "Desconocida"))
        w_val = info.get("manipulabilidad", "-")
        q_teo = info.get("q_teorico_deg", [])
        print(f"  [{idx}] {nombre_traj:<13} | Config: {conf_str:<13} | W: {w_val} | q_dest(°): {q_teo}")

    print("-" * 75)
    print(" Opciones de ejecución:")
    print("  - ENTER o 't' : Ejecutar TODAS las configuraciones en secuencia (Home -> Conf_i -> Medir)")
    print("  - Número      : Ejecutar solo esa configuración (ej: 1, 2, ...)")
    print("  - 'q'         : Salir")

    opcion = input("\nSeleccione cómo desea ejecutar la prueba: ").strip().lower()
    if opcion == 'q':
        return

    if opcion in ('', 't'):
        trajs_a_ejecutar = lista_trajs
    elif opcion.isdigit() and (1 <= int(opcion) <= len(lista_trajs)):
        trajs_a_ejecutar = [lista_trajs[int(opcion) - 1]]
    else:
        print("[!] Opción inválida.")
        return

    # Ejecutar la secuencia seleccionada y medir el desempeño en cada configuración
    resultados = []

    for nombre_traj in trajs_a_ejecutar:
        info = metadata.get(nombre_traj, {})
        conf_str = str(info.get("conf", "N/A"))

        print("\n" + "-" * 65)
        print(f">>> 1. Llevando el robot a HOME antes de '{nombre_traj}' (Config {conf_str})...")
        robot.GoHome(speed=30)
        time.sleep(1.0)

        print(f">>> 2. Ejecutando '{nombre_traj}' con robot.traj_moveit()...")
        robt_alcanzado = robot.traj_moveit(
            archivo=archivo,
            traj_name=nombre_traj,
            tool=tool
        )

        # Esperar asentamiento mecánico para medir encoders reales
        time.sleep(1.5)
        q_real_deg = leer_q_actual_deg(robot)
        T_real_tcp = robot.cobot_tb.fkine(np.deg2rad(q_real_deg)) * tool
        xyz_real = T_real_tcp.t

        # Cálculo de errores cartesianos respecto a la pose objetivo
        delta_z = xyz_real[2] - xyz_objetivo[2]
        error_3d = float(np.linalg.norm(xyz_real - xyz_objetivo))

        print(f">>> 3. Medición en Pose Objetivo ({nombre_traj}):")
        print(f"       XYZ Teórico : [{xyz_objetivo[0]:6.2f}, {xyz_objetivo[1]:6.2f}, {xyz_objetivo[2]:6.2f}] mm")
        print(f"       XYZ Medido  : [{xyz_real[0]:6.2f}, {xyz_real[1]:6.2f}, {xyz_real[2]:6.2f}] mm")
        print(f"       Deflexión dZ: {delta_z:+6.2f} mm | Error 3D Total: {error_3d:6.2f} mm")

        resultados.append({
            "traj": nombre_traj,
            "conf": conf_str,
            "xyz_real": xyz_real,
            "delta_z": delta_z,
            "error_3d": error_3d,
            "q_real_deg": np.round(q_real_deg, 1)
        })

        # Pausa breve en la pose para observar físicamente la configuración del brazo
        time.sleep(2.0)

    # Regresar a Home de forma segura al terminar
    print("\n>>> Regresando a HOME para finalizar la prueba...")
    robot.GoHome(speed=30)

    # Imprimir tabla resumen comparativa
    print("\n" + "=" * 80)
    print(" RESUMEN COMPARATIVO DE EXACTITUD POR CONFIGURACIÓN (MISMA POSE CARTESIANA)")
    print("=" * 80)
    print(f"{'Trayectoria':<14} | {'Config [c1,c2,c3]':<18} | {'Z Real (mm)':<12} | {'Delta Z (mm)':<13} | {'Error 3D (mm)':<13}")
    print("-" * 80)
    for r in resultados:
        print(f"{r['traj']:<14} | {r['conf']:<18} | {r['xyz_real'][2]:>10.2f}   | {r['delta_z']:>+10.2f}    | {r['error_3d']:>10.2f}")
    print("=" * 80)