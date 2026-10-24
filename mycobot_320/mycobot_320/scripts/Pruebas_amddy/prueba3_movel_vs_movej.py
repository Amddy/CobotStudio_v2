import os
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from spatialmath import SE3
from Cobot_sdk import *

def run(robot: BaseRobotController, **kwargs):
    print("\n" + "="*70)
    print(" PRUEBA 3: EVALUACIÓN COMPARATIVA CARTESIANA VS ARTICULAR ")
    print("="*70)

    es_robot_real = (robot.__class__.__name__ == 'MyCobotController')
    print(f"Entorno detectado: {'ROBOT FÍSICO' if es_robot_real else 'SIMULACIÓN ROS'}\n")

    # Carpeta de salida dedicada en el directorio del script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "resultados_prueba3")
    os.makedirs(output_dir, exist_ok=True)

    # Pose de origen segura dentro de la envolvente de trabajo
    q_origen = np.array([0.0, 90.0, -100.0, 20.0, -90.0, 0.0])
    
    # Pose nominal de la brida
    T_brida_origen = robot.cobot_tb.fkine(np.deg2rad(q_origen))
    
    # Desplazamiento cartesiano comandado sobre la brida
    vector_desplazamiento = np.array([0.0, 50.0, 75.0])
    T_brida_destino = SE3.Rt(T_brida_origen.R, T_brida_origen.t + vector_desplazamiento)

    config_nom = list(robot.cobot_tb.calc_conf(np.deg2rad(q_origen)))
    robt_destino = RobTarget(T_brida_destino, config_nom)
    
    # Cálculo articular nominal directo sobre la brida
    q_destino_calc = np.rad2deg(robot.cobot_tb.ikine(T_brida_destino, config_nom)[0])

    metodos = [
        {"nombre": "Movimiento Articular (MoveJ)", "tipo": "joint"},
        {"nombre": "Movimiento Cartesiano (MoveL)", "tipo": "cartesian"}
    ]

    registros = []

    for met in metodos:
        nombre = met["nombre"]
        print(f"\n>>> Probando: {nombre} <<<")

        # Posicionamiento inicial en la pose de partida
        robot.MoveJ_q(q_origen, 25, 'deg')
        time.sleep(3.0 if es_robot_real else 1.0)

        t_inicio = time.time()
        if met["tipo"] == "joint":
            robot.MoveJ_q(q_destino_calc, 20, 'deg')
        else:
            if es_robot_real:
                # Comandar directamente las coordenadas de la brida
                coords = list(T_brida_destino.t) + list(T_brida_destino.rpy(order='zyx', unit='deg'))
                robot.mc.sync_send_coords(coords, 20, 1)
            else:
                robot.MoveL(robt_destino, speed=20, tool=SE3())

        duracion = time.time() - t_inicio
        time.sleep(3.0 if es_robot_real else 1.0)

        if es_robot_real:
            raw = robot.mc.get_angles()
            q_final = np.array(raw[:6], dtype=float) if raw else q_destino_calc.copy()
            moving_flag = robot.mc.get_servo_data(5, 0x42, 0) or 0
            status_error = robot.mc.get_servo_data(5, 0x41, 0) or 0
        else:
            q_sim = getattr(robot, 'q_current', None)
            q_final = np.rad2deg(q_sim[:6]) if q_sim is not None else q_destino_calc.copy()
            moving_flag, status_error = 0, 0

        # Verificación cinemática sobre la brida
        pose_alcanzada = robot.cobot_tb.fkine(np.deg2rad(q_final))
        error_xyz = (pose_alcanzada.t - T_brida_destino.t).flatten()
        error_3d = float(np.linalg.norm(error_xyz))

        print(f"  Duración: {duracion:.2f} s")
        print(f"  Error Cartesiano [dx, dy, dz]: {error_xyz.round(3)} mm | ||Error||: {error_3d:.3f} mm")
        if es_robot_real:
            print(f"  Servo 5 -> Moving sign: {moving_flag} | Byte de estado: 0x{status_error:02X}")

        registros.append({
            "Metodo": nombre,
            "Duracion_s": duracion,
            "dx_mm": float(error_xyz[0]),
            "dy_mm": float(error_xyz[1]),
            "dz_mm": float(error_xyz[2]),
            "Error_3D_mm": error_3d,
            "dq_wrist_J5": float(q_final[4] - q_destino_calc[4]),
            "Status_J5": status_error
        })

    # Retorno seguro
    robot.MoveJ_q(np.zeros(6), 20, 'deg')
    df = pd.DataFrame(registros)

    print("\n" + "="*70)
    print(" COMPARATIVA DE EXACTITUD SEGÚN MÉTODO ")
    print("="*70)
    print(df[["Metodo", "Duracion_s", "dx_mm", "dy_mm", "dz_mm", "Error_3D_mm", "dq_wrist_J5"]].to_string(index=False))

    # Guardado de archivos
    timestamp = int(time.time())
    nombre_base = f"prueba3_movel_vs_movej_{'real' if es_robot_real else 'sim'}_{timestamp}"
    ruta_csv = os.path.join(output_dir, f"{nombre_base}.csv")
    ruta_png = os.path.join(output_dir, f"{nombre_base}.png")

    df.to_csv(ruta_csv, index=False)
    print(f"\nDatos exportados en: {ruta_csv}")

    # -------------------------------------------------------------
    # Generación y Guardado de Gráficos
    # -------------------------------------------------------------
    print("Generando gráficos comparativos de interpolación...")
    metodos_nombres = df["Metodo"].to_numpy()
    dx = df["dx_mm"].to_numpy()
    dy = df["dy_mm"].to_numpy()
    dz = df["dz_mm"].to_numpy()
    err_3d = df["Error_3D_mm"].to_numpy()
    duraciones = df["Duracion_s"].to_numpy()

    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))
    fig.suptitle(f"Prueba 3: Comparación de Desempeño MoveL vs MoveJ\n({nombre_base})", fontsize=12, fontweight='bold')

    # Panel 1: Desglose de Errores Cartesianos
    x = np.arange(len(df))
    bar_width = 0.18
    ax1.bar(x - 1.5*bar_width, dx, bar_width, label='dx [mm]', color='#1f77b4')
    ax1.bar(x - 0.5*bar_width, dy, bar_width, label='dy [mm]', color='#ff7f0e')
    ax1.bar(x + 0.5*bar_width, dz, bar_width, label='dz [mm]', color='#2ca02c')
    ax1.bar(x + 1.5*bar_width, err_3d, bar_width, label='Error 3D [mm]', color='#d62728')

    ax1.set_xticks(x)
    ax1.set_xticklabels(metodos_nombres, fontsize=9, fontweight='bold')
    ax1.set_ylabel("Discrepancia Cartesiana [mm]", fontsize=10, fontweight='bold')
    ax1.set_title("Error Cartesiano Final Respecto a la Pose Objetivo", fontsize=11)
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc='best', frameon=True)

    # Panel 2: Duración de la Trayectoria
    barras_t = ax2.bar(metodos_nombres, duraciones, color=['#3b528b', '#5ec962'], width=0.45, alpha=0.85)
    ax2.set_ylabel("Tiempo Total de Trayectoria [s]", fontsize=10, fontweight='bold')
    ax2.set_title("Tiempo de Ejecución por Tipo de Movimiento", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.6)

    for barra in barras_t:
        altura = barra.get_height()
        ax2.annotate(f"{altura:.2f} s",
                     xy=(barra.get_x() + barra.get_width() / 2, altura),
                     xytext=(0, 4), textcoords="offset points",
                     ha='center', va='bottom', fontweight='bold')

    plt.tight_layout()
    plt.savefig(ruta_png, dpi=300)
    print(f"Gráfico guardado en: {ruta_png}")
    plt.show()