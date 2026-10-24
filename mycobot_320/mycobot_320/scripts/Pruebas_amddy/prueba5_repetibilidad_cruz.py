import os
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from spatialmath import SE3
from Cobot_sdk import *

def run(robot: BaseRobotController, **kwargs):
    print("\n" + "="*70)
    print(" PRUEBA 5: REPETIBILIDAD MULTIDIRECCIONAL EN CRUZ (ISO 9283) ")
    print("="*70)

    es_robot_real = (robot.__class__.__name__ == 'MyCobotController')
    print(f"Entorno detectado: {'ROBOT FÍSICO' if es_robot_real else 'SIMULACIÓN ROS'}\n")

    # Carpeta de salida dedicada en el directorio del script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "resultados_prueba5")
    os.makedirs(output_dir, exist_ok=True)

    pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)

    q_centro = np.array([90.0, 100.0, -80.0, -20.0, -50.0, 0.0])
    T_centro_nom = robot.cobot_tb.fkine(np.deg2rad(q_centro)) * pinza

    delta_dist = 40.0
    perturbaciones = [
        {"eje": "+X", "offset": np.array([delta_dist, 0.0, 0.0])},
        {"eje": "-X", "offset": np.array([-delta_dist, 0.0, 0.0])},
        {"eje": "+Y", "offset": np.array([0.0, delta_dist, 0.0])},
        {"eje": "-Y", "offset": np.array([0.0, -delta_dist, 0.0])}
    ]

    conf_calc = robot.cobot_tb.calc_conf(np.deg2rad(q_centro))
    conf_centro = list(conf_calc)

    N_CICLOS = 4  # 16 llegadas al centro en total
    llegadas = []

    print(f"Punto Central Nominal [X, Y, Z]: {T_centro_nom.t.round(2)} mm\n")

    for ciclo in range(1, N_CICLOS + 1):
        print(f"\n>>> INICIANDO CICLO {ciclo}/{N_CICLOS} <<<")
        for pert in perturbaciones:
            origen = pert["eje"]
            # 1. Movimiento al cuadrante exterior manteniendo orientación
            T_ext = SE3.Rt(T_centro_nom.R, T_centro_nom.t + pert["offset"])
            q_ext = np.rad2deg(robot.cobot_tb.ikine(T_ext * pinza.inv(), conf_centro)[0])

            robot.MoveJ_q(q_ext, 30, 'deg')
            time.sleep(2.0 if es_robot_real else 0.5)

            # 2. Llegada al centro desde dicha dirección
            robot.MoveJ_q(q_centro, 25, 'deg')
            time.sleep(3.0 if es_robot_real else 0.8)

            # 3. Adquisición de posición
            if es_robot_real:
                raw = robot.mc.get_angles()
                q_leido = np.array(raw[:6], dtype=float) if (raw and len(raw) >= 6) else q_centro.copy()
            else:
                q_sim = getattr(robot, 'q_current', None)
                q_leido = np.rad2deg(q_sim[:6]) if q_sim is not None else q_centro.copy()

            T_alcanzada = robot.cobot_tb.fkine(np.deg2rad(q_leido)) * pinza
            pos = T_alcanzada.t.flatten()

            print(f"  Llegada desde {origen:2s} -> Centro: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] mm")

            llegadas.append({
                "Ciclo": ciclo,
                "Direccion_Aproximacion": origen,
                "X_mm": pos[0],
                "Y_mm": pos[1],
                "Z_mm": pos[2],
                "Error_Nominal_mm": float(np.linalg.norm(pos - T_centro_nom.t.flatten()))
            })

    robot.MoveJ_q(np.zeros(6), 20, 'deg')
    df = pd.DataFrame(llegadas)

    # Procesamiento
    bar_x = float(df["X_mm"].mean())
    bar_y = float(df["Y_mm"].mean())
    bar_z = float(df["Z_mm"].mean())

    dist_baricentro = np.sqrt(
        (df["X_mm"] - bar_x)**2 +
        (df["Y_mm"] - bar_y)**2 +
        (df["Z_mm"] - bar_z)**2
    )
    df["Distancia_Baricentro_mm"] = dist_baricentro

    bar_l = float(dist_baricentro.mean())
    s_l = float(dist_baricentro.std())
    rp_iso = bar_l + 3 * s_l

    print("\n" + "="*70)
    print(" RESULTADOS DE REPETIBILIDAD (ISO 9283) ")
    print("="*70)
    print(f"Baricentro Experimental [X, Y, Z]: [{bar_x:.3f}, {bar_y:.3f}, {bar_z:.3f}] mm")
    print(f"Desviación media al baricentro (l_bar): {bar_l:.3f} mm")
    print(f"Desviación estándar (S_l): {s_l:.3f} mm")
    print(f"Repetibilidad Cartesiana (Rp = l_bar + 3*S_l): {rp_iso:.3f} mm")

    # Guardado de archivos
    timestamp = int(time.time())
    nombre_base = f"prueba5_repetibilidad_iso_{'real' if es_robot_real else 'sim'}_{timestamp}"
    ruta_csv = os.path.join(output_dir, f"{nombre_base}.csv")
    ruta_png = os.path.join(output_dir, f"{nombre_base}.png")

    df.to_csv(ruta_csv, index=False)
    print(f"\nDatos exportados en: {ruta_csv}")

    # -------------------------------------------------------------
    # Generación y Guardado de Gráficos
    # -------------------------------------------------------------
    print("Generando gráficos de repetibilidad multidireccional...")
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))
    fig.suptitle(f"Prueba 5: Repetibilidad Multidireccional ISO 9283\n({nombre_base})", fontsize=12, fontweight='bold')

    colores = {'+X': '#d62728', '-X': '#1f77b4', '+Y': '#2ca02c', '-Y': '#ff7f0e'}

    # Panel 1: Nube de dispersión en el plano XY
    for orig in ['+X', '-X', '+Y', '-Y']:
        sub = df[df['Direccion_Aproximacion'] == orig]
        ax1.scatter(sub['X_mm'].to_numpy(), sub['Y_mm'].to_numpy(), 
                    color=colores[orig], label=f'Desde {orig}', s=50, alpha=0.8)

    ax1.scatter([bar_x], [bar_y], color='black', marker='X', s=130, label=f'Baricentro ({bar_x:.1f}, {bar_y:.1f})')
    
    # Círculo de repetibilidad Rp proyectado
    circulo_rp = plt.Circle((bar_x, bar_y), max(rp_iso, 0.1), color='purple', fill=False, 
                            linestyle='--', linewidth=1.5, label=f'$R_p = {rp_iso:.2f}$ mm')
    ax1.add_patch(circulo_rp)
    ax1.set_xlabel("X [mm]", fontsize=10, fontweight='bold')
    ax1.set_ylabel("Y [mm]", fontsize=10, fontweight='bold')
    ax1.set_title("Dispersión de Poses en el Plano XY", fontsize=11)
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc='best', frameon=True)
    ax1.axis('equal')

    # Panel 2: Distancia euclídea al baricentro por arribo
    iteraciones = np.arange(1, len(df) + 1)
    barras_colores = [colores[o] for o in df['Direccion_Aproximacion']]
    ax2.bar(iteraciones, dist_baricentro.to_numpy(), color=barras_colores, alpha=0.7)
    ax2.axhline(bar_l, color='black', linestyle='-', linewidth=1.5, label=f'Media $\\bar{{l}} = {bar_l:.2f}$ mm')
    ax2.axhline(rp_iso, color='purple', linestyle='--', linewidth=1.5, label=f'ISO 9283 $R_p = {rp_iso:.2f}$ mm')
    ax2.set_xlabel("N° de Llegada al Centro", fontsize=10, fontweight='bold')
    ax2.set_ylabel("Distancia al Baricentro $l_j$ [mm]", fontsize=10, fontweight='bold')
    ax2.set_title("Desviación Respecto al Baricentro por Arribo", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc='best', frameon=True)

    plt.tight_layout()
    plt.savefig(ruta_png, dpi=300)
    print(f"Gráfico guardado en: {ruta_png}")
    plt.show()