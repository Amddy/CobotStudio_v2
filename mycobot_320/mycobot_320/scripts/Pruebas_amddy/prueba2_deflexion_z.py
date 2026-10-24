import os
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from spatialmath import SE3
from Cobot_sdk import *

def resolver_postura_horizontal(robot, pinza, r_target, z_target, q_semilla):
    """
    Calcula los ángulos articulares exactos (q2, q3, q4) para situar la herramienta
    en Radio=r_target y Altura=z_target garantizando Z constante y avance horizontal.
    """
    def costo(q_vars):
        q2, q3, q4 = q_vars
        q_cand = np.array([90.0, q2, q3, q4, -90.0, 0.0])
        pose_calc = robot.cobot_tb.fkine(np.deg2rad(q_cand)) * pinza
        r_calc = np.linalg.norm(pose_calc.t[:2])
        z_calc = pose_calc.t[2]
        
        # Minimizar discrepancia en radio y forzar fuertemente Z constante
        err_r = (r_calc - r_target)**2
        err_z = 25.0 * (z_calc - z_target)**2
        # Mantener actitud geométrica natural del cabezal
        err_geom = 0.02 * (q2 + q3 + q4)**2
        return err_r + err_z + err_geom

    res = minimize(costo, q_semilla, method='Nelder-Mead', options={'maxiter': 1500, 'xatol': 1e-4, 'fatol': 1e-4})
    q2_opt, q3_opt, q4_opt = res.x
    return np.array([90.0, q2_opt, q3_opt, q4_opt, -90.0, 0.0])

def run(robot: BaseRobotController, **kwargs):
    print("\n" + "="*70)
    print(" PRUEBA 2: RIGIDEZ ARTICULAR Y DEFLEXIÓN GRAVITATORIA ESTÁTICA ")
    print("="*70)

    es_robot_real = (robot.__class__.__name__ == 'MyCobotController')
    print(f"Entorno detectado: {'ROBOT FÍSICO' if es_robot_real else 'SIMULACIÓN ROS'}\n")

    # Carpeta de salida dedicada
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "resultados_prueba2")
    os.makedirs(output_dir, exist_ok=True)

    pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)

    # Definición de plano horizontal estricto (Z fija) y radios crecientes
    Z_fijo = 280.0  # Altura constante en mm sobre el plano de trabajo
    radios_objetivo = [180.0, 220.0, 260.0, 300.0]  # mm

    print(f"Calculando cinemática exacta para plano horizontal (Z = {Z_fijo:.1f} mm)...")
    posturas = []
    q_semilla = [35.0, -60.0, 25.0]

    for r_obj in radios_objetivo:
        q_sol = resolver_postura_horizontal(robot, pinza, r_obj, Z_fijo, q_semilla)
        q_semilla = [q_sol[1], q_sol[2], q_sol[3]]  # Semilla para el siguiente punto
        
        pose_verif = robot.cobot_tb.fkine(np.deg2rad(q_sol)) * pinza
        r_verif = float(np.linalg.norm(pose_verif.t[:2]))
        z_verif = float(pose_verif.t[2])

        posturas.append({
            "nombre": f"Extensión R = {int(r_obj)} mm",
            "q": q_sol,
            "r_teorico": r_verif,
            "z_teorico": z_verif,
            "pose_teorica": pose_verif
        })
        print(f"  -> {posturas[-1]['nombre']}: R={r_verif:.2f} mm | Z={z_verif:.2f} mm | q2={q_sol[1]:.1f}°, q3={q_sol[2]:.1f}°")

    registros = []

    for pos in posturas:
        nombre = pos["nombre"]
        q_cmd = pos["q"]
        pose_teorica = pos["pose_teorica"]
        r_teorico = pos["r_teorico"]
        z_teorico = pos["z_teorico"]

        print(f"\n--- Comandando: {nombre} ---")
        robot.MoveJ_q(q_cmd, 25, 'deg')
        time.sleep(3.5 if es_robot_real else 1.0)

        if es_robot_real:
            raw_angles = robot.mc.get_angles()
            q_leido = np.array(raw_angles[:6], dtype=float) if raw_angles else q_cmd.copy()
            
            temps = robot.mc.get_servo_temps() or [25]*6
            volts = robot.mc.get_servo_voltages() or [12.0]*6
            corrientes = robot.mc.get_servo_currents() or [0.0]*6
            
            # Carga específica (Registro 0x3C, Little-Endian -> mode=1)
            load_j2 = (robot.mc.get_servo_data(2, 0x3C, 1) or 0) * 0.1
            load_j3 = (robot.mc.get_servo_data(3, 0x3C, 1) or 0) * 0.1
            temp_j2 = temps[1]
            volt_j2 = volts[1]
            curr_j2 = corrientes[1]
        else:
            q_sim = getattr(robot, 'q_current', None)
            q_leido = np.rad2deg(q_sim[:6]) if q_sim is not None else q_cmd.copy()
            load_j2, load_j3, temp_j2, volt_j2, curr_j2 = 0.0, 0.0, 25, 12.0, 0.0

        pose_real = robot.cobot_tb.fkine(np.deg2rad(q_leido)) * pinza
        delta = (pose_real.t - pose_teorica.t).flatten()
        r_real = float(np.linalg.norm(pose_real.t[:2]))
        z_real = float(pose_real.t[2])

        print(f"  Radio: {r_real:.2f} mm | Z Teórico: {z_teorico:.2f} mm | Z Real: {z_real:.2f} mm")
        print(f"  Deflexión Delta_Z: {delta[2]:.3f} mm | ||Error||: {np.linalg.norm(delta):.3f} mm")
        if es_robot_real:
            print(f"  J2 -> Carga: {load_j2:.1f}% | Corriente: {curr_j2} mA | Tensión: {volt_j2:.1f}V | Temp: {temp_j2}°C")

        registros.append({
            "Postura": nombre,
            "Radio_mm": r_real,
            "Z_Real_mm": z_real,
            "Delta_Z_mm": delta[2],
            "Error_3D_mm": float(np.linalg.norm(delta)),
            "dq2_deg": q_leido[1] - q_cmd[1],
            "dq3_deg": q_leido[2] - q_cmd[2],
            "Load_J2_pct": load_j2,
            "Load_J3_pct": load_j3,
            "Corriente_J2_mA": curr_j2,
            "Temp_J2_C": temp_j2
        })

    robot.MoveJ_q(np.zeros(6), 20, 'deg')
    df = pd.DataFrame(registros)

    print("\n" + "="*70)
    print(" RESUMEN DE DEFLEXIÓN ESTÁTICA ")
    print("="*70)
    print(df[["Postura", "Radio_mm", "Z_Real_mm", "Delta_Z_mm", "Error_3D_mm", "dq2_deg", "dq3_deg", "Load_J2_pct"]].to_string(index=False))

    timestamp = int(time.time())
    nombre_base = f"prueba2_deflexion_z_{'real' if es_robot_real else 'sim'}_{timestamp}"
    ruta_csv = os.path.join(output_dir, f"{nombre_base}.csv")
    ruta_png = os.path.join(output_dir, f"{nombre_base}.png")

    df.to_csv(ruta_csv, index=False)
    print(f"\nDatos exportados en: {ruta_csv}")

    # -------------------------------------------------------------
    # Generación y Guardado de Gráficos
    # -------------------------------------------------------------
    print("Generando gráficos de deflexión...")
    radios = df["Radio_mm"].to_numpy()
    delta_z = df["Delta_Z_mm"].to_numpy()
    error_3d = df["Error_3D_mm"].to_numpy()
    load_j2 = df["Load_J2_pct"].to_numpy()
    load_j3 = df["Load_J3_pct"].to_numpy()

    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    fig.suptitle(f"Prueba 2: Rigidez Articular vs Extensión del Brazo (Z = {Z_fijo:.1f} mm)\n({nombre_base})", fontsize=13, fontweight='bold')

    ax1.plot(radios, delta_z, 'o-', color='#d62728', linewidth=2, label=r'Deflexión $\Delta Z$ [mm]')
    ax1.plot(radios, error_3d, 's--', color='#1f77b4', linewidth=1.8, label=r'Error 3D $||\Delta||$ [mm]')
    ax1.set_ylabel("Error Posicional [mm]", fontsize=10, fontweight='bold')
    ax1.set_title(f"Deflexión Estática a Altura Constante Z = {Z_fijo:.1f} mm", fontsize=11)
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="best", frameon=True)

    ax2.plot(radios, load_j2, 'o-', color='#2ca02c', linewidth=2, label=r'Carga Servo 2 ($J_2$)')
    ax2.plot(radios, load_j3, 's--', color='#ff7f0e', linewidth=1.8, label=r'Carga Servo 3 ($J_3$)')
    ax2.set_xlabel("Radio Horizontal $R = \sqrt{X^2 + Y^2}$ [mm]", fontsize=10, fontweight='bold')
    ax2.set_ylabel("Carga Motor [% PWM]", fontsize=10, fontweight='bold')
    ax2.set_title("Carga Electromecánica en Función del Radio de Extensión", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="best", frameon=True)

    plt.tight_layout()
    plt.savefig(ruta_png, dpi=300)
    print(f"Gráfico guardado en: {ruta_png}")
    plt.show()