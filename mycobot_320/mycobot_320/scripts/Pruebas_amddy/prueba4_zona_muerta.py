import os
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from Cobot_sdk import *

def leer_posicion_encoder(robot, servo_id=1, reintentos=3):
    """Lee el registro 0x38 (posición actual) asegurando que el valor sea válido (0 a 4095)."""
    for _ in range(reintentos):
        pos = robot.mc.get_servo_data(servo_id, 0x38, 1)
        if pos is not None and 0 <= pos <= 4095:
            return pos
        time.sleep(0.05)
    return None

def run(robot: BaseRobotController, **kwargs):
    print("\n" + "="*70)
    print(" PRUEBA 4: DETERMINACIÓN EXPERIMENTAL DE ZONA MUERTA (EJE 1) ")
    print("="*70)

    es_robot_real = (robot.__class__.__name__ == 'MyCobotController')
    print(f"Entorno detectado: {'ROBOT FÍSICO' if es_robot_real else 'SIMULACIÓN ROS'}\n")

    # Carpeta de salida dedicada en el directorio del script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "resultados_prueba4")
    os.makedirs(output_dir, exist_ok=True)

    resolucion_paso = 360.0 / 4096.0  # 0.08789° por pulso (12 bits)
    pasos_test = [1, 2, 4, 8, 16]
    sentidos = [
        {"nombre": "Horario (CW - Registro 0x1A)", "signo": 1.0, "reg": 0x1A},
        {"nombre": "Antihorario (CCW - Registro 0x1B)", "signo": -1.0, "reg": 0x1B}
    ]

    q_base = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.MoveJ_q(q_base, 25, 'deg')
    time.sleep(3.0 if es_robot_real else 1.0)

    dead_cw, dead_ccw = 0, 0
    if es_robot_real:
        cw_val = robot.mc.get_servo_data(1, 0x1A, 0)
        ccw_val = robot.mc.get_servo_data(1, 0x1B, 0)
        dead_cw = cw_val if (cw_val is not None and cw_val >= 0) else 0
        dead_ccw = ccw_val if (ccw_val is not None and ccw_val >= 0) else 0
        print(f"Zona muerta leída en EEPROM Servo 1 -> CW: {dead_cw} pasos | CCW: {dead_ccw} pasos\n")

    registros = []

    for s_info in sentidos:
        sentido_nombre = s_info["nombre"]
        signo = s_info["signo"]
        print(f"\n--- Probando sentido: {sentido_nombre} ---")

        for pasos in pasos_test:
            delta_deg = signo * (pasos * resolucion_paso)
            q_test = q_base.copy()
            q_test[0] += delta_deg

            # Lectura inicial robusta de encoder
            pos_inicial = leer_posicion_encoder(robot, 1) if es_robot_real else 2048

            robot.MoveJ_q(q_test, 10, 'deg')
            time.sleep(2.0 if es_robot_real else 0.4)

            # Lectura final robusta de encoder
            if es_robot_real:
                pos_final = leer_posicion_encoder(robot, 1)
                if pos_final is not None and pos_inicial is not None:
                    delta_pasos = pos_final - pos_inicial
                else:
                    print("  [ADVERTENCIA] Error de lectura serie del encoder.")
                    delta_pasos = 0
                delta_medido_deg = delta_pasos * resolucion_paso
            else:
                q_sim = getattr(robot, 'q_current', None)
                q_act = np.rad2deg(q_sim[0]) if q_sim is not None else q_test[0]
                delta_medido_deg = q_act - q_base[0]
                delta_pasos = round(delta_medido_deg / resolucion_paso)

            respondio = abs(delta_pasos) >= 1

            print(f"  Orden: {delta_deg:+.3f}° ({pasos:2d} pasos) | "
                  f"Medido: {delta_medido_deg:+.3f}° ({delta_pasos:+2d} pasos) | "
                  f"Estado: {'RESPONDIÓ' if respondio else 'BLOQUEADO EN ZONA MUERTA'}")

            registros.append({
                "Sentido": sentido_nombre,
                "Pasos_Ordenados": pasos,
                "Delta_Ordenado_deg": delta_deg,
                "Pasos_Medidos": delta_pasos,
                "Delta_Medido_deg": delta_medido_deg,
                "Error_Seguimiento_deg": abs(delta_deg - delta_medido_deg),
                "Respondio": respondio
            })

            # Retorno a la posición base entre cada micropaso
            robot.MoveJ_q(q_base, 15, 'deg')
            time.sleep(1.5 if es_robot_real else 0.4)

    robot.MoveJ_q(np.zeros(6), 20, 'deg')
    df = pd.DataFrame(registros)

    print("\n" + "="*70)
    print(" RESUMEN DE SENSIBILIDAD POR MICROPASOS ")
    print("="*70)
    print(df[["Sentido", "Pasos_Ordenados", "Delta_Ordenado_deg", "Pasos_Medidos", "Respondio"]].to_string(index=False))

    # Guardado de archivos
    timestamp = int(time.time())
    nombre_base = f"prueba4_zona_muerta_{'real' if es_robot_real else 'sim'}_{timestamp}"
    ruta_csv = os.path.join(output_dir, f"{nombre_base}.csv")
    ruta_png = os.path.join(output_dir, f"{nombre_base}.png")

    df.to_csv(ruta_csv, index=False)
    print(f"\nDatos exportados en: {ruta_csv}")

    # -------------------------------------------------------------
    # Generación y Guardado de Gráficos
    # -------------------------------------------------------------
    print("Generando gráficos de zona muerta...")
    df_cw = df[df["Sentido"].str.contains("Horario")]
    df_ccw = df[df["Sentido"].str.contains("Antihorario")]

    pasos_cw = df_cw["Pasos_Ordenados"].to_numpy()
    cmd_cw = df_cw["Delta_Ordenado_deg"].to_numpy()
    med_cw = df_cw["Delta_Medido_deg"].to_numpy()

    pasos_ccw = df_ccw["Pasos_Ordenados"].to_numpy()
    cmd_ccw = df_ccw["Delta_Ordenado_deg"].to_numpy()
    med_ccw = df_ccw["Delta_Medido_deg"].to_numpy()

    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), sharey=False)
    fig.suptitle(f"Prueba 4: Umbral de Zona Muerta y Sensibilidad Angular (Servo 1)\n({nombre_base})", fontsize=12, fontweight='bold')

    # Panel Horario (CW)
    ax1.plot(pasos_cw, cmd_cw, 'k--', linewidth=1.8, label="Ideal Comandado")
    ax1.plot(pasos_cw, med_cw, 'o-', color="#d62728", linewidth=2, label="Respuesta Medida")
    ax1.set_xlabel("Micropasos de cuantización [1 paso = 0.088°]", fontsize=10, fontweight='bold')
    ax1.set_ylabel("Desplazamiento Angular [°]", fontsize=10, fontweight='bold')
    ax1.set_title(f"Sentido Horario (+CW)\nEEPROM Reg 0x1A = {dead_cw} pasos", fontsize=11)
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="upper left", frameon=True)

    # Panel Antihorario (CCW)
    ax2.plot(pasos_ccw, cmd_ccw, 'k--', linewidth=1.8, label="Ideal Comandado")
    ax2.plot(pasos_ccw, med_ccw, 's-', color="#1f77b4", linewidth=2, label="Respuesta Medida")
    ax2.set_xlabel("Micropasos de cuantización [1 paso = 0.088°]", fontsize=10, fontweight='bold')
    ax2.set_ylabel("Desplazamiento Angular [°]", fontsize=10, fontweight='bold')
    ax2.set_title(f"Sentido Antihorario (-CCW)\nEEPROM Reg 0x1B = {dead_ccw} pasos", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="lower left", frameon=True)

    plt.tight_layout()
    plt.savefig(ruta_png, dpi=300)
    print(f"Gráfico guardado en: {ruta_png}")
    plt.show()