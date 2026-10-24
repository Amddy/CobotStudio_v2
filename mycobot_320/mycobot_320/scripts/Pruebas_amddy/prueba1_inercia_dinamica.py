import os
import time
import threading
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from Cobot_sdk import *

def run(robot: BaseRobotController, **kwargs):
    print("\n" + "="*70)
    print(" PRUEBA 1: RESPUESTA DINÁMICA E INERCIA ARTICULAR BAJO CARGA ")
    print("="*70)

    es_robot_real = (robot.__class__.__name__ == 'MyCobotController')
    print(f"Entorno detectado: {'ROBOT FÍSICO' if es_robot_real else 'SIMULACIÓN ROS'}\n")

    # Obtener el directorio donde reside este script y definir la subcarpeta de salida
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "resultados_prueba1")
    os.makedirs(output_dir, exist_ok=True)

    q_origen = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q_destino = np.array([0.0, -90.0, 45.0, 0.0, 0.0, 45.0])
    

    print("Posicionando en pose inicial...")
    robot.MoveJ_q(q_origen, 25, 'deg')
    time.sleep(3.0 if es_robot_real else 1.0)

    telemetria = []
    muestreando = True
    t_inicio = time.time()

    def hilo_muestreo():
        while muestreando:
            t_act = time.time() - t_inicio
            if es_robot_real:
                angles = robot.mc.get_angles() or [0.0]*6
                currents = robot.mc.get_servo_currents() or [0.0]*6
                speeds = robot.mc.get_servo_speeds() or [0.0]*6
                volts = robot.mc.get_servo_voltages() or [12.0]*6
            else:
                q_sim = getattr(robot, 'q_current', None)
                angles = np.rad2deg(q_sim[:6]) if q_sim is not None else q_origen.copy()
                # Perfil estimado para visualización en simulación
                currents = [350.0 if t_act < 2.0 else 120.0]*6
                speeds = [45.0 if t_act < 2.0 else 0.0]*6
                volts = [12.0]*6

            telemetria.append({
                "Tiempo_s": round(t_act, 3),
                "q2_deg": angles[1],
                "q3_deg": angles[2],
                "Corriente_J2_mA": currents[1],
                "Corriente_J3_mA": currents[2],
                "Velocidad_J2_step_s": speeds[1],
                "Velocidad_J3_step_s": speeds[2],
                "Voltaje_J2_V": volts[1]
            })
            time.sleep(0.05)

    # Iniciar muestreo continuo en hilo paralelo
    t_thread = threading.Thread(target=hilo_muestreo, daemon=True)
    t_thread.start()

    print("Ejecutando movimiento dinámico hacia zona de alta inercia...")
    robot.MoveJ_q(q_destino, 40, 'deg')
    time.sleep(1.5)  # Margen de asentamiento final

    muestreando = False
    t_thread.join()

    # Rutas absolutas para el guardado en la subcarpeta
    df = pd.DataFrame(telemetria)
    timestamp = int(time.time())
    nombre_archivo = f"prueba1_inercia_{'real' if es_robot_real else 'sim'}_{timestamp}"
    ruta_csv = os.path.join(output_dir, f"{nombre_archivo}.csv")
    ruta_png = os.path.join(output_dir, f"{nombre_archivo}.png")

    df.to_csv(ruta_csv, index=False)
    print(f"\nDatos dinámicos guardados en: {ruta_csv}")

    # Retorno seguro a Home
    robot.MoveJ_q(np.zeros(6), 20, 'deg')

    # -------------------------------------------------------------
    # Generación y Guardado de Gráficos
    # -------------------------------------------------------------
    print("Generando gráficos de telemetría...")
    t = df["Tiempo_s"].to_numpy()
    q2 = df["q2_deg"].to_numpy()
    q3 = df["q3_deg"].to_numpy()
    i_j2 = df["Corriente_J2_mA"].to_numpy()
    i_j3 = df["Corriente_J3_mA"].to_numpy()
    v_j2 = df["Velocidad_J2_step_s"].to_numpy()
    v_j3 = df["Velocidad_J3_step_s"].to_numpy()

    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f"Prueba 1: Telemetría de Inercia y Respuesta Dinámica\n({nombre_archivo})", fontsize=13, fontweight='bold')

    # Gráfico 1: Posición Angular
    ax1.plot(t, q2, label=r"$q_2$ (Hombro / Eje 2)", color="#1f77b4", linewidth=2)
    ax1.plot(t, q3, label=r"$q_3$ (Codo / Eje 3)", color="#ff7f0e", linewidth=2)
    ax1.set_ylabel("Posición [°]", fontsize=10, fontweight='bold')
    ax1.set_title("Evolución Angular de Articulaciones", fontsize=11)
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="upper right", frameon=True)

    # Gráfico 2: Corrientes
    ax2.plot(t, i_j2, label=r"$I_{J2}$ (Servo 2)", color="#d62728", linewidth=1.8)
    ax2.plot(t, i_j3, label=r"$I_{J3}$ (Servo 3)", color="#9467bd", linewidth=1.8)
    ax2.set_ylabel("Corriente [mA]", fontsize=10, fontweight='bold')
    ax2.set_title("Consumo de Corriente en Motores", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="upper right", frameon=True)

    # Gráfico 3: Velocidad
    ax3.plot(t, v_j2, label=r"Velocidad $J_2$", color="#2ca02c", linewidth=1.8)
    ax3.plot(t, v_j3, label=r"Velocidad $J_3$", color="#8c564b", linewidth=1.8, linestyle="--")
    ax3.set_xlabel("Tiempo [s]", fontsize=10, fontweight='bold')
    ax3.set_ylabel("Velocidad [step/s]", fontsize=10, fontweight='bold')
    ax3.set_title("Velocidad de Salida", fontsize=11)
    ax3.grid(True, linestyle="--", alpha=0.6)
    ax3.legend(loc="upper right", frameon=True)

    plt.tight_layout()
    plt.savefig(ruta_png, dpi=300)
    print(f"Gráfico guardado en: {ruta_png}")
    plt.show()