from Cobot_sdk import *
import os
import time
import numpy as np
import matplotlib.pyplot as plt


def graficar_comparativa_configuraciones(pose_objetivo, opciones, conf_actual=None):
    """
    Abre una ventana de Matplotlib con gráfico de barras de ángulos articulares (q1..q6)
    y tabla comparativa de configuraciones, manipulabilidad y estado de colisión en MoveIt.
    """
    n = len(opciones)
    if n == 0:
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))
    fig.suptitle("Explorador Gráfico de Configuraciones para la Pose Objetivo", fontsize=13, fontweight='bold')

    # --- Panel Izquierdo: Gráfico de barras con los ángulos (q1..q6 en grados) ---
    ax_bar = axes[0]
    x = np.arange(6)
    ancho = 0.8 / n

    for idx, opt in enumerate(opciones):
        q_deg = np.rad2deg(opt['q'])
        etiqueta = f"#{idx+1}: {opt['conf']} ({'OK' if opt['ok'] else 'COLISIÓN'})"
        ax_bar.bar(x + idx * ancho, q_deg, width=ancho, label=etiqueta)

    ax_bar.set_xticks(x + ancho * (n - 1) / 2)
    ax_bar.set_xticklabels(['q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
    ax_bar.set_ylabel("Ángulo articular (°)")
    ax_bar.set_title("Comparación en Espacio Articular (Grados)")
    ax_bar.axhline(0, color='black', linewidth=0.8, linestyle='--')
    ax_bar.grid(True, alpha=0.3)
    ax_bar.legend(fontsize=8)

    # --- Panel Derecho: Tabla resumen de Soluciones del PCI ---
    ax_tab = axes[1]
    ax_tab.axis('off')
    ax_tab.set_title("Resumen de Soluciones del PCI (Misma Pose Cartesiana)")

    filas_tabla = []
    colores_filas = []
    for idx, opt in enumerate(opciones):
        q_deg_str = ", ".join([f"{val:5.1f}°" for val in np.rad2deg(opt['q'])])
        es_actual = " (ACTUAL)" if (conf_actual is not None and list(opt['conf']) == list(conf_actual)) else ""
        estado = "Libre" if opt['ok'] else "Colisión"
        filas_tabla.append([
            f"#{idx+1}{es_actual}",
            str(opt['conf']),
            f"{opt['manip']:.3f}",
            estado,
            f"[{q_deg_str}]"
        ])
        colores_filas.append(["#d4edda" if opt['ok'] else "#f8d7da"] * 5)

    columnas = ["Opción", "Config [c1,c2,c3]", "Manipulab.", "MoveIt", "Ángulos q1..q6 (°)"]
    tabla = ax_tab.table(
        cellText=filas_tabla,
        colLabels=columnas,
        cellColours=colores_filas,
        loc='center',
        cellLoc='center'
    )
    tabla.auto_set_font_size(False)
    tabla.set_fontsize(8.5)
    tabla.scale(1.1, 1.6)

    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)


def obtener_q_actual(robot: BaseRobotController) -> np.ndarray:
    """Obtiene el vector articular actual (en rad) sin disparar warnings de GhostDriver en ROS."""
    #  Solo consultar robot.mc si estamos en el controlador físico real
    if robot.__class__.__name__ == "MyCobotController" and hasattr(robot, "mc"):
        try:
            q_deg = robot.mc.get_angles()
            if q_deg is not None and len(q_deg) == 6:
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


def ir_a_home(robot: BaseRobotController, speed: int = 30):
    """Lleva el robot a la posición Home [0, 0, 0, 0, 0, 0] de manera compatible con ROS y Real."""
    print(">>> Moviendo el robot a HOME [0, 0, 0, 0, 0, 0]...")
    if hasattr(robot, "GoHome"):
        robot.GoHome(speed=speed)
    elif hasattr(robot, "MoveJ_q"):
        robot.MoveJ_q([0, 0, 0, 0, 0, 0], speed, 'deg')
    elif hasattr(robot, "MoveJAngles"):
        robot.MoveJAngles(np.zeros(6), speed=speed, unit='rad')
    time.sleep(2.0)


def guardar_metadata_en_archivo(nombre_archivo: str, metadata: dict, pose_objetivo: SE3):
    """Agrega un diccionario METADATA_CONFIGS al archivo de trayectorias para facilitar su uso posterior."""
    carpeta_traj = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Trayectorias")
    os.makedirs(carpeta_traj, exist_ok=True)
    ruta = os.path.join(carpeta_traj, f"{nombre_archivo}.py")

    with open(ruta, "a", encoding="utf-8") as f:
        f.write("\n# =====================================================================\n")
        f.write("# METADATA DE CONFIGURACIONES GENERADAS PARA LA POSE OBJETIVO\n")
        f.write("# =====================================================================\n")
        f.write(f"POSE_OBJETIVO_XYZ = {list(np.round(pose_objetivo.t, 3))}\n")
        f.write(f"METADATA_CONFIGS = {repr(metadata)}\n")


def run(robot: BaseRobotController, **kwargs):
    # Definir herramienta (TCP) y Workobject
    tool = SE3(-1.71, 106.91, 28.33)
    wobj = SE3()
    archivo_traj = "reconfiguraciones"

    if hasattr(robot, "load_scene"):
        robot.load_scene('mesa_scene')

    #  Definir la Pose Cartesiana objetivo usando SE3.OA para orientar el TCP fácilmente
    pose_objetivo = SE3(250, -150, 300) * SE3.OA([0, 0, -1], [1, 0, 0])
    robt_base = RobTarget(pose_objetivo)

    # Mostrar la terna objetivo en RViz si estamos en ROS
    if hasattr(robot, "show_tf"):
        robot.show_tf(wobj * pose_objetivo, "Pose_Objetivo")

    #  Buscar todas las configuraciones analíticas válidas con el modelo del robot
    configs_validas = robt_base.find_valid_configs(tool, wobj)
    if not configs_validas:
        print("[!] La pose elegida está fuera del alcance para cualquier configuración.")
        return

    #  Calcular para cada configuración sus ángulos exactos, manipulabilidad y colisión
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

    # Leer estado inicial y abrir gráfico comparativo
    q_now = obtener_q_actual(robot)
    conf_actual = robot.cobot_tb.calc_conf(q_now)
    graficar_comparativa_configuraciones(pose_objetivo, opciones, conf_actual)

    #  Bucle interactivo
    ultimo_idx = 0
    q_home_rad = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    while True:
        q_now = obtener_q_actual(robot)
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
        print("  - 's' + Número  : MOVER SEGURO vía HOME (Home -> MoveJ Config N) [Recomendado Real]")
        print("  - 'm' + Número  : MOVER DIRECTO con MoveJ desde posición actual (ej: m1, m2)")
        print("  - 'p' + Número  : PLANIFICAR con MoveIt (Home -> Config N) y guardar (ej: p1)")
        print("  - 'r' + Número  : REPRODUCIR trayectoria guardada con traj_moveit (ej: r1, r2)")
        print("  - 'a'           : GENERAR Y GUARDAR TODAS las trayectorias válidas desde HOME")
        print("  - 't'           : PROBAR TODAS las configs válidas en secuencia (vía HOME)")
        print("  - 'h'           : Ir a posición HOME [0, 0, 0, 0, 0, 0]")
        print("  - 'g'           : Actualizar ventana gráfica comparativa")
        print("  - 'q'           : Salir")

        entrada = input("\nSeleccione acción: ").strip().lower()

        if entrada == 'q':
            plt.close('all')
            break

        elif entrada == 'g':
            plt.close('all')
            graficar_comparativa_configuraciones(pose_objetivo, opciones, conf_actual)
            continue

        elif entrada == 'h':
            if hasattr(robot, "toggle_ros_connection"):
                robot.toggle_ros_connection(False)
            ir_a_home(robot, speed=30)
            if hasattr(robot, "toggle_ros_connection"):
                robot.toggle_ros_connection(True)
            continue

        elif entrada == 'a':
            # Generar y guardar todas las trayectorias estandarizadas saliendo desde HOME
            if not (hasattr(robot, "moveit_manager") and robot.moveit_manager is not None):
                print("[!] Debés estar en modo ROS con MoveIt activo para planificar y guardar.")
                continue

            # Limpiar archivo previo para que quede prolijo y sin variables repetidas
            carpeta_traj = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Trayectorias")
            os.makedirs(carpeta_traj, exist_ok=True)
            ruta_limpia = os.path.join(carpeta_traj, f"{archivo_traj}.py")
            if os.path.exists(ruta_limpia):
                os.remove(ruta_limpia)

            metadata_guardada = {}
            print(f"\n>>> Generando todas las trayectorias desde HOME en Trayectorias/{archivo_traj}.py...")

            for idx_gen, opt in enumerate(opciones, start=1):
                if not opt['ok']:
                    print(f"  [-] Saltando Conf #{idx_gen} {opt['conf']} (En colisión).")
                    continue

                nombre_var = f"TRAJ_CONF_{idx_gen}"
                print(f"  [+] Planificando HOME -> Conf #{idx_gen} {opt['conf']}...")
                q_plan = robot.moveit_manager.plan_and_execute(
                    target=opt['robt'],
                    start=q_home_rad,
                    tool=tool,
                    wobj=wobj,
                    execute=True,
                    update_rviz=True
                )
                if q_plan is not None:
                    exito = robot.moveit_manager.save_trajectory(archivo_traj, nombre_var)
                    if exito:
                        metadata_guardada[nombre_var] = {
                            "indice": idx_gen,
                            "conf": opt['conf'],
                            "q_teorico_deg": [round(float(x), 2) for x in np.rad2deg(opt['q'])],
                            "manipulabilidad": round(opt['manip'], 3)
                        }
                        print(f"      [OK] Guardada '{nombre_var}'.")

            guardar_metadata_en_archivo(archivo_traj, metadata_guardada, pose_objetivo)
            print(f">>> ¡Proceso completo! Se guardaron {len(metadata_guardada)} trayectorias listas para usar.")
            continue

        elif entrada == 't':
            # Probar todas las configuraciones válidas en secuencia pasando por Home
            print("\n>>> Iniciando recorrido seguro por TODAS las configuraciones válidas (vía HOME)...")
            if hasattr(robot, "toggle_ros_connection"):
                robot.toggle_ros_connection(False)

            for idx_seq, opt in enumerate(opciones, start=1):
                if not opt['ok']:
                    print(f"  [-] Saltando Conf #{idx_seq} {opt['conf']} por colisión.")
                    continue

                print(f"\n--- [Configuración #{idx_seq}: {opt['conf']}] Paso 1: Subiendo a HOME ---")
                ir_a_home(robot, speed=30)

                print(f"--- [Configuración #{idx_seq}: {opt['conf']}] Paso 2: Bajando a Pose Objetivo ---")
                robot.MoveJ(opt['robt'], speed=25, tool=tool, wobj=wobj, robt_name=f"Conf_{idx_seq}")
                time.sleep(1.0)

                q_medido = obtener_q_actual(robot)
                pose_real = wobj.inv() * robot.cobot_tb.fkine(q_medido) * tool
                err_mm = np.linalg.norm(pose_objetivo.t - pose_real.t)
                print(f"    -> Pose alcanzada | Desviación TCP: {err_mm:.2f} mm")

                resp = input("    Presione ENTER para ir a la siguiente (o 'q' para abortar secuencia): ").strip().lower()
                if resp == 'q':
                    break

            ir_a_home(robot, speed=30)
            if hasattr(robot, "toggle_ros_connection"):
                robot.toggle_ros_connection(True)
            continue

        # Detectar comandos indexados: ver ('v'), seguro ('s'), directo ('m'), MoveIt ('p'), reproducir ('r')
        modo = 'v'
        if entrada.startswith(('s', 'm', 'p', 'r')) and entrada[1:].isdigit():
            modo = entrada[0]
            idx = int(entrada[1:]) - 1
        elif entrada.isdigit():
            idx = int(entrada) - 1
        else:
            print("[!] Entrada no válida.")
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

        elif modo in ('s', 'm'):
            if hasattr(robot, "toggle_ros_connection"):
                robot.toggle_ros_connection(False)

            if modo == 's':
                print(f">>> [Modo Seguro] Llevando primero a HOME antes de ir a Conf #{idx+1}...")
                ir_a_home(robot, speed=30)

            print(f">>> Ejecutando MoveJ hacia configuración #{idx+1}: {elegida['conf']}...")
            robot.MoveJ(robt_elegido, speed=25, tool=tool, wobj=wobj, robt_name=f"Conf_{idx+1}")

            if hasattr(robot, "toggle_ros_connection"):
                robot.toggle_ros_connection(True)

        elif modo == 'p':
            if not elegida['ok']:
                print("[!] Esa configuración está en colisión. MoveIt rechazará el plan.")
                continue

            if hasattr(robot, "moveit_manager") and robot.moveit_manager is not None:
                nombre_var = f"TRAJ_CONF_{idx+1}"
                print(f">>> Planificando en MoveIt desde HOME hacia Conf #{idx+1}: {elegida['conf']}...")
                q_plan = robot.moveit_manager.plan_and_execute(
                    target=robt_elegido,
                    start=q_home_rad,
                    tool=tool,
                    wobj=wobj,
                    execute=True,
                    update_rviz=True
                )
                if q_plan is not None:
                    robot.moveit_manager.save_trajectory(archivo_traj, nombre_var)
                    print(f">>> Guardada en Trayectorias/{archivo_traj}.py como '{nombre_var}'.")
            else:
                print("[!] MoveIt no está activo. Usando MoveJ seguro vía Home...")
                ir_a_home(robot, speed=30)
                robot.MoveJ(robt_elegido, speed=25, tool=tool, wobj=wobj)

        elif modo == 'r':
            nombre_var = f"TRAJ_CONF_{idx+1}"
            print(f">>> Reproduciendo trayectoria '{nombre_var}' desde Trayectorias/{archivo_traj}.py...")
            robt_fin = robot.traj_moveit(archivo_traj, nombre_var, tool=tool)
            if robt_fin is not None:
                err_mm = np.linalg.norm(pose_objetivo.t - robt_fin.pose.t)
                print(f">>> Finalizado. Error cartesiano del TCP respecto al objetivo: {err_mm:.2f} mm")