import numpy as np
import sys
import os
import time
import select
import termios
import tty
from pathlib import Path
from spatialmath import SE3
from CobotStudio import ROSManager

# TCP exacto de tu código
pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)


def formatear_robtarget(nombre_var: str, robt) -> str:
    """Genera la línea exacta de código Python con el formato RobTarget de tus rutinas."""
    pos = robt.pose.t
    rpy = robt.pose.rpy(unit="deg")
    cfg = list(robt.config)
    return (
        f"{nombre_var} = RobTarget("
        f"SE3({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) * "
        f"SE3.RPY([{rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}], unit='deg'), "
        f"{cfg})"
    )


def leer_tecla_no_bloqueante(timeout: float = 0.2):
    """Detecta si se presionó una tecla en la terminal sin frenar el monitoreo."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None


def run(robot: ROSManager, **kwargs):
    if hasattr(robot, "load_scene"):
        try:
            robot.load_scene("NDLM_scene_2")
        except Exception:
            pass

    # Archivo donde se irán acumulando todos los RobTargets que captures
    carpeta_actual = Path(getattr(robot, "project_root", os.path.dirname(os.path.abspath(__file__))))
    archivo_salida = carpeta_actual / "puntos_capturados.py"

    if not archivo_salida.exists():
        archivo_salida.write_text("from Cobot_sdk import *\n\n# Puntos capturados desde monitor_tcp.py\n", encoding="utf-8")

    contador = 1
    guardados_sesion = []

    print("\n=====================================================================================")
    print(" MONITOREO DE POSE EN TIEMPO REAL + CAPTURA RÁPIDA DE ROBTARGETS")
    print("-------------------------------------------------------------------------------------")
    print(" Atajos de teclado:")
    print("   [ESPACIO] o [ENTER] : Guardar pose actual rápido (Robt_pos1, Robt_pos2, ...)")
    print("   [n]                 : Guardar pose actual escribiendo el nombre de la variable")
    print("   [q] o [Ctrl+C]      : Salir y mostrar resumen de todos los puntos capturados")
    print(f" Archivo de guardado   : {archivo_salida}")
    print("=====================================================================================\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        # Poner terminal en modo cbreak para detectar teclas al instante sin apretar Enter
        tty.setcbreak(fd)

        while True:
            _, robt = robot.get_current_q(get_robt=True, tool=pinza)

            pos = robt.pose.t
            rpy = robt.pose.rpy(unit="deg")
            cfg = list(robt.config)

            output = (
                f"\r X: {pos[0]:7.2f} | Y: {pos[1]:7.2f} | Z: {pos[2]:7.2f} (mm) || "
                f"R: {rpy[0]:6.2f}° | P: {rpy[1]:6.2f}° | Y: {rpy[2]:6.2f}° || "
                f"Cfg: {cfg}   "
            )
            sys.stdout.write(output)
            sys.stdout.flush()

            tecla = leer_tecla_no_bloqueante(timeout=0.25)

            if tecla in (' ', '\n', '\r'):
                nombre_var = f"Robt_pos{contador}"
                linea = formatear_robtarget(nombre_var, robt)
                with open(archivo_salida, "a", encoding="utf-8") as f:
                    f.write(linea + "\n")
                guardados_sesion.append(linea)
                contador += 1
                print(f"\n[GUARDADO] {linea}")

            elif tecla in ('n', 'N'):
                # Restaurar terminal temporalmente para que puedas escribir el nombre
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                nombre_custom = input("\nNombre para este RobTarget (ej. Robt_entrega): ").strip()
                tty.setcbreak(fd)

                if not nombre_custom:
                    nombre_custom = f"Robt_pos{contador}"
                    contador += 1

                linea = formatear_robtarget(nombre_custom, robt)
                with open(archivo_salida, "a", encoding="utf-8") as f:
                    f.write(linea + "\n")
                guardados_sesion.append(linea)
                print(f"[GUARDADO] {linea}")

            elif tecla in ('q', 'Q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        # Restaurar siempre la configuración normal de la terminal al salir
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\n\nMonitoreo finalizado.")
        if guardados_sesion:
            print("\n--- RESUMEN DE ROBTARGETS CAPTURADOS (Listos para copiar a tu rutina) ---")
            for item in guardados_sesion:
                print(item)
            print(f"\nTambién quedaron guardados en: {archivo_salida}")