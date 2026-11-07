import time
import numpy as np
from spatialmath import SE3
from CobotStudio import BaseRobotController
from RobtManager import RobTarget

def run(robot: BaseRobotController, **kwargs):
    # Si usas una herramienta (TCP), defínela aquí; si quieres la brida, deja SE3()
    tool = SE3()  # Ejemplo con pinza: SE3(-1.71, 106.91, 28.33)
    robot.logger.info("Servidor listo. Mostrando pose actual en pantalla (Ctrl+C para salir)...")

    while True:
        # 1. Leer de la memoria de CobotServer (si --bridge está activo) para no bloquear el puerto serie
        q_deg = None
        if getattr(robot, "_server_ref", None) is not None:
            latest = robot._server_ref.get_latest()
            q_deg = latest.get("angles")
        elif hasattr(robot, "mc"):
            q_deg = robot.mc.get_angles()

        if q_deg and len(q_deg) == 6:
            q_rad = np.deg2rad(q_deg)
            # Calcular matriz SE3 y RobTarget con tu modelo cinemático DHRobotGT
            robt = RobTarget.from_q(q_rad, tool=tool)
            xyz_rpy = robot.matrix_to_pose(robt.pose)  # [x, y, z, rx, ry, rz] en mm y grados

            q_str = ", ".join([f"{a:6.1f}°" for a in q_deg])
            p_str = (f"X:{xyz_rpy[0]:6.1f} Y:{xyz_rpy[1]:6.1f} Z:{xyz_rpy[2]:6.1f} mm | "
                     f"R:{xyz_rpy[3]:5.1f}° P:{xyz_rpy[4]:5.1f}° Y:{xyz_rpy[5]:5.1f}° | Conf:{robt.config}")

            # Imprime actualizando la línea en la terminal de la Raspi
            print(f"\r[q]: [{q_str}]  -->  [Pose]: {p_str}   ", end="", flush=True)

        time.sleep(1)