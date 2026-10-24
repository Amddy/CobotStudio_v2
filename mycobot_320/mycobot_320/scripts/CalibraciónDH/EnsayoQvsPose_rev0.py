from Cobot_sdk import *
import random
import os

# script_dir = os.path.dirname(os.path.abspath(__file__))
# filename = os.path.join(script_dir, "QvsPose30-04.txt")


# Límites articulares
joint_limits_deg = [
    (-170, 170),
    (-120, 120),
    (-148, 148),
    (-120, 135),
    (-168, 168),
    (-180, 180)
]
def generar_q_respetando_limites():
    """Genera un vector de 6 valores dentro de los límites articulares del robot."""
    return [round(random.uniform(lim[0]*0.7, lim[1]*0.7), 2) for lim in joint_limits_deg]

def run (robot: BaseRobotController, **kwargs):
    # Archivo de salida
    filename = robot._resolve_project_path('Datos', '12-30', True)
    with open(filename, "a") as f:
        for i in range(1, 6):
            q_random = generar_q_respetando_limites()
            robot.logger.debug(f'q_random = {q_random}')
            robot.MoveJAngles(q_random, 30, 'deg')
            # mc.send_angles(qs_prueba[i-1], 30)
            # mc.send_angles([0, 0, 0, 0, 0, 0], 30)
            time.sleep(2)

            q_real = robot.mc.get_angles()

            # Con estas 2 líneas se introduce el error del control!
            # mc.send_angles(q_real, 30)
            # time.sleep(2)

            pose = robot.mc.get_coords()

            # print(f"{i}\t{[round(q,2) for q in q_real]}\t{[round(p,2) for p in pose]}\n")
            # f.write(f"{i}; {[round(q,2) for q in q_real]}; {[round(p,2) for p in pose]}; {[round(ang,2) for ang in q_random]}\n")
            robot.GoHome(40)
            print(f"Iteración {i} completada")
            time.sleep(5)
    print(f"\nDatos guardados en {filename}")