# from scripts.CobotStudio import (
#     RobTarget,
#     SimManager,
#     MyCobotController,
#     checkQ,
#     checkPose,
#     pose_to_matrix,
#     TCP_4puntos,
#     TCP_4puntos_extendido,
#     joystick_adjust,
# )
# from spatialmath import SE3
# import numpy as np
# from DHRobotGT import myCobot320
# import time
# from pymycobot import MyCobotSocket

from Cobot_sdk import *

def generar_q_random():
    """
    Devuelve una lista de 6 valores random (en radianes) para testear movimientos articulares.
    """
    # Ejemplo: valores entre -pi y pi
    return np.random.uniform(-np.pi, np.pi, 6).tolist()

# cob = MyCobotController()

pose_1 = [104.7, -153.7, 180, -91.56, -66.61, 1.86]
pose_2 = [-102.4, -154.7, 180, -90.35, 38.05, 0.7]
pose_3 = [3.8, -154.2, 180, -90, 12.48, -1.79]
pose_4 = [-5.6, -123.7, 180, -91.56, 26.16, 0.48]

q_pose_1 = [0, 15, -45, -20, 0, -15]
q_pose_2 = [0, -15, 45, 20, 0, 15]
q_pose_3 = [0, 0, 0, 0, 0, 0]
q_pose_4 = [54, 2, 12, -14, -108, 27]
poses_prueba = [q_pose_1, q_pose_2, q_pose_3, q_pose_4]
# checkQ(np.degrees(q_pose_2), True) # Visualización de las poses grabadas

ensayo = [[19.4, 280.9, 426.4, -93.86, 0.88, 15.41], [19.4, 280.9, 426.4, -93.86, 0.88, 15.41], [19.4, 281.0, 426.2, -93.86, 0.88, 15.41], [19.4, 280.9, 426.4, -93.77, 0.88, 15.41], [19.4, 280.9, 426.4, -93.77, 0.88, 15.41], [19.4, 280.9, 426.4, -93.77, 0.88, 15.41], [19.5, 280.9, 426.4, -93.77, 0.87, 15.32], [19.5, 280.9, 426.4, -93.77, 0.87, 15.32], [19.4, 280.9, 426.4, -93.77, 0.88, 15.41], [19.5, 280.9, 426.4, -93.77, 0.87, 15.32], [20.8, 282.0, 425.0, -93.86, 0.51, 14.9], [20.6, 282.5, 424.5, -94.04, 0.51, 14.9], [20.8, 281.8, 425.3, -93.77, 0.51, 14.9], [20.9, 281.4, 425.7, -93.6, 0.59, 14.9], [20.9, 281.4, 425.7, -93.6, 0.59, 14.9]], [[132.3, 255.5, 224.1, 24.51, -36.99, 53.42], [132.3, 255.5, 224.1, 24.51, -36.99, 53.42], [132.3, 255.5, 224.1, 24.51, -36.99, 53.42], [132.3, 255.5, 224.1, 24.51, -36.99, 53.42], [132.3, 255.5, 224.1, 24.51, -36.99, 53.42], [132.3, 255.5, 224.1, 24.51, -36.99, 53.42], [132.3, 255.6, 224.1, 24.46, -36.91, 53.45], [132.3, 255.6, 224.1, 24.46, -36.91, 53.45], [132.3, 255.6, 224.1, 24.46, -36.91, 53.45], [132.3, 255.6, 224.1, 24.46, -36.91, 53.45], [133.7, 254.8, 225.3, 24.76, -37.08, 53.13], [134.1, 254.5, 225.7, 24.88, -37.12, 53.03], [134.2, 254.7, 225.1, 24.78, -36.97, 53.09], [134.2, 254.6, 225.5, 24.83, -37.05, 53.06], [134.2, 254.9, 224.7, 24.67, -36.82, 53.15]], [[71.5, 212.1, 389.2, -75.85, -43.55, 96.24], [70.9, 212.4, 388.4, -75.62, -43.27, 96.88], [70.5, 213.9, 387.0, -75.46, -42.86, 97.04], [68.7, 216.5, 384.6, -75.03, -42.31, 97.7], [68.0, 217.9, 383.2, -74.8, -42.0, 97.97], [67.9, 219.3, 381.7, -74.64, -41.59, 98.05], [67.9, 219.2, 381.3, -74.59, -41.6, 98.1], [68.0, 219.0, 381.2, -74.59, -41.69, 98.17], [67.9, 217.7, 381.7, -74.55, -42.07, 98.47], [67.9, 217.2, 382.0, -74.62, -42.24, 98.52], [68.2, 215.0, 382.6, -74.91, -43.16, 98.5], [68.2, 215.0, 382.6, -74.91, -43.16, 98.5], [68.1, 214.7, 382.9, -74.95, -43.24, 98.53], [68.1, 214.3, 383.2, -74.98, -43.33, 98.55], [68.0, 214.0, 383.6, -75.04, -43.33, 98.61]], [[-23.9, 235.4, 419.1, -96.64, 25.83, -78.76], [-23.9, 235.4, 419.1, -96.64, 25.83, -78.76], [-23.9, 235.9, 418.7, -96.66, 25.75, -78.77], [-23.9, 235.9, 418.7, -96.66, 25.75, -78.77], [-23.9, 235.9, 418.7, -96.66, 25.75, -78.77], [-23.9, 236.1, 418.4, -96.68, 25.66, -78.78], [-23.9, 235.7, 419.0, -96.68, 25.84, -78.7], [-23.9, 235.7, 419.0, -96.68, 25.84, -78.7], [-23.9, 236.2, 418.4, -96.72, 25.67, -78.71], [-23.9, 236.0, 418.6, -96.7, 25.75, -78.71], [-22.8, 236.3, 418.5, -96.63, 25.83, -79.11], [-22.8, 236.3, 418.5, -96.62, 25.74, -79.1], [-22.8, 236.3, 418.5, -96.62, 25.74, -79.1], [-22.8, 236.3, 418.5, -96.63, 25.91, -79.12], [-22.8, 236.3, 418.5, -96.63, 25.91, -79.12]]

# Vectores q calculados mediante ikine para las poses del martes 12/08
q1 = [1.81495107, -0.99961651,  0.25200104,  0.68146848,  1.59315131,  0.0119061]
q2 = [1.42920751, -1.94073726,  0.26673288,  0.85508014, -1.4667272,  -1.96505626]
q3 = [1.40419587, -1.39823072,  0.63197706,  0.37183331, -2.6219612,  -1.10804047]
q4 = [-1.58003319,  0.56696686,  0.5863963,  -0.75813289, -2.86706463,  0.83456788]
q_martes = np.rad2deg([q1, q2, q3, q4])

# TCP calculado con las poses del martes 12/08
# tcp_offset, tcp_offset_real, _, ecm, rmse = TCP_4puntos(q_martes)
# print(f"---Método TCP_4puntos---\nOffset TCP: {tcp_offset}\nOffset TCP real: {tcp_offset_real}\nECM: {ecm:.4f}\nRMSE: {rmse:.4f}")

# p_tool, p_tool_real, _, ecm_ext, rmse_ext = TCP_4puntos_extendido(q_martes)
# print(f"---Método TCP_4puntos_Pablo---\nOffset TCP: {p_tool}\nOffset TCP real: {p_tool_real}\nECM: {ecm_ext:.4f}\nRMSE: {rmse_ext:.4f}")

# poses_TCP = cob.recolectar_puntos_TCP(ajuste = True)
# poses_TCP = cob.grabar_poses(cant_poses = 4, ajuste = True)
# tcp_offset, tcp_offset_real, _, ecm, rmse = TCP_4puntos(poses_TCP)
# print(f"---Método TCP_4puntos---\nOffset TCP: {tcp_offset}\nOffset TCP real: {tcp_offset_real}\nECM: {ecm:.4f}\nRMSE: {rmse:.4f}")

# p_tool, p_tool_real, _, ecm_ext, rmse_ext = TCP_4puntos_extendido(poses_TCP)
# print(f"---Método TCP_4puntos_Pablo---\nOffset TCP: {p_tool}\nOffset TCP real: {p_tool_real}\nECM: {ecm_ext:.4f}\nRMSE: {rmse_ext:.4f}")


# Datos no calculados:
# Pose 1: [-113.20000000000003, 65.47333333333334, -37.526, -13.258666666666665, -21.238666666666667, -30.420666666666673]
# Pose 2: [-124.44666666666663, 136.30999999999997, -106.85999999999999, -59.05066666666666, 9.806666666666668, -76.10999999999999]
# Pose 3: [-110.55999999999996, -18.528, 89.26733333333333, -11.419999999999996, -16.960000000000004, -110.6493333333333]
# Pose 4: [-131.21133333333336, 25.038666666666657, 91.97599999999998, -4.829999999999999, 77.69000000000003, -62.49733333333331]

# pose_0 = [139.92000000000002, -99.65199999999999, 78.73466666666667, 58.50199999999998, 69.52, -45.61000000000001]
# pose_mod = joystick_adjust(np.deg2rad(pose_0), mover_callback=lambda r: cob.MoveJ(r, 20, SE3(), SE3()))
pose_pick_manual = RobTarget(SE3(0, -220, 25)* SE3.Ry(-np.pi), [-1, 1, 1]) 
pose_pick_manual2 = RobTarget(SE3(215.46, -190.81, 135.07) * SE3.RPY(np.deg2rad([146.27, -11.84, -146.69])), [-1, 1, -1])
wobj = SE3()
pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2)
# pinza = SE3(-2.84011157, 115.08057356, 22.28604936) * SE3.Rx(-np.pi/2)
# ''' Movimiento alrededor del TCP'''
def run(robot: BaseRobotController, **kwargs):
    robot.GripperState(0)
    angle = 0
    robot.MoveJ(pose_pick_manual2.relTool(0, 0, 0, angle, angle, angle), 80, pinza, wobj)
    time.sleep(3)
    # Aisla un solo eje para ver cuál introduce la oscilación
    robot.MoveL(pose_pick_manual2.relTool(0, 0, 0, -25, 0, 0), 10, pinza, wobj)
    robot.MoveL(pose_pick_manual2.relTool(0, 0, 0, -25, -25, 0), 10, pinza, wobj)  
    robot.MoveL(pose_pick_manual2.relTool(0, 0, 0, -25, -25, -25), 10, pinza, wobj)
    robot.MoveL(pose_pick_manual2.relTool(0, 0, 0, angle, angle, angle), 10, pinza, wobj)
    robot.MoveL(pose_pick_manual2.relTool(0, 0, 0, 25, 25, 25), 10, pinza, wobj)
    # while angle >= -25:
    #     robot.MoveJ(pose_pick_manual2.relTool(0, 0, 0, angle, angle, angle), 100, pinza, wobj)
    #     angle -= 5
    # angle = -25
    # while angle <= 25:
    #     robot.MoveJ(pose_pick_manual2.relTool(0, 0, 0, angle, angle, angle), 100, pinza, wobj)
    #     angle += 5
