#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from spatialmath import SE3
import time

# # Asegurar que Python encuentre las clases de tu SDK (RobTarget, myCobot320)
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# from RobtManager import RobTarget
# from DHRobotGT import myCobot320

# # Nombres oficiales del URDF usados en CobotClient
# ARM_JOINTS = [
#     'joint2_to_joint1',
#     'joint3_to_joint2',
#     'joint4_to_joint3',
#     'joint5_to_joint4',
#     'joint6_to_joint5',
#     'joint6output_to_joint6'
# ]

# class MonitorTCPNode(Node):
#     def __init__(self, tool: SE3, wobj: SE3, refresh_hz: float = 5.0):
#         super().__init__('monitor_tcp_robt')
#         self.tool = tool
#         self.wobj = wobj
#         self.cobot_dh = myCobot320()
#         self.latest_q_rad = None
#         self.latest_gripper_pct = None

#         # 1. Suscripción pasiva a /joint_states (cero tráfico extra en el bus serie)
#         self.sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self._joint_state_cb,
#             10
#         )

#         # 2. Timer para refrescar la pantalla de forma legible (ej. 5 veces por segundo)
#         self.timer = self.create_timer(1.0 / refresh_hz, self._mostrar_datos)
#         self.get_logger().info("Monitor TCP activo. Escuchando /joint_states...")

#     def _joint_state_cb(self, msg: JointState):
#         """Extrae los 6 ángulos en el orden correcto del URDF sin importar quién publique."""
#         try:
#             q_rad = []
#             for j_name in ARM_JOINTS:
#                 idx = msg.name.index(j_name)
#                 q_rad.append(float(msg.position[idx]))
#             self.latest_q_rad = q_rad

#             # Si también viene el gripper (-0.7 a 0.3 rad -> 0 a 100%)
#             if 'gripper_controller' in msg.name:
#                 g_idx = msg.name.index('gripper_controller')
#                 g_rad = float(msg.position[g_idx])
#                 self.latest_gripper_pct = max(0.0, min(100.0, (g_rad + 0.7) * 100.0))
#         except ValueError:
#             pass

#     def _mostrar_datos(self):
#         if self.latest_q_rad is None:
#             print("\r[Esperando datos en el tópico /joint_states...]", end="", flush=True)
#             return

#         q_rad = self.latest_q_rad
#         q_deg = np.round(np.rad2deg(q_rad), 2)

#         # Calcular el RobTarget actual a partir de los ángulos leídos del tópico
#         robt = RobTarget.from_q(q_rad, tool=self.tool)
#         if self.wobj != SE3():
#             robt = self.wobj.inv() * robt

#         # Extraer traslación (X, Y, Z en mm) y orientación (Roll, Pitch, Yaw en grados)
#         xyz = np.round(robt.pose.t, 2)
#         rpy_zyx = np.round(robt.pose.rpy(unit='deg', order='zyx'), 2)
#         cfg = robt.config

#         # Línea lista para copiar y pegar en cualquier rutina
#         linea_codigo = (
#             f"RobTarget(SE3({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}) * "
#             f"SE3.RPY([{rpy_zyx[0]:.2f}, {rpy_zyx[1]:.2f}, {rpy_zyx[2]:.2f}], "
#             f"unit='deg', order='zyx'), {cfg})"
#         )

#         # Limpiar consola e imprimir panel actualizado
#         os.system('clear')
#         print("======================================================================")
#         print("       MONITOR EN VIVO: TCP Y ROBTARGET (vía /joint_states)           ")
#         print("======================================================================")
#         print(f" ÁNGULOS (deg) : J1:{q_deg[0]:6.2f}° | J2:{q_deg[1]:6.2f}° | J3:{q_deg[2]:6.2f}°")
#         print(f"                 J4:{q_deg[3]:6.2f}° | J5:{q_deg[4]:6.2f}° | J6:{q_deg[5]:6.2f}°")
#         if self.latest_gripper_pct is not None:
#             print(f" PINZA         : {self.latest_gripper_pct:5.1f} %")
#         print("----------------------------------------------------------------------")
#         print(f" POSICIÓN TCP  : X = {xyz[0]:7.2f} mm | Y = {xyz[1]:7.2f} mm | Z = {xyz[2]:7.2f} mm")
#         print(f" ORIENTACIÓN   : R(Z)= {rpy_zyx[0]:6.2f}° | P(Y)= {rpy_zyx[1]:6.2f}° | Y(X)= {rpy_zyx[2]:6.2f}°")
#         print(f" CONFIGURACIÓN : {cfg}  ([Brazo, Codo, Muñeca])")
#         print("----------------------------------------------------------------------")
#         print(" COPIAR PARA TU RUTINA:")
#         print(f" robt_actual = {linea_codigo}")
#         print("======================================================================")
#         print(" Presiona Ctrl+C para salir.")


# def main(args=None):
#     rclpy.init(args=args)

#     # Ajusta aquí el TCP de tu herramienta (por ejemplo, la pinza de tu proyecto)
#     pinza = SE3(-1.71, 106.91, 28.33)
#     wobj = SE3()

#     nodo = MonitorTCPNode(tool=pinza, wobj=wobj, refresh_hz=5.0)
#     try:
#         rclpy.spin(nodo)
#     except KeyboardInterrupt:
#         print("\nMonitor cerrado.")
#     finally:
#         nodo.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import sys
import time
from CobotStudio import ROSManager

pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)

def run(robot: ROSManager, **kwargs):

    robot.load_scene()
    print("\n--- MONITOREO DE POSE EN TIEMPO REAL (Ctrl+C para salir) ---")
    try:
        while True:
            _, robt = robot.get_current_q(get_robt=True, tool=pinza)

            pos = robt.pose.t
            rpy = robt.pose.rpy(unit="deg")

            # \r al inicio regresa el cursor al comienzo de la línea
            output = f"\r X: {pos[0]:7.2f} | Y: {pos[1]:7.2f} | Z: {pos[2]:7.2f} (mm) || R: {rpy[0]:6.2f}° | P: {rpy[1]:6.2f}° | Y: {rpy[2]:6.2f}°"

            sys.stdout.write(output)
            sys.stdout.flush()

            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nMonitoreo finalizado.")
