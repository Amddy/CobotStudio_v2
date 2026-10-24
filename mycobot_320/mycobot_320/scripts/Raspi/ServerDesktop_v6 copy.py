#!/usr/bin/env python3
"""
TCPJointStateClient
-----------------------------
Nodo de ROS 2 que actúa como cliente TCP.
Se conecta al RobotServer (corriendo en la Raspberry Pi del cobot),
recibe la telemetría en tiempo real (JSON) y publica en el tópico /joint_states.

Permite visualizar el movimiento del robot real en RViz al ejecutar una rutina.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import threading
import time
import json
import math

# IP de la Raspberry Pi (el servidor)
ROBOT_IP = "192.168.88.10"
PORT = 65432

# Nombres de joints: deben coincidir con el URDF
JOINT_NAMES = [
    'joint2_to_joint1', 
    'joint3_to_joint2', 
    'joint4_to_joint3', 
    'joint5_to_joint4', 
    'joint6_to_joint5', 
    'joint6output_to_joint6',
    'gripper_controller'
]

class TCPJointStateClient(Node):
    def __init__(self):
        super().__init__('tcp_joint_state_client')

        # Publisher de ROS
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Variables compartidas
        self._lock = threading.Lock()
        self._latest_angles_deg = None
        self._latest_data = None
        self._latest_ts = None

        # Iniciar el hilo del cliente
        self._sock_thread = threading.Thread(target=self._socket_client_loop, daemon=True)
        self._sock_thread.start()

        # Timer para publicar en ROS (30 Hz)
        self.create_timer(1.0/30.0, self._publish_latest)

        self.get_logger().info(f"Cliente Bridge iniciado. Intentando conectar a {ROBOT_IP}:{PORT}...")

    def _socket_client_loop(self):
        """
        Bucle principal que intenta conectarse al servidor (Raspi).
        Si se cae la conexión, reintenta infinitamente.
        """
        while rclpy.ok():
            try:
                self.get_logger().info(f"Conectando a {ROBOT_IP}:{PORT}...")

                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(5.0) # Timeout para la conexión
                    s.connect((ROBOT_IP, PORT))
                    s.settimeout(2.0) # Timeout para recibir datos
                    
                    self.get_logger().info("¡Conexión establecida con el Robot!")
                    
                    # Bucle de lectura mientras la conexión esté viva
                    self._read_from_socket(s)
                    
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                self.get_logger().warn(f"No se pudo conectar o se perdió conexión: {e}")
            
            # Leve pausa antes de reintentar
            self.get_logger().info("Reintentando en 2 segundos...")
            time.sleep(2.0)

    def _read_from_socket(self, conn):
        buffer = ""
        while rclpy.ok():
            try:
                data = conn.recv(4096)
                if not data:
                    self.get_logger().warn("El servidor cerró la conexión.")
                    break # Salir para reconectar
                
                buffer += data.decode('utf-8', errors='replace')
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    
                    parsed = self._parse_line(line)
                    if parsed is not None:
                        with self._lock:
                            self._latest_data = parsed
                            self._latest_ts = time.time()
                            
            except socket.timeout:
                # Es normal si el robot no envía nada por un momento, seguimos esperando
                continue
            except Exception as e:
                self.get_logger().error(f"Error leyendo datos: {e}")
                break

    def _parse_line(self, line: str):
        # Intenta JSON
        if line.startswith('{'):
            try:
                payload = json.loads(line)
                # Busca la key 'angles', 'angle' o 'joints'
                angles = payload.get("angles") or payload.get("angle") or payload.get("joints")
                gripper_raw = payload.get("gripper", 0.0)
                if isinstance(angles, list) and len(angles) == 6:
                    # return [float(a) for a in angles]
                    return (angles, gripper_raw)
            except json.JSONDecodeError:
                return None
        # Intenta CSV como fallback
        else:
            try:
                parts = [p for p in line.split(',') if p.strip()]
                if len(parts) == 6:
                    return [float(p) for p in parts]
            except ValueError:
                pass
        return None

    def _publish_latest(self):
        with self._lock:
            data = self._latest_data
            # angles_deg = self._latest_angles_deg
            # Opcional: usar ts para chequear si la data es muy vieja
        
        if data is None:
            return

        angles_deg, gripper_raw = data
        # Conversión Grados -> Radianes
        angles_rad = [a * math.pi / 180.0 for a in angles_deg]
        griper_rad = (gripper_raw * 0.01) -0.7
        angles_rad.append(griper_rad)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = angles_rad
        
        # MyCobot a veces envía arrays vacíos de velocity/effort, mejor dejarlos vacíos si no los tenemos
        msg.velocity = []
        msg.effort = []

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TCPJointStateClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()