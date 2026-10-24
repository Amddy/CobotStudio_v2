#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import threading
import time
import json
import math
import subprocess

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

class CobotClient(Node):
    """
    Nodo de ROS 2 que actúa como cliente TCP.
    Se conecta al RobotServer (corriendo en la Raspberry Pi del cobot),
    recibe la telemetría en tiempo real (JSON) y publica en el tópico /joint_states.

    Permite visualizar el movimiento del robot real en RViz al ejecutar una rutina.
    """
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
        """Lee el stream TCP, maneja el buffer y extrae líneas JSON completas."""
        buffer = ""
        while rclpy.ok():
            try:
                data = conn.recv(4096)
                if not data:
                    self.get_logger().warn("El servidor cerró la conexión.")
                    break # Salir para reconectar
                
                buffer += data.decode('utf-8', errors='replace')
                
                # Procesar líneas completas
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
                # Si el robot no envía nada por un momento, seguir esperando
                continue
            except Exception as e:
                self.get_logger().error(f"Error leyendo datos: {e}")
                break

    def _parse_line(self, line: str):
        """Parsea el JSON recibido. Retorna (angles_list, gripper_float) o None."""
        # Chequear que corresponda a JSON
        if not line.startswith('{'):
            return None
        
        try:
            payload = json.loads(line)

            # Busca la key 'angles', 'angle' o 'joints'
            angles = payload.get("angles") or payload.get("angle") or payload.get("joints")
            gripper_raw = payload.get("gripper", 0.0)

            if isinstance(angles, list) and len(angles) == 6:
                return (angles, gripper_raw)
        
        except json.JSONDecodeError:
            return None
        return None

    def _publish_latest(self):
        """Publica el estado actual en ROS."""
        with self._lock:
            data = self._latest_data
        
        # Si todavía no hay datos
        if data is None:
            return

        angles_deg, gripper_raw = data

        # Convertir grados a radianes
        angles_rad = [math.radians(a) for a in angles_deg]

        # Conversión del gripper
        # Apertura      URDF (radianes)
        #    0              -0.7
        #   100              0.3
        griper_rad = (gripper_raw * 0.01) -0.7

        # Unir con los valores del brazo
        angles_rad.append(griper_rad)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = angles_rad
        
        # Arrays vacíos indican "no medido/no controlado" en ROS standard
        msg.velocity = []
        msg.effort = []

        self.pub.publish(msg)

def gestionar_publicadores_locales(activar_al_salir: bool):
    """
    Libera el tópico /joint_states tanto si está abierto demo.launch.py (MoveIt)
    como si está abierto move.launch.py (RViz base).
    """
    if not activar_al_salir:
        print("[Bridge] Liberando el tópico /joint_states en la PC...")
        
        # 1. Caso MoveIt (demo.launch.py): Desactivar joint_state_broadcaster
        try:
            subprocess.run(
                ["ros2", "control", "switch_controllers", "--deactivate", "joint_state_broadcaster"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2.5
            )
        except Exception:
            pass

        # 2. Caso RViz base (move.launch.py): Cerrar joint_state_publisher si está activo
        try:
            subprocess.run(
                ["pkill", "-f", "joint_state_publisher"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.5
            )
        except Exception:
            pass
    else:
        print("\n[Bridge] Restaurando joint_state_broadcaster (si MoveIt está activo)...")
        try:
            subprocess.run(
                ["ros2", "control", "switch_controllers", "--activate", "joint_state_broadcaster"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2.5
            )
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    # Al arrancar CobotClient, apaga los publicadores locales de MoveIt o RViz base
    gestionar_publicadores_locales(activar_al_salir=False)
    
    node = CobotClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al cerrar con Ctrl+C, vuelve a activar joint_state_broadcaster para MoveIt
        gestionar_publicadores_locales(activar_al_salir=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()