#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
import tkinter as tk
from sensor_msgs.msg import JointState
import socket
import threading
import time
import json
import math
import subprocess
import numpy as np
from scipy.interpolate import CubicSpline

# IP de la Raspberry Pi (el servidor)
ROBOT_IP = "192.168.88.10"
PORT = 65432
GRIPPER_SPEED = 30

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
    Nodo de ROS 2 que actúa como cliente TCP y Action Server de MoveIt.
    Se conecta al RobotServer (corriendo en la Raspberry Pi del cobot),
    recibe la telemetría en tiempo real (JSON) y publica en el tópico /joint_states.

    Además expone el servidor de acción '/arm_controller/follow_joint_trajectory'
    para recibir trayectorias calculadas por MoveIt en la PC y enviarlas por TCP
    a la Raspberry Pi para su ejecución en tiempo real.
    """
    def __init__(self):
        super().__init__('tcp_joint_state_client')

        # Publisher de ROS
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # EL SERVIDOR DE ACCIÓN (ESCUCHA A MOVEIT)
        # Este servidor espera recibir mensajes de trayectoria en este tópico
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_callback
        )

        # SERVIDOR DE ACCIÓN DE LA PINZA (ESCUCHA A MOVEIT Y RVIZ)
        self._gripper_action_server = ActionServer(
            self,
            GripperCommand,
            '/grip_action_controller/gripper_cmd',
            self.execute_gripper_callback
        )

        # ATAJOS RÁPIDOS DE ROS 2 ( TÓPICO PORCENTUAL Y SERVICIO BINARIO ABRIR/CERRAR )
        self.create_subscription(Float32, '/cobot/gripper_pct', self._gripper_pct_cb, 10)
        self.create_service(SetBool, '/cobot/set_gripper', self._gripper_srv_cb)

        # Variables compartidas
        self._lock = threading.Lock()
        self._latest_angles_deg = None
        self._latest_data = None
        self._latest_ts = None
        self._traj_status = "IDLE"
        self._active_sock = None

        # Iniciar el hilo del cliente
        self._sock_thread = threading.Thread(target=self._socket_client_loop, daemon=True)
        self._sock_thread.start()

        # Timer para publicar en ROS (30 Hz)
        self.create_timer(1.0/30.0, self._publish_latest)

        self.get_logger().info(f"Cliente Bridge iniciado. Intentando conectar a {ROBOT_IP}:{PORT}...")

    def _send_to_server(self, payload: dict) -> bool:
        """Envía un mensaje JSON al CobotServer de la Raspberry Pi a través del socket activo."""
        with self._lock:
            sock = self._active_sock
        if sock is None:
            self.get_logger().error("No hay conexión TCP activa con la Raspberry Pi.")
            return False
        try:
            message = json.dumps(payload).encode('utf-8') + b'\n'
            sock.sendall(message)
            return True
        except Exception as e:
            self.get_logger().error(f"Error enviando comando a la Raspberry Pi: {e}")
            return False

    # -------------------------------------------------------------------------
    # CONTROL DE PINZA (0 a 100%, Velocidad 30)
    # -------------------------------------------------------------------------

    def send_gripper_command(self, apertura_pct: float, speed: int = 30) -> bool:
        """
        Envía el comando de apertura de pinza al CobotServer en la Raspberry Pi.

        Si no hay conexión TCP activa con el robot físico (modo solo simulador),
        actualiza el estado local en memoria para que pueda probarse en RViz.

        Args:
            apertura_pct (float): Apertura deseada entre 0.0 (cerrado) y 100.0 (abierto).
            speed (int, optional): Velocidad del movimiento (1-100). Defaults to 30.

        Returns:
            bool: True si se envió al servidor o se actualizó en simulación local.
        """
        val = max(0.0, min(100.0, float(apertura_pct)))
        spd = max(1, min(100, int(speed)))

        with self._lock:
            connected = self._active_sock is not None

        if connected:
            self.get_logger().info(f"Enviando orden de pinza a Raspi: {val:.0f}% ( spd={spd} )")
            return self._send_to_server({"cmd": "SET_GRIPPER", "value": val, "speed": spd})
        else:
            self.get_logger().warn(f"Sin conexión a Raspi. Simulando pinza al {val:.0f}% en RViz.")
            with self._lock:
                angles = self._latest_data[0] if self._latest_data is not None else [0.0] * 6
                self._latest_data = (angles, val)
            return True

    def _gripper_pct_cb(self, msg: Float32):
        """Callback del tópico '/cobot/gripper_pct' para control directo de 0 a 100%."""
        self.send_gripper_command(msg.data, speed=30)

    def _gripper_srv_cb(self, request: SetBool.Request, response: SetBool.Response):
        """
        Servicio estándar '/cobot/set_gripper' (std_srvs/srv/SetBool).
        data=True -> Abre la pinza al 100%.
        data=False -> Cierra la pinza al 0%.
        """
        target_pct = 100.0 if request.data else 0.0
        ok = self.send_gripper_command(target_pct, speed=30)
        response.success = ok
        response.message = f"Pinza comandada al {target_pct:.0f}% (vel 30)"
        return response

    def send_gripper_command(self, apertura_pct: float, speed: int = 30) -> bool:
        """
        Envía el comando de apertura de pinza al CobotServer en la Raspberry Pi.

        Si no hay conexión TCP activa con el robot físico (modo solo simulador),
        actualiza el estado local en memoria para que pueda probarse en RViz.

        Args:
            apertura_pct (float): Apertura deseada entre 0.0 (cerrado) y 100.0 (abierto).
            speed (int, optional): Velocidad del movimiento (1-100). Defaults to 30.

        Returns:
            bool: True si se envió al servidor o se actualizó en simulación local.
        """
        val = max(0.0, min(100.0, float(apertura_pct)))
        spd = max(1, min(100, int(speed)))

        with self._lock:
            connected = self._active_sock is not None

        if connected:
            self.get_logger().info(f"Enviando orden de pinza a Raspi: {val:.0f}% ( spd={spd} )")
            return self._send_to_server({"cmd": "SET_GRIPPER", "value": val, "speed": spd})
        else:
            self.get_logger().warn(f"Sin conexión a Raspi. Simulando pinza al {val:.0f}% en RViz.")
            with self._lock:
                angles = self._latest_data[0] if self._latest_data is not None else [0.0] * 6
                self._latest_data = (angles, val)
            return True

    def _gripper_pct_cb(self, msg: Float32):
        """Callback del tópico '/cobot/gripper_pct' para control directo de 0 a 100%."""
        self.send_gripper_command(msg.data, speed=30)

    def _gripper_srv_cb(self, request: SetBool.Request, response: SetBool.Response):
        """
        Servicio estándar '/cobot/set_gripper' (std_srvs/srv/SetBool).
        data=True -> Abre la pinza al 100%.
        data=False -> Cierra la pinza al 0%.
        """
        target_pct = 100.0 if request.data else 0.0
        ok = self.send_gripper_command(target_pct, speed=30)
        response.success = ok
        response.message = f"Pinza comandada al {target_pct:.0f}% (vel 30)"
        return response

    def execute_gripper_callback(self, goal_handle):
        """
        Callback del ActionServer '/grip_action_controller/gripper_cmd'.
        Recibe la posición en radianes desde MoveIt/RViz (-0.7 a 0.3 rad),
        la convierte a escala porcentual (0 a 100%) y la ejecuta en el cobot real.
        """
        pos_rad = float(goal_handle.request.command.position)
        # Mapeo inverso al de _publish_latest: (-0.7 rad -> 0%, 0.3 rad -> 100%)
        apertura_pct = max(0.0, min(100.0, (pos_rad + 0.7) * 100.0))

        self.get_logger().info(
            f"¡Acción de pinza recibida de MoveIt! ({pos_rad:.3f} rad -> {apertura_pct:.0f}%)"
        )

        ok = self.send_gripper_command(apertura_pct, speed=30)
        result = GripperCommand.Result()

        if not ok:
            goal_handle.abort()
            return result

        # Tiempo de asentamiento mecánico para que la pinza física complete el recorrido
        time.sleep(0.5)
        result.position = pos_rad
        result.reached_goal = True
        goal_handle.succeed()
        return result

    # -------------------------------------------------------------------------
    # CONTROL DE TRAYECTORIAS DEL BRAZO (MoveIt -> Raspberry Pi)
    # ------------------------------------------------------------------------

    def _interpolate_trajectory(self, points, idx_map, freq_hz: float = 25.0, max_servo_deg_s: float = 140.0):
        """
        Re-muestrea los waypoints de MoveIt a frecuencia constante (25 Hz) usando CubicSpline
        y calcula la velocidad dinámica (1-100) de cada tramo según la derivada articular.
        """
        t_orig = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points], dtype=float)
        q_orig_deg = np.array([[math.degrees(p.positions[i]) for i in idx_map] for p in points], dtype=float)

        # Filtrar posibles puntos con tiempos duplicados
        _, unique_idx = np.unique(t_orig, return_index=True)
        t_orig = t_orig[unique_idx]
        q_orig_deg = q_orig_deg[unique_idx]

        total_time = float(t_orig[-1])
        if len(t_orig) < 2 or total_time <= 0.05:
            q_end = [round(float(x), 3) for x in q_orig_deg[-1]]
            return [q_end + [20, round(total_time, 4)]]

        # Ajustar Spline Cúbico con velocidad inicial y final nula (clamped)
        cs = CubicSpline(t_orig, q_orig_deg, axis=0, bc_type='clamped')

        dt = 1.0 / freq_hz
        t_interp = np.arange(0.0, total_time, dt)
        if t_interp[-1] < total_time:
            t_interp = np.append(t_interp, total_time)

        q_interp = cs(t_interp)
        # Derivada primera: velocidad angular real en grados/s para cada articulación
        dq_interp = np.abs(cs(t_interp, 1))
        max_vel_deg_s = np.max(dq_interp, axis=1)

        points_payload = []
        for i in range(len(t_interp)):
            # Mapear la velocidad angular requerida a la escala 1..100 de pymycobot
            spd = int(np.clip(round((max_vel_deg_s[i] / max_servo_deg_s) * 100.0), 6, 100))
            q_list = [round(float(val), 3) for val in q_interp[i]]
            points_payload.append(q_list + [spd, round(float(t_interp[i]), 4)])

        return points_payload

    def execute_callback(self, goal_handle):
        """Esta función se activa cuando MoveIt manda una trayectoria."""
        self.get_logger().info('¡Trayectoria recibida de MoveIt! Interpolando y enviando a la Raspberry Pi...')

        # Extraemos la lista de puntos calculados por MoveIt
        points = goal_handle.request.trajectory.points
        traj_joint_names = goal_handle.request.trajectory.joint_names

        if not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Asegurar el orden correcto de las 6 articulaciones del brazo según URDF
        arm_joints = JOINT_NAMES[:6]
        try:
            idx_map = [traj_joint_names.index(jn) for jn in arm_joints]
        except ValueError:
            idx_map = list(range(6))

        # Interpolar a 25 Hz con velocidades dinámicas suaves
        points_payload = self._interpolate_trajectory(points, idx_map, freq_hz=25.0, max_servo_deg_s=140.0)

        with self._lock:
            self._traj_status = "EXECUTING"

        # Enviar la trayectoria interpolada al CobotServer en la Raspberry Pi
        if not self._send_to_server({"cmd": "EXECUTE_TRAJ", "points": points_payload}):
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # BUCLE DE MONITOREO MIENTRAS LA RASPBERRY PI EJECUTA LA TRAYECTORIA
        while rclpy.ok():
            # Si MoveIt detecta un choque inminente o el usuario cancela la acción
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('¡PARADA DE EMERGENCIA solicitada por MoveIt!')
                self._send_to_server({"cmd": "CANCEL_TRAJ"})
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            with self._lock:
                status = self._traj_status

            if status == "SUCCEEDED":
                self._send_to_server({"cmd": "RESET_STATUS"})
                goal_handle.succeed()
                self.get_logger().info('Movimiento completado con éxito.')
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            elif status in ("ABORTED", "CANCELED"):
                self._send_to_server({"cmd": "RESET_STATUS"})
                goal_handle.abort()
                self.get_logger().error(f'Movimiento finalizado con estado: {status}')
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result

            time.sleep(0.01)


    # -------------------------------------------------------------------------
    # RECEPCIÓN TCP Y PUBLICACIÓN EN /joint_states
    # -------------------------------------------------------------------------

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
                    
                    with self._lock:
                        self._active_sock = s

                    self.get_logger().info("¡Conexión establecida con el Robot!")
                    
                    # Bucle de lectura mientras la conexión esté viva
                    self._read_from_socket(s)
                    
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                self.get_logger().warn(f"No se pudo conectar o se perdió conexión: {e}")
            finally:
                with self._lock:
                    self._active_sock = None
            
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
                        angles, gripper_raw, traj_status = parsed
                        with self._lock:
                            self._latest_data = (angles, gripper_raw)
                            self._latest_ts = time.time()
                            if traj_status is not None:
                                self._traj_status = traj_status
                            
            except socket.timeout:
                # Si el robot no envía nada por un momento, seguir esperando
                continue
            except Exception as e:
                self.get_logger().error(f"Error leyendo datos: {e}")
                break

    def _parse_line(self, line: str):
        """Parsea el JSON recibido. Retorna (angles_list, gripper_float, traj_status) o None."""
        # Chequear que corresponda a JSON
        if not line.startswith('{'):
            return None
        
        try:
            payload = json.loads(line)

            # Busca la key 'angles', 'angle' o 'joints'
            angles = payload.get("angles") or payload.get("angle") or payload.get("joints")
            gripper_raw = payload.get("gripper", 0.0)
            traj_status = payload.get("traj_status", None)

            if isinstance(angles, list) and len(angles) == 6:
                return (angles, gripper_raw, traj_status)
        
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
        griper_rad = (gripper_raw * 0.01) - 0.7

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
    Libera el tópico /joint_states y el servidor de acción /arm_controller/follow_joint_trajectory
    tanto si está abierto demo.launch.py (MoveIt) como si está abierto move.launch.py (RViz base).
    """
    if not activar_al_salir:
        print("[Bridge] Liberando el tópico /joint_states y /arm_controller simulados en la PC...")
        
        # 1. Caso MoveIt (demo.launch.py): Desactivar joint_state_broadcaster y arm_controller
        for ctrl in ["joint_state_broadcaster", "arm_controller","grip_action_controller"]:
            try:
                subprocess.run(
                    ["ros2", "control", "switch_controllers", "--deactivate", ctrl],
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
        print("\n[Bridge] Restaurando joint_state_broadcaster y arm_controller (si MoveIt está activo)...")
        for ctrl in ["joint_state_broadcaster", "arm_controller","grip_action_controller"]:
            try:
                subprocess.run(
                    ["ros2", "control", "switch_controllers", "--activate", ctrl],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=2.5
                )
            except Exception:
                pass

def bucle_teclado_pinza(client_node: CobotClient):
    """
    Atajo interactivo por consola para abrir/cerrar la pinza en tiempo real
    mientras se planifican movimientos del brazo en RViz2.

    Comandos admitidos:
    - 'o' / 'a' + ENTER -> Abre la pinza completamente (100%) a velocidad 30.
    - 'c'       + ENTER -> Cierra la pinza completamente (0%) a velocidad 30.
    - '0'..'100'+ ENTER -> Mueve la pinza al porcentaje indicado a velocidad 30.
    - 'q'       + ENTER -> Cierra CobotClient de forma limpia.
    """
    print("\n" + "=" * 62)
    print(" ATAJO DE PINZA ACTIVO (Velocidad fija: 30)")
    print("  [o / a] + ENTER -> ABRIR pinza del todo (100%)")
    print("  [c]     + ENTER -> CERRAR pinza del todo (0%)")
    print("  [0-100] + ENTER -> Apertura parcial (ej: 50)")
    print("  [q]     + ENTER -> Salir y restaurar controladores")
    print("=" * 62 + "\n")

    while rclpy.ok():
        try:
            cmd = input().strip().lower()
            if not cmd:
                continue
            if cmd in ('o', 'a', 'open', 'abrir'):
                client_node.send_gripper_command(100.0, speed=GRIPPER_SPEED)
            elif cmd in ('c', 'close', 'cerrar'):
                client_node.send_gripper_command(0.0, speed=GRIPPER_SPEED)
            elif cmd in ('q', 'quit', 'exit', 'salir'):
                print("Saliendo de CobotClient...")
                break
            else:
                try:
                    val = float(cmd)
                    if 0.0 <= val <= 100.0:
                        client_node.send_gripper_command(val, speed=GRIPPER_SPEED)
                    else:
                        print("[!] Ingresa un valor entre 0 y 100, 'o' (abrir) o 'c' (cerrar).")
                except ValueError:
                    print("[!] Comando no reconocido. Usa 'o' (100%), 'c' (0%), un número 0-100 o 'q'.")
        except EOFError:
            # Si se ejecuta en segundo plano sin stdin interactivo, mantener vivo el proceso
            while rclpy.ok():
                time.sleep(0.5)
            break

    

def main(args=None):
    rclpy.init(args=args)
    
    # Al arrancar CobotClient, apaga los publicadores locales de MoveIt o RViz base
    gestionar_publicadores_locales(activar_al_salir=False)
    
    node = CobotClient()

    # Ejecutamos MultiThreadedExecutor en un hilo daemon para que ROS 2 procese
    # /joint_states y ActionServers en paralelo con la lectura de teclado
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    
    try:
        bucle_teclado_pinza(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al cerrar con Ctrl+C, vuelve a activar joint_state_broadcaster y arm_controller para MoveIt
        gestionar_publicadores_locales(activar_al_salir=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()