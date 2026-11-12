#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
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
        self._traj_metrics = None
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

    def execute_gripper_callback(self, goal_handle):
        """
        Callback del ActionServer '/grip_action_controller/gripper_cmd'.
        Envía el comando a la Raspberry Pi, espera la lectura real del encoder de la pinza
        y reporta la posición real alcanzada (detectando si un objeto detuvo el cierre).
        """
        pos_rad = float(goal_handle.request.command.position)
        apertura_pct = max(0.0, min(100.0, (pos_rad + 0.7) * 100.0))

        self.get_logger().info(
            f"¡Acción de pinza recibida de MoveIt! ({pos_rad:.3f} rad -> {apertura_pct:.0f}%)"
        )

        ok = self.send_gripper_command(apertura_pct, speed=30)
        result = GripperCommand.Result()

        if not ok:
            goal_handle.abort()
            return result

        # Esperar a que CobotServer ejecute el movimiento, mida get_gripper_value() y actualice el JSON
        time.sleep(0.65)
        with self._lock:
            real_pct = float(self._latest_data[1]) if self._latest_data is not None else apertura_pct

        real_rad = (real_pct * 0.01) - 0.7
        if apertura_pct <= 10.0 and real_pct >= 12.0:
            self.get_logger().info(
                f"[Gripper] ¡Objeto detectado entre los dedos! "
                f"Comando: {apertura_pct:.0f}% | Apertura real retenida: {real_pct:.0f}% ({real_rad:.3f} rad)"
            )

        result.position = real_rad
        result.reached_goal = True
        goal_handle.succeed()
        return result

    # -------------------------------------------------------------------------
    # CONTROL DE TRAYECTORIAS DEL BRAZO (MoveIt -> Raspberry Pi)
    # ------------------------------------------------------------------------

    def _interpolate_trajectory(self, points, idx_map, freq_hz: float = 25.0,
                                max_servo_deg_s: float = 150.0, lead_gain: float = 1.10,
                                min_speed: int = 6, max_speed: int = 95):
        """
        Re-muestrea los waypoints de MoveIt a 25 Hz usando CubicSpline, calcula la velocidad
        dinámica (6-95) con cola anti-caída y estima el factor de escala de aceleración (acc_scale)
        para ajustar automáticamente el registro 0x29 en la SRAM de los servos.
        """
        t_orig = []
        q_orig_deg = []
        for p in points:
            t_sec = float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9
            if len(t_orig) > 0 and t_sec <= t_orig[-1]:
                continue
            t_orig.append(t_sec)
            q_orig_deg.append([math.degrees(p.positions[i]) for i in idx_map])

        t_orig = np.asarray(t_orig, dtype=float)
        q_orig_deg = np.asarray(q_orig_deg, dtype=float)

        total_time = float(t_orig[-1])
        if len(t_orig) < 2 or total_time <= 0.05:
            q_end = [round(float(x), 2) for x in q_orig_deg[-1]]
            return [q_end + [15, round(total_time, 4)]], 0.30

        cs = CubicSpline(t_orig, q_orig_deg, axis=0, bc_type='clamped')

        dt = 1.0 / float(freq_hz)
        t_interp = np.arange(0.0, total_time, dt)
        if not np.isclose(t_interp[-1], total_time):
            t_interp = np.append(t_interp, total_time)

        q_interp_deg = cs(t_interp)
        vel_inst_deg_s = np.abs(cs(t_interp, 1))
        acc_inst_rad_s2 = np.deg2rad(np.abs(cs(t_interp, 2)))

        # Mejora 1: Estimar el acc_scale real de la trayectoria respecto a joint_limits.yaml
        max_acc_yaml = np.array([3.00, 2.50, 3.00, 3.50, 4.00, 4.00], dtype=float)
        peak_acc_per_joint = np.max(acc_inst_rad_s2, axis=0)
        est_acc_scale = float(np.clip(np.max(peak_acc_per_joint / max_acc_yaml), 0.15, 1.00))

        n_pts = len(t_interp)
        points_payload = []
        for k in range(n_pts):
            v_inst_max = float(np.max(vel_inst_deg_s[k]))
            if k > 0:
                dt_step = max(1e-4, float(t_interp[k] - t_interp[k - 1]))
                dq_step_deg = np.abs(q_interp_deg[k] - q_interp_deg[k - 1])
                v_sec_max = float(np.max(dq_step_deg) / dt_step)
            else:
                v_sec_max = v_inst_max

            v_req_deg_s = max(v_inst_max, v_sec_max) * float(lead_gain)
            raw_pct = (v_req_deg_s / float(max_servo_deg_s)) * 100.0
            spd = int(np.clip(round(raw_pct), min_speed, max_speed))

            q_list = [round(float(val), 2) for val in q_interp_deg[k]]
            points_payload.append(q_list + [spd, round(float(t_interp[k]), 4)])

        return points_payload, round(est_acc_scale, 2)

    def execute_callback(self, goal_handle):
        """
        Recibe la trayectoria desde MoveIt, la interpola a 25 Hz, la envía con el acc_scale
        estimado a CobotServer, publica Feedback de ROS 2 en vivo e informa las métricas finales.
        """
        from builtin_interfaces.msg import Duration

        self.get_logger().info('¡Trayectoria recibida de MoveIt! Interpolando y enviando a la Raspberry Pi...')

        points = goal_handle.request.trajectory.points
        traj_joint_names = goal_handle.request.trajectory.joint_names

        if not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        arm_joints = JOINT_NAMES[:6]
        try:
            idx_map = [traj_joint_names.index(jn) for jn in arm_joints]
        except ValueError:
            idx_map = list(range(6))

        # Interpolar a 25 Hz y obtener también el acc_scale estimado de esta trayectoria
        points_payload, est_acc_scale = self._interpolate_trajectory(
            points, idx_map, freq_hz=25.0, max_servo_deg_s=150.0
        )

        with self._lock:
            self._traj_status = "EXECUTING"
            self._traj_metrics = None

        # Enviar la trayectoria y la escala de aceleración al CobotServer en la Raspberry Pi
        if not self._send_to_server({
            "cmd": "EXECUTE_TRAJ",
            "points": points_payload,
            "acc_scale": est_acc_scale
        }):
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        t_exec_start = time.perf_counter()
        last_fb_time = 0.0

        # BUCLE DE MONITOREO Y PUBLICACIÓN DE FEEDBACK EN TIEMPO REAL
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('¡PARADA DE EMERGENCIA solicitada por MoveIt!')
                self._send_to_server({"cmd": "CANCEL_TRAJ"})
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            with self._lock:
                status = self._traj_status
                metrics = self._traj_metrics
                latest_data = self._latest_data

            # Mejora 3: Publicar Feedback de FollowJointTrajectory a 20 Hz (cada 0.05 s)
            elapsed = time.perf_counter() - t_exec_start
            if (elapsed - last_fb_time) >= 0.05 and latest_data is not None:
                last_fb_time = elapsed
                actual_deg = latest_data[0]
                actual_rad = [math.radians(a) for a in actual_deg]

                # Buscar el waypoint deseado correspondiente al tiempo transcurrido
                des_pt = points_payload[-1]
                for wp in points_payload:
                    if wp[7] >= elapsed:
                        des_pt = wp
                        break
                desired_rad = [math.radians(a) for a in des_pt[:6]]
                error_rad = [d - a for d, a in zip(desired_rad, actual_rad)]

                fb = FollowJointTrajectory.Feedback()
                fb.header.stamp = self.get_clock().now().to_msg()
                fb.joint_names = arm_joints
                fb.desired.positions = desired_rad
                fb.actual.positions = actual_rad
                fb.error.positions = error_rad
                sec = int(elapsed)
                fb.actual.time_from_start = Duration(sec=sec, nanosec=int((elapsed - sec) * 1e9))
                goal_handle.publish_feedback(fb)

            if status == "SUCCEEDED":
                self._send_to_server({"cmd": "RESET_STATUS"})
                goal_handle.succeed()

                # Mejora 2: Mostrar las métricas cuantitativas reportadas por los encoders de la Raspi
                if metrics is not None and metrics.get("max_err_deg") is not None:
                    self.get_logger().info(
                        f"[Métricas MoveIt Online] Tiempo: {metrics['t_real']:.2f}s "
                        f"(Plan: {metrics['t_plan']:.2f}s, Diff: {metrics['t_diff']:+.3f}s) | "
                        f"Max Err Articular: {metrics['max_err_deg']:.2f}° | "
                        f"Err/Eje (°): {metrics['err_per_joint_deg']}"
                    )
                else:
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
                    break
                
                buffer += data.decode('utf-8', errors='replace')
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    
                    parsed = self._parse_line(line)
                    if parsed is not None:
                        angles, gripper_raw, traj_status, traj_metrics = parsed
                        with self._lock:
                            self._latest_data = (angles, gripper_raw)
                            self._latest_ts = time.time()
                            if traj_status is not None:
                                self._traj_status = traj_status
                            if traj_metrics is not None:
                                self._traj_metrics = traj_metrics
                            
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Error leyendo datos: {e}")
                break

    def _parse_line(self, line: str):
        """Parsea el JSON recibido. Retorna (angles_list, gripper_float, traj_status, traj_metrics) o None."""
        if not line.startswith('{'):
            return None
        
        try:
            payload = json.loads(line)
            angles = payload.get("angles") or payload.get("angle") or payload.get("joints")
            gripper_raw = payload.get("gripper", 0.0)
            traj_status = payload.get("traj_status", None)
            traj_metrics = payload.get("traj_metrics", None)

            if isinstance(angles, list) and len(angles) == 6:
                return (angles, gripper_raw, traj_status, traj_metrics)
        
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