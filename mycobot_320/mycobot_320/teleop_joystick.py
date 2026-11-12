import time
import threading
import numpy as np
from spatialmath import SE3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Header
from sensor_msgs.msg import Joy, JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import ApplyPlanningScene

from Cobot_sdk import *
from DHRobotGT import IKineError
from CobotStudio import joint_names, ROSManager, MyCobotController


class JoyStickTeleop(Node):
    def __init__(self, robot: BaseRobotController, speed_default: int = 30):
        super().__init__('gamesir_teleop_node')
        self.robot = robot
        self.es_ros = isinstance(robot, ROSManager)
        self.es_real = isinstance(robot, MyCobotController)
        self.speed_default = int(np.clip(speed_default, 10, 80))

        # =====================================================================
        # CONFIGURACIÓN DE ALINEACIÓN CON LA MESA Y SENTIDOS DEL STICK
        # =====================================================================
        # Si aún notas una leve inclinación respecto al borde de tu mesa en RViz,
        # cambia este valor (ej. -30.0, -37.0, 53.0 o -60.0):
        self.ANGULO_MESA_DEG = -37.0  # Alineado con el RPY(180, 0, 53°) -> 53° - 90° = -37°

        # Si Izquierda/Derecha o Adelante/Atrás te quedan invertidos a tu vista,
        # cambia aquí +1.0 por -1.0:
        self.SIGNO_ADELANTE =  1.0
        self.SIGNO_LATERAL  = -1.0   # Invertido para que Izquierda mueva a la izquierda
        self.SIGNO_Z        =  1.0

        # Modos de referencia conmutables con el Botón X:
        # 0 = Alineado con la Mesa (-37°) | 1 = Alineado con el Brazo/Pinza | 2 = Base (0°)
        self.modo_referencia = 0
        self.nombres_referencia = [
            f"MESA ({self.ANGULO_MESA_DEG}°)",
            "VISTA DEL BRAZO / PINZA (Dinámico)",
            "BASE DEL ROBOT (0°)"
        ]

        # 1. En ROS desactivamos joint_state_broadcaster para que FakeSystem no devuelva el robot a Home
        if self.es_ros and hasattr(self.robot, "toggle_ros_connection"):
            self.robot.toggle_ros_connection(False)

        # 2. En Real activamos fresh_mode(1) para movimiento continuo sin frenar entre pasos
        if self.es_real and hasattr(self.robot, "mc") and self.robot.mc is not None:
            try:
                self.robot.mc.set_fresh_mode(1)
            except Exception:
                pass
            if getattr(self.robot, "_server_ref", None) is not None:
                self.robot._server_ref.pause_serial_reading(True)

        # 3. Postura inicial con la PINZA APUNTANDO HACIA ABAJO
        self.pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
        T_pinza_abajo = SE3(250.0, -140.0, 268.0) * SE3.RPY([180.0, 0.0, 53.0], unit='deg')
        T_brida_abajo = T_pinza_abajo * self.pinza.inv()

        try:
            self.q_pose_segura = np.asarray(self.robot.cobot_tb.ikine(T_brida_abajo, [1, -1, 1])[0], dtype=float)[:6]
        except Exception:
            self.q_pose_segura = np.deg2rad([-20.1, -45.0, -30.0, -15.0, 90.0, 0.0])

        q_leido = self._leer_q_inicial_rad()

        if np.linalg.norm(q_leido) < 0.05:
            print("[*] Posicionando robot con la pinza apuntando hacia abajo...")
            self.q_robot_rad = self.q_pose_segura.copy()
            if self.es_real:
                self.robot.MoveJ_q(self.q_robot_rad, spd=25, unit='rad')
        else:
            self.q_robot_rad = q_leido.copy()

        self.q_goal_rad = self.q_robot_rad.copy()
        self.pose_actual = self.robot.cobot_tb.fkine(self.q_goal_rad)
        self.config_actual = self.robot.cobot_tb.calc_conf(self.q_goal_rad)
        self.gripper_val_rad = -0.2

        self.modo_robot_en_vivo = False
        self.deadzone = 0.15
        self.ultimo_joy = None
        self.buttons_prev = []
        self.pose_counter = 1
        self._lock_accion = threading.Lock()
        self.last_goal_srv_time = 0.0

        # Publicar estado inicial y mostrar también la terna de la mesa en RViz para guiar la vista
        if self.es_ros:
            self.robot.node_tf.add_robt(self.pinza, 'tool', 'link6')
            self.robot.node_tf.add_wobj(SE3(250.0, -140.0, 0.0) * SE3.Rz(self.ANGULO_MESA_DEG, unit='deg'), 'ejes_mesa_joy')
            self._publicar_joint_state_rviz(self.q_robot_rad)
            self._actualizar_goal_naranja_async(self.q_goal_rad, forzar_rviz=True)

        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.04, self.control_loop)  # 25 Hz (40 ms)

        self._imprimir_instrucciones()

    def _imprimir_instrucciones(self):
        modo_txt = "ROBOT EN VIVO" if self.modo_robot_en_vivo else "GOAL NARANJA (MoveIt)"
        entorno_txt = "SIMULACIÓN ROS / MOVEIT" if self.es_ros else "ROBOT REAL (HARDWARE)"
        ref_txt = self.nombres_referencia[self.modo_referencia]
        print("\n" + "=" * 74)
        print(f"  GAMESIR NOVA LITE ACTIVO | Entorno: {entorno_txt}")
        print(f"  Modo: [{modo_txt}] | Referencia XY: [{ref_txt}]")
        print("-" * 74)
        print("  - Stick Izquierdo (axes 0,1) : Mover Recto en la Mesa (Adelante/Atrás/Lados)")
        print("  - Stick Derecho   (axes 3)   : Subir / Bajar en Z PURO (Vertical recto)")
        print("  - Stick Derecho   (axes 2)   : Rotar pinza en Yaw (RZ)")
        print("  - Cruceta D-Pad   (axes 6,7) : Inclinar pinza en Roll (RX) y Pitch (RY)")
        print("  - Botón X (idx 3 o 2)        : CAMBIAR REFERENCIA XY (Mesa / Brazo / Base)")
        print("  - Botón Y (idx 4 o 3)        : Alternar [GOAL NARANJA] <-> [ROBOT EN VIVO]")
        print("  - Botón A (idx 0)            : Guardar Pose en 'Poses/poses_joystick.py'")
        print("  - Botón B (idx 1)            : Ejecutar Plan & Execute en MoveIt hacia el Goal")
        print("  - Botones LB (6) / RB (7)    : Abrir Pinza (100%) / Cerrar Pinza (0%)")
        print("  - Botón Select (10)          : Volver a pose con pinza apuntando hacia abajo")
        print("=" * 74 + "\n")

    def _leer_q_inicial_rad(self) -> np.ndarray:
        if self.es_ros:
            q = self.robot.get_current_q(prefer_gripper=False, timeout=1.0)
            if q is not None and len(q) >= 6:
                return np.asarray(q[:6], dtype=float)
        elif self.es_real and hasattr(self.robot, "mc"):
            ang = self.robot.mc.get_angles()
            if ang is not None and len(ang) >= 6:
                return np.deg2rad(np.asarray(ang[:6], dtype=float))
        return np.zeros(6, dtype=float)

    def filtrar_dz(self, val: float) -> float:
        return float(val) if abs(val) > self.deadzone else 0.0

    def _publicar_joint_state_rviz(self, q_rad_6: np.ndarray):
        if not self.es_ros:
            return
        q_full = np.append(q_rad_6[:6], self.gripper_val_rad)
        msg = JointState()
        msg.header.stamp = self.robot.node_joint.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = q_full.tolist()
        self.robot.node_joint.publisher.publish(msg)
        self.robot.q_current = q_full.copy()

    def _actualizar_goal_naranja_async(self, q_rad_6: np.ndarray, forzar_rviz: bool = False):
        if not self.es_ros or self.robot.moveit_manager is None:
            return

        T_goal_tcp = self.robot.cobot_tb.fkine(q_rad_6[:6]) * self.pinza
        self.robot.node_tf.add_robt(T_goal_tcp, 'robt_joystick', 'base')

        now = time.time()
        if not forzar_rviz and (now - self.last_goal_srv_time) < 0.12:
            return
        self.last_goal_srv_time = now

        mm = self.robot.moveit_manager
        if not mm._apply_cli.service_is_ready():
            return

        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        rs = RobotState()
        js = JointState()
        js.header = Header()
        js.header.stamp = mm.get_clock().now().to_msg()
        js.name = mm.joint_names_full[:6]
        js.position = q_rad_6[:6].tolist()
        rs.joint_state = js
        req.scene.robot_state = rs

        future = mm._apply_cli.call_async(req)
        future.add_done_callback(lambda _fut: mm._pub_update.publish(Empty()))

    def _guardar_pose_actual(self):
        q_target = self.q_robot_rad if self.modo_robot_en_vivo else self.q_goal_rad
        q_deg = np.round(np.rad2deg(q_target[:6]), 2)
        q_rad = np.round(q_target[:6], 6)
        T_brida = self.robot.cobot_tb.fkine(q_target[:6])
        T_tcp = T_brida * self.pinza
        conf = self.robot.cobot_tb.calc_conf(q_target[:6])
        xyz = np.round(T_tcp.t, 2)
        rpy = np.round(T_tcp.rpy(order='zyx', unit='deg'), 2)

        nombre_var = f"Pose_{self.pose_counter}"
        self.pose_counter += 1

        try:
            self.robot._save_se3_definition(
                folder="Poses",
                filename="poses_joystick",
                var_name=nombre_var,
                se3_obj=T_tcp,
                q_vals=[q_deg.tolist()]
            )
        except Exception as e:
            print(f"[!] Advertencia al guardar en archivo: {e}")

        print("\n" + "=" * 68)
        print(f"  POSE GUARDADA COMO '{nombre_var}' EN Poses/poses_joystick.py")
        print(f"  TCP XYZ (mm) : [{xyz[0]}, {xyz[1]}, {xyz[2]}] | RPY (°): [{rpy[0]}, {rpy[1]}, {rpy[2]}]")
        print(f"  Config       : {conf}")
        print(f"  q_deg        = {q_deg.tolist()}")
        print(f"  q_rad        = {q_rad.tolist()}")
        print(f"  RobTarget    = RobTarget(SE3({xyz[0]}, {xyz[1]}, {xyz[2]}) * SE3.RPY({rpy.tolist()}, unit='deg'), {conf})")
        print("=" * 68 + "\n")

    def _ejecutar_plan_moveit(self):
        if not self.es_ros or self.robot.moveit_manager is None:
            print("[!] Plan & Execute de MoveIt solo aplica en modo 'ros'.")
            return

        print("\n[MoveIt] >>> Planificando y ejecutando trayectoria hacia el Goal Naranja...")
        self.robot.toggle_ros_connection(True)
        time.sleep(0.15)

        q_plan = self.robot.moveit_manager.plan_and_execute(
            target=self.q_goal_rad[:6].tolist(),
            start=self.q_robot_rad[:6].tolist(),
            execute=True,
            update_rviz=False
        )

        self.robot.toggle_ros_connection(False)
        if q_plan is not None and len(q_plan) > 0:
            self.q_robot_rad = np.asarray(q_plan[-1][:6], dtype=float)
            self.q_goal_rad = self.q_robot_rad.copy()
            self.pose_actual = self.robot.cobot_tb.fkine(self.q_goal_rad)
            self.config_actual = self.robot.cobot_tb.calc_conf(self.q_goal_rad)
            self._publicar_joint_state_rviz(self.q_robot_rad)
            print("[MoveIt] >>> Trayectoria ejecutada con éxito.")
        else:
            print("[MoveIt] [!] Falló la planificación hacia ese punto.")

    def _accion_pinza(self, apertura_pct: int):
        self.gripper_val_rad = (float(apertura_pct) / 100.0) - 0.7

        def _worker():
            with self._lock_accion:
                try:
                    if self.es_real:
                        self.robot.GripperState(apertura_pct, 30)
                        if hasattr(self.robot, "mc"):
                            self.robot.mc.set_fresh_mode(1)
                        if getattr(self.robot, "_server_ref", None) is not None:
                            self.robot._server_ref.pause_serial_reading(True)
                    elif self.es_ros and self.robot.moveit_manager is not None:
                        self.robot.moveit_manager.move_gripper(apertura_pct, wait_for_result=False)
                except Exception as e:
                    print(f"[Pinza] Error: {e}")

        threading.Thread(target=_worker, daemon=True).start()
        estado_p = "ABIERTA (100%)" if apertura_pct >= 50 else "CERRADA (0%)"
        print(f"[JoyStick] Pinza -> {estado_p}")

    def joy_callback(self, msg: Joy):
        self.ultimo_joy = msg
        btns = list(msg.buttons)
        if not self.buttons_prev:
            self.buttons_prev = [0] * len(btns)

        es_layout_19 = (len(btns) >= 15)
        idx_x   = 3 if es_layout_19 else 2
        idx_y   = 4 if es_layout_19 else 3
        idx_lb  = 6 if es_layout_19 else 4
        idx_rb  = 7 if es_layout_19 else 5
        idx_sel = 10 if es_layout_19 else 6

        for idx, (curr, prev) in enumerate(zip(btns, self.buttons_prev)):
            if curr == 1 and prev == 0:
                if idx == 0:
                    self._guardar_pose_actual()
                elif idx == 1:
                    threading.Thread(target=self._ejecutar_plan_moveit, daemon=True).start()
                elif idx == idx_x:
                    # Botón X: Alterna entre referencia de la Mesa, del Brazo o de la Base
                    self.modo_referencia = (self.modo_referencia + 1) % 3
                    print(f"\n[JoyStick] >>> Referencia XY cambiada a: [{self.nombres_referencia[self.modo_referencia]}]")
                elif idx == idx_y:
                    self.modo_robot_en_vivo = not self.modo_robot_en_vivo
                    if self.modo_robot_en_vivo:
                        self.q_goal_rad = self.q_robot_rad.copy()
                        self.pose_actual = self.robot.cobot_tb.fkine(self.q_goal_rad)
                        self.config_actual = self.robot.cobot_tb.calc_conf(self.q_goal_rad)
                        print("\n[JoyStick] >>> MODO: MOVIENDO ROBOT EN VIVO")
                    else:
                        print("\n[JoyStick] >>> MODO: SOLO GOAL STATE (NARANJA EN MOVEIT)")
                elif idx == idx_lb:
                    self._accion_pinza(100)
                elif idx == idx_rb:
                    self._accion_pinza(0)
                elif idx == idx_sel:
                    print("\n[JoyStick] Volviendo a postura con pinza hacia abajo...")
                    self.q_goal_rad = self.q_pose_segura.copy()
                    self.pose_actual = self.robot.cobot_tb.fkine(self.q_goal_rad)
                    self.config_actual = self.robot.cobot_tb.calc_conf(self.q_goal_rad)
                    if self.modo_robot_en_vivo:
                        self.q_robot_rad = self.q_goal_rad.copy()
                        if self.es_real:
                            self.robot.mc.send_angles(np.rad2deg(self.q_robot_rad).tolist(), 30)
                    else:
                        self._actualizar_goal_naranja_async(self.q_goal_rad, forzar_rviz=True)

        self.buttons_prev = btns

    def control_loop(self):
        if self.es_ros:
            self._publicar_joint_state_rviz(self.q_robot_rad)

        if self.ultimo_joy is None or len(self.ultimo_joy.axes) < 8:
            return

        axes = self.ultimo_joy.axes

        stick_lx = self.filtrar_dz(axes[0])
        stick_ly = self.filtrar_dz(axes[1])
        stick_rx = self.filtrar_dz(axes[2])
        stick_ry = self.filtrar_dz(axes[3])
        dpad_x   = self.filtrar_dz(axes[6])
        dpad_y   = self.filtrar_dz(axes[7])

        # Filtro Anti-Diagonal: si empujas casi recto un eje del stick, anula la desviación transversal
        if abs(stick_ly) > 1.8 * abs(stick_lx):
            stick_lx = 0.0
        elif abs(stick_lx) > 1.8 * abs(stick_ly):
            stick_ly = 0.0

        if abs(stick_ry) > 1.8 * abs(stick_rx):
            stick_rx = 0.0
        elif abs(stick_rx) > 1.8 * abs(stick_ry):
            stick_ry = 0.0

        escala_vel = 1.0
        if axes[4] < 0.2:
            escala_vel = 0.30
        elif axes[5] < 0.2:
            escala_vel = 1.80

        # Desplazamiento deseado en los ejes de tu referencia (Mesa / Brazo / Base)
        avance_local  = self.SIGNO_ADELANTE * 3.0 * stick_ly * escala_vel
        lateral_local = self.SIGNO_LATERAL  * 3.0 * stick_lx * escala_vel
        dz            = self.SIGNO_Z        * 3.0 * stick_ry * escala_vel

        # Rotación del plano XY según la referencia activa (Modo 0 = Mesa por defecto)
        if self.modo_referencia == 0:
            theta_ref = np.deg2rad(self.ANGULO_MESA_DEG)
        elif self.modo_referencia == 1:
            # Sigue la orientación horizontal hacia donde apunta el brazo actualmente
            theta_ref = float(np.arctan2(self.pose_actual.t[1], self.pose_actual.t[0]))
        else:
            theta_ref = 0.0

        dx = avance_local * np.cos(theta_ref) - lateral_local * np.sin(theta_ref)
        dy = avance_local * np.sin(theta_ref) + lateral_local * np.cos(theta_ref)

        d_roll  = 1.2 * dpad_x * escala_vel
        d_pitch = 1.2 * dpad_y * escala_vel
        d_yaw   = 1.5 * stick_rx * escala_vel

        moviendo = any(abs(v) > 1e-4 for v in (dx, dy, dz, d_roll, d_pitch, d_yaw))
        if not moviendo:
            return

        pose_backup = self.pose_actual
        q_backup = self.q_goal_rad.copy()

        T_nueva = SE3(dx, dy, dz) * self.pose_actual * SE3.RPY([d_roll, d_pitch, d_yaw], order='zyx', unit='deg')

        if T_nueva.t[2] < 20.0 and dz < 0:
            print("[Seguridad] Límite inferior Z = 20 mm alcanzado.")
            return

        try:
            res = self.robot.cobot_tb.ikine(T_nueva, self.config_actual)
            q_sol = np.asarray(res[0], dtype=float)[:6]

            q_sol_deg = np.rad2deg(q_sol)
            for j, (_, (q_min, q_max)) in enumerate(self.robot.joint_limits.items()):
                if q_sol_deg[j] < q_min or q_sol_deg[j] > q_max:
                    raise IKineError(f"Límite articular alcanzado en eje {j+1} ({q_sol_deg[j]:.1f}°)")

            if np.max(np.abs(np.rad2deg(q_sol - self.q_goal_rad))) > 25.0:
                return

            self.q_goal_rad = q_sol
            self.pose_actual = T_nueva

        except Exception:
            self.pose_actual = pose_backup
            self.q_goal_rad = q_backup
            return

        if self.modo_robot_en_vivo:
            self.q_robot_rad = self.q_goal_rad.copy()
            if self.es_ros:
                self._publicar_joint_state_rviz(self.q_robot_rad)
                self.robot.node_tf.add_robt(self.pose_actual * self.pinza, 'robt_joystick', 'base')
            elif self.es_real and hasattr(self.robot, "mc"):
                q_deg_list = [round(float(x), 2) for x in np.rad2deg(self.q_robot_rad)]
                self.robot.mc.send_angles(q_deg_list, self.speed_default)
                if getattr(self.robot, "_server_ref", None) is not None:
                    self.robot._server_ref.set_external_angles(q_deg_list)
        else:
            self._actualizar_goal_naranja_async(self.q_goal_rad)


def run(robot: BaseRobotController, **kwargs):
    speed = kwargs.get("speed", 20)

    rclpy_iniciado_aqui = False
    if not rclpy.ok():
        rclpy.init()
        rclpy_iniciado_aqui = True

    teleop_node = JoyStickTeleop(robot, speed_default=speed)

    usar_executor_existente = isinstance(robot, ROSManager) and hasattr(robot, "executor")
    if usar_executor_existente:
        robot.executor.add_node(teleop_node)

    try:
        if usar_executor_existente:
            while rclpy.ok():
                time.sleep(0.1)
        else:
            rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        print("\n[*] Cerrando teleoperación con JoyStick...")
    finally:
        if usar_executor_existente:
            try:
                robot.executor.remove_node(teleop_node)
            except Exception:
                pass

        if isinstance(robot, ROSManager) and rclpy.ok():
            try:
                robot.toggle_ros_connection(True)
            except Exception:
                pass
        elif isinstance(robot, MyCobotController) and hasattr(robot, "mc"):
            try:
                robot.mc.set_fresh_mode(0)
            except Exception:
                pass
            if getattr(robot, "_server_ref", None) is not None:
                robot._server_ref.pause_serial_reading(False)

        teleop_node.destroy_node()
        if rclpy_iniciado_aqui and rclpy.ok():
            rclpy.shutdown()