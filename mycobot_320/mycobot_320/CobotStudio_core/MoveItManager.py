# moveit_manager.py
#!/usr/bin/env python3
import time
import threading

from typing import List, Optional
from pathlib import Path
from spatialmath import SE3
import numpy as np
from typing import TYPE_CHECKING, Optional
from scipy.interpolate import CubicSpline

if TYPE_CHECKING:
    from CobotStudio import ROSManager 

from rclpy.node import Node
from std_msgs.msg import Empty, Header
from sensor_msgs.msg import JointState

from moveit_msgs.msg import DisplayTrajectory, RobotState, CollisionObject, ObjectColor
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA
from scipy.spatial.transform import Rotation as R

from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, RobotState, AttachedCollisionObject, ObjectColor
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import SwitchController


class MoveItManager(Node):
    """
    Nodo opcional que encapsula interacciones con MoveIt/RViz:
    """
    def __init__(self,
                 node_name: str = "moveit_manager",
                 apply_scene_srv_name: str = "/apply_planning_scene",
                 update_topic: str = "/rviz/moveit/update_goal_state",
                 display_traj_topic: str = "/display_planned_path",
                 project_path: str = None,
                 joint_names: list = None):
        super().__init__(node_name)


        if project_path:
            self.project_root = Path(project_path).resolve()
        else:
            self.project_root = Path.cwd()
        
        self.joint_names_full = joint_names
        self.apply_scene_srv_name = apply_scene_srv_name
        self.update_topic = update_topic
        self.display_traj_topic = display_traj_topic
        self.ROSManager_ref: Optional['ROSManager'] = None

        # Crear los client necesarios (sin esperar conexión aún)
        self._apply_cli = self.create_client(ApplyPlanningScene, self.apply_scene_srv_name)
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._collision_client = self.create_client(GetStateValidity, "/check_state_validity")
        self._switch_controller_cli = self.create_client(SwitchController, '/controller_manager/switch_controller')
        # Flags de disponibilidad
        self._apply_available = False
        self.plan_available = False
        self._collision_available = False
        self._switch_available = False

        # Publisher para actualizar el Goal State en RViz con un mensaje vacío
        self._pub_update = self.create_publisher(Empty, self.update_topic, 10)

        # Subscripción a DisplayTrajectory para capturar trayectorias planeadas con MotionPlanning
        self._last_display_trajectory = None
        self._display_sub = self.create_subscription(
            DisplayTrajectory,
            self.display_traj_topic,
            self._display_cb,
            10
        )

        self.last_robot_trajectory = None   # RobotTrajectory completo
        self.last_q_trajectory = None       # lista de vectores q
        
        # Lock para proteger datos y evitar condiciones de carrera
        self._lock = threading.Lock()

        self.get_logger().info("MoveItManager inciado.")

    def move_goal(self, target, tool=SE3(), wobj=SE3(), publish_tf:bool = False,  tf_frame_name: str = "moveit_robt", tf_reference: str = "moveit_wobj", timeout=10.0) -> bool:
        """
        Mueve el Goal State (robot naranja) de MoveIt a un vector de variables articulares o robtarget determinado.
        Llama a /apply_planning_scene para setear planning_scene.robot_state.
        Devuelve True si el servicio respondió success=True.

        Args:
            target (RobTarget, lista, array): Destino representado por un robtarget o vector de variables articulares.
            tool (SE3): Herramienta para el robtarget. Se asume nula por defecto (brida).
            wobj (SE3): Workobject para el robtarget. Se asume nulo por defecto (base del robot).
            publish_tf (bool): Si es `True`, publica la terna del goal mediante TF.
            tf_frame_name (str): Nombre del frame TF a publicar si `publish_tf=True`.
            tf_reference (str): Frame de referencia para el TF publicado.
            timeout (float): Tiempo máximo de espera por la respuesta del servicio.
        Returns:
            bool: True si el servicio respondió.
        """
        # Leer variables articulares o robtargets
        positions = self.ROSManager_ref._parse_to_joint_list(target, tool, wobj)

        # apply_planning_scene tiene que estar disponible
        if not self._ensure_service_connection(self._apply_cli, '_apply_available', timeout=2.0):
             # Loguea el error internamente el helper
             return False
        
        # Determinar joints a usar
        if len(positions) == 7:
            used_joints = self.joint_names_full
        elif len(positions) == 6:
            used_joints = self.joint_names_full[:6]
        else:
            self.get_logger().error(f"move_goal: longitud inválida de posiciones ({len(positions)}")
            return False
        
        # Construir request
        req = ApplyPlanningScene.Request()
        scene = req.scene
        scene.is_diff = True

        # Enviar la pose a la planning scene
        rs = RobotState()
        js = JointState()
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = used_joints
        js.position = positions
        rs.joint_state = js
        scene.robot_state = rs

        self.get_logger().info("Actualizando Goal State en MoveIt...")
        future = self._apply_cli.call_async(req)
        
        res = self._wait_for_future(future, timeout_sec=timeout)

        if res is not None and res.success:
            self.get_logger().info("Goal State actualizado (Success).")
            
            self._trigger_rviz_update_goal()
            
            if self.ROSManager_ref:
                self.ROSManager_ref._manipulability(target, wobj, tool, debug=True)

            time.sleep(0.5)

            # Publicar la terna de la pose según la herramienta mediante TF
            if publish_tf and getattr(self, 'ROSManager_ref', None) is not None:
                try:
                    # SimManager maneja las ternas con TF y calcula FK con la toolbox
                    self.ROSManager_ref._publish_goal_tf(positions, tool, wobj, tf_frame_name, tf_reference)
                except Exception as e:
                    self.get_logger().warn(f"publish_goal_tf fallo: {e}")
            return True
        else:
            self.get_logger().error("No hubo respuesta de apply_planning_scene (timeout o error).")
            return False

    def plan_and_execute(self, target, start=None, tool=SE3(), wobj=SE3(), execute=False,
                         update_rviz: bool = True, timeout: float = 5.0, 
                         vel_scale: float = 0.3, acc_scale: float = 0.3,
                         pipeline_id: str = "ompl",
                         planner_id: str = "RRTConnectkConfigDefault"):
        """
        Orquesta la planificación y ejecución de movimiento.

        1. Convierte inputs (RobTargets, listas) a vectores articulares.
        2. Llama a MoveIt (plan_joint_trajectory).
        3. Si execute=True, construye la trayectoria y la envía al ActionServer.
        4. Actualiza RViz.

        Args:
            target (RobTarget, lista, array): Destino representado por un robtarget o vector de variables articulares.
            q_start (RobTarget, lista, array, optional): Inicio expresado como robtarget o vector de variables articulares. Si es None, se usa el estado actual.
            tool (SE3): Herramienta para el robtarget. Se asume nula por defecto (brida).
            wobj (SE3): Workobject para el robtarget. Se asume nulo por defecto (base del robot).
            execute (bool): `True` mueve el robot en RViz. `False` planifica la trayectoria sin ejecutarla.
            update_rviz (bool): Mover al goal state a la pose final mediante `move_goal`.
            timeout (float): Tiempo máximo de espera por la respuesta del servicio.
        
        Returns:
            list: Lista de waypoints (q_list) planificados, o None si falló.
        """
        # Convertir pose inicial y final usando el helper
        q_goal_list = self.ROSManager_ref._parse_to_joint_list(target, tool, wobj)
        q_start_list = None

        if start is not None:
            q_start_list = self.ROSManager_ref._parse_to_joint_list(start, tool, wobj)

        if q_goal_list is None:
            self.get_logger().error("plan_and_execute: target inválido o IK falló.")
            return None
        
        traj_msg, q_list = self._plan_joint_trajectory(
            q_goal_list, q_start=q_start_list, vel_scale=vel_scale, acc_scale=acc_scale,
            timeout=timeout, pipeline_id=pipeline_id, planner_id=planner_id
        )
        
        if q_list is None or traj_msg is None:
            return None

        with self._lock:
            self.last_q_trajectory = q_list

        joint_names_used, q_start_h, q_goal_h, _ = self._harmonize_joint_inputs(
            self.joint_names_full, q_start_list, q_goal_list
        )

        if execute:
            # Usar directamente el JointTrajectory con los timestamps reales de MoveIt
            jt = traj_msg.joint_trajectory
            self.get_logger().debug(f"Enviando trayectoria de MoveIt. Joints: {len(jt.joint_names)}, Puntos: {len(jt.points)}")

            ok = self._send_trajectory_action(jt)
            if not ok:
                self.get_logger().warn("Fallo al ejecutar trayectoria.")
        
        if update_rviz:
            try:
                self.move_goal(q_goal_h)
                time.sleep(0.5)
            except Exception:
                self.get_logger().warn("move_goal falló o no disponible.")
            
            try:
                self._trigger_rviz_update_goal()
            except Exception:
                self.get_logger().warn("trigger_rviz_update_goal falló o no disponible.")
        
        return q_list


    def save_trajectory(self, trajectory_name: str, file_name: str = 'moveit_traj.py',
                        directory: str = 'Trayectorias', sample_hz: float = 25.0,
                        max_servo_deg_s: float = 150.0, lead_gain: float = 1.10,
                        min_speed: int = 6, max_speed: int = 90) -> bool:
        """
        Re-muestrea la última trayectoria planificada (por código o desde RViz) mediante CubicSpline
        a frecuencia constante (25 Hz) y traduce el perfil de velocidades al comando escalar speed (1..100)
        con cola de asentamiento anti-caída en los últimos 3 puntos.
        """
        # 1. Obtener la última trayectoria planificada (por código o por panel de RViz)
        with self._lock:
            robot_traj = self.last_robot_trajectory
            display_traj = self._last_display_trajectory

        jt = None
        if robot_traj is not None and hasattr(robot_traj, "joint_trajectory") and robot_traj.joint_trajectory.points:
            jt = robot_traj.joint_trajectory
        elif display_traj is not None and hasattr(display_traj, "trajectory") and len(display_traj.trajectory) > 0:
            jt = display_traj.trajectory[0].joint_trajectory

        if jt is None or not jt.points:
            self.get_logger().error("No hay una trayectoria planificada válida para guardar.")
            return False

        points = jt.points

        # Asegurar el orden físico de las 6 articulaciones del brazo
        arm_order = self.joint_names_full[:6] if self.joint_names_full else []
        try:
            idx_map = [jt.joint_names.index(jn) for jn in arm_order]
        except ValueError:
            idx_map = list(range(6))

        # 2. Extraer tiempos y posiciones originales en radianes
        t_orig = []
        q_orig_rad = []
        for p in points:
            t_sec = float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9
            if len(t_orig) > 0 and t_sec <= t_orig[-1]:
                continue  # Filtrar timestamps duplicados
            t_orig.append(t_sec)
            q_orig_rad.append([float(p.positions[i]) for i in idx_map])

        t_orig = np.asarray(t_orig, dtype=float)
        q_orig_rad = np.asarray(q_orig_rad, dtype=float)
        q_orig_deg = np.rad2deg(q_orig_rad)

        total_time = float(t_orig[-1])
        dt = 1.0 / float(sample_hz)

        # 3. Interpolación con Spline Cúbico en el tiempo original de MoveIt
        if len(t_orig) >= 2 and total_time > dt:
            t_interp = np.arange(0.0, total_time, dt)
            if not np.isclose(t_interp[-1], total_time):
                t_interp = np.append(t_interp, total_time)

            cs_rad = CubicSpline(t_orig, q_orig_rad, bc_type='clamped')
            cs_deg = CubicSpline(t_orig, q_orig_deg, bc_type='clamped')

            q_interp_rad = cs_rad(t_interp)
            q_interp_deg = cs_deg(t_interp)
            vel_inst_deg_s = np.abs(cs_deg(t_interp, 1))
        else:
            t_interp = t_orig
            q_interp_rad = q_orig_rad
            q_interp_deg = q_orig_deg
            vel_inst_deg_s = np.zeros_like(q_orig_deg)

        # 4. Cálculo del comando escalar único 'speed' (1..100) con rampa final anti-caída
        n_pts = len(t_interp)
        traj_export = []
        for k in range(n_pts):
            q_rad_list = [round(float(val), 6) for val in q_interp_rad[k]]
            t_val = round(float(t_interp[k]), 4)

            v_inst_max = float(np.max(vel_inst_deg_s[k]))
            if k > 0:
                dt_step = max(1e-4, float(t_interp[k] - t_interp[k - 1]))
                dq_step_deg = np.abs(q_interp_deg[k] - q_interp_deg[k - 1])
                v_sec_max = float(np.max(dq_step_deg) / dt_step)
            else:
                v_sec_max = v_inst_max

            v_req_deg_s = max(v_inst_max, v_sec_max) * float(lead_gain)
            raw_pct = (v_req_deg_s / float(max_servo_deg_s)) * 100.0
            spd_int = int(np.clip(round(raw_pct), min_speed, max_speed))

            # Cola de asentamiento progresiva en los últimos 3 puntos para eliminar caída por inercia
            if n_pts >= 4:
                if k == n_pts - 3:
                    spd_int = min(spd_int, 16)
                elif k == n_pts - 2:
                    spd_int = min(spd_int, 11)
                elif k == n_pts - 1:
                    spd_int = min(spd_int, 8)

            traj_export.append((q_rad_list, spd_int, t_val))

        # 5. Guardar dentro de self.project_root / directory para que load_data lo encuentre siempre
        if not file_name.endswith(".py"):
            file_name += ".py"
        target_dir = self.project_root / directory
        target_dir.mkdir(parents=True, exist_ok=True)
        filepath = target_dir / file_name

        mode = 'a' if filepath.exists() else 'w'
        with open(filepath, mode, encoding='utf-8') as f:
            f.write(f"\n{trajectory_name} = [\n")
            for q_r, spd, t_s in traj_export:
                f.write(f"    ({q_r}, {spd}, {t_s}),\n")
            f.write("]\n")

        self.get_logger().info(
            f"Trayectoria '{trajectory_name}' guardada en {filepath} "
            f"({n_pts} pts @ {sample_hz}Hz, T={total_time:.2f}s)."
        )
        return True
        
    def _display_cb(self, msg: DisplayTrajectory):
        """
        Callback que recibe la trayectoria planeada por MoveIt.
        """
        with self._lock:
            # Mantener el último mensaje de DisplayTrajectory
            self._last_display_trajectory = msg

    def _ensure_service_connection(self, client, flag_attr_name: str, timeout: float = 3.0) -> bool:
        """
        Helper interno: Verifica si un servicio está listo. Si no, espera 'timeout' segundos.
        Actualiza el flag de disponibilidad automáticamente.
        
        Args:
            client: El cliente ROS2 (self._apply_cli o self.plan_client).
            flag_attr_name: Nombre del atributo flag (ej: '_apply_available').
            timeout: Tiempo máximo de espera si no está conectado.
        """
        # Si el cliente devuelve que está listo el servicio, se actualiza el flag
        if client.service_is_ready():
            setattr(self, flag_attr_name, True)
            return True

        # Si no está listo, esperar según timeout
        self.get_logger().info(f"Conectando a servicio '{client.srv_name}' (Esperando {timeout}s)...")
        
        if client.wait_for_service(timeout_sec=timeout):
            setattr(self, flag_attr_name, True)
            self.get_logger().info(f"Conectado a '{client.srv_name}'.")
            return True
        else:
            setattr(self, flag_attr_name, False)
            self.get_logger().error(f"Fallo conexión con '{client.srv_name}' tras {timeout}s. Asegúrese de haber ejecutado el paquete de MoveIt.")
            return False

    def _wait_for_future(self, future, timeout_sec):
        """Helper para esperar futuros de forma segura sin bloquear el executor de fondo."""
        start_t = time.time()
        while not future.done():
            if time.time() - start_t > timeout_sec:
                return None # Timeout
            time.sleep(0.01)
        
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"Excepción en future: {e}")
            return None 

    def _trigger_rviz_update_goal(self, n_publish:int = 3, delay:float = 0.2):
        """
        Actualiza el Goal State del MotionPlanning a la posición actual (<current>) enviando un mensaje vacío. Requiere External Comm. activado en el plugin desde RViz.

        Args:
            n_publish (int): Número de mensajes Empty a publicar.
            delay (float): Retardo entre publicaciones en segundos.
        """
        e = Empty()
        for i in range(n_publish):
            self._pub_update.publish(e)
            time.sleep(delay)

    def _last_planned_trajectory(self) -> Optional[DisplayTrajectory]:
        """
        Acceso al último DisplayTrajectory. Necesario para guardarla.
        """
        with self._lock:
            return self._last_display_trajectory
        
    def _reset_last_trajectory(self):
        """
        Limpia la última trayectoria guardada. Usar antes de solicitar un nuevo plan.
        """
        with self._lock:
            self._last_display_trajectory = None
   
    def _plan_joint_trajectory(self, q_goal, q_start=None, group_name=None, timeout=5.0,
                               vel_scale: float = 0.3 , acc_scale: float = 0.3,
                               pipeline_id: str = "ompl",
                               planner_id: str = "RRTConnectkConfigDefault"):
        """
        Planea una trayectoria con el plugin MotionPlanning de MoveIt. No la ejecuta por lo que la posición del robot no cambia.
        Devuelve (traj_msg, q_list) o (None, None) en caso de error.
        q_goal debe ser lista de len == len(joint_names) (sin gripper si el grupo no lo incluye).
        """
        # Limpiar la memoria antes de planificar nuevamente
        self._reset_last_trajectory()
        
        # Verificar disponibilidad del servicio
        if not self._ensure_service_connection(self.plan_client, 'plan_available', timeout=2.0):
            return None, None

        if q_start is None and self.ROSManager_ref:
            # Pedimos q_current sin forzar pinza (devolverá lo que haya)
            q_start = self.ROSManager_ref.get_current_q(timeout=1.0)
            # q_start puede ser None o array
            if q_start is not None:
                q_start = list(q_start)

        # Decidir joint_names a usar y group_name si no especificado
        joint_names_used, q_start_h, q_goal_h, decided_group = self._harmonize_joint_inputs(
            self.joint_names_full, q_start, q_goal
        )

        if group_name is None:
            group_name = decided_group

        # Rechazar si aún falta q_start (cuando no había y SimManager no dio nada)
        if q_start_h is None:
            self.get_logger().warn("q_start no provisto y no se pudo obtener q_current.")
            return None, None

        # Construir GetMotionPlan.Request
        req = GetMotionPlan.Request()
        mp = MotionPlanRequest()
        mp.group_name = group_name
        mp.pipeline_id = pipeline_id
        mp.planner_id = planner_id
        mp.num_planning_attempts = 5
        mp.allowed_planning_time = float(timeout)
        mp.max_velocity_scaling_factor = float(np.clip(vel_scale, 0.01, 1.0))
        mp.max_acceleration_scaling_factor = float(np.clip(acc_scale, 0.01, 1.0))

        start_js = JointState()
        start_js.name = joint_names_used
        start_js.position = q_start_h
        mp.start_state = RobotState(joint_state=start_js)

        goal = Constraints()
        for name, q in zip(joint_names_used, q_goal_h):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = q
            jc.tolerance_above = 0.0001
            jc.tolerance_below = 0.0001
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        mp.goal_constraints.append(goal)
        req.motion_plan_request = mp
        
        # Llamar al servicio
        self.get_logger().info("Llamando a /plan_kinematic_path ...")
        future = self.plan_client.call_async(req)

        res = self._wait_for_future(future, timeout_sec=timeout)


        if res is None or res.motion_plan_response.error_code.val != 1:
            err = res.motion_plan_response.error_code.val if res else "Timeout"
            self.get_logger().warn(f"Planificación falló. Code: {err}")
            return None, None

        traj = res.motion_plan_response.trajectory

        # Convertir puntos de la trayectoria (waypoints) en lista de vectores q
        q_list = [list(pt.positions) for pt in traj.joint_trajectory.points]

        self.get_logger().info(f"Planificación OK. {len(q_list)} waypoints generados.")

        with self._lock:
            self.last_robot_trajectory = traj
            self.last_q_trajectory = q_list
        
        return traj, q_list
   
    def _build_joint_trajectory(self, joint_names: List[str], q_list: List[List[float]],
                            dt: float = 0.1, q_start: List[float] = None, prepend_start: bool = True) -> JointTrajectory:
        """
        Construye un mensaje JointTrajectory a partir de una lista de waypoints.

        Manejo de Gripper/Joints estáticos:
        Si los puntos de 'q_list' tienen menos elementos que 'joint_names', se rellenan
        los valores faltantes usando los valores correspondientes de 'q_start'.
        Esto evita que la pinza se cierre (0.0) si MoveIt solo planificó para el brazo.

        Args:
            joint_names (List[str]): Nombres de las articulaciones (e.g. 6 brazo + 1 gripper).
            q_list (List[List[float]]): Lista de waypoints (e.g. solo 6 brazo).
            dt (float): Paso de tiempo entre puntos.
            q_start (List[float]): Estado inicial completo. Fundamental para padding seguro.
            prepend_start (bool): Si True, agrega q_start como punto en t=0.0.

        Returns:
            JointTrajectory: Mensaje listo para enviar al controlador.
        """
        jt = JointTrajectory()
        jt.joint_names = list(joint_names)
        n_target = len(joint_names)

        if not q_list:
            self.get_logger().error("q_list vacía en _build_joint_trajectory")
            return jt

        points = []
        # Determinar valores de relleno (padding): 
        # si q_start tiene el tamaño correcto (7), y q_list tiene (6),
        # se extrae el valor del gripper (índice 6) de q_start.
        padding_values = []
        dim_plan = len(q_list[0])
        
        if dim_plan < n_target:
            if q_start and len(q_start) >= n_target:
                # Copiar la cola (tail) de q_start
                padding_values = q_start[dim_plan:]
            else:
                # Fallback si q_start no puede determinar el padding
                self.get_logger().warn("Faltan datos en q_start para rellenar joints estáticos (ej. gripper). Usando 0.0")
                padding_values = [0.0] * (n_target - dim_plan)

        # Helper local
        def _pad_point(q_in):
            # Si ya tiene el tamaño, cortar o devolver
            if len(q_in) >= n_target:
                return q_in[:n_target]
            # Si falta, agregar lo que sacamos de q_start
            return list(q_in) + padding_values

        points = []

        # punto inicial (time = 0)
        if prepend_start:
            if q_start is None:
                # Si no hay q_start se usa el primer punto de q_list (normalizado)
                p0_q = _pad_point(q_list[0])
            else:
                # Si hay q_start, se asume completo
                p0_q = q_start[:n_target] # Asegurar longitud

            p0 = JointTrajectoryPoint()
            p0.positions = [float(x) for x in p0_q]
            p0.time_from_start = Duration(sec=0, nanosec=0)
            points.append(p0)

        # Agregar todos los waypoints de q_list con tiempos dt, 2*dt, ...
        t = dt
        for q in q_list:
            # Rellenar con el gripper
            qc = _pad_point(q)

            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in qc]
            
            # Duración
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            p.time_from_start = Duration(sec=sec, nanosec=nanosec)
            points.append(p)
            t += dt

        jt.points = points
        return jt

    def _send_trajectory_action(self, jt: JointTrajectory,
                            controller_action_name: str = '/arm_controller/follow_joint_trajectory',
                            wait_for_result: bool = True,
                            action_timeout: float = 60.0,
                            gripper_action_name: str = '/grip_action_controller/gripper_cmd') -> bool:
        """
        Envía JointTrajectory al action server FollowJointTrajectory.
        Si jt contiene 'gripper_controller' lo separa: envía los joints del brazo al
        controller_action_name y la orden de pinza al gripper_action_name (GripperCommand).
        - Devuelve True solo si los goals aceptados/completados según wait_for_result.

        Args:
            jt (JointTrajectory): Trayectoria a enviar.
            controller_action_name (str): Nombre del action server del brazo.
            wait_for_result (bool): Si es True espera el resultado del action server.
            action_timeout (float): Tiempo máximo de espera por el resultado.
            gripper_action_name (str): Nombre del action server de la pinza.
        Returns:
            bool: True si los goals fueron aceptados/completados según wait_for_result.
        """
        # Validaciones
        if not jt.points:
            self.get_logger().error("_send_trajectory_action: jt sin puntos. No hay nada para enviar.")
            return False

        # Detectar si la pinza está en la lista de joints
        gripper_name = 'gripper_controller'
        has_gripper = gripper_name in jt.joint_names

        arm_jt = JointTrajectory()
        target_gripper_pos = None

        if has_gripper:
            # Separar joints del gripper
            arm_joint_names = [jn for jn in jt.joint_names if jn != gripper_name]
            arm_jt.joint_names = arm_joint_names
            
            # Reconstruir puntos filtrando el gripper
            gripper_idx = jt.joint_names.index(gripper_name)
            
            for p in jt.points:
                new_p = JointTrajectoryPoint()
                new_p.time_from_start = p.time_from_start
                # Excluir el valor del índice del gripper
                new_p.positions = [val for i, val in enumerate(p.positions) if i != gripper_idx]
                arm_jt.points.append(new_p)

            # Obtener valor final del gripper (del último punto)
            target_gripper_pos = float(jt.points[-1].positions[gripper_idx])
            
        else:
            # Si no hay gripper, la trayectoria pasa directa
            arm_jt = jt
            target_gripper_pos = None

        # Ejecución en el brazo: Arm Controller
        # Inicializar cliente
        if not hasattr(self, "_traj_action_client") or self._traj_action_client is None:
            self._traj_action_client = ActionClient(self, FollowJointTrajectory, controller_action_name)

        if not self._traj_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Action server {controller_action_name} no disponible.")
            return False

        # Enviar goal
        goal_msg = FollowJointTrajectory.Goal(trajectory=arm_jt)
        send_future = self._traj_action_client.send_goal_async(goal_msg)
        
        # Llamar al cliente
        goal_handle = self._wait_for_future(send_future, 5.0)

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal de brazo rechazado.")
            return False
        
        self.get_logger().info("Ejecutando trayectoria de brazo...")

        # Ejecución en el gripper: Gripper Controller
        gripper_client = None
        gripper_goal_handle = None
        gripper_ok = True # Se asume true si no hay gripper o si funciona

        if target_gripper_pos is not None:
            try:
                gripper_client = ActionClient(self, GripperCommand, gripper_action_name)
                if gripper_client.wait_for_server(timeout_sec=1.0):
                    g_goal = GripperCommand.Goal()
                    g_goal.command.position = target_gripper_pos
                    g_goal.command.max_effort = 0.0
                    
                    g_future = gripper_client.send_goal_async(g_goal)
                    # Llamada al cliente
                    gripper_goal_handle = self._wait_for_future(g_future, 5.0)
                    
                    if not gripper_goal_handle or not gripper_goal_handle.accepted:
                        self.get_logger().warn("Goal gripper rechazado.")
                        gripper_ok = False
                    else:
                        self.get_logger().info(f"Comando gripper enviado: {target_gripper_pos}")
                else:
                    self.get_logger().warn(f"Server gripper {gripper_action_name} no disponible.")
                    gripper_ok = False
            except Exception as e:
                self.get_logger().error(f"Error enviando gripper: {e}")
                gripper_ok = False

        # Esperar resultados
        if not wait_for_result:
            return True

        # Esperar brazo
        arm_ok = True
        res_future = goal_handle.get_result_async()
        # Nueva llamada al cliente
        res = self._wait_for_future(res_future, action_timeout)
        
        # Verificar resultado
        if res and res.result:
            self.get_logger().info(f"Brazo OK. Code: {res.status}")
            pass
        else:
            self.get_logger().error("Fallo o timeout esperando resultado del brazo.")
            arm_ok = False

        # Esperar gripper (si se envió)
        if gripper_client and gripper_goal_handle:
            g_res_fut = gripper_goal_handle.get_result_async()
            g_res = self._wait_for_future(g_res_fut, 5.0)
            if not g_res:
                self.get_logger().warn("No se recibió confirmación del gripper.")

        return arm_ok and gripper_ok


    def _init_collision_service(self, timeout=3.0):
        """
        Inicializa el cliente de GetStateValidity si aún no existe.
        
        Args:
            timeout (float): Tiempo máximo de espera para que el servicio esté disponible.
        """
        if hasattr(self, "_collision_client"):
            return

        self._collision_client = self.create_client(GetStateValidity, "/check_state_validity")
        if not self._collision_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn("Servicio /check_state_validity NO disponible. ¿move_group levantado?")
            self._collision_available = False
        else:
            self.get_logger().info("Servicio /check_state_validity disponible.")
            self._collision_available = True

    def _check_collision(self, positions, tool, wobj, group_name="arm", timeout=5.0):

        """
        Consulta a MoveIt2 si una configuración articular está en colisión.
        Devuelve:
            True  -> válido (sin colisión)
            False -> en colisión
            None  -> error o servicio no disponible
        """
        # Inicializar servicio on-demand
        self._init_collision_service()

        if not self._ensure_service_connection(self._collision_client, '_collision_available', timeout=2.0):
            return False # Fail-safe
        
        q = self.ROSManager_ref._parse_to_joint_list(positions, tool, wobj)

        # Construir RobotState
        js = JointState()
        js.name = self.joint_names_full[:6]
        js.position = q
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()

        rs = RobotState()
        rs.joint_state = js

        # Construir request
        req = GetStateValidity.Request()
        req.group_name = group_name
        req.robot_state = rs

        # Llamada al servicio asíncrona
        future = self._collision_client.call_async(req) 

        # 4. Espera manual (thread-safe)
        start_t = time.time()
        while not future.done():
            if time.time() - start_t > timeout: # Timeout corto para checks
                self.get_logger().error("Timeout en check_collision")
                return False # Fail-safe: ante timeout se asume colisión
            time.sleep(0.005) # Sleep muy breve para ser rápido

        # Procesar resultado
        try:
            resp = future.result()

            if resp.valid:
                self.get_logger().info("Configuración sin colisión.")
                return True
            else:
                self.get_logger().info("Configuración EN COLISIÓN.")
                return False
            
        except Exception as e:
            self.get_logger().error(f"Excepción en check_collision: {e}")
            return False # Asumir colisión si falla        
    
    def _switch_controllers(self, activate_list: list = [], deactivate_list: list = [], 
                           strictness: int = 1, timeout: float = 5.0) -> bool:
        """
        Activa o desactiva controladores de ros2_control dinámicamente.
        
        Args:
            activate_list (list[str]): Nombres de controladores a activar.
            deactivate_list (list[str]): Nombres de controladores a desactivar.
            strictness (int): 1=BEST_EFFORT, 2=STRICT (Default: BEST_EFFORT).
            timeout (float): Tiempo de espera.
            
        Returns:
            bool: True si el cambio fue exitoso.
        """
        if  self._switch_controller_cli is None:
            self.get_logger().error("switch_controllers: cliente no disponible.")
            return False

        # Conexión bajo demanda
        if not self._ensure_service_connection(self._switch_controller_cli, '_switch_available', timeout=2.0):
            return False

        req = SwitchController.Request()
        req.activate_controllers = activate_list
        req.deactivate_controllers = deactivate_list
        req.strictness = strictness
        req.start_asap = True
        # req.timeout = Duration(sec=int(timeout), nanosec=0).to_msg()

        self.get_logger().info(f"Conmutando controladores: +{activate_list} / -{deactivate_list}")

        future = self._switch_controller_cli.call_async(req)
        
        # Helper thread-safe
        res = self._wait_for_future(future, 2)

        if res is not None and res.ok:
            self.get_logger().info("Controladores conmutados exitosamente.")
            return True
        else:
            self.get_logger().error("Fallo al conmutar controladores.")
            return False

    def _harmonize_joint_inputs(self, joint_names_full, q_start, q_goal, default_val=0.0):
        """
        Normaliza q_start, q_goal y decide joint_names y group_name.
        - joint_names_full: lista completa (7 names).
        - q_start, q_goal: listas o None.
        Retorna: (joint_names_used, q_start_h, q_goal_h, group_name)
        """
        n_full = len(joint_names_full)
        if n_full < 6:
            raise ValueError("joint_names_full debe tener al menos 6 elementos")

        # longitudes (0 si None)
        n_start = len(q_start) if q_start is not None else 0
        n_goal  = len(q_goal)  if q_goal  is not None else 0

        # Si alguno de los parámetros incluye al gripper se usa arm_with_gripper
        if n_start >= 7 or n_goal >= 7:
            joint_names = joint_names_full[:7]
            group = "arm_with_gripper"
            target_n = 7
        else:
            joint_names = joint_names_full[:6]
            group = "arm"
            target_n = 6


        def smart_pad(q_in, reference_val=None):
            """
            Helper interno para normalizar vectores articulares.
            """
            if q_in is None: return None

            q_list = list(q_in)
            current_len = len(q_list)
            
            if current_len == target_n:
                return q_list
            
            if current_len > target_n:
                return q_list[:target_n]
            
            # Si falta rellenar (ej. tenemos 6, queremos 7)
            if target_n == 7 and current_len == 6:
                # Si hay un valor de referencia se usa, si no, usar default
                val_to_add = reference_val if reference_val is not None else default_val
                return q_list + [val_to_add]
            
            # Fallback genérico para otros tamaños
            return (q_list + [default_val] * (target_n - current_len))[:target_n]

        # Primera opción: usar el valor del gripper del start para el goal
        start_gripper_val = q_start[6] if (n_start >= 7) else None
        
        # Segunda opción: usar el valor del gripper del goal para el start
        goal_gripper_val = q_goal[6] if (n_goal >= 7) else None

        # Si start no tiene gripper (n=6) y el goal sí, se busca el valor actual real
        if n_start == 6 and target_n == 7:
             if hasattr(self, 'ROSManager_ref') and self.ROSManager_ref:
                 curr = self.ROSManager_ref.get_current_q(prefer_gripper=True, timeout=0.2)
                 if curr is not None and len(curr) >= 7:
                     start_gripper_val = curr[6]

        # Normalizar
        q_start_norm = smart_pad(q_start, reference_val=goal_gripper_val)
        q_goal_norm  = smart_pad(q_goal,  reference_val=start_gripper_val)

        return joint_names, q_start_norm, q_goal_norm, group
    


    def _load_stl_as_mesh_msg(self, stl_path_or_uri: str, scale: tuple) -> Optional[Mesh]:
        """Lee un archivo STL (binario o ASCII) y construye un shape_msgs/msg/Mesh escalado."""
        import struct
        from urllib.parse import urlparse, unquote

        if stl_path_or_uri.startswith("file://"):
            file_path = Path(unquote(urlparse(stl_path_or_uri).path))
        else:
            file_path = Path(stl_path_or_uri)

        if not file_path.exists():
            self.get_logger().error(f"No se encontró el archivo STL: {file_path}")
            return None

        sx, sy, sz = float(scale[0]), float(scale[1]), float(scale[2])
        mesh_msg = Mesh()

        # para STL binario
        try:
            with open(file_path, 'rb') as f:
                header = f.read(80)
                num_triangles = struct.unpack('<I', f.read(4))[0]
                for i in range(num_triangles):
                    data = struct.unpack('<12fH', f.read(50))
                    # data[0:3] es la normal; data[3:12] son los 3 vértices (x,y,z)
                    for v_idx in range(3):
                        vx = data[3 + v_idx * 3] * sx
                        vy = data[4 + v_idx * 3] * sy
                        vz = data[5 + v_idx * 3] * sz
                        mesh_msg.vertices.append(Point(x=float(vx), y=float(vy), z=float(vz)))
                    base_idx = i * 3
                    mesh_msg.triangles.append(
                        MeshTriangle(vertex_indices=[base_idx, base_idx + 1, base_idx + 2])
                    )
            return mesh_msg
        except Exception as e:
            self.get_logger().error(f"Error leyendo STL '{file_path}': {e}")
            return None

    def add_collision_object(self, name: str, pose_init: tuple, size: tuple,
                             color: tuple = (0.5, 0.5, 0.5, 1.0),
                             shape: int = 1, mesh: str = None,
                             rot_euler: tuple = (0.0, 0.0, 0.0),
                             frame_id: str = "base",
                             touch_links: list = None) -> bool:
        """
        Agrega un objeto al PlanningScene de MoveIt como obstáculo de colisión.
        Si se especifican 'touch_links' (ej. ['base', 'base_mesa']), MoveIt deshabilita
        la colisión entre este objeto y dichos eslabones (equivalente a Adjacent/Never).
        """
        if not self._ensure_service_connection(self._apply_cli, '_apply_available', timeout=20.0):
            return False

        co = CollisionObject()
        co.header.frame_id = frame_id
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = name
        co.operation = CollisionObject.ADD

        pose = Pose()
        pose.position.x = float(pose_init[0])
        pose.position.y = float(pose_init[1])
        pose.position.z = float(pose_init[2])
        quat = R.from_euler('xyz', rot_euler).as_quat()  # [x, y, z, w]
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])

        if mesh is not None or shape == 10:
            mesh_msg = self._load_stl_as_mesh_msg(mesh, size)
            if mesh_msg is None:
                return False
            co.meshes.append(mesh_msg)
            co.mesh_poses.append(pose)
        else:
            prim = SolidPrimitive()
            if shape == 1:
                prim.type = SolidPrimitive.BOX
                prim.dimensions = [float(size[0]), float(size[1]), float(size[2])]
            elif shape == 2:
                prim.type = SolidPrimitive.SPHERE
                prim.dimensions = [float(size[0]) / 2.0]
            elif shape == 3:
                prim.type = SolidPrimitive.CYLINDER
                prim.dimensions = [float(size[2]), float(size[0]) / 2.0]
            else:
                prim.type = SolidPrimitive.BOX
                prim.dimensions = [float(size[0]), float(size[1]), float(size[2])]
            co.primitives.append(prim)
            co.primitive_poses.append(pose)

        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True

        # Si hay touch_links definidos, se registra con touch_links para ignorar colisiones con la base
        if touch_links:
            aco = AttachedCollisionObject()
            aco.link_name = frame_id
            aco.object = co
            aco.touch_links = list(touch_links)
            req.scene.robot_state.attached_collision_objects.append(aco)
            req.scene.robot_state.is_diff = True
        else:
            req.scene.world.collision_objects.append(co)

        obj_color = ObjectColor()
        obj_color.id = name
        obj_color.color = ColorRGBA(
            r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3])
        )
        req.scene.object_colors.append(obj_color)

        future = self._apply_cli.call_async(req)
        res = self._wait_for_future(future, timeout_sec=5.0)

        if res is not None and res.success:
            self.get_logger().info(
                f"Obstáculo '{name}' agregado a MoveIt (touch_links={touch_links})."
            )
            return True
        else:
            self.get_logger().error(f"Fallo al agregar obstáculo '{name}' a MoveIt.")
            return False



    def move_gripper(self, apertura_pct: float, wait_for_result: bool = True,
                     gripper_action_name: str = '/grip_action_controller/gripper_cmd') -> bool:
        """
        Envía un comando de apertura/cierre al servidor de acción de la pinza (GripperCommand).

        Permite que la función GripperState() del SDK opere de manera transparente tanto
        contra el controlador virtual de MoveIt (FakeSystem) como contra CobotClient (Robot Real).

        Args:
            apertura_pct (float): Apertura deseada de 0.0 (cerrado) a 100.0 (abierto).
            wait_for_result (bool, optional): Si es True, bloquea hasta completar la acción.
            gripper_action_name (str, optional): Nombre del servidor de acción de la pinza.

        Returns:
            bool: True si el servidor de acción aceptó y ejecutó el comando.
        """
        val_rad = (float(apertura_pct) * 0.01) - 0.7

        if not hasattr(self, "_gripper_action_client") or self._gripper_action_client is None:
            self._gripper_action_client = ActionClient(self, GripperCommand, gripper_action_name)

        if not self._gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"Servidor de pinza {gripper_action_name} no disponible.")
            return False

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = val_rad
        goal_msg.command.max_effort = 0.0

        send_future = self._gripper_action_client.send_goal_async(goal_msg)
        goal_handle = self._wait_for_future(send_future, timeout_sec=3.0)

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal de pinza rechazado por el Action Server.")
            return False

        if wait_for_result:
            res_future = goal_handle.get_result_async()
            self._wait_for_future(res_future, timeout_sec=5.0)

        return True
              
