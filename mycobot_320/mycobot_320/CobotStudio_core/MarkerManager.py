import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, TransformStamped
from builtin_interfaces.msg import Duration as MsgDuration
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np

class ManagedObject:
    """
    Definición de objetos (Markers) mediante ROS.
    """
    def __init__(self, name, shape, size, color, pose_init, movable=True, mesh=None,
                 marker_id=0, marker_ns='objects'):
        self.name = name
        self.pose_fixed = pose_init     # Pose inicial o última pose al soltar
        self.attached = False
        self.movable = movable          # Flag para permitir o no su movimiento
        self.rel_pose = None            # Pose relativa tool->obj cuando es tomado por la pinza

        # Configuración de RViz
        self.marker = Marker()
        self.marker.ns = marker_ns
        self.marker.id = marker_id
        self.marker.type = shape
        self.shape = shape
        self.marker.action = Marker.ADD
        self.size = size
        self.color = color
        self.mesh = mesh

        # Se admite el uso de RGB o RGBA (transparencia)
        if len(color) == 3:
            r, g, b = color
            a = 1.0
        elif len(color) == 4:
            r, g, b, a = color
        else:
            raise ValueError("El color debe ser una tupla/lista con 3 (RGB) o 4 (RGBA) valores")
    
        self.marker.color.r, self.marker.color.g, self.marker.color.b, self.marker.color.a = (r, g, b, a)
        self.marker.scale.x, self.marker.scale.y, self.marker.scale.z = size
        
        self.marker.lifetime = MsgDuration(sec=0)  # Visible indefinidamente

        if mesh and shape == Marker.MESH_RESOURCE:
            self.marker.mesh_resource = mesh
            self.marker.mesh_use_embedded_materials = True

class MarkerManager(Node):
    """
    Nodo que maneja Markers en ROS y sus interacciones con la herramienta del robot.
    """

    # Umbrales para lógica de agarre
    GRIPPER_CLOSE_DELTA = -0.4  # Cambio negativo indica cierre
    GRIPPER_OPEN_DELTA = 0.01   # Cambio positivo indica apertura

    def __init__(self, base_frame='base', tool_frame='link6', freq=30.0):
        super().__init__("MarkerManager")
        self.base_frame = base_frame
        self.tool_frame = tool_frame    # Se define a la brida como herramienta porque siempre está publicada
        
        self.publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.objects = {}  # Diccionario para objetos
        self._next_id = 0  # Contador para IDs únicos

        self.prev_gripper_state = None

        # Distancia máxima permitida entre la tool y el objeto a tomar
        self.attach_distance_threshold = 0.15

        # Guardar último timestamp recibido de joint_states
        self.last_stamp = None

        # Suscripción a /joint_states
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        timer_period = 1.0 / freq  # segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Limpiar objetos que hayan podido quedar de ejecuciones previas
        self.clear_scene() 

        self.get_logger().info(f"MarkerManager iniciado. Actualizando marcadores a {freq} Hz.")

    def timer_callback(self):
        """Ciclo principal de actualización visual."""
        self._update_markers()

    def joint_state_callback(self, msg):
        """Callback de sensores para detectar eventos de la pinza."""
        # Guardar el último timestamp
        self.last_stamp = msg.header.stamp
        self._handle_auto_attach(msg)

    def find_closest_object(self):
        """
        Encuentra el objeto móvil más cercano más cercano al TCP del robot.

        Returns:
            (str, float): Nombre del objeto y distancia.
        """
        try:
            # Obtener la pose de la herramienta en el marco base
            timeout = rclpy.duration.Duration(seconds=1.0)
            tf_tool = self.tf_buffer.lookup_transform(self.base_frame, 'tool', rclpy.time.Time(), timeout=timeout)
            
            tool_pos = np.array([
                tf_tool.transform.translation.x,
                tf_tool.transform.translation.y,
                tf_tool.transform.translation.z
            ])

            min_dist = float('inf')
            closest_obj_name = None

            # Recorrer objetos: se buscan objetos móviles no tomados por la pinza
            for name, obj in self.objects.items():
                if obj.movable and not obj.attached:
                    obj_pos = np.array([
                        obj.pose_fixed.position.x,
                        obj.pose_fixed.position.y,
                        obj.pose_fixed.position.z
                    ])

                    # Distancia euclídea de la herramienta la terna del objeto
                    dist = np.linalg.norm(tool_pos - obj_pos)
                    if dist < min_dist:
                        min_dist = dist
                        closest_obj_name = name
            
            return closest_obj_name, min_dist

        except Exception as e:
            self.get_logger().warn(f"No se pudo encontrar el objeto más cercano: {e}")
            return None, float('inf')

    def publish_object_tf(self, obj_name):
        """
        Publicar las ternas de los objetos.
        """
        if obj_name not in self.objects:
            self.get_logger().warn(f"Objeto {obj_name} no existe")
            return
        
        obj = self.objects[obj_name]
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base'           # Terna de referencia
        t.child_frame_id = f"{obj_name}_frame"

        # Posición
        t.transform.translation.x = obj.pose_fixed.position.x
        t.transform.translation.y = obj.pose_fixed.position.y
        t.transform.translation.z = obj.pose_fixed.position.z

        # Orientación
        t.transform.rotation = obj.pose_fixed.orientation

        self.tf_broadcaster.sendTransform(t)

    def add_object(self, name, pose_init, size, color=(0.5, 0.5, 0.5, 1.0),
                   shape=Marker.CUBE, movable=True, mesh=None, rot_euler=(0.0, 0.0, 0.0)):
        """
        Instancia un ManagedObject, configura el Marker de ROS y lo registra en el diccionario interno.
        
        Realiza la conversión de Euler a Cuaternión y asigna IDs únicos.
        Ver docstring de 'ROSManager.add_scene_object' para detalles de parámetros.
        """
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pose_init

        quat = R.from_euler('xyz', rot_euler, degrees=False).as_quat()      # Rotación como cuaternión
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat

        # Definir al objeto con la clase correspondiente
        obj = ManagedObject(name, shape, size, color, pose, movable, mesh,
                            marker_id=self._alloc_id(), marker_ns="objects")
        obj.marker.pose = pose
        obj.marker.mesh_use_embedded_materials = False # Para cambiar el color del STL
        obj.marker.header.frame_id = self.base_frame

        # Agregar al diccionario y publicar su terna
        self.objects[name] = obj
        self._update_markers()
        self.get_logger().info(f"Objeto agregado: '{name}'.")

    def attach(self, name):
        """
        Tomar objetos con la herramienta del robot. 
        Su posición y orientación respecto a la pinza se mantiene constante mientras persista el estado.
        """
        if name not in self.objects:
            self.get_logger().warn(f"Objeto {name} no existe")
            return
        obj = self.objects[name]

        # Verificar que el objeto sea móvil
        if not obj.movable:
            self.get_logger().warn(f"Objeto {name} es fijo, no puede adherirse")
            return
        
        try:
            # Tool en coordenadas de base
            timeout = rclpy.duration.Duration(seconds=1.0)
            tf_bt = self.tf_buffer.lookup_transform(
                self.base_frame, 
                self.tool_frame,
                rclpy.time.Time(),
                timeout=timeout
            )
            Tb_tool = self.tf_to_matrix(tf_bt.transform) # Pasar a matriz homogénea

            # T_base_obj actual (el marker fijo en base antes de tomar)
            Tb_obj = self.pose_to_matrix(obj.pose_fixed) # Objeto respecto a base

            # Guardar el offset relativo: objeto respecto a tool = base respecto a tool * objeto respecto a base
            obj.rel_pose = np.linalg.inv(Tb_tool) @ Tb_obj
            obj.attached = True

            self.get_logger().info(f"Objeto {name} adherido a {self.tool_frame}")
        except Exception as e:
            self.get_logger().warn(f"Fallo al tomar el objeto {name}: {e}")

    def detach(self, name):
        """
        Suelta un objeto tomado por la herramienta.
        """
        if name not in self.objects:
            self.get_logger().warn(f"Objeto {name} no existe")
            return
        
        # Verificar que haya un objeto tomado por la herramienta
        obj = self.objects[name]
        if not obj.attached:
            return

        try:
            # T_base_tool
            timeout = rclpy.duration.Duration(seconds=1.0)
            tf_bt = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time(),
                timeout=timeout
            )
            Tb_tool = self.tf_to_matrix(tf_bt.transform)

            # Pose real del objeto al soltar: T_base_obj = T_base_tool * T_tool_obj
            Tb_obj = Tb_tool @ obj.rel_pose if obj.rel_pose is not None else Tb_tool
            obj.pose_fixed = self.matrix_to_pose(Tb_obj)

        except Exception as e:
            self.get_logger().warn(f"Error al leer TF {'base'}<-{self.tool_frame}: {e}")

        obj.attached = False
        obj.rel_pose = None
        self.get_logger().info(f"Objeto {name} liberado en posición actual")

    def clear_scene(self):
        """
        Borra TODOS los objetos visuales de RViz y limpia el diccionario interno.
        """
        # Marker.DELETEALL
        delete_all_marker = Marker()
        delete_all_marker.action = 3 
        delete_all_marker.header.frame_id = self.base_frame
        
        arr = MarkerArray()
        arr.markers.append(delete_all_marker)
        self.publisher.publish(arr)

        # Limpiar la memoria interna del script
        self.objects.clear()
        self._next_id = 0
    
    def _handle_auto_attach(self, msg: JointState):
        """
        Lógica para agarrar/soltar objetos.
        Detecta cambios en la posición de la pinza para inferir comandos de abrir/cerrar.
        """
        try:
            # Encontrar el estado actual de la pinza
            current_gripper_state = msg.position[6]

            if self.prev_gripper_state is not None:

                delta = current_gripper_state - self.prev_gripper_state
                # Detectar cierre: el valor del joint disminuye
                if delta < self.GRIPPER_CLOSE_DELTA:
                    # Buscar objeto más cercano
                    closest_obj, min_dist = self.find_closest_object()

                    # Si la distancia es menor al valor umbral, se toma dicho objeto
                    if closest_obj and min_dist < self.attach_distance_threshold:
                        self.get_logger().info(f"Cierre de pinza detectado. Objeto más cercano: {closest_obj} a {min_dist:.3f}m.")
                        self.attach(closest_obj)

                # Detectar apertura: el valor del joint aumenta
                elif delta > self.GRIPPER_OPEN_DELTA:
                    # Buscar si hay un objeto adherido para soltarlo
                    for obj in self.objects.values():
                        if obj.attached:
                            self.get_logger().info(f"Apertura de pinza detectada. Soltando {obj.name}.")
                            self.detach(obj.name)
                            break # Se asume que solo se puede tener un objeto a la vez

            self.prev_gripper_state = current_gripper_state

        except (ValueError, IndexError):
            # Si el joint de la pinza no está en el mensaje, se ignora
            pass
        except Exception as e:
            self.get_logger().error(f"Error en _handle_auto_attach: {e}")
    
    def _update_markers(self):
        """
        Actualiza la posición de todos los objetos según corresponda.
        """
        arr = MarkerArray()
        Tb_tool = None

        try:
            timeout = rclpy.duration.Duration(seconds=0.5)
            tf_bt = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time(),
                timeout=timeout
            )
            Tb_tool = self.tf_to_matrix(tf_bt.transform)
        except Exception:
            pass
            # self.get_logger().warn(f"No pude actualizar objeto {obj.name}: {e}")
        
        for obj in self.objects.values():
            obj.marker.header.stamp = rclpy.time.Time().to_msg()

            # Actualizar objeto tomado por la herramienta
            if obj.attached and obj.rel_pose is not None and Tb_tool is not None:
                # T_base_obj = T_base_tool * T_tool_obj
                Tb_obj = Tb_tool @ obj.rel_pose

                obj.marker.header.frame_id = 'base'
                obj.marker.pose = self.matrix_to_pose(Tb_obj)

            # Los objetos sin interacción permanecen en su pose fija
            else:
                obj.marker.header.frame_id = 'base'
                obj.marker.pose = obj.pose_fixed

            arr.markers.append(obj.marker)
            # self.publish_object_tf(obj.name)  # Permite ver la terna de cada objeto

        self.publisher.publish(arr)

    def _alloc_id(self):
        """
        Helper intenro para identificar objetos.
        """
        mid = self._next_id
        self._next_id += 1
        return mid

    """ Helpers """
    @staticmethod
    def tf_to_matrix(tf):
        """Convierte Transform geometry_msgs a matriz 4x4."""
        T = np.eye(4)
        t = [tf.translation.x, tf.translation.y, tf.translation.z]
        q = [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
        T[:3, :3] = R.from_quat(q).as_matrix()
        T[:3,  3] = t
        return T
    
    @staticmethod
    def pose_to_matrix(pose: Pose):
        """Convierte Pose geometry_msgs a matriz 4x4."""
        T = np.eye(4)
        T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        T[:3, :3] = R.from_quat(q).as_matrix()
        return T
    
    @staticmethod
    def matrix_to_pose(T):
        """Convierte matriz 4x4 a Pose geometry_msgs."""
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = T[:3, 3]
        q = R.from_matrix(T[:3, :3]).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        return pose
