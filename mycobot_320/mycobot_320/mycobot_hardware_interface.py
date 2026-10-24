#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from pymycobot import MyCobot320
import math
import time

class MyCobotActionServer(Node):
    def __init__(self):
        super().__init__('mycobot_hardware_interface')
        
        # 1. CONEXIÓN AL HARDWARE REAL
        self.get_logger().info("Conectando al MyCobot 320 Pi físico...")
        try:
            self.mc = MyCobot320('/dev/ttyAMA0', 115200)
            self.get_logger().info("¡Hardware conectado exitosamente!")
        except Exception as e:
            self.get_logger().error(f"Fallo de conexión: {e}")
            raise e

        # Nombres de las articulaciones
        self.joint_names = [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
            'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
        ]

        # 2. EL SERVIDOR DE ACCIÓN (ESCUCHA A MOVEIT)
        # Este servidor espera recibir mensajes en este tópico
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_callback
        )

        # 3. PUBLICADOR DEL ESTADO REAL (HABLA CON MOVEIT)
        # MoveIt necesita saber dónde está el robot a cada instante.
        # Leemos los motores a 20Hz (cada 0.05 segundos)
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        self.get_logger().info("Action Server iniciado. Esperando trayectorias...")

    def publish_joint_states(self):
        """Lee los encoders reales (grados), los pasa a radianes y los publica."""
        try:
            angles_deg = self.mc.get_angles()
            if angles_deg and len(angles_deg) == 6:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = [math.radians(ang) for ang in angles_deg]
                self.state_pub.publish(msg)
        except Exception as e:
            pass # Ignoramos pequeños errores de lectura serial para no saturar la consola

    def execute_callback(self, goal_handle):
        """Esta función se activa cuando MoveIt manda una trayectoria."""
        self.get_logger().info('¡Trayectoria recibida de MoveIt! Empezando movimiento...')
        
        # Extraemos la lista de puntos calculados por MoveIt
        points = goal_handle.request.trajectory.points

        if not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # PREPARAR EL ROBOT PARA MOVIMIENTO CONTINUO
        self.mc.set_fresh_mode(1) # Ignorar frenado entre puntos
        
        # Guardamos el tiempo exacto en el que empezamos
        start_time = time.time()
        
        for point in points:
            # ¿A qué segundo exacto debo mandar esta posición?
            t_sec = point.time_from_start.sec + (point.time_from_start.nanosec * 1e-9)
            
            # Pasamos de radianes (MoveIt) a grados (Robot)
            angles_deg = [math.degrees(q) for q in point.positions]
            
            # Enviamos el comando de posición a los motores. 
            # Ponemos una velocidad alta fija (ej. 60) porque el ritmo real 
            # lo va a marcar el bucle 'while' de abajo.
            self.mc.send_angles(angles_deg, 60)
            
            # BUCLE DE SINCRONIZACIÓN
            # Esperamos hasta que sea el momento exacto de pasar al siguiente punto.
            target_absolute_time = start_time + t_sec
            while time.time() < target_absolute_time:
                time.sleep(0.001) 
                
                # Si MoveIt detecta un choque inminente, cancelará la acción.
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn('¡PARADA DE EMERGENCIA solicitada por MoveIt!')
                    self.mc.stop()
                    self.mc.set_fresh_mode(0)
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()

        # 5. FINALIZAR MOVIMIENTO
        self.mc.set_fresh_mode(0) # Volver al comportamiento normal
        goal_handle.succeed()
        self.get_logger().info('Movimiento completado con éxito.')
        
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = MyCobotActionServer()
    
    # IMPORTANTE: Usamos MultiThreadedExecutor.
    # Esto permite que la función publish_joint_states y execute_callback
    # se ejecuten al mismo tiempo. Si no lo usamos, el robot se movería 
    # pero en RViz se vería congelado.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(action_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.mc.stop()
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()