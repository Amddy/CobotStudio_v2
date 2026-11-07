import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )

class DynamicCameraTF(Node):
    def __init__(self):
        super().__init__("camera_base_tf_publisher")

        pos_desc = ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=-1.5, to_value=1.5, step=0.005)]
        )
        ang_desc = ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=-180.0, to_value=180.0, step=0.5)]
        )

        self.declare_parameter("frame_id", "base")
        self.declare_parameter("child_frame_id", "camera_link")
        self.declare_parameter("x", 0.60, pos_desc)
        self.declare_parameter("y", 0.00, pos_desc)
        self.declare_parameter("z", 0.35, pos_desc)
        self.declare_parameter("roll_deg", 0.0, ang_desc)
        self.declare_parameter("pitch_deg", 0.0, ang_desc)
        self.declare_parameter("yaw_deg", 180.0, ang_desc)

        # Variables en caché local (evita el retraso de get_parameter en callbacks)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.roll_deg = float(self.get_parameter("roll_deg").value)
        self.pitch_deg = float(self.get_parameter("pitch_deg").value)
        self.yaw_deg = float(self.get_parameter("yaw_deg").value)

        # Publicamos tanto en /tf_static (para sobreescribir el launch) como en /tf
        self.static_br = StaticTransformBroadcaster(self)
        self.dyn_br = TransformBroadcaster(self)

        self.add_on_set_parameters_callback(self.on_params_changed)
        self.timer = self.create_timer(0.05, self.broadcast_tf)
        self.broadcast_tf()
        self.get_logger().info("TF Dinámico listo (actualizando /tf y /tf_static en tiempo real).")

    def on_params_changed(self, params):
        for p in params:
            if p.name == "x":
                self.x = float(p.value)
            elif p.name == "y":
                self.y = float(p.value)
            elif p.name == "z":
                self.z = float(p.value)
            elif p.name == "roll_deg":
                self.roll_deg = float(p.value)
            elif p.name == "pitch_deg":
                self.pitch_deg = float(p.value)
            elif p.name == "yaw_deg":
                self.yaw_deg = float(p.value)
            elif p.name == "frame_id":
                self.frame_id = str(p.value)
            elif p.name == "child_frame_id":
                self.child_frame_id = str(p.value)

        # Enviar inmediatamente la nueva rotación sin esperar al timer
        self.broadcast_tf()
        self.print_launch_args()
        return SetParametersResult(successful=True)

    def print_launch_args(self):
        r = math.radians(self.roll_deg)
        p = math.radians(self.pitch_deg)
        yw = math.radians(self.yaw_deg)
        print(
            f"\n---> Para tu demo.launch.py (RPY en deg: {self.roll_deg:.1f}°, {self.pitch_deg:.1f}°, {self.yaw_deg:.1f}°):\n"
            f'     "--x", "{self.x:.3f}", "--y", "{self.y:.3f}", "--z", "{self.z:.3f}",\n'
            f'     "--roll", "{r:.4f}", "--pitch", "{p:.4f}", "--yaw", "{yw:.4f}",',
            flush=True
        )

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        r = math.radians(self.roll_deg)
        p = math.radians(self.pitch_deg)
        yw = math.radians(self.yaw_deg)
        qx, qy, qz, qw = euler_to_quaternion(r, p, yw)

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.static_br.sendTransform(t)
        self.dyn_br.sendTransform(t)

def main():
    rclpy.init()
    node = DynamicCameraTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
