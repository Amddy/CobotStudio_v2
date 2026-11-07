import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
import numpy as np

class WorkspaceCropBox(Node):
    def __init__(self):
        super().__init__("workspace_cropbox_node")

        # Descriptores de rango para habilitar sliders en rqt_reconfigure (paso de 5 mm)
        xy_desc = ParameterDescriptor(
            description="Límite horizontal en metros respecto a base",
            floating_point_range=[FloatingPointRange(from_value=-1.0, to_value=1.0, step=0.005)]
        )
        z_desc = ParameterDescriptor(
            description="Límite vertical en metros respecto a base",
            floating_point_range=[FloatingPointRange(from_value=-0.30, to_value=1.0, step=0.005)]
        )

        # Parámetros con valores por defecto enfocados en el MyCobot 320
        self.declare_parameter("target_frame", "base")
        self.declare_parameter("min_x", -0.35, xy_desc)  # Atrás de la base
        self.declare_parameter("max_x",  0.55, xy_desc)  # Frente de la mesa
        self.declare_parameter("min_y", -0.42, xy_desc)  # Lado derecho
        self.declare_parameter("max_y",  0.42, xy_desc)  # Lado izquierdo
        self.declare_parameter("min_z",  0.02, z_desc)   # Corte de mesa (2 cm sobre base)
        self.declare_parameter("max_z",  0.65, z_desc)   # Techo del espacio de trabajo

        # Cache local de variables para no consultar el servidor de parámetros en cada frame
        self.target_frame = self.get_parameter("target_frame").value
        self.min_x = float(self.get_parameter("min_x").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.min_y = float(self.get_parameter("min_y").value)
        self.max_y = float(self.get_parameter("max_y").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)

        self.add_on_set_parameters_callback(self.on_params_changed)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            PointCloud2, "/point_cloud/points", self.cloud_cb, qos
        )
        self.pub = self.create_publisher(PointCloud2, "/cropped_points", qos)
        self.get_logger().info(
            "CropBox con sliders activo (/point_cloud/points -> /cropped_points). "
            "Ajusta en vivo con 'ros2 run rqt_reconfigure rqt_reconfigure'"
        )

    def on_params_changed(self, params):
        for p in params:
            if p.name == "target_frame":
                self.target_frame = str(p.value)
            elif p.name == "min_x":
                self.min_x = float(p.value)
            elif p.name == "max_x":
                self.max_x = float(p.value)
            elif p.name == "min_y":
                self.min_y = float(p.value)
            elif p.name == "max_y":
                self.max_y = float(p.value)
            elif p.name == "min_z":
                self.min_z = float(p.value)
            elif p.name == "max_z":
                self.max_z = float(p.value)

        print(
            f"\n---> Límites CropBox actualizados para demo.launch.py:\n"
            f'     "-p", "min_x:={self.min_x:.3f}", "-p", "max_x:={self.max_x:.3f}",\n'
            f'     "-p", "min_y:={self.min_y:.3f}", "-p", "max_y:={self.max_y:.3f}",\n'
            f'     "-p", "min_z:={self.min_z:.3f}", "-p", "max_z:={self.max_z:.3f}",'
        )
        return SetParametersResult(successful=True)

    def cloud_cb(self, msg: PointCloud2):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, rclpy.time.Time()
            )
        except Exception:
            return

        pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        if pts.shape[0] == 0:
            return

        xyz = pts[:, :3]

        q = trans.transform.rotation
        t = trans.transform.translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
            [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ], dtype=np.float32)
        T = np.array([t.x, t.y, t.z], dtype=np.float32)

        xyz_base = xyz @ R.T + T

        mask = (
            (xyz_base[:, 0] >= self.min_x) & (xyz_base[:, 0] <= self.max_x) &
            (xyz_base[:, 1] >= self.min_y) & (xyz_base[:, 1] <= self.max_y) &
            (xyz_base[:, 2] >= self.min_z) & (xyz_base[:, 2] <= self.max_z)
        )

        pts_cropped = np.empty((np.count_nonzero(mask), 4), dtype=np.float32)
        pts_cropped[:, :3] = xyz_base[mask]
        pts_cropped[:, 3] = pts[mask, 3]

        out_msg = pc2.create_cloud(msg.header, msg.fields, pts_cropped)
        out_msg.header.frame_id = self.target_frame
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    node = WorkspaceCropBox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
