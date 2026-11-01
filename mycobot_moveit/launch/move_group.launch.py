from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mycobot_moveit").to_moveit_configs()

    # Formato exacto de parámetros del occupancy_map_monitor
    moveit_config.sensors_3d = {
        "octomap_frame": "base", 
        "octomap_resolution": 0.025, # Define el tamaño de cada voxel (metros)
        "max_range": 1.0, # Distancia maxima del procesado de puntos del optomap ( metros)
        "sensors": ["xtion_pointcloud"],
        "xtion_pointcloud.sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",
        "xtion_pointcloud.point_cloud_topic": "/point_cloud/points",
        "xtion_pointcloud.max_range": 1.0,
        "xtion_pointcloud.point_subsample": 2,  # Cantidad de puntos de la nube para construir un voxel
        "xtion_pointcloud.padding_offset": 0.04, # Margen de seguridad en las mallas de robot
        "xtion_pointcloud.padding_scale": 1.0, # Factor multiplicador del volumen de los eslabones del robot para el auto-filtrado.
        "xtion_pointcloud.max_update_rate": 10.0, # Cantidad máxima de veces por segundo que MoveIt recalcula el mapa de vóxeles.
        "xtion_pointcloud.filtered_cloud_topic": "/filtered_points", # topico donde se publica en moveit la nube filtrada
    }

    return generate_move_group_launch(moveit_config)