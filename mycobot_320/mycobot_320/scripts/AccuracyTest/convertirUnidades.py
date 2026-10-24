from scripts.CobotStudio_rev4 import RobTarget, BaseRobotController, MyCobotController, matrix_to_pose
from spatialmath import SE3
import numpy as np
from scripts.DHRobotGT import myCobot320

cobot_tb = myCobot320(rotar_base=True, metros=False)
# --- Punto 1 - 2022-04-21 11:31:21 ---
q_robot = [75.58, 34.8, 44.82, 103.0, 75.76, -179.56]
coords_robot = [38.3, -272.2, 208.5, -92.55, 1.08, 179.79]
pose_pedida = [40.00000000000002, -272.5, 218.0, -90.00000000000001, -7.016709298534876e-15, 180.0]
q_tb = [1.327258160047382, 0.5822513311047102, 0.7667305413057859, 1.792610781179297, 1.327258160047382, -3.141592653589793]
pose_alc_tb = [146.96720375420676, -125.48057895066975, 82.71749070528358, 81.88973750506112, 5.978383748147761, 28.723193826193803]

q_tb_deg = np.rad2deg(q_tb).tolist()
print(f'q_tb = {q_tb_deg}')

pose_alc_tb = matrix_to_pose(cobot_tb.fkine(np.deg2rad(q_robot)))
print(f'pose_alc_tb = {pose_alc_tb}')
