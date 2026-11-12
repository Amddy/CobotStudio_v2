# Archivo de definiciones: Poses
from spatialmath import SE3, UnitQuaternion
import numpy as np

# Guardado: 2026-11-10T15:14:39.829803
Pose_1_q = [1.622566e-16, 7.071068e-01, 7.071068e-01, -1.849776e-16]
Pose_1_t = np.array([1.573623e+02, -1.502568e+02, 2.065498e+02])
Pose_1 = SE3(Pose_1_t) * UnitQuaternion(Pose_1_q).SE3()

# Guardado: 2026-11-10T15:15:00.594599
Pose_2_q = [4.281849e-06, -7.071547e-01, -7.070588e-01, -1.455188e-05]
Pose_2_t = np.array([1.573509e+02, -1.502803e+02, 2.065488e+02])
Pose_2 = SE3(Pose_2_t) * UnitQuaternion(Pose_2_q).SE3()

# Guardado: 2026-11-10T15:15:18.236548
Pose_3_q = [5.795817e-01, -5.795817e-01, -4.050742e-01, 4.050742e-01]
Pose_3_t = np.array([1.573623e+02, -1.502568e+02, 2.065498e+02])
Pose_3 = SE3(Pose_3_t) * UnitQuaternion(Pose_3_q).SE3()

# Guardado: 2026-11-10T15:21:14.239957
Pose_1_q = [1.874159e-02, -8.941842e-01, -4.457122e-01, -3.773549e-02]
Pose_1_t = np.array([2.029130e+02, -2.759663e+02, 1.802331e+02])
Pose_1 = SE3(Pose_1_t) * UnitQuaternion(Pose_1_q).SE3()

# Guardado: 2026-11-10T15:21:15.517783
Pose_2_q = [1.874159e-02, -8.941842e-01, -4.457122e-01, -3.773549e-02]
Pose_2_t = np.array([2.029130e+02, -2.759663e+02, 1.802331e+02])
Pose_2 = SE3(Pose_2_t) * UnitQuaternion(Pose_2_q).SE3()

