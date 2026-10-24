# Archivo de definiciones: Workobjects
from spatialmath import SE3, UnitQuaternion
import numpy as np

# Guardado: 2026-03-15T18:51:58.262160
wobj_quat_R = np.array([
    [9.972521376306726637e-01, 6.763739003404624361e-02, 3.022180438749712955e-02],
    [7.023455967925786025e-02, -9.929690859167040751e-01, -9.528641582307005586e-02],
    [2.356439300543514137e-02, 9.714719699059018443e-02, -9.949910258385031847e-01]
  ])
wobj_quat_t = np.array([-6.733863131391804302e+01, -2.662989943810728164e+02, 6.290940661880289042e+01])

# Guardado: 2026-03-15T20:36:43.684224
wobj_quat2_q = [4.81976e-02, 9.98150e-01, 3.45319e-02, 1.34715e-02]
wobj_quat2_t = np.array([-6.73386e+01, -2.66299e+02, 6.29094e+01])
wobj_quat2 = SE3(wobj_quat2_t) * UnitQuaternion(wobj_quat2_q).SE3()

# Guardado: 2026-03-15T20:38:59.863803
wobj_quat3_q = [5.16382e-01, 4.56921e-01, 5.20201e-01, -5.03949e-01]
wobj_quat3_t = np.array([-1.60981e+02, -4.06186e+02, 3.48017e+02])
wobj_quat3 = SE3(wobj_quat3_t) * UnitQuaternion(wobj_quat3_q).SE3()

# Guardado: 2026-03-15T20:40:32.973132
wobj_quat3_q = [5.16382e-01, 4.56921e-01, 5.20201e-01, -5.03949e-01]
wobj_quat3_t = np.array([-1.60981e+02, -4.06186e+02, 3.48017e+02])
wobj_quat3 = SE3(wobj_quat3_t) * UnitQuaternion(wobj_quat3_q).SE3()

