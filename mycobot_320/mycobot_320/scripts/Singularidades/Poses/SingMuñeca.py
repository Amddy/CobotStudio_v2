# Archivo de definiciones: Poses
from spatialmath import SE3, UnitQuaternion
import numpy as np

# Guardado: 2026-04-01T13:32:24.811938
PuntoA_q = [2.504995e-01, 6.872045e-01, 2.329163e-01, 6.408978e-01]
PuntoA_t = np.array([1.141200e+02, 1.275900e+02, 2.961900e+02])
PuntoA = SE3(PuntoA_t) * UnitQuaternion(PuntoA_q).SE3()

# Guardado: 2026-04-01T13:32:24.812486
PuntoB_q = [2.504995e-01, 6.872045e-01, 2.329163e-01, 6.408978e-01]
PuntoB_t = np.array([1.941200e+02, 1.275900e+02, 2.961900e+02])
PuntoB = SE3(PuntoB_t) * UnitQuaternion(PuntoB_q).SE3()

