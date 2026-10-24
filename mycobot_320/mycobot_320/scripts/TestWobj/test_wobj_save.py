from Cobot_sdk import *
import time
import datetime
import pandas as pd
from importlib import import_module


def run(robot: ROSManager, **kwargs):


    # pinza = SE3(-1.71, 106.91, 28.33) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza_palp = SE3(0, 148, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza_aux = pinza*SE3(0,0,25)

    q_11 = [
    [-54.4, -11.86, -92.63, -12.65, 16.69, 117.59],
    [-11.68, -21.7, -113.37, -11.77, -26.01, 99.49],
    [-21.62, -129.63, 86.3, -10.63, 12.12, 58.0],
    [-76.11, -77.25, -4.3, -6.32, 20.03, 58.0],
    [-82.17, -125.77, 61.61, 29.44, 28.21, 57.04],
    [-80.77, -79.89, -20.56, 44.47, 20.3, 89.03],
]
    
    q_71 = [
    [41.04, 61.17, 52.64, -26.45, 99.31, -119.26],
    [40.86, 59.32, 74.26, -34.01, 95.53, -124.36],
    [42.71, 85.42, 63.45, -48.86, 94.65, -132.71],
    [40.42, 29.79, 24.69, 47.81, 59.67, -126.91],
    [52.47, 19.77, 36.47, 63.19, 76.2, -119.35],
    [71.8, 23.99, 28.03, 55.72, 62.05, -126.38],
]

    # robot.load_wobj('Prueba_quats', f'wobj_quat', pinza, True, q_11)
    robot.teach_and_save_wobj('Prueba_quats', f'wobj_quat3', pinza, method='6points', q_test=q_71)
    wobj = robot.load_wobj('Prueba_quats', 'wobj_quat3')
    for i, q in enumerate(q_71):
        robot.view_pose(np.radians(q), pinza_palp, robt_name=f'terna{i}')
        time.sleep(1)



    print(wobj)

    robot.show_tf(wobj, 'wobj_enseñado3')