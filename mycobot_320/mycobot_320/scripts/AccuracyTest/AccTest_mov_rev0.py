from CobotStudio_core.Cobot_sdk import *
import datetime


log_point_counter = 0

def run(robot: BaseRobotController, speed = 10, **kwargs):
    robot.load_scene('AccTest_Scene_rev0')
    def data_reg(robt, tool, filename, sleep=1):
        global log_point_counter
        time.sleep(sleep)
        
        try:
            lectura_robot = robot.mc.get_angles_coords()
            q_robot = lectura_robot[:6]
            coords_robot = lectura_robot[6:] # Asumo que las coordenadas son los últimos elementos
        except Exception as e:
            print(f"No se pudo acceder al controlador del robot: {e}. Devolviendo q de ROS2:")
            q_robot = robot.q_current[:6]
            coords_robot = [] # Dejar vacío o con un valor por defecto si no se pueden obtener
            print(f'q_ROS\n{q_robot}')

        pose_pedida = robot.matrix_to_pose(robt.pose * tool.inv())
        print(f"En xyzrpy mandamos al cobot a\n{pose_pedida}")

        q_tb = robot.cobot_tb.ikine(robt.pose * tool.inv(), robt.config)[0]
        print(f'Según la tb se llegaba con los ángulos\n{q_tb}')

        pose_alc_tb = robot.matrix_to_pose(robot.cobot_tb.fkine(q_robot))
        print(f'Los ángulos devueltos por el robot según la tb van a\n{pose_alc_tb}')

        file_path = robot._resolve_project_path('Datos', filename)
        try:
            with open(file_path, 'a') as f:
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                f.write(f"# --- Punto {log_point_counter} - {timestamp} ---\n")

                f.write(f"q_robot = {list(q_robot)}\n")
                f.write(f"coords_robot = {list(coords_robot)}\n")
                f.write(f"pose_pedida = {list(pose_pedida)}\n")
                f.write(f"q_tb = {list(q_tb)}\n")
                f.write(f"pose_alc_tb = {list(pose_alc_tb)}\n")
                f.write("\n") # Añadir una línea en blanco para separar los puntos

            print(f"Datos del punto {log_point_counter} guardados en '{filename}'")
            log_point_counter += 1

        except Exception as e:
            print(f"Error al guardar los datos en el archivo: {e}")

        time.sleep(sleep)

    # TCPs
    # pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    l_marker = 35
    # d_offset = 15
    pinza_marker = pinza * SE3(0, 0, l_marker)
    wobj = SE3() # wobj nulo por simplicidad
    home = RobTarget(robot.cobot_tb.fkine(np.repeat(0, 6)), [1, 1, 1])

    def ensayo1 (d_offset = 15, iteracion = 0):
        file_log = f'ensayo1_v{iteracion}'
        punto_centro = RobTarget(SE3(0, -260, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1]) #[-1, 1, 1], [1, -1, -1]
        # robot.testPose(punto_centro, pinza_marker, wobj)
        punto1 = RobTarget(SE3(40, -300, 60)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto2 = RobTarget(SE3(-40, -300, 60)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto3 = RobTarget(SE3(40, -220, 60)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto4 = RobTarget(SE3(-40, -220, 60)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        puntos = [punto_centro, punto1, punto2, punto3, punto4]

        robot.MoveJ(punto_centro.offset(0, 0, d_offset), 30, pinza_marker, robt_name='Centro')
        print(punto_centro.offset(0, 0, d_offset).pose)
        robot.GripperState(100, 30)
        time.sleep(7)
        robot.GripperState(0, 30)
        time.sleep(3)
        for idx, punto in enumerate(puntos[1:], 1):
            robot.MoveJ(punto.offset(0, 0, d_offset), 30, pinza_marker, wobj)
            robot.MoveL(punto, 30, pinza_marker, wobj, robt_name=f'p{idx}')
            print(f'Llegamos al punto {idx}')
            # data_reg(punto, pinza_marker, file_log)
            robot.MoveL(punto.offset(0, 0, d_offset), 30, pinza_marker, wobj)
            robot.MoveJ(punto_centro.offset(0, 0, d_offset), 30, pinza_marker, wobj, robt_name='Centro')
            # data_reg(punto_centro, pinza_marker, file_log)
        
        robot.MoveJ(punto_centro.offset(0, 0, 50), 30, pinza_marker, wobj)
        print(f'Volvemos al centro y terminamos.')
        robot.GripperState(100, 30)
        time.sleep(7)
    
        # Volvemos a Home
        robot.MoveJAngles(np.zeros(6), 30)
        # robot.MoveJ(home, 30, SE3(), wobj)
 
    def ensayo2 (d_offset = 15, iteracion = 0):
        file_log = f'ensayo2_v{iteracion}'
        punto_centro = RobTarget(SE3(100, 200, 35)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, 1]) #(-1, 1, 1) [1, -1, 1]
        punto1 = RobTarget(SE3(70, 230, 35)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, 1])
        punto2 = RobTarget(SE3(130, 230, 35)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, 1])
        punto3 = RobTarget(SE3(70, 170, 35)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, 1])
        punto4 = RobTarget(SE3(130, 170, 35)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, 1])
        puntos = [punto_centro, punto1, punto2, punto3, punto4]

        robot.MoveJ(punto_centro.offset(0, 0, d_offset), 30, pinza_marker, robt_name='Centro')
        print(punto_centro.offset(0, 0, d_offset).pose)
        robot.GripperState(100, 30)
        time.sleep(7)
        robot.GripperState(0, 30)
        time.sleep(3)

        for idx, punto in enumerate(puntos[1:], 1):
            # robot.testPose(punto, pinza_marker, wobj)
            robot.MoveJ(punto.offset(0, 0, d_offset), 30, pinza_marker, wobj)
            robot.MoveL(punto, 30, pinza_marker, wobj, robt_name=f'punto{idx}')
            print(f'Llegamos al punto {idx}')
            data_reg(punto, pinza_marker, file_log)
            robot.MoveL(punto.offset(0, 0, d_offset), 30, pinza_marker, wobj)
            robot.MoveJ(punto_centro.offset(0, 0, d_offset), 30, pinza_marker, wobj, robt_name='Centro')
            data_reg(punto_centro, pinza_marker, file_log)
        
        robot.MoveJ(punto_centro.offset(0, 0, 20), 30, pinza_marker, wobj)
        print(f'Volvemos al centro y terminamos.')
        robot.GripperState(100, 30)
        time.sleep(7)
    
        # Volvemos a Home
        robot.MoveJAngles(np.zeros(6), 30)
        # robot.MoveJ(home, 30, SE3(), wobj)

    def ensayo3(iteracion = 0):
        file_log = f'ensayo3_v{iteracion}'
        robot.GripperState(100, 30)
        time.sleep(7)
        robot.GripperState(0, 30)
        time.sleep(3)

        punto5 = RobTarget(SE3(150, -250, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto6 = RobTarget(SE3(100, -250, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto7 = RobTarget(SE3(100, -235, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto8 = RobTarget(SE3(150, -235, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])

        puntos = [punto5, punto6, punto7, punto8]

        # robot.MoveJ(punto5.offset(0, 0, 15), 30, pinza_marker, wobj)
        for punto in puntos:
            if punto == punto6 or punto == punto8:
                robot.MoveL(punto, 30, pinza_marker, wobj)
            else: 
                robot.MoveJ(punto.offset(0, 0, 15), 30, pinza_marker, wobj)
                time.sleep(2)
                robot.MoveJ(punto, 30, pinza_marker, wobj)
            data_reg(punto, pinza_marker, file_log)

        robot.MoveJ(punto8.offset(0, 0, 15), 30, pinza_marker, wobj)
        time.sleep(3)
        robot.GripperState(100, 30)
        time.sleep(2)
        robot.MoveJAngles(np.zeros(6), 30)
    
    def ensayo4(iteracion = 0):
        file_log = f'ensayo4_v{iteracion}'
        robot.GripperState(100, 30)
        time.sleep(7)
        robot.GripperState(0, 30)
        time.sleep(3)

        punto1 = RobTarget(SE3(150, -150, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, -1]) #[-1, 1, 1], [1, -1, -1]
        punto2 = RobTarget(SE3(150, -230, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, -1]) 
        punto3 = RobTarget(SE3(135, -230, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, -1]) 
        punto4 = RobTarget(SE3(135, -150, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, -1])

        puntos = [punto1, punto2, punto3, punto4]
        for punto in puntos:
            if punto == punto2 or punto == punto4:
                robot.MoveL(punto, 15, pinza_marker, wobj)
            else:
                robot.MoveJ(punto.offset(0, 0, 5), 30, pinza_marker, wobj)
                time.sleep(2)
                robot.MoveJ(punto, 30, pinza_marker, wobj)
            data_reg(punto, pinza_marker, file_log)

        robot.MoveJ(punto4.offset(0, 0, 15), 30, pinza_marker, wobj)
        time.sleep(3)
        robot.GripperState(100, 30)
        time.sleep(2)
        robot.MoveJAngles(np.zeros(6), 30)

    ensayo = int(input("Seleccione ensayo a ejecutar (1, 2, 3 ó 4): "))
    it = 25
    if ensayo == 1:
        ensayo1(iteracion = it)
    elif ensayo == 2:
        ensayo2(iteracion = it)
    elif ensayo == 3:
        ensayo3(iteracion = it)
    elif ensayo == 4:
        ensayo4(iteracion = it)
    else:
        print("Opción no válida.")
        return
