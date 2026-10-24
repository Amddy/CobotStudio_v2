from Cobot_sdk import *

def menu_interactivo_cebado(funcion_movimiento, wobj = SE3()):
    """
    Muestra el menú y llama a la función que recibe como parámetro.
    """
    ultima_opcion = None

    while True:
        print("\n" + "="*30)
        print("   MENÚ DE CEBADO")
        print("="*30)
        default_txt = f" (Default: {ultima_opcion})" if ultima_opcion else ""

        print(" 1 - Poquito")
        print(" 2 - Medio")
        print(" 3 - Mucho")
        print(" 4 - Repetir")
        print(" 5 - Gracias")
        print("="*30)

        prompt = f" Opción{default_txt}: "
        entrada = input(prompt).strip()

        if entrada == '5':
            print(">>> Provecho.")
            break

        opcion = 0
        
        # Lógica de repetición y validación
        if not entrada or entrada == '4':
            if ultima_opcion is None:
                print("(!) No hay anterior.")
                primer_mate = True
                continue
            opcion = ultima_opcion
        elif entrada in ['1', '2', '3']:
            opcion = int(entrada)
            ultima_opcion = opcion
        else:
            print("(!) Opción inválida.")
            continue

        # Llamar a la función que ejecuta los movimientos
        try:
            funcion_movimiento(opcion, wobj)
        except Exception as e:
            print(f"Error: {e}")

def run(robot: BaseRobotController, **kwargs):
    # Escena
    robot.load_scene('mate_scene_v2')

    # TCP
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)

    # Robtargets termo
    tomar_termo = RobTarget(SE3(338, -65, 344)*SE3.RPY([89, 0, 59.38], unit='deg'), [1, 1, -1])
    tomar_termo_wobj = RobTarget(SE3(60.5, 178.71, -167)*SE3.RPY([-1.59, 1.22e-16, -1.56]), [1, 1, -1])
    tomar_termo_wobjmate = RobTarget(SE3(342.06, 48.46+65, -182)*SE3.Rx(-np.pi/2), [1, 1, -1])

    # Robtargets cebado
    cebar = RobTarget(SE3(273, -185, 307)*SE3.RPY([83, -2, -12], unit='deg'), [1, 1, -1])
    cebar_wobj = RobTarget(SE3(64.17, 315.3, -130)*SE3.RPY([-1.69, 0.035, -0.314]), [1, 1, -1])
    cebar_wobjmate = RobTarget(SE3(205.63, 52.17, -145)*SE3.RPY([-1.69, 0.035, 1.25]), [1, 1, -1])

    # TCP en el pico de la botella
    pico = SE3(-18, 132.5, 7)*SE3.Rx(-np.pi/2)*SE3.Rz(-np.pi/2)
    pinza_pico = pinza*pico

    # Workobject
    mesa_mate = SE3(125, -337, 162)*SE3.Rz(np.deg2rad(60))*SE3.Rx(np.pi)
    mesa_mate_quat = SE3(125, -337, 162)*UnitQuaternion([0.0, 0.8660, 0.5000, 0.0]).SE3()   # Wobj como cuaternión

    def moveit(robot:ROSManager):
        # Pose de pick del termo
        robot.moveit_manager.move_goal(tomar_termo.relTool(0, 30, 0), pinza, publish_tf=True)

        # Test de poses para cebar
        robot.check_pose_configs(cebar.relTool(0,-10,0,10), pinza_pico, filtrar=False)
        robot.moveit_manager.move_goal(cebar.relTool(0,30,0,-10), pinza_pico, publish_tf=True)
        robot.moveit_manager.move_goal(cebar.relTool(0,10,0,-5), pinza_pico, publish_tf=True)
        robot.moveit_manager.move_goal(cebar.relTool(0,0,0,0), pinza_pico, publish_tf=True)
        robot.moveit_manager.move_goal(cebar.relTool(0,-10,0,5), pinza_pico, publish_tf=True)
        robot.moveit_manager.move_goal(cebar.relTool(0,-10,0,10), pinza_pico, publish_tf=True)
        robot.moveit_manager.move_goal(cebar.relTool(0,-10,0,20), pinza_pico, publish_tf=True)
        robot.moveit_manager.move_goal(cebar.relTool(0,-15,0,25), pinza_pico, publish_tf=True)

        print(RobTarget.from_q(robot.get_current_q(), pinza_pico))
        robot.show_tf(RobTarget.from_q(robot.get_current_q(), SE3()).pose*pinza_pico, 'pico')
        robot.moveit_manager.move_goal(tomar_termo, pinza, publish_tf=True)

    def acercarse_termo(wobj = mesa_mate):
        robot.GripperState(100, 30)
        # Acercarse al termo
        robot.MoveJ(tomar_termo_wobjmate.relTool(0, 0, -85, -7), 30, pinza, wobj)
        robot.MoveJ(tomar_termo_wobjmate.relTool(0, 0, 0, -7), 30, pinza, wobj)


    def cebar_mate_wobj(opcion, wobj = mesa_mate):
        print(f"\n>>> Ejecutando secuencia de cebado (Opción {opcion})...")

        # Tomar termo
        time.sleep(1)
        robot.GripperState(40, 30)
        time.sleep(2)

        # Levantar termo
        robot.MoveJ(tomar_termo_wobjmate.relTool(0, 30, 0, -7), 30, pinza, wobj)

        # Pre cebado
        robot.MoveJ(cebar_wobjmate.relTool(0,30,0,-10), 30, pinza_pico, wobj)

        # Cebar
        if opcion == 1:
            robot.MoveJ(cebar_wobjmate.relTool(0,-10,0,5), 10, pinza_pico, wobj)
        elif opcion == 2:
            robot.MoveJ(cebar_wobjmate.relTool(0,-10,0,20), 10, pinza_pico, wobj)
        elif opcion == 3:
            robot.MoveJ(cebar_wobjmate.relTool(0,-15,0,25), 10, pinza_pico, wobj)
        time.sleep(1)
        robot.MoveJ(cebar_wobjmate.relTool(0,-10,0,10), 20, pinza_pico, wobj)
        robot.MoveJ(cebar_wobjmate.relTool(0,15,20,-5), 20, pinza_pico, wobj)
       
        # Devolver termo
        robot.MoveJ(tomar_termo_wobjmate.relTool(0, 30, 0, -7), 30, pinza, wobj)
        robot.MoveJ(tomar_termo_wobjmate.relTool(0, 0, 0, -7), 10, pinza, wobj)
        time.sleep(1)
        robot.GripperState(100, 30)
        time.sleep(2)
        # robot.MoveJ(tomar_termo_wobjmate.relTool(0, 0, -20, -7), 30, pinza, wobj)

        
    def finalizar(wobj=mesa_mate):
        robot.MoveJ(tomar_termo_wobjmate.relTool(0, 0, -85, -7), 30, pinza, wobj)
        robot.GoHome(30)

    acercarse_termo(mesa_mate)
    menu_interactivo_cebado(cebar_mate_wobj, mesa_mate)
    finalizar(mesa_mate)
    
