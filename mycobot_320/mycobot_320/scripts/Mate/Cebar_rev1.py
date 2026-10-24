from __future__ import annotations
from spatialmath import SE3
import numpy as np
import time
from CobotStudio_core.RobtManager import RobTarget
from CobotStudio_core.CobotStudio import BaseRobotController
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from CobotStudio_core.CobotStudio import SimManager

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

        # Llamamos a la función que nos pasaron.
        # No hace falta saber qué robot es ni qué wobj usa.
        try:
            funcion_movimiento(opcion, wobj)
        except Exception as e:
            print(f"Error: {e}")

def run(robot: BaseRobotController, **kwargs):
    # Escena
    robot.load_scene('mate_scene_v2')

    # TCP
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)


    tomar_termo = RobTarget(SE3(338, -65, 344)*SE3.RPY([89, 0, 59.38], unit='deg'), [1, 1, -1])
    tomar_termo_wobj = RobTarget(SE3(60.5, 178.71, -167)*SE3.RPY([-1.59, 1.22e-16, -1.56]), [1, 1, -1])
    tomar_termo_wobjmate = RobTarget(SE3(342.06, 48.46+65, -182)*SE3.RPY([-1.59, -1.32e-18, 1.08e-2]), [1, 1, -1])
    # tomar_termo_wobjmate = RobTarget(SE3(342.06, 48.46, -182)*SE3.Rx(-np.pi/2), [1, 1, -1])

    cebar = RobTarget(SE3(273, -185, 307)*SE3.RPY([83, -2, -12], unit='deg'), [1, 1, -1])
    cebar_wobj = RobTarget(SE3(64.17, 315.3, -130)*SE3.RPY([-1.69, 0.035, -0.314]), [1, 1, -1])
    cebar_wobjmate = RobTarget(SE3(205.63, 52.17, -145)*SE3.RPY([-1.69, 0.035, 1.25]), [1, 1, -1])


    pico = SE3(-18, 132.5, 7)*SE3.Rx(-np.pi/2)*SE3.Rz(-np.pi/2)
    pinza_pico = pinza*pico
    mesa = SE3(375, 120, 177)*SE3.Rz(np.deg2rad(-30))*SE3.Rx(np.pi)
    mesa_mate = SE3(125, -337, 162)*SE3.Rz(np.deg2rad(60))*SE3.Rx(np.pi)



    def moveit(robot:SimManager):
       
        q_tomar = np.deg2rad([7.2, -107.92, 63.72, -44.29, -94.39, 142.99, -40])
        # robot.moveit_adapter.plan_and_execute(tomar_termo_wobjmate.relTool(0, 0, 0, -7), tool=pinza, wobj=mesa_mate, execute=True)
        # robot.moveit_adapter.plan_and_execute(q_tomar, tool=pinza, wobj=mesa_mate, execute=True)
        # robot.moveit_adapter.plan_and_execute(tomar_termo_wobjmate.relTool(0, 0, 0, -7), tool=pinza, wobj=mesa_mate, execute=True)
        #robot.moveit_adapter.save_last_planned_trajectory('trajsmvt', 'prueba1')
        robot.toggle_ros_connection(False)
        robot.traj_moveit('trajsmvt', 'prueba1', pinza)
        time.sleep(3)
        # robot.moveit_adapter.apply_goal_state(np.zeros(6), pinza, publish_tf=True)
        # robot.explore_ik_configs(cebar.relTool(0,15,20,-5), pinza_pico)
        # robot.moveit_adapter.apply_goal_state(q_tomar, pinza, publish_tf=True)
        # robot.MostrarTerna(tomar_termo_wobjmate.relTool(0, 0, 0, -7).pose*mesa_mate.inv(), )
        # robot.moveit_adapter.apply_goal_state(cebar.relTool(0,15,20,-5), pinza_pico, publish_tf=True)
        # robot.explore_ik_configs(cebar.relTool(0,-10,0,10), pinza_pico, filtrar=False)
        # robot.moveit_adapter.apply_goal_state(tomar_termo.relTool(0, 30, 0), pinza, publish_tf=True)

        # robot.moveit_adapter.apply_goal_state(cebar_wobjmate, pinza_pico, mesa_mate, True)

        # robot.moveit_adapter.apply_goal_state(cebar.relTool(0,30,0,-10), pinza_pico, publish_tf=True)
        # robot.moveit_adapter.apply_goal_state(cebar.relTool(0,10,0,-5), pinza_pico, publish_tf=True)
        # robot.moveit_adapter.apply_goal_state(cebar.relTool(0,0,0,0), pinza_pico, publish_tf=True)
        # robot.moveit_adapter.apply_goal_state(cebar.relTool(0,-10,0,5), pinza_pico, publish_tf=True)
        # robot.moveit_adapter.apply_goal_state(cebar.relTool(0,-10,0,10), pinza_pico, publish_tf=True)
        # robot.moveit_adapter.apply_goal_state(cebar.relTool(0,-10,0,20), pinza_pico, publish_tf=True)
        # robot.moveit_adapter.apply_goal_state(cebar.relTool(0,-15,0,25), pinza_pico, publish_tf=True)


        # robot.moveit_adapter.plan_and_execute(cebar, tool=pinza_pico, execute=True)
        # robot.moveit_adapter.plan_and_execute(tomar_termo.relTool(0, 0, 65, -7), tool=pinza, execute=True)
        # robot.moveit_adapter.plan_and_execute(cebar_pico3.relTool(0, 30, 0), cebar_pico3.relTool(0,110,0,-35), tool=pinza_pico, execute=False)
        # robot.moveit_adapter.plan_and_execute(cebar_pico3.relTool(0,20,0,-15), cebar_pico3.relTool(0,110,0,-35), tool=pinza_pico, execute=False)
        # robot.moveit_adapter.plan_and_execute(cebar_pico3.relTool(0,30,0,0), cebar_pico3.relTool(0,20,0,-15), tool=pinza_pico, execute=False)
        # robot.moveit_adapter.plan_and_execute(cebar_pico.relTool(0,0,0,0), tool=pinza_pico, execute=True)
        # robot.moveit_adapter.plan_and_execute(tomar_termo31.relTool(0, 0, 50), tool=pinza, execute=True)


        # print(RobTarget.from_q(robot.get_current_q(), pinza_pico))
        # robot.MostrarTerna(RobTarget.from_q(robot.get_current_q(), SE3()).pose*pinza_pico, 'pico')
        # robot.MostrarTerna(terna_pinza*pico, 'pico')
        # robot.moveit_adapter.apply_goal_state(tomar_termo, pinza, publish_tf=True)
        # robot.MostrarTerna(cebar_max2.offset(0, 0, 10).pose*pico, 'pico_cebando')

    def movs(wobj = SE3()):

        robot.GripperState(100, 30)
        # Acercarse al termo
        robot.MoveJ(tomar_termo.relTool(0, 0, -20, -7), 30, pinza, wobj)
        # Tomar termo
        robot.MoveJ(tomar_termo.relTool(0, 0, 65, -7), 30, pinza, wobj)
        time.sleep(1)
        robot.GripperState(20, 30)
        time.sleep(2)

        # Levantar termo
        robot.MoveJ(tomar_termo.relTool(0, 30, 65, -7), 30, pinza, wobj)

        # Pre cebado
        robot.MoveJ(cebar.relTool(0,30,0,-10), 30, pinza_pico, wobj)

        # Cebar
        robot.MoveJ(cebar.relTool(0,-10,0,5), 10, pinza_pico, wobj)
        robot.MoveJ(cebar.relTool(0,-10,0,20), 10, pinza_pico, wobj)
        robot.MoveJ(cebar.relTool(0,-15,0,25), 10, pinza_pico, wobj)
        time.sleep(1)
        robot.MoveJ(cebar.relTool(0,-10,0,10), 10, pinza_pico, wobj)
        robot.MoveJ(cebar.relTool(0,15,20,-5), 10, pinza_pico, wobj)
        # robot.MoveJ(cebar.relTool(0,0,0,-10), 30, pinza_pico)

        # time.sleep(1)
        
        # Devolver termo
        robot.MoveJ(tomar_termo.relTool(0, 30, 65, -7), 30, pinza, wobj)
        robot.MoveJ(tomar_termo.relTool(0, 0, 65, -7), 10, pinza, wobj)
        time.sleep(1)
        robot.GripperState(100, 30)
        time.sleep(2)

        robot.MoveJ(tomar_termo.relTool(0, 0, -20, -7), 30, pinza, wobj)


    def acercarse_termo(wobj = mesa_mate):
        robot.GripperState(100, 30)
        # Acercarse al termo
        print(tomar_termo_wobjmate.relTool(0, 0, -85, -7).pose*wobj.inv())
        robot.MostrarTerna(tomar_termo_wobjmate.relTool(0, 0, -85, -7).pose*wobj.inv())
        robot.MoveJ(tomar_termo_wobjmate.relTool(0, 0, -85, -7), 30, pinza, wobj)
        robot.MoveL(tomar_termo_wobjmate.relTool(0, 0, 0, -7), 30, pinza, wobj)


    def cebar_mate_wobj(opcion, wobj = mesa_mate):
        print(f"\n>>> Ejecutando secuencia de cebado (Opción {opcion})...")

        # Tomar termo
        time.sleep(1)
        robot.GripperState(20, 30)
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
        robot.MoveJ(cebar_wobjmate.relTool(0,-10,0,10), 10, pinza_pico, wobj)
        robot.MoveJ(cebar_wobjmate.relTool(0,15,20,-5), 10, pinza_pico, wobj)
        # robot.MoveJ(cebar_wobjmate.relTool(0,0,0,-10), 30, pinza_pico)

        # time.sleep(1)
        
        # Devolver termo
        robot.MoveJ(tomar_termo_wobjmate.relTool(0, 30, 0, -7), 30, pinza, wobj)
        robot.MoveJ(tomar_termo_wobjmate.relTool(0, 0, 0, -7), 10, pinza, wobj)
        time.sleep(1)
        robot.GripperState(100, 30)
        time.sleep(2)
        # robot.MoveJ(tomar_termo_wobjmate.relTool(0, 0, -20, -7), 30, pinza, wobj)

        
    def finalizar(wobj=mesa_mate):
        robot.MoveL(tomar_termo_wobjmate.relTool(0, 0, -85, -7), 30, pinza, wobj)
        robot.GoHome(30)

    def tests():
        # moveit(robot)
        # movs()
        # robot.MostrarTerna(mesa, 'esquina')
        robot.MostrarTerna(mesa_mate, 'mesa_mate')
        
        # robot.MostrarTerna(SE3(375, 120, 177)*SE3.Rz(np.deg2rad(-30))*SE3.Rx(np.pi), 'esquina')
        robot.MostrarTerna(tomar_termo.pose, 'tomar_og')
        robot.MostrarTerna(tomar_termo_wobjmate.pose, 'tomar_wobj', mesa_mate)
        robot.MostrarTerna(cebar.pose, 'cebar_og')
        robot.MostrarTerna(cebar_wobjmate.pose, 'cebar_wobj', mesa_mate)
        time.sleep(2)
        # print((mesa.inv()*tomar_termo.pose).rpy())
        # print((mesa.inv()*cebar.pose).rpy())
        print((mesa_mate.inv()*cebar.pose).t)
        print((mesa_mate.inv()*cebar.pose).rpy())
        # robot.MoveJ(cebar_wobj.relTool(0,-10,0,5), 10, pinza_pico, mesa)
    
    def test_sim(robot:SimManager):
        robot.GripperState(20)
        q_test = np.deg2rad([7.2, 107.92, 63.72, -44.29, 94.39, 142.99])
        robot.VerPose(q_test, pinza, wobj_name='mesa_mate', robt_name='test_robtarget')
        robot.testPose(cebar_wobjmate, pinza_pico, mesa_mate)
        robot.get_current_q(True, get_robt=True, tool=pinza_pico, wobj=mesa_mate)
    # wobj_actual = mesa_mate
    # acercarse_termo(wobj_actual)
    # menu_interactivo_cebado(cebar_mate_wobj, wobj_actual)
    # finalizar(wobj_actual)
    
    # tests()
    moveit(robot)

    # test_sim(robot)
    time.sleep(2)

    
