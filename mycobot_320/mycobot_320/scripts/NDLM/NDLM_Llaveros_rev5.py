from Cobot_sdk import *
import random

def run(robot: ROSManager, speed):
    robot.load_scene("NDLM_scene")
    
    # TCP
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    # Wobjs y robt comunes
    wobj_ref = SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/6.6)*SE3(7, 282, 0)
    dar = RobTarget(SE3(280, -135.721, 200)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi), [-1, 1, 1])

    def no_llavero():
        """
        Cierre prematuro de la pinza, no da llavero.
        """
        robot.GripperState(0, 30) # Cierre de la pinza
        time.sleep(3)
        nop = RobTarget(SE3(215, 0.5, 300)*SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, 1, 1]) # 111 | 1-11 | -11-1 | -1-1-1!!
        robot.MoveJ(nop, 50, pinza, SE3())
        robot.MoveJ(nop.relTool(-30, 0, -5, 0, -10, 0), 30, pinza, SE3())
        robot.MoveJ(nop.relTool(30, 0, -5, 0, 10, 0), 30, pinza, SE3())
        robot.MoveJ_q(np.zeros(6), speed)
        robot.GripperState(100, 30)
        
    def amague(pose_pick):
        """
        Cierre parcial de la pinza en la posición del llavero.
        """
        robot.GripperState(20, 30) # Cierre de la pinza
        time.sleep(3)
        robot.MoveJ(pose_pick.offset(0, 0, -40), speed, pinza, wobj_ref)
        nop = RobTarget(SE3(215, 0.5, 300)*SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, 1, 1]) # 111 | 1-11 | -11-1 | -1-1-1!!
        robot.MoveJ(nop, 50, pinza, SE3())
        robot.MoveJ(nop.relTool(-30, 0, -5, 0, -10, 0), 30, pinza, SE3())
        robot.MoveJ(nop.relTool(30, 0, -5, 0, 10, 0), 30, pinza, SE3())
        robot.MoveJ_q(np.zeros(6), speed)
        robot.GripperState(100, 30)

    def dar_llavero(llavero, speed = 30):
        if llavero == 1:
            pick = RobTarget(SE3(0, 10, -40), [-1, 1, 1])

        elif llavero == 2:
            # pick = RobTarget(SE3(115, 10, -40)*SE3.Rz(-0.06), [-1, 1, 1])
            pick = RobTarget(SE3(115, 10, -40), [-1, 1, 1])
            
        elif llavero == 3:
            pick = RobTarget(SE3(-105, 10, -40), [-1, 1, 1])

        robot.GripperState(100, 30) # Abrir la pinza
        # Acercamiento al llavero
        robot.MoveJ(pick.offset(0, 0, -40), speed, pinza, wobj_ref)

        dado = random.randint(1, 6) # Define si da o no llavero
        dado = 1
        if dado == 2: # No da llavero y cierra la pinza antes
            no_llavero()
            return

        # Posición de pick
        robot.MoveL(pick, speed, pinza, wobj_ref)

        if dado == 4: # No da llavero por no cerrar del todo la pinza
            amague(pick)
            return
        
        robot.GripperState(0, 40) # Cierre de la pinza
        time.sleep(3) # Tiempo para que cierre
        # Sacar llavero de la base
        robot.MoveL(pick.offset(0, 0, -40), 30, pinza, wobj_ref)
        # Dar el llavero
        robot.MoveJ(dar, speed, pinza)
        robot.MoveJ(dar.relTool(0, 0, 50), 30, pinza)
        time.sleep(2)
        robot.GripperState(100, 30) # Abrir pinza
        time.sleep(5) # Tiempo para agarrar el llavero
        # Volver al home
        robot.MoveJ_q(np.zeros(6), speed)


    def saludar():
        saludo1 = RobTarget(SE3(215, 0.5, 425)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi/2), [-1, 1, 1])
        saludo2 = RobTarget(SE3(250, 0.5, 350)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi/2)*SE3.Rx(np.pi/4), [-1, 1, 1])
        robot.MoveJ(saludo1, 50, pinza, SE3())
        robot.MoveJ(saludo2, 40, pinza, SE3())
        robot.MoveJ(saludo1, 30, pinza, SE3())

    def recorrer():
        punto1 = RobTarget(SE3(324, -30, 90)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])
        punto2 = RobTarget(SE3(223, -225, 90)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])
        punto3 = RobTarget(SE3(270, -130, 90)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])
        # robot.testPose(punto1, pinza, SE3())
        robot.MoveJ(punto1, 50, pinza, SE3())
        robot.MoveJ(punto2, 30, pinza, SE3())
        robot.MoveJ(punto3, 30, pinza, SE3())

    def testPoses():
        """
        Chequeo de las posiciones de los porta llaveros.
        """
        llav = int(input("Seleccione llavero 1, 2 o 3:"))
        dar_llavero(llav)


    saludar()
    recorrer()
    random_num = random.randint(1, 3)
    dar_llavero(random_num)