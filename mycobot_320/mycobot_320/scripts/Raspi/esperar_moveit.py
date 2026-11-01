import time
from CobotStudio import BaseRobotController

def run(robot: BaseRobotController, **kwargs):
    robot.logger.info("Servidor listo. Esperando trayectorias de MoveIt desde la PC (Ctrl+C para salir)...")
    while True:
        time.sleep(1.0)