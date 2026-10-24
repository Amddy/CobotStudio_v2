from CobotStudio_core.Cobot_sdk import *

def setup_scene(robot: ROSManager, caso = 1):
        altura_mesa = 0.05 # Altura de los bloques de madera a apilar
        brida_base = 0.02
    
        # Agregamos los objetos según sus dimensiones        
        # robot_manager.node_obj.add_object("papel", pose_init=(0.0, -425.00e-3, altura_mesa/2- brida_base),
        #                 size=(300e-3, 300e-3, altura_mesa),
        #                 color=(0.88, 0.88, 0.88, 0.8),
        #                 shape=CUBE, movable=True, rot_euler=np.deg2rad((0, 0, 0)))

        caso = int(input("Seleccione ensayo para cargar la escena (1, 2, 3, 4): "))

        if caso == 1:
            posicion_marcador = (0.0, -260.0e-3, 100.0e-3)
        elif caso == 2:
             posicion_marcador = (100.0e-3, 200.0e-3, 100.0e-3)
        elif caso == 3:
             posicion_marcador = (0.0e-3, -182.0e-3, 417.0e-3)
        elif caso == 4:
             posicion_marcador = (0.0e-3, -182.0e-3, 417.0e-3)
        else:
             print('Opción no válida.')
             return
        
        robot.add_scene_object(name="marcador",
        pose_init=posicion_marcador,
        size=(18e-3, 18e-3, 100e-3),  # escala en metros
        color=(1.0, 0.0, 0.0, 0.8),
        shape=Shapes.CYLINDER,
        movable=True,
        rot_euler=(0, 0, 0))