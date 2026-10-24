from Cobot_sdk import *

def run(robot:BaseRobotController, **kwargs):

    TA_hombro=SE3([
    [ -1,   0,   0,    103.6 ],
    [  0,  -1,   0,   -89.1  ],
    [  0,   0,   1,    331.4 ],
    [  0,   0,   0,   1      ]
    ])

    TB_hombro=SE3([
    [ -1,   0,   0,   -103.6  ],
    [  0,  -1,   0,   -89.1   ],
    [  0,   0,   1,    331.4  ],
    [  0,   0,   0,   1       ]
    ])
    
    TA_codo=SE3([
    [  0,  -1,  0,   300  ],
    [ -1,   0,  0,  -90.2 ],
    [  0,   0,  -1,  86   ],
    [  0,   0,  0,   1    ]
    ])
    
    TB_codo=SE3([
    [  0,  -1,  0,   348  ],
    [ -1,   0,  0,  -90.2 ],
    [  0,   0,  -1,  86   ],
    [  0,   0,  0,   1    ]
    ])

    TA_muñeca=robot.load_data('Poses', 'SingMuñeca', 'PuntoA')
    TB_muñeca=robot.load_data('Poses', 'SingMuñeca', 'PuntoB')

    def traj_sing(robot:BaseRobotController, spd, pose1, pose2, config):
            robt1 = RobTarget(pose1, config=config)
            robt2 = RobTarget(pose2, config=config)

            robot.MoveJ(robt1, spd)
            time.sleep(2)
            robot.MoveL(robt2, spd)
            robot.logger.info("Trayectoria completada.")

    def validar_input(prompt, max_val):
        while True:
            try:
                val_str = input(prompt)
                val = int(val_str)
                if 0 <= val <= max_val:
                    return val
                print(f"Ingrese un número entre 0 y {max_val}.")
            except ValueError:
                print("Entrada no válida. Ingrese un número entero.")

    menu_options = {
    1: {
        "name": "Hombro",
        "configs": [[1, 1, 1], [-1, -1, -1], [-1, 1, 1], [1, -1, -1]],
        "poses": (TA_hombro, TB_hombro)
    },
    2: {
        "name": "Codo",
        "configs": [[1, 1, 1], [1, -1, 1], [-1, 1, -1]],
        "poses": (TA_codo, TB_codo)
    },
    3: {
        "name": "Muñeca",
        "configs": [[1, -1, 1]],
        "poses": (TA_muñeca, TB_muñeca)
    }
}

    while True:
        print("\nMenú de singularidades")
        for key, data in menu_options.items():
            print(f"{key}: {data['name']}")
        
        sel_main = validar_input("Seleccione singularidad (0 para salir): ", len(menu_options))

        if sel_main == 0:
            print("Finalizando análisis de singularidades.")
            break

        data = menu_options[sel_main]
        nombre = data['name']
        configs = data['configs']
        poses = data['poses']
        
        while True:
            print(f"\nConfiguraciones disponibles para {nombre}:")
            for i, conf in enumerate(configs, start=1):
                print(f"  {i}: {conf}")
                
            sel_conf = validar_input("Seleccione configuración (0 para volver): ", len(configs))
            
            if sel_conf == 0:
                break
                
            conf_elegida = configs[sel_conf - 1]
            print(f"\nEjecutando: {nombre} | Conf: {conf_elegida}")
            
            traj_sing(robot, 30, poses[0], poses[1], conf_elegida)
    
    robot.GoHome()