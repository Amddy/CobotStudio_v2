from Cobot_sdk import *
import os
import json
import inspect


def elegir_configuracion(robot, robt_base: RobTarget, tool: SE3 = SE3(), wobj: SE3 = SE3(), filtrar: bool = True):
    """
    Lista las configuraciones válidas (misma lógica que check_pose_configs) y devuelve
    el RobTarget elegido por el usuario, para poder ejecutarlo después con MoveJ.
    Opción 'r' para previsualizar en RViz antes de decidir (usa check_pose_configs original).
    """
    configs = robt_base.find_valid_configs(tool, wobj)
    if not configs:
        print("[!] No se encontraron configuraciones válidas para esa pose.")
        return None

    resultados = []
    for conf in configs:
        target = RobTarget(robt_base.pose, list(conf))
        ok = True
        try:
            ok = bool(robot.moveit_manager._check_collision(target, tool, wobj))
        except Exception as e:
            print(f"    (no se pudo chequear colisión para {conf}: {e})")
        resultados.append({"conf": conf, "ok": ok, "target": target})

    display_list = [r for r in resultados if r["ok"]] if filtrar else resultados
    if not display_list:
        print("[!] Ninguna configuración pasó el filtro de colisiones.")
        return None

    while True:
        print("\n" + "=" * 40)
        print(f" Configuraciones disponibles: {len(display_list)}")
        print("=" * 40)
        for i, item in enumerate(display_list, start=1):
            estado = "OK" if item["ok"] else "En colisión"
            print(f"{i}: {item['conf']}  ({estado})")
        print("-" * 40)
        print("Número para elegir | 'r' para previsualizar en RViz | 'q' para omitir esta pose")

        user_in = input("Opción: ").strip().lower()

        if user_in == "q":
            return None
        if user_in == "r":
            robot.check_pose_configs(robt=robt_base, tool=tool, wobj=wobj, filtrar=filtrar)
            continue
        if not user_in.isdigit():
            print("[!] Ingresá un número válido, 'r' o 'q'.")
            continue

        idx = int(user_in)
        if 1 <= idx <= len(display_list):
            elegido = display_list[idx - 1]
            print(f"-> Configuración elegida: {elegido['conf']}")
            return elegido["target"]
        print("[!] Índice fuera de rango.")


def run(robot: BaseRobotController, speed=30, **kwargs):
    def moveit(robot: ROSManager):
        poses_candidatas = {
            "punto_A": SE3(120, -120, 320) * SE3.Rx(0),
            "punto_B": SE3(120, 120, 320) * SE3.Rx(0),
        }

        posiciones_seguras = {}
        carpeta = os.path.join(os.path.dirname(__file__), "Posiciones_seguras")
        os.makedirs(carpeta, exist_ok=True)

        for nombre, pose in poses_candidatas.items():
            print(f"\n--- {nombre} ---")
            robt_base = RobTarget(pose)

            target = elegir_configuracion(robot, robt_base, tool=SE3(), wobj=SE3(), filtrar=True)
            if target is None:
                print(f"[!] {nombre} omitido (sin configuración elegida).")
                continue

            # Movimiento real (función global: funciona igual en 'ros' y en 'real')
            robot.MoveJ(target, speed=speed, tool=SE3(), wobj=SE3())

            # Lectura confirmada de la posición real alcanzada
            q = robot.get_current_q(prefer_gripper=False)
            entrada = {"q": q.tolist(), "conf": list(target.conf) if hasattr(target, "conf") else None}

            # --- Intento opcional de capturar también la pose cartesiana ---
            # OJO: no confirmamos si get_current_q admite get_robt=True en esta versión
            # del código (la doc dice que sí, pero ya vimos que la doc no siempre coincide).
            # Se intenta una sola vez y se avisa si no funciona, sin frenar el resto del script.
            try:
                q2, robt_actual = robot.get_current_q(prefer_gripper=False, get_robt=True)
                entrada["coords"] = robt_actual.pose.t.tolist()  # traslación x,y,z
            except TypeError:
                print("    (get_current_q no acepta 'get_robt' en esta versión — se omite pose cartesiana)")
            except Exception as e:
                print(f"    (no se pudo obtener pose cartesiana: {e})")

            posiciones_seguras[nombre] = entrada
            print(f"{nombre} -> {entrada}")

        with open(os.path.join(carpeta, "posiciones.py"), "w") as f:
            f.write("posiciones_seguras = " + repr(posiciones_seguras) + "\n")
        with open(os.path.join(carpeta, "posiciones.json"), "w") as f:
            json.dump(posiciones_seguras, f, indent=2)

        print(f"\nGuardado en {carpeta}/posiciones.py y posiciones.json")

    moveit(robot)