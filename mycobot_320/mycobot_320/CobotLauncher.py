#!/usr/bin/env python3
"""
CobotLauncher
-------------
Punto de entrada principal para la ejecución de rutinas robóticas en el proyecto.
Maneja la inicialización del entorno (ROS2 o Robot Real), la carga dinámica
de scripts de usuario y la configuración del puente de comunicación (Bridge).

Uso:
    python3 launcher.py ruta/a/rutina.py [ros|real] [--bridge] [--speed 50]
"""

import sys
import argparse
import importlib.util
import time
import traceback
from pathlib import Path
from typing import Optional

# Rutas e imports
# Determinar la raíz del proyecto de acuerdo a la ubicación de este script
LAUNCHER_PATH = Path(__file__).resolve()
SCRIPT_DIR = LAUNCHER_PATH.parent

CORE_DIR = SCRIPT_DIR / "CobotStudio_core"

# Agregar rutas al path para permitir importaciones absolutas y relativas
if str(SCRIPT_DIR) not in sys.path:
    sys.path.append(str(SCRIPT_DIR))
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

try:
    # Importaciones del núcleo del proyecto
    from CobotStudio import MyCobotController, ROS_OK
    from CobotServer import CobotServer
    if ROS_OK:
         from CobotStudio import ROSManager
    else:
        ROSManager = None 
except ImportError as e:
    print(f"\n[!] CRITICAL ERROR: No se pudieron importar las librerías base.")
    print(f"    Detalle: {e}")
    print(f"    PYTHONPATH actual: {sys.path}\n")
    sys.exit(1)


def get_parser() -> argparse.ArgumentParser:
    """Configura y devuelve el parser de argumentos de línea de comandos."""
    parser = argparse.ArgumentParser(
        description="CobotLauncher: Orquestador de ejecución de rutinas robóticas.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("filepath", type=str, help="Ruta al archivo .py de la rutina (ej: Proyecto/Rutina.py)")
    parser.add_argument("mode", choices=["ros", "real"], help="Modo de ejecución: Robot Virtual (ROS) o Hardware (Real)")
    parser.add_argument("--bridge", action="store_true", help="Activar CobotServer para streaming de datos (solo en mode real)")
    parser.add_argument("--speed", type=int, default=30, help="Velocidad global de ejecución (1-100)")
    return parser


def load_routine_module(file_path: Path):
    """
    Carga dinámicamente un módulo Python desde una ruta de archivo.
    """
    if not file_path.exists():
        # Intentar buscar relativo a la carpeta de scripts si no es absoluta
        alt_path = SCRIPT_DIR / 'scripts' / file_path
        if alt_path.exists():
            file_path = alt_path
        else:
            raise FileNotFoundError(f"No se encontró el archivo de rutina: {file_path}")

    module_name = file_path.stem # nombre sin extensión
    
    # Agregar la carpeta de la rutina al path para que pueda importar sus dependencias locales
    routine_dir = file_path.parent
    if str(routine_dir) not in sys.path:
        sys.path.append(str(routine_dir))

    try:
        spec = importlib.util.spec_from_file_location(module_name, str(file_path))
        if spec is None or spec.loader is None:
            raise ImportError(f"No se pudo crear la especificación para: {file_path}")
        
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        
        if not hasattr(module, "run"):
            raise AttributeError(f"El módulo '{module_name}' no tiene la función requerida 'run(robot, **kwargs)'.")
            
        return module
    except Exception as e:
        raise ImportError(f"Error cargando el script '{file_path.name}': {e}")


def _get_angles_safe(robot) -> Optional[list]:
    """Helper para leer ángulos de forma segura independientemente de la implementación del driver."""
    try:
        # Lectura mediante pymycobot
        if hasattr(robot, "mc") and hasattr(robot.mc, "get_angles"):
            return robot.mc.get_angles()
    except Exception:
        pass
    return None


def initialize_environment(mode: str, project_path: Path, enable_bridge: bool):
    """
    Instancia el controlador del robot y servicios auxiliares según el modo.
    
    Returns:
        tuple: (robot_instance, server_instance)
    """
    robot = None
    server = None

    print(f"[*] Inicializando entorno: {mode.upper()}")

    if mode == "ros":
        if not ROS_OK or ROSManager is None:
            raise RuntimeError("Modo 'ros' solicitado pero ROS no está disponible o falló la importación.")
        
        # ROSManager se encarga de instanciar ROS y conectar con MoveIt
        robot = ROSManager(project_path=str(project_path))

    elif mode == "real":
        # MyCobotController maneja la conexión serial/socket
        robot = MyCobotController(project_path=str(project_path))
        
        if enable_bridge:
            print("[Bridge] Configurando servidor de transmisión de datos...")
            server = CobotServer(
                read_angles_callable=lambda: _get_angles_safe(robot),
                port=65432
            )
            server.start()

            # Observer para replicar comandos de gripper
            if hasattr(robot, "_set_server_observer"):
                robot._set_server_observer(server)
            
            print("\n" + "="*50)
            print("   MODO BRIDGE ACTIVO: ESPERANDO CLIENTE ROS2")
            print("="*50 + "\n")
            
            if server.wait_for_connection(timeout=30): # Timeout explícito
                print("[Bridge] Cliente conectado. Iniciando ejecución.")
                time.sleep(1.0)
            else:
                print("[Bridge] Advertencia: Timeout esperando cliente. Se continuará sin bridge.")

    return robot, server


def main():
    parser = get_parser()
    args = parser.parse_args()

    routine_path = Path(args.filepath)
    project_directory = routine_path.parent.resolve()

    # Carga de la Rutina
    try:
        print(f"[*] Cargando script: {routine_path.name} ...")
        routine_module = load_routine_module(routine_path)
    except Exception as e:
        print(f"[!] Error Fatal cargando rutina: {e}")
        sys.exit(1)

    # Inicialización del Hardware/ROS
    robot = None
    server = None
    
    try:
        robot, server = initialize_environment(args.mode, project_directory, args.bridge)
    except Exception as e:
        print(f"[!] Error inicializando sistema robótico: {e}")
        sys.exit(1)

    # Ejecución
    start_time = time.time()
    try:
        if robot:
            print(f"[*] Ejecutando 'run' en {routine_path.name}...")
            print(f"[*] Parametros: Speed={args.speed}%")
            
            # Inyección de dependencias a la rutina
            routine_module.run(robot, speed=args.speed)
            
            print(f"[*] Rutina finalizada con éxito ({time.time() - start_time:.2f}s).")

    except KeyboardInterrupt:
        print("\n[!] Ejecución detenida por el usuario (Ctrl+C).")
    except Exception as e:
        print(f"\n[!] Excepción durante la ejecución de la rutina:")
        print(f"    {e}")
        traceback.print_exc()
    finally:
        # Limpieza y apagado
        if server:
            print("    - Deteniendo CobotServer...")
            server.stop()
        
        if robot and hasattr(robot, 'shutdown'):
            print("    - Apagando ROSManager...")
            try:
                robot.shutdown()
            except Exception:
                pass # Ignorar errores en shutdown
        
if __name__ == "__main__":
    main()