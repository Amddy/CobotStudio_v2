import numpy as np
from scipy.spatial.transform import Rotation as R
from DHRobotGT import myCobot320, IKineError
import time
import datetime
import threading
import logging
from pymycobot import MyCobotSocket, MyCobot320
from abc import ABC, abstractmethod
from spatialmath import SE3
from pynput import keyboard
from pathlib import Path
import importlib.util
from RobtManager import RobTarget
from typing import TYPE_CHECKING

# Typehinting: no se ejecuta en runtime, solo ofrece autocompletado.
if TYPE_CHECKING:
    from MoveItManager import MoveItManager

# Imports de ROS2
try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from MarkerManager import MarkerManager
    ROS_OK = True
    print('>>> Librerías de ROS2 importadas correctamente. <<<')
except ImportError:
    ROS_OK = False
    print('>>> No se pudieron importar las librerías de ROS2. Funciones de visualización deshabilitadas. <<<')
    class Node: pass

# Si se importó correctamente lo referido a ROS2, buscar MoveIt
if ROS_OK:
    try:
        from moveit_msgs.msg import DisplayRobotState, RobotState
        from sensor_msgs.msg import JointState
        MoveIt_OK = True
    except ImportError:
        MoveIt_OK = False

class BaseRobotController(ABC):
    """
    Clase abstracta: maneja tanto al robot real como al virtual según el argumento del launcher.
    """
    def __init__(self, project_path, logger_name='BaseController'):
        
        # Ruta del proyecto, pasada como argumento por consola
        if project_path:
            self.project_root = Path(project_path).resolve()
        else:
            self.project_root = Path.cwd()

        # Logger propio
        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(logging.DEBUG)
        if not self.logger.handlers:
            ch = logging.StreamHandler()
            ch.setLevel(logging.DEBUG) 
            
            formatter = logging.Formatter(
                '[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s',
                datefmt='%H:%M:%S'
            )
            ch.setFormatter(formatter)
            self.logger.addHandler(ch)

        # Modelo del robot en roboticstoolbox
        self.cobot_tb = myCobot320(rotar_base=True, metros=False)

        # Límites de los ejes
        self.joint_limits = {
        "joint_1": (-168.0, 168.0),
        "joint_2": (-135.0, 135.0),
        "joint_3": (-145.0, 145.0),
        "joint_4": (-145.0, 145.0),
        "joint_5": (-168.0, 168.0),
        "joint_6": (-180.0, 180.0),
        }
            
        self.logger.info(f"Project Root: {self.project_root}")

    # Métodos abstractos: cada clase los maneja a su manera.    
    @abstractmethod
    def MoveJ(self, robt, speed:int = 30, tool=SE3(), wobj=SE3(), wobj_name='MoveJ_wobj', robt_name='MoveJ_robtarget'):
        """
        Realiza un movimiento articular (joint) hacia un objetivo (robtarget).
        
        Calcula la cinemática inversa para alcanzar la pose definida por 'robt' 
        dentro del sistema de coordenadas 'wobj' usando la herramienta 'tool'.

        Args:
            robt (RobTarget): Objetivo de pose y configuración del robot.
            speed (int (0-100)): Velocidad de movimiento.
            tool (SE3): Transformación de la herramienta respecto a la brida (TCP).
            wobj (SE3): Objeto de trabajo respecto a la base, referencia para `robt`.
            wobj_name (str, optional): Nombre de la referencia en logs/visualización.
            robt_name (str, optional): Nombre del punto objetivo en logs/visualización.

        Raises:
            IKineError: Si no se encuentra solución a la cinemática inversa.
        """
        pass

    @abstractmethod
    def MoveL(self, robt, speed:int = 30, tool=SE3(), wobj=SE3(), wobj_name='MoveL_wobj', robt_name='MoveL_robt'):
        """
        Realiza un movimiento cartesiano (lineal) hacia un objetivo (robtarget).
        
        Calcula la cinemática inversa para alcanzar la pose definida por 'robt' 
        dentro del sistema de coordenadas 'wobj' usando la herramienta 'tool'.

        Args:
            robt (RobTarget): Objetivo de pose y configuración del robot.
            speed (int, optional): Velocidad de movimiento (0-100).
            tool (SE3, optional): Transformación de la herramienta respecto a la brida (TCP).
            wobj (SE3, optional): Objeto de trabajo respecto a la base, referencia para `robt`.
            wobj_name (str, optional): Nombre de la referencia en logs/visualización.
            robt_name (str, optional): Nombre del punto objetivo en logs/visualización.

        Raises:
            IKineError: Si no se encuentra solución a la cinemática inversa.
        """
        pass

    @abstractmethod
    def GripperState(self, apertura: float, spd: int):
        """
        Modifica el estado de la pinza del robot permitiendo aperturas parciales.

        Args:
            apertura (float 0-100): Valor de apertura de la pinza (0 cerrado, 100 abierto).
            spd (int 0-100): Velocidad de movimiento de la pinza.
        """
        pass

    @abstractmethod
    def MoveJ_q(self, q, speed:int = 30, unit = 'rad'):
        """
        Envía al robot al vector de variables articulares determinado.

        Args:
            q (Array(1,6)): Vector de variables articulares.
            speed (int 0-100): Velocidad de movimiento.
            unit (str ('deg', 'rad')) : Unidad de las variables articulares.
        """
        pass

    def GoHome(self, speed:int=30):
        """
        Envía al robot a su posición home: q = [0, 0, 0, 0, 0, 0].

        Args:
            spd (int 0-100): Velocidad de movimiento.
        """
        q = np.zeros(6)
        self.MoveJ_q(q, speed)
    
    @abstractmethod
    def teach_poses(self, cantidad: int, ajuste: bool = False, q_list: list = None):
        """
        Grabación de poses del robot.

        El comportamiento varía según la implementación:
        
        - **ROSManager**: Se reconocen las poses dadas por la lista `q_list` que contiene los vectores de variables articulares.
        - **MyCobotController**: Se inicia la rutina de grabación de poses manual en el robot.

        Args:
            cantidad (int): Cantidad de poses a grabar.
            ajuste (bool): Si es True, permite ajuste interactivo de cada pose antes de grabarla a través de un joystick.
            q_list (list, optional): Lista de vectores articulares para simular la grabación (solo en ROSManager).
        """
        pass
    
    def joystick_adjust(self, q: list, mover_callback, step_fino: float = 5.0, step_grueso: float = 10.0):
        """
        Ajuste interactivo de una pose emulando el comportamiento de un joystick con el teclado.
        Al presionar las teclas se genera un nuevo robtarget desplazado de la posición actual respecto al workobject (offset).
        Luego se calcula la cinemática inversa para enviar el movimiento al robot. Si falla el cálculo del PCI
        se recupera el último robtarget válido.

        Args:
            q (list or np.ndarray): Vector de variables articulares inicial.
            mover_callback (function): Función que mueve el robot (ej: ROSManager.MoveJ).
            step_fino (float): Paso fino para desplazamientos en mm.
            step_grueso (float): Paso grueso para desplazamientos en mm.

        Returns:
            robt (RobTarget): RobTarget final luego de los ajustes.

        Control:
            - Flechas: ejes X e Y.
            - PgUp/PgDn: eje Z.
            - Shift + flecha/PgUp/PgDn = movimiento de paso grueso.
            - Esc = salir.
        """
        # Calcular la pose inicial y generar el robtarget
        print(f'Pose recibida = {q}')
        pose = self.cobot_tb.fkine(q)
        robt = RobTarget(pose, self.cobot_tb.calc_conf(q))
        print(f"RobTarget inicial: \n{robt.pose} | Configuración: {robt.config}")
        paso = step_fino

        print("\n--- Modo joystick ---")
        print("Flechas = mover XY | PgUp/PgDn = mover Z | Shift = paso grueso | Esc = salir")

        def on_press(key):
            nonlocal robt, paso
            # Guardar el estado actual antes de desplazarlo
            robt_backup = robt
            # Desplazamiento según la tecla presionada usando robtarget.offset()
            try:
                if key == keyboard.Key.shift:
                    paso = step_grueso
                elif key == keyboard.Key.up:
                    robt = robt.offset(dy=paso)
                elif key == keyboard.Key.down:
                    robt = robt.offset(dy=-paso)
                elif key == keyboard.Key.left:
                    robt = robt.offset(dx=-paso)
                elif key == keyboard.Key.right:
                    robt = robt.offset(dx=paso)
                elif key == keyboard.Key.page_up:
                    robt = robt.offset(dz=paso)
                elif key == keyboard.Key.page_down:
                    robt = robt.offset(dz=-paso)
                elif key == keyboard.Key.esc:
                    return False
                print(f"RobTarget ajustado: \n{robt.pose} | Configuración: {robt.config}")

                # Ejecutar el movimiento: se puede elegir MoveJ o MoveL según el caso
                mover_callback(robt)
                
            except IKineError as e:
                # Si falla el PCI se recupera el último robtarget con solución válida para continuar desplazando desde el mismo
                print(f'Error con el movimiento del robot: {e}. Restaurando a la última pose válida...')
                robt = robt_backup
            except Exception as e:
                print(f'Error inesperado: {e}. Restaurando a la última pose válida...')
                robt = robt_backup

        def on_release(key):
            # Shift debe mantenerse presionado para movimientos de paso grueso
            nonlocal paso
            if key == keyboard.Key.shift:
                paso = step_fino

        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

        return robt
    
    def teach_and_save_wobj(self, filename: str, wobj_name: str, tool: SE3 = SE3(), save_q: bool = False, method: str = '3points', q_test: list = None) -> SE3:
        """
        Enseña un workobject referido a la base del robot y lo guarda en "Workobjects/filename.py", creando el directorio si no existe.
        Opcionalmente guarda los q leídos por el cobot en 'Workobjects/filename_q.py'.

        Args:
            filename (str) : Nombre del archivo .py (sin .py opcional).
            wobj_name (str) : Nombre de la variable dentro del archivo.
            tool (SE3) : Herramienta usada para enseñar el wobj.
            save_q (bool) : Si `True`, guarda los q usados en la enseñanza en un archivo separado.
            method (str) : Método de enseñanza: '3points' o '6points'.
            q_test (list, optional) : Vectores de variables articulares considerados como puntos de enseñanza. Se utiliza en el robot virtual.
        Returns:
            wobj_calculated (SE3) : Terna que representa al workobject respecto a la base del robot.          
        """
        # Determinar cuántos puntos pedir
        n_puntos = 6 if method == '6points' else 3
        print("\n" + "="*60)
        print(f"Enseñanza de workobject: {wobj_name}. Método: {n_puntos} puntos.")
        print("="*60 + "\n")
        
        # Grabar poses (o leerlas en el caso del robot virtual)
        q_vals, _ = self.teach_poses(n_puntos, ajuste=True, q_list=q_test)
        
        # Calcular el workobject
        print(f"\n[Calculando] Procesando geometría ({method})...")
        wobj_calculated = self.cobot_tb.calc_wobj(q_vals, tool)
        print(f"[Resultado] Wobj:\n{wobj_calculated}")
        
        # Delegar guardado
        qs_to_save = q_vals if save_q else None
        self._save_se3_definition(
            folder="Workobjects",
            filename=filename,
            var_name=wobj_name,
            se3_obj=wobj_calculated,
            q_vals=qs_to_save
            )

        return wobj_calculated

    def teach_and_save_TCP(self, filename: str, tcp_name: str, save_q: bool = False, q_test: list = None) -> SE3:
        """
        Enseña un TCP con el método de los 4 puntos y lo guarda en carpeta "TCPs" dentro del proyecto.

        Args:
            filename (str) : Nombre del archivo .py (sin .py opcional).
            tcp_name (str) : Nombre de la variable.
            save_q (bool) : Si `True`, guarda los q usados en la enseñanza en un archivo separado.
            q_test (list, optional) : Vectores de variables articulares considerados como puntos de enseñanza. Se utiliza en el robot virtual.

        Returns:
            tcp_calculated (SE3) : Objeto SE3 que representa la traslación al TCP enseñado.
        """
        print("\n" + "="*60)
        print(f"  ENSEÑANZA DE TCP: {tcp_name}")
        print("="*60 + "\n")

        # Obtener datos físicos: 4 veces el mismo punto con orientaciones distintas
        q_vals, _ = self.teach_poses(4, ajuste=True, q_list = q_test)

        print(f"\n[Calculando] Procesando geometría TCP...")
        tcp_result_tuple = self.cobot_tb.TCP_4puntos(q_vals)
        
        _, p_tool_real, _, _, rmse = tcp_result_tuple

        print(f"[Resultado] TCP (Traslación): {p_tool_real}")
        print(f"[Calidad]   RMSE: {rmse:.5f}")

        # Guardar el TCP con helper específico
        qs_to_save = q_vals if save_q else None

        self._save_tcp_definition(
            folder="TCPs",
            filename=filename,
            var_name=tcp_name,
            tcp_data=tcp_result_tuple,
            q_vals=qs_to_save
        )

        # Se devuelve la traslación, permitiendo guardarlo en una variable
        return SE3.Trans(p_tool_real)

    def load_data(self, subfolder: str, filename: str, var_name: str, verbose: bool = False) -> any:
        """
        Importa dinámicamente una variable específica desde un archivo Python externo.

        Busca un archivo en la carpeta del proyecto para cargarlo como módulo y extraer la variable (terna, trayectoria, lista, etc.)

        Args:
            subfolder (str) : Subcarpeta. Ejemplo: "Workobjects", "TCPs".
            filename (str) : Nombre del archivo .py (con o sin extensión).
            var_name (str) : Nombre de la variable a cargar.
            verbose (bool) : Log con información adicional.

        Returns:
            val : Variable leída. Puede ser una lista, array de numpy, 
             o clase personalizada dependiendo del archivo fuente.

        Raises:
            FileNotFoundError: Si la ruta construida no existe.
            ImportError: Si el módulo no puede ser cargado o ejecutado (ej. sintaxis inválida).
            AttributeError: Si "var_name" no existe dentro del archivo cargado.
        """
        # Obtención del path al archivo con la variable
        path = self._resolve_project_path(subfolder, filename, create_dir=False)
        print(f'Buscando en {path}')
        
        if not path.exists():
             raise FileNotFoundError(f"Archivo no encontrado: {path}")
             
        # Importación del módulo
        try:
            spec = importlib.util.spec_from_file_location("mod_dyn", path)
            if spec is None or spec.loader is None:
                 raise ImportError(f"No se pudo crear especificación para {path}")
            
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        except Exception as e:
            raise ImportError(f"Error al cargar módulo dinámico '{filename}': {e}")
        
        # Chequeo de existencia de la variable
        if not hasattr(module, var_name):
            raise AttributeError(f"Variable '{var_name}' no existe en {path.name}")
        
        val = getattr(module, var_name)

        # Log adicional
        if verbose:
            try:
                display_path = path.relative_to(self.project_root) # Ruta relativa
            except ValueError:
                # Si por alguna razón el archivo está fuera del root, se muestra el nombre solo
                display_path = path.name
                
            # Log conciso: [Carga] Archivo -> Variable (Tipo)
            type_name = type(val).__name__
            info_extra = f" (Len: {len(val)})" if hasattr(val, '__len__') else ""
            
            self.logger.info(f"[Load] {display_path} -> '{var_name}' <{type_name}>{info_extra}")
            
        return val
        
    def load_wobj(self, archivo: str, wobj_name: str, tool: SE3 = SE3(), auto_teach: bool = True, q_teach = None) -> SE3:
        """
        Intenta cargar un Wobj buscando en la carpeta "Workobjects" del proyecto. Si falla y `auto_teach` es `True`, 
        ofrece al usuario enseñarlo manualmente.
        
        Args:
            archivo (str) : Nombre del archivo .py (sin .py opcional).
            wobj_name (str) : Nombre de la variable dentro del archivo.
            tool (SE3) : Herramienta usada para enseñar (solo si falla la carga).
            auto_teach (bool) : Si True, ofrece la opción de proceder con la enseñanza de la terna.
            q_teach (list, optional) : Vectores de variables articulares considerados como punto de enseñanza. Se utiliza en el robot virtual.

        Returns:
            wobj (SE3) : Terna que representa al workobject respecto a la base del robot.

        Raises:
            FileNotFoundError: Si el archivo no existe y no se realiza la enseñanza (`auto_teach=False` o usuario cancela).
            AttributeError: Si la variable no existe en el archivo y no se realiza la enseñanza.
            TypeError: Si el objeto cargado existe pero no es una instancia de SE3.
    
        """
        try:
            wobj = self.load_data("Workobjects", archivo, wobj_name)
            
            # Validación de tipo
            if not isinstance(wobj, SE3):
                raise TypeError(f"El objeto '{wobj_name}' no es de tipo SE3, es {type(wobj)}.")
            else:
                self.logger.info(f"Wobj '{wobj_name}' cargado correctamente.")
                
            return wobj

        except (FileNotFoundError, AttributeError) as e:
            # Si no existe archivo o variable
            self.logger.error(f"\n[!] Error al cargar Wobj: {e}")
            
            # Si se desactiva la enseñanza automática, falla normalmente
            if not auto_teach:
                raise e 

            # Interacción con el usuario
            print(f"¿Desea ejecutar la rutina de enseñanza para '{wobj_name}' ahora?")
            resp = input("Escribe 's' para enseñar, o cualquier tecla para cancelar: ").lower().strip()
            
            if resp == 's':
                self.logger.info(f"--- Iniciando recuperación interactiva para {wobj_name} ---")
                new_wobj = self.teach_and_save_wobj(
                    filename=archivo, 
                    wobj_name=wobj_name, 
                    tool=tool, 
                    save_q=True,
                    q_test = q_teach
                )
                return new_wobj
            else:
                self.logger.warning("[!] Operación cancelada por el usuario.")
                raise e # Se relanza el error original para detener el script
    
    @abstractmethod
    def load_scene(self, scene_filename: str, subfolder: str = ""):
        """
        Interfaz para la carga de objetos de la celda robótica en RViz.

        El comportamiento varía según la implementación:
        
        - **ROSManager**: Llama al nodo `MarkerArray` a través el archivo `scene_filename` para crear la escena.
        - **MyCobotController**: Se ignora (implementación vacía).

        Args:
            scene_filename (str): Nombre del archivo de definición de la escena.
        """
        pass
    
    @abstractmethod
    def traj_moveit(self, archivo: str, traj_name: str = "TRAJ", tool = SE3()):
        """
        Lee una trayectoria generada con MoveIt y la envía al cobot la publica en RViz según corresponda.

        Args:
            archivo (str): Nombre del archivo .py.
            traj_name (str): Nombre de la variable dentro del archivo.
            tool (SE3): Herramienta.

        Returns:
            last_robt (RobTarget): RobTarget correspondiente al último punto de la trayectoria ejecutada.
        """
        pass
    
    @staticmethod
    def pose_to_matrix(pose):
        """
        Convierte pose dada como lista [x, y, z, r, p, y] en una matriz homogénea SE3.

        Args:
            pose (list): Lista con 6 elementos [x, y, z, r, p, y] en grados.
        Returns:
            T_se3 (SE3): Matriz homogénea SE3.
        """
        x, y, z, rx, ry, rz = pose
        rx, ry, rz = np.deg2rad([rx, ry, rz])
        T_se3 = SE3(x, y, z) * SE3.RPY([rx, ry, rz], order='zyx')
        return T_se3

    @staticmethod
    def matrix_to_pose(T_matrix):
        """
        Convierte una matriz homogénea SE3 en lista [x, y, z, r, p, y] en grados.
        
        Args:
            T_matrix (np.ndarray o SE3): Matriz homogénea SE3.
        Returns:
            pose (list): Lista con 6 elementos [x, y, z, r, p, y] en grados.
        """
        T = SE3(T_matrix)
        x, y, z = T.t
        # Extraer rotación como RPY (en radianes), orden ZYX
        rx, ry, rz = T.rpy(order='zyx', unit='rad')
        # Conversión a grados
        rx, ry, rz = np.rad2deg([rx, ry, rz])
        return [x, y, z, rx, ry, rz]

    def _resolve_project_path(self, subfolder: str, filename: str, create_dir: bool = False) -> Path:
        """
        Helper para construir rutas absolutas dentro de `self.project_root` y asegurar la extensión ".py".

        Nota:
            - Fuerza la extensión '.py' si no está presente.
            - Si create_dir=True, genera los directorios faltantes (efecto secundario).

        Returns:
            Path: Ruta pathlib completa lista para usarse.
        """
        target_folder = self.project_root / subfolder

        # Creación del directorio (si es necesario)
        if create_dir:
            target_folder.mkdir(parents=True, exist_ok=True)

        # Agregado de extensión si no fue especificada
        if not filename.endswith(".py"):
            filename += ".py"
        return target_folder / filename
     
    def _save_aux_q_data(self, folder: str, base_filename: str, var_name: str, q_vals: list):
        """
        Helper interno para guardar una lista con los valores articulares (q) utilizados en la enseñanza de una terna (ej: TCP, workobject) en un archivo .py separado.
        
        El archivo creado comienza con el mismo nombre que aquel que contiene la variable guardada, luego se agrega el sufijo "_q".
        La misma lógica aplica al nombre de la variable guardada.
        """
        # Agregar sufijo _q y construir ruta
        name_q = base_filename.replace(".py", "") + "_q.py"
        path_q = self._resolve_project_path(folder, name_q) 
        
        # Mismo caso para la variable
        variable_name_q = f"{var_name}_q"

        # Escritura de la variable
        mode = 'a' if path_q.exists() else 'w'
        try:
            with open(path_q, mode) as f:
                if mode == 'w':
                    f.write(f"# Config Articulares (Q) para {folder}\n\n")
                f.write(f"# Q Values para: {var_name}\n")
                f.write(f"{variable_name_q} = [\n")
                for row in q_vals:
                    clean_row = [float(x) for x in row]
                    f.write(f"    {clean_row},\n")
                f.write("]\n\n")
            self.logger.info(f"[Guardado] Q Values -> {path_q.name}")
        except Exception as e:
            self.logger.error(f"Falló guardado Q en {name_q}: {e}")

    def _save_tcp_definition(self, folder: str, filename: str, var_name: str, tcp_data: tuple, q_vals: list = None):
        """
        Helper interno para guardar un TCP definido por vector de traslación (SE3) y sus métricas de calidad en un archivo .py.

        Permite también guardar los vectores de variables articulares asociados a la enseñanza llamando al helper correspondiente.

        Nota :
            Genera los directorios faltantes (efecto secundario).
        """

        # p_tool_raw: calculado directo, p_tool_real: corregido con z_aux. Desempaquetado de métricas de calidad.
        p_tool_raw, p_tool_real, residuals, ecm, rmse = tcp_data

        # Creación del archivo (si no existe) y escritura del mismo
        if not filename.endswith(".py"): filename += ".py"
        path_tcp = self._resolve_project_path(folder, filename, create_dir=True)
        mode = 'a' if path_tcp.exists() else 'w'

        try:
            with open(path_tcp, mode) as f:
                # Si el archivo no existe, se agregan los imports necesarios en el encabezado
                if mode == 'w':
                    f.write(f"# Archivo de definiciones TCP: {folder}\n")
                    f.write("from spatialmath import SE3\n")
                    f.write("import numpy as np\n\n")

                # Se registran las métricas como comentarios para control de calidad
                f.write(f"# --- Registro: {var_name} ---\n")
                f.write(f"# Fecha: {datetime.datetime.now().isoformat()}\n")
                f.write(f"# RMSE (Error Medio Cuadrático): {rmse:.6f} mm\n")
                f.write(f"# ECM: {ecm:.6f}\n")
                if hasattr(residuals, 'tolist'):
                    f.write(f"# Residuales: {residuals.flatten().tolist()}\n")
                
                # Se guarda adicionalmente el vector crudo (sin corrección)
                raw_str = "[" + ", ".join([f"{val:.6f}" for val in p_tool_raw]) + "]"
                f.write(f"# Raw calc (sin corrección z_aux): {raw_str}\n")

                # Se guarda el TCP con la corrección como objeto SE3
                t_str = "[" + ", ".join([f"{val:.6f}" for val in p_tool_real]) + "]"
                # SE3 de traslación pura
                f.write(f"{var_name}_t = np.array({t_str})\n")
                f.write(f"{var_name} = SE3.Trans({var_name}_t)\n\n")

            self.logger.info(f"[Guardado] TCP {var_name} (RMSE: {rmse:.4f}) -> {path_tcp.name}")

        except Exception as e:
            self.logger.error(f"Falló guardado TCP en {filename}: {e}")

        # Opcional: guardado de los q implicados en la enseñanza del TCP
        if q_vals:
            self._save_aux_q_data(folder, filename, var_name, q_vals)

    def _save_se3_definition(self, folder: str, filename: str, var_name: str, se3_obj: SE3, q_vals: list = None):
            """
            Helper interno para guardar una terna como objeto SE3. Se utiliza para workobjects.

            Permite también guardar los vectores de variables articulares asociados a la construcción de la terna llamando al helper correspondiente.

            Nota :
                Genera los directorios faltantes (efecto secundario).
            """
            # Construcción del path y de directorios (si no existen)
            path_se3 = self._resolve_project_path(folder, filename, create_dir=True)
            
            # Escritura del archivo
            mode = 'a' if path_se3.exists() else 'w'
            try:
                with open(path_se3, mode) as f:
                    # Escribir encabezado con imports solo si es archivo nuevo
                    if mode == 'w':
                        f.write(f"# Archivo de definiciones: {folder}\n")
                        f.write("from spatialmath import SE3, UnitQuaternion\n")
                        f.write("import numpy as np\n\n")

                    # Extraemos el cuaternión desde la matriz de rotación del SE3.
                    # .vec nos devuelve un array de 4 elementos: [s, vx, vy, vz]
                    print(f'DEBUG se3_obj: {se3_obj.R}')
                    quat = SE3.UnitQuaternion(se3_obj).vec
                    print(f'DEBUG quat: {quat}')
                    
                    # Formateo de alta precisión para facilitar la reconstrucción del objeto
                    q_str = "[" + ", ".join([f"{val:.6e}" for val in quat]) + "]"
                    t_str = "[" + ", ".join([f"{val:.6e}" for val in se3_obj.t.flatten()]) + "]"
                    
                    f.write(f"# Guardado: {datetime.datetime.now().isoformat()}\n") # Marca de tiempo
                    f.write(f"{var_name}_q = {q_str}\n")                  # Escritura de la matriz de rotación
                    f.write(f"{var_name}_t = np.array({t_str})\n")                  # Escritura del vector traslación
                    f.write(f"{var_name} = SE3({var_name}_t) * UnitQuaternion({var_name}_q).SE3()\n\n")
                self.logger.info(f"[Guardado] {folder}/{var_name} -> {path_se3.name}")
            except Exception as e:
                self.logger.error(f"[Error] Falló guardado SE3: {e}")

            # Opcional: guardado de los q implicados en la construcción de la terna
            if q_vals:
                self._save_aux_q_data(folder, filename, var_name, q_vals)

    def _check_limits(self, trajectory_arm: np.ndarray, unit: str = 'rad'):
        """
        Chequea que, para una trayectoria dada, cada valor de q esté dentro de los límites admisibles para el robot y notifica.

        Args:
            trajectory_arm: lista/array con los valores articulares.
            unit (str): unidad de los valores en la trayectoria ('rad' o 'deg').
        Returns:
            list de índices (o nombres) de ejes con valores fuera de límite
        """
        # Se convierte a grados para homogeneizar unidades con los límites
        if unit == 'rad':
            traj_q = np.rad2deg(trajectory_arm)
        else:
            traj_q = trajectory_arm

        q_limit = []

        # Recorrer elementos de la trayectoria (vectores de 6 variables articulares)
        for j, (name, (q_min, q_max)) in enumerate(self.joint_limits.items()):
            q_vals = traj_q[:, j] 
            if np.any(q_vals < q_min) or np.any(q_vals > q_max):
                q_limit.append(j+1)  # Eje en base 1

        if q_limit:
            # Caso de trayectorias que exceden el límite de una sola articulación
            if len(q_limit) == 1:
                self.logger.warning(f"Valor fuera de límite para el eje {q_limit[0]}")
            # Se exceden límites en más de una articulación
            else:
                ejes = ", ".join(map(str, q_limit))
                self.logger.warning(f"Se sobrepasan límites en los ejes: {ejes}")
    
    def _manipulability(self, objetivo, wobj: SE3, tool: SE3, q_unit = 'rad', debug = False):
        """
        Chequeo de manipulabilidad en la pose de destino. Puede analizarse Robtarget o un vector de variables articulares.

        Args:
            objetivo (RobTarget or list/np.ndarray/tuple): Pose destino como robtarget o vector de variables articulares.
            wobj (SE3): Workobject aplicado.
            tool (SE3): Herramienta aplicada.
            q_unit (str): Unidad de las variables articulares ("rad" o "deg").
            debug (bool): Si `True`, muestra información adicional.

        Returns:
            W_ratio (float): Manipulabilidad normalizada entre 0 y 1.
        """
        W_MAX = 4.653e+06 # Manipulabilidad máxima del MyCobot320 calculada con Max_manipulability.py

        # Caso robtarget
        if isinstance(objetivo, RobTarget):
            # Calcular el vector de variables articulares
            pose_calc = wobj * objetivo.pose * tool.inv()
            res = self.cobot_tb.ikine(pose_calc, objetivo.config)
            q_sol = res[0]

        # Caso vector q
        elif isinstance(objetivo, (list, np.ndarray, tuple)):
            # Manejo de unidades
            if q_unit == 'deg':
                q_sol = np.deg2rad(objetivo[:6])
            else:
                q_sol = objetivo[:6]

        # Manipulabilidad con roboticstoolbox
        W = self.cobot_tb.manipulability(q_sol)

        W_ratio = W / W_MAX # Normalización entre 0 y 1

        LIMIT_SINGULARIDAD = 0.001  # 0.1% Robot bloqueado
        LIMIT_ADVERTENCIA  = 0.1    # Cerca de la singularidad (10%)

        if W_ratio < LIMIT_SINGULARIDAD:
            self.logger.warning(f"La pose de destino corresponde a una singularidad. Manipulabilidad: {W_ratio:.2f}")

        elif W_ratio < LIMIT_ADVERTENCIA:
            self.logger.warning(f"La pose de destino se encuentra próxima a una singularidad. Manipulabilidad: {W_ratio:.2f}")

        elif debug:
            print(f"Manipulabilidad en la pose de destino: {W_ratio:.2f}")
        
        return W_ratio

# Instanciar clases de ROS solo si se encuentra disponible
if ROS_OK or TYPE_CHECKING:

    # Declaración global de nombres de articulaciones
    joint_names = ['joint2_to_joint1',
                   'joint3_to_joint2',
                   'joint4_to_joint3',
                   'joint5_to_joint4',
                   'joint6_to_joint5',
                   'joint6output_to_joint6',
                   'gripper_controller']

    class joint_pub(Node):
        """
        Nodo encargado de la publicación de estados articulares en ROS.
        """
        def __init__(self):
            super().__init__('trajectory_publisher')
            self.publisher = self.create_publisher(JointState, '/joint_states', 10)

            # Subscripción para leer estado actual
            self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                10
            )

            # Estado actual
            self.q_current = None
            self.q_current_time = None

        def joint_state_callback(self, msg):
            """
            Callback para actualizar la posición actual `q_current` con las variables articulares que muestra ROS.
            """
            self.last_joint_msg = msg
            # Ordenar posiciones segun joint_names
            position_map = {name: pos for name, pos in zip(msg.name, msg.position)}
            ordered_positions = [position_map.get(name, 0.0) for name in joint_names]
            self.q_current = np.array(ordered_positions)
            self.q_current_time = msg.header.stamp
        
        def publish_trajectory(self, trajectory, joint_names: dict, dt: float = 1.0):
            """
            Publica una trayectoria articular en el topic /joint_states de ROS.

            Args:
                trajectory: Lista con las posiciones articulares de la trayectoria.
                joint_names (dict): Nombre de las articulaciones. Debe coincidir con la definición del robot en URDF.
                dt (float): Tiempo de espera entre la publicación de cada pose de la trayectoria.
            """
            # Recorrer y publicar cada uno de los puntos de la trayectoria
            for q in trajectory:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = joint_names
                msg.position = q.tolist()
                self.publisher.publish(msg)
                time.sleep(dt)

    class TFPublisher(Node):
        """
        Nodo encargado del manejo de ternas en ROS. Publica y actualiza workobjects y robtargets, permitiendo especificar referencias y nombres.
        """

        def __init__(self):
            super().__init__('multi_tf_publisher')
            self.br = TransformBroadcaster(self)
            self.timer = self.create_timer(0.1, self.publish_all_transforms)
            
            # Diccionario para almacenar las ternas: {frame_name: SE3}
            self.transforms = {}

        def add_wobj(self, transform: SE3, name: str):
            """
            Agrega un workobject. Se asume referido a la terna base.
            
            Args:
                transform (SE3): Transformación del workobject respecto a la base.
                name (str): Nombre del workobject.
            """
            wobj_m = SE3(np.array(transform))
            wobj_m.t = wobj_m.t / 1000.0 # ROS trabaja en metros
            self.transforms[name] = (wobj_m, 'base')

        def add_robt(self, transform: SE3, name: str, reference_frame: str = 'base'):
            """
            Agrega un robtarget referido a otra terna: puede ser 'base' o un wobj, por ejemplo.
            
            Args:
                transform (SE3): Transformación del robtarget respecto a la referencia.
                name (str): Nombre del robtarget.
                reference_frame (str): Nombre de la terna de referencia.
            """
            robt_m = SE3(np.array(transform))
            robt_m.t = robt_m.t / 1000.0 # ROS trabaja en metros
            self.transforms[name] = (robt_m, reference_frame)

        def publish_all_transforms(self):
            """
            Actualización de ternas. Recorre las ternas almacenadas y publica sus transformaciones en ROS.
            """
            now = self.get_clock().now().to_msg()

            for child, (se3, parent) in self.transforms.items():
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = parent
                t.child_frame_id = child

                pos = se3.t
                rot = se3.R
                quat = R.from_matrix(rot).as_quat()

                t.transform.translation.x = pos[0]
                t.transform.translation.y = pos[1]
                t.transform.translation.z = pos[2]
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                self.br.sendTransform(t)
        
        def remove_transform(self, name: str): 
            """
            Elimina una terna específica del diccionario para que deje de publicarse.
            """
            # En el caso de la terna tool provoca un salto en la terna que se puede solucionar congelándola en la posición actual y 
            # cambiándole el parent a 'base' para que no se intente actualizar con el movimiento de la brida.
            if name in self.transforms:
                del self.transforms[name]
                self.get_logger().info(f"Terna '{name}' eliminada de la publicación.")
            else:
                self.get_logger().warn(f"Se intentó eliminar la terna '{name}', pero no existía.")

    class ROSManager(BaseRobotController):
        """
        Visualización del robot virtual en ROS. Instrucciones de movimiento, control del gripper, visualización de objetos, publicación de ternas y funcionalidades adicionales mediante MoveIt. Inicia y coordina todos los nodos correspondientes a las herramientas previas.
        """
        moveit_manager: 'MoveItManager' # Typehinting en funciones de MoveIt

        def __init__(self, project_path=None):
            super().__init__(project_path=project_path)

            # Instanciar el modelo cinemático de roboticstoolbox
            RobTarget.set_default_model(self.cobot_tb) 

            rclpy.init()
            self.logger = rclpy.logging.get_logger("ROSManager")    # Logger específico de la clase
            self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

            self.node_tf = TFPublisher()    # Manejo de ternas
            self.node_joint = joint_pub()   # Publicación de variables articulares
            self.node_obj = MarkerManager() # Manejo de objetos en RViz

            # Nombres de las articulaciones según URDF
            self.ARM_JOINT_NAMES = joint_names[:6]
            self.JOINT_NAMES_FULL = joint_names

            # Funcionalidades de MoveIt (opcionales)
            self.moveit_manager = None
            if MoveIt_OK:
                try:
                    from MoveItManager import MoveItManager
                    self.moveit_manager = MoveItManager(
                        project_path=self.project_root,
                        joint_names=self.JOINT_NAMES_FULL
                        )
                    self.moveit_manager.ROSManager_ref = self
                except Exception as e:
                    self.logger.error(f"Error a inicializar MoveIt: {e}")
            else:
                self.logger.warning("MoveIt no detectado. Funciones de planificación deshabilitadas.")

            # Publicación de posición home inicial
            msg = JointState()
            msg.name = joint_names
            msg.position = [0.0] * len(joint_names)
            self.node_joint.publisher.publish(msg)

            # Executor multithread para los nodos
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node_tf)
            self.executor.add_node(self.node_joint)
            self.executor.add_node(self.node_obj)

            # Permitir funcionamiento sin MoveIt
            if self.moveit_manager is not None:
                self.executor.add_node(self.moveit_manager)

            # Arranque del executor en un hilo aparte
            self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.executor_thread.start()

           # Esperar hasta tener el primer estado articular
            while self.node_joint.q_current is None:
                time.sleep(0.1)

            self.q_current = self.node_joint.q_current

            # Definición vacía de la API pymycobot por si se quiere acceder en modo ROS
            self.mc = GhostDriver(self.logger)

            self.logger.info("Entorno de ROS iniciado.")

        def MoveJ(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), wobj_name='MoveJ_wobj', robt_name='MoveJ_robt'):
            """
            Implementación de MoveJ para el robot virtual (ROS2).
            
            Genera una trayectoria interpolada en el espacio articular, visualiza 
            las ternas "tool", "workobject" y "robtarget" en RViz mediante TFPublisher y ejecuta el movimiento virtual
            a través de joint_pub.
            """
            # Publicación de ternas
            self.node_tf.add_wobj(wobj, wobj_name)
            self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)
            self.node_tf.add_robt(tool, 'tool', 'link6')

            # Se calcula la pose objetivo y luego se hallan los q con el PCI
            pose_calc = wobj * robtarget.pose * tool.inv()
            q_goal_arm = self.cobot_tb.ikine(pose_calc, robtarget.config)[0]

            # Hay que asegurarse de tomar los 6 primeros joints
            q_start_arm = self.get_current_q()
            
            # Generar trayectoria con el interpolador trapezoidal. qb = qc
            traj_keypoints = np.vstack([q_start_arm, q_goal_arm, q_goal_arm])
            self.cobot_tb.genTrJoint(traj_keypoints, np.zeros(traj_keypoints.shape[0]))
            full_traj = self.cobot_tb.q_ref

            # Control de velocidad mediante subsampling de la trayectoria (solo brazo)
            skip = self._get_sim_skip(speed)
            traj_arm_subsampled = full_traj[::skip]
            
            # Corrección de punto final: asegurar que se respeta el objetivo exacto
            if not np.allclose(traj_arm_subsampled[-1], full_traj[-1]):
                traj_arm_subsampled = np.vstack([traj_arm_subsampled, full_traj[-1]])

            # Chequear manipulabilidad
            self._manipulability(robtarget, wobj, tool)

            # Publicación de las poses delegada al helper
            self._execute_arm_trajectory(traj_arm_subsampled, robt_name, wobj_name)

        def MoveL(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), wobj_name='MoveL_wobj', robt_name='MoveL_robt'):
            """
            Implementación de MoveL para el robot virtual (ROS2).
            
            Genera una trayectoria interpolada en el espacio articular, visualiza 
            las ternas "tool", "workobject" y "robtarget" en RViz mediante TFPublisher y ejecuta el movimiento virtual
            a través de joint_pub.
            """
            # Publicación de ternas
            self.node_tf.add_wobj(wobj, wobj_name)
            self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)
            self.node_tf.add_robt(tool, 'tool', 'link6')
            
            # Se obtienen las poses objetivo e inciales leyendo los ángulos actuales y aplicando el PCD
            pose_goal = wobj * robtarget.pose * tool.inv()
            q_start_arm = self.get_current_q()
            pose_start = self.cobot_tb.fkine(q_start_arm)

            # Generador cartesiano con SLERP y perfil trapezoidal
            self.cobot_tb.genTrCart([pose_start*tool, pose_goal*tool, pose_goal*tool], 0*np.ones(3), conf=robtarget.config, tool = tool)
            full_traj = self.cobot_tb.q_ref

            # Control de velocidad mediante subsampling de la trayectoria (solo brazo)
            skip = self._get_sim_skip(speed)
            traj_arm_subsampled = full_traj[::skip]
            
            # Corrección de punto final: asegurar que se respeta el objetivo exacto
            if not np.allclose(traj_arm_subsampled[-1], full_traj[-1]):
                traj_arm_subsampled = np.vstack([traj_arm_subsampled, full_traj[-1]])

            # Publicación de las poses con el helper
            self._execute_arm_trajectory(traj_arm_subsampled, robt_name, wobj_name)
        
        def GripperState(self, apertura: float, spd: int = 30):
            """
            Implementación de GripperState para el robot virtual (ROS2). La pinza se mueve de forma instantánea,
            sin interpolar posiciones.      
            """
            if not (0 <= apertura <= 100):
                raise ValueError("La apertura debe estar entre 0 y 100%.")
            
            # Mapear al rango que interpreta ROS: -0.7 a 0.3
            val_gripper = apertura / 100 - 0.7

            q_full = self.get_current_q(prefer_gripper=True)
            q_full[6] = val_gripper

            # Enviar como movimiento de un solo punto
            self._send_move([q_full], "Gripper", "Tool")

        def MoveJ_q(self, q, spd = 30, unit = 'deg'):
            """
            Implementación de MoveJ_q para el robot virtual (ROS2).

            Genera una trayectoria interpolada en el espacio de variables articulares
            desde la pose actual hasta el vector `q` dado como objetivo. Luego la publica mediante
            joint_pub. Mantiene el estado actual de la pinza.
            """
            if unit == 'deg':
                q_brazo = np.deg2rad(q).tolist()
            elif unit == 'rad':
                q_brazo = q

            # Obtención del q actual: se toman los ángulos asociados a los 6 joints
            q_start_arm = self.get_current_q()

            # Generar la trayectoria joint para el brazo
            traj_keypoints = np.vstack([q_start_arm, q_brazo, q_brazo])
            self.cobot_tb.genTrJoint(traj_keypoints, np.zeros(traj_keypoints.shape[0]))
            full_traj = self.cobot_tb.q_ref

            # Control de velocidad mediante subsampling de la trayectoria (solo brazo)
            skip = self._get_sim_skip(spd)
            traj_arm_subsampled = full_traj[::skip]
            
            # Corrección de punto final: asegurar que se respeta el objetivo exacto
            if not np.allclose(traj_arm_subsampled[-1], full_traj[-1]):
                traj_arm_subsampled = np.vstack([traj_arm_subsampled, full_traj[-1]])
            
            self._execute_arm_trajectory(traj_arm_subsampled, "MoveJ_q", "JointSpace")

        def get_current_q(self, prefer_gripper=False, get_robt=False, tool: SE3 = SE3(), wobj: SE3 = SE3(), timeout=2.0):
            """
            Devuelve el vector de variables articulares correspondiente a la pose actual ordenado como np.array, permitiendo generar un robtarget a partir de la misma.
            
            A veces MoveIt altera el orden de los joints y devuelve un que no corresponde con la definición de joint_names. La función incluye un método para mapear los nombres y variables articulares, logrando compatibilidad con `self.q_current`.

            Args:
                prefer_gripper (bool): Si `True`, devuelve el estado del gripper junto con las 6 variables articulares del brazo.
                timeout (float): Tiempo máximo de espera en segundos.
                get_robt (bool): Si `True`, devuelve también el RobTarget actual.
                tool (SE3): Herramienta aplicada para generar el RobTarget (si `get_robt=True`).
                wobj (SE3): Workobject aplicado (si `get_robt=True`).
                timeout (float): Tiempo máximo de espera en segundos.

            Returns:
                ordered_q (array): Vector de variables articulares.
                robt (RobTarget, optional): RobTarget generado a partir de la pose actual (si get_robt=True).
            """
            t0 = time.time()

            # Definir qué nombres de articulaciones se utilizarán
            if prefer_gripper and hasattr(self, "JOINT_NAMES_FULL"):
                names = self.JOINT_NAMES_FULL
            else:
                names = self.ARM_JOINT_NAMES

            while time.time() - t0 < timeout:
                # Se pide el mensaje crudo, que incluye los joint_names
                last_msg = getattr(self.node_joint, "last_joint_msg", None)

                if last_msg is not None and hasattr(last_msg, "name"):
                    # position_map = {n: p for n, p in zip(last_msg.name, last_msg.position)}
                    position_map = dict(zip(last_msg.name, last_msg.position)) # Otra forma
                    
                    # ordered_list = [position_map.get(n, 0.0) for n in names]
                    ordered_q = np.array([position_map.get(n, 0.0) for n in names])

                    # Si se lo solicita, generar el Robtarget
                    if get_robt:
                        q_arm = ordered_q[:6]   # Siempre se usan los joints del brazo
                        robt = RobTarget.from_q(q_arm, tool, wobj)
                        self.logger.info(f'Robtarget generado = {robt}')
                        return ordered_q, robt
                    
                    return ordered_q
                
                time.sleep(0.05)
            
            # Timeout
            self.node_joint.get_logger().warning("get_current_q: timeout esperando joint_states")
            return None     
        
        def show_tf(self, terna, nombre='terna1', ref = SE3()):
            """
            Muestra en RViz una terna dada por un objeto SE3 definida respecto a `ref`.

            Args:
                terna (SE3): Objeto SE3 a mostrar.
                nombre (str): Nombre de la terna en RViz.
                ref (SE3): Referencia respecto a la cual se define la terna.
            """
            self.node_tf.add_wobj(ref*terna, nombre)
            time.sleep(1.0) # Esperar para que se procese la terna

        def hide_tf(self, terna = 'tool'):
            """
            Limpia específicamente una de la visualización. Por defecto se aplica a la terna `tool`.
            """
            self.node_tf.remove_transform(terna)

        def view_pose(self, target, tool: SE3 | None = SE3(), wobj: SE3 = SE3(), wobj_name='wobj1', robt_name='robt1'):
            """
            Muestra un robtarget en RViz de forma inmediata, sin generar trayectorias.

            Args:
                target (RobTarget or list/np.ndarray): Robtarget a mostrar o vector de variables articulares.
                tool (SE3, optional): Herramienta aplicada. Si no se especifica, se asume nula (brida).
                wobj (SE3): Workobject aplicado.
                wobj_name (str): Nombre del workobject para visualizar en RViz.
                robt_name (str): Nombre del robtarget para visualizar en RViz.
            """
            # Determinar el q asociado al robtarget 
            # Si se recibe un q directamente entonces se usa dicho argumento
            q_sol = self._parse_to_joint_list(target, tool, wobj)
           
            # Se agregan también las ternas del workobject y robtarget
            self.node_tf.add_wobj(wobj, wobj_name)
            # Si se pasa un robtarget se publica su pose asociada
            if isinstance(target, RobTarget):
                self.node_tf.add_robt(target.pose, robt_name, wobj_name)
            # Si se pasa un q, se calcula la pose
            else:
                self.node_tf.add_robt(self.cobot_tb.fkine(q_sol)*tool, robt_name, wobj_name)
            
            self.node_tf.add_robt(tool, 'tool', 'link6')
            
            q_arm_target = np.array([q_sol]) 

            # El helper publica la pose
            self._execute_arm_trajectory(q_arm_target, robt_name, wobj_name)
            time.sleep(0.1)
        
        def view_pose_configs(self, robt: SE3, tool: SE3 = SE3(), wobj: SE3 = SE3()):
            """
            Permite analizar todas las configuraciones posibles para una pose en RViz mediante un menu interactivo. No se analizan colisiones.

            Args:
                robt (RobTarget): Pose a analizar.
                tool (SE3): Herramienta aplicada.
                wobj (SE3): Workobject aplicado.
            """
            # Obtención de configuraciones a través del PCI
            confs = robt.find_valid_configs(tool, wobj)
            if not confs:
                self.logger.warning("No se encontraron configuraciones válidas para esa pose.")
                return
            
            last_conf = None
            
            # Menú interactivo
            while True:
                print("\n" + "="*30)
                print(f"Menú de configuraciones: se hallaron {len(confs)} soluciones")
                print("="*30)
                for i, conf in enumerate(confs, start = 1):
                    marker = " *" if conf == last_conf else "" # Marca la actual
                    print(f"{i}: {conf}{marker}")

                print("\nSeleccione un número para ver la configuración.")
                print("Presione 'q' para salir.\n")

                user_in = input("Opción: ")

                if user_in.lower() == 'q':   # Salir del menú
                    if last_conf:
                        self.logger.info(f"Saliendo del menú. La última configuración seleccionada fue: {last_conf}")
                    else:
                        self.logger.info("Saliendo del menú. No se seleccionó ninguna configuración.")
                    break

                try:
                    idx = int(user_in)
                    if 1 <= idx <= len(confs):
                        last_conf = confs[idx - 1]
                        print(f"Configuración seleccionada #{idx}: {last_conf}")

                        # Creación y visualización del robtarget
                        robt_sel = RobTarget(robt.pose, list(last_conf))
                        self.view_pose(robt_sel, tool, wobj)

                    else:
                        print(f"Índice fuera de rango. Elija entre 1 y {len(confs)}.")
                except ValueError:
                    print(f"Entrada inválida, use un número entre 1 y {len(confs)} o 'q' para salir.")

        def check_pose_configs(self, robt, tool: SE3=SE3(), wobj: SE3=SE3(), filtrar: bool=True):
            """
            Recorre todas las configuraciones con solución analítica del PCI,
            evalúa colisiones mediante MoveIt y muestra un menú interactivo.

            Args:
                robt (RobTarget): Pose a analizar.
                tool (SE3): Herramienta aplicada.
                wobj (SE3): Workobject aplicado.
                filtrar (bool) : `True` para filtrar colisiones. `False` para visualizar todas las configuraciones que satisfacen el PCD.
            """
            # Obtención de configuraciones a través del PCI
            configs = robt.find_valid_configs(tool, wobj)
            if not configs:
                self.logger.warning("No se encontraron configuraciones válidas para esa pose.")
                return
            
            self.logger.info(f"Analizando colisiones para {len(configs)} configuraciones...")
            results = [] 

            # Recorrer las configuraciones encontradas analíticamente
            for conf in configs:
                # Construir el robtarget con la configuración actual
                target = RobTarget(robt.pose, list(conf))
                collision_free = True   # Por defecto se asume sin colisión para que el filtrado no la elimine
                # Análisis en MoveIt
                try:
                    collision_free = bool(self.moveit_manager._check_collision(target, tool, wobj))
                except Exception as e:
                    print(f"Error al chequear colisión para {conf}: {e}")

                # results.append((conf, collision_free))
                results.append({
                'conf': conf,
                'ok': collision_free,
                'target': target
                })

            # Filtrado
            if filtrar:
                display_list = [opt for opt in results if opt['ok']]
            else:
                display_list = results

            if not display_list:
                self.logger.info("Ninguna configuración cumple con el criterio de filtrado.")
                return

            last_conf = None
            while True:
                print("\n" + "="*40)
                print(f" Análisis de Colisiones: {len(display_list)} opciones")
                print("="*40)
                
                for i, item in enumerate(display_list, start=1):
                    mark = " *" if i-1 == last_conf else "" # Marcar actual
                    estado = "OK" if item['ok'] else "En colisión" # Agregar estado al menu
                    print(f"{i}: {item['conf']}  ({estado}){mark}")

                print("-" * 40)
                print("Seleccione número para visualizar o 'q' para salir.")

                user_in = input("Opción: ").strip()
                
                if user_in.lower() == 'q':
                    self.logger.info("Finalizando menú de colisiones.")
                    break

                # Validación de la entrada por pantalla
                if not user_in.isdigit():
                    print("[!] Por favor ingrese un número válido.")
                    continue
                
                idx = int(user_in)
                if 1 <= idx <= len(display_list):
                    conf = display_list[idx - 1]
                    last_conf = idx - 1
                    print(f"Configuración seleccionada: {conf['conf']} → {'OK' if conf['ok'] else 'En colisión'}")
                    # Mostrar en RViz
                    self.moveit_manager.move_goal(conf['target'], tool, wobj)
                else:
                    print("Índice fuera de rango. Intente nuevamente.")

        # def traj_moveit(self, archivo: str, traj_name: str = "TRAJ", tool = SE3()):
        #     """
        #     Implementación de `traj_moveit` en el robot virtual (ROS2).
        #     Soporta tanto el formato nuevo pre-procesado (q_deg, speed, t_sec)
        #     como formatos anteriores.
        #     """
        #     self.node_tf.add_robt(tool, 'tool', 'link6')
        #     trajectory_points = self.load_data("Trayectorias", archivo, traj_name, verbose=True)

        #     if not trajectory_points:
        #         self.logger.error(f"No se pudo cargar la trayectoria {traj_name} de {archivo}")
        #         return None

        #     first_elem = trajectory_points[0]

        #     # Caso 1: Formato nuevo sincronizado -> (q_deg, speed, t_sec)
        #     if isinstance(first_elem, tuple) and len(first_elem) == 3:
        #         q_rad_list = [np.deg2rad(pt[0][:6]) for pt in trajectory_points]
        #         # Calcular dt promedio entre puntos para publicar en RViz al ritmo real
        #         if len(trajectory_points) > 1:
        #             dt = (trajectory_points[-1][2] - trajectory_points[0][2]) / (len(trajectory_points) - 1)
        #         else:
        #             dt = 0.05

        #         traj_arm = np.asarray(q_rad_list)
        #         self._check_limits(traj_arm, unit='rad')

        #         q_full_now = self.get_current_q(prefer_gripper=True, timeout=0.5)
        #         gripper_val = q_full_now[6] if q_full_now is not None else 0.0
        #         gripper_col = np.full((traj_arm.shape[0], 1), gripper_val)
        #         traj_full = np.c_[traj_arm, gripper_col]

        #         self._send_move(traj_full, f'{traj_name}', 'Moveit', dt=dt)
        #         last_q = q_rad_list[-1]

        #     # Caso 2: Formato tupla anterior -> (q_rad, t_sec)
        #     elif isinstance(first_elem, tuple) and len(first_elem) == 2:
        #         q_rad_list = [np.asarray(pt[0][:6], dtype=float) for pt in trajectory_points]
        #         self._execute_arm_trajectory(np.asarray(q_rad_list), f'{traj_name}', 'Moveit')
        #         last_q = q_rad_list[-1]

        #     # Caso 3: Lista simple de vectores q en radianes
        #     else:
        #         first_len = len(first_elem)
        #         if first_len not in (6, 7):
        #             raise ValueError(f"Dimensión inesperada en el primer punto: {first_len}")
        #         if first_len == 7:
        #             self._send_move(trajectory_points, f'{traj_name}', 'Moveit')
        #         else:
        #             self._execute_arm_trajectory(np.asarray(trajectory_points), f'{traj_name}', 'Moveit')
        #         last_q = trajectory_points[-1][:6]

        #     return RobTarget.from_q(last_q, tool)


        def traj_moveit(self, archivo: str, traj_name: str = "TRAJ", tool = SE3()):
            """
            Implementación de `traj_moveit` en el robot virtual (ROS2).
            Reproduce la trayectoria en RViz de forma fluida sin colisionar con ros2_control.
            """
            self.node_tf.add_robt(tool, 'tool', 'link6')
            trajectory_points = self.load_data("Trayectorias", archivo, traj_name, verbose=True)

            if not trajectory_points:
                self.logger.error(f"No se pudo cargar la trayectoria {traj_name} de {archivo}")
                return None

            first_elem = trajectory_points[0]

            # 1. Extraer lista de posiciones (en radianes) y tiempos (t_sec)
            if isinstance(first_elem, tuple) and len(first_elem) == 3:
                # Formato nuevo: (q_deg, speed, t_sec)
                q_rad_list = [np.deg2rad(pt[0][:6]).tolist() for pt in trajectory_points]
                t_sec_list = [float(pt[2]) for pt in trajectory_points]
            elif isinstance(first_elem, tuple) and len(first_elem) == 2:
                # Formato tupla intermedio: (q_rad, t_sec)
                q_rad_list = [list(pt[0][:6]) for pt in trajectory_points]
                t_sec_list = [float(pt[1]) for pt in trajectory_points]
            else:
                # Formato lista simple de vectores q en radianes
                q_rad_list = [list(pt[:6]) for pt in trajectory_points]
                t_sec_list = [i * 0.05 for i in range(len(q_rad_list))]

            self._check_limits(np.asarray(q_rad_list), unit='rad')

            # 2. Llevar el robot suavemente al primer punto (t=0) si no está ahí, igual que en el robot real
            q_now = self.get_current_q(prefer_gripper=False, timeout=0.5)
            if q_now is not None and not np.allclose(q_now[:6], q_rad_list[0], atol=0.02):
                self.logger.info("Moviendo robot virtual al punto inicial de la trayectoria...")
                if self.moveit_manager is not None:
                    self.moveit_manager.plan_and_execute(q_rad_list[0], execute=True, update_rviz=False)
                else:
                    self.MoveJ_q(q_rad_list[0], spd=30, unit='rad')
                time.sleep(0.2)

            # 3. Si estamos en el entorno de MoveIt (demo.launch.py), enviar al ActionServer arm_controller
            # para que FakeSystem ejecute la curva suave a 100 Hz sin pelear por /joint_states
            if self.moveit_manager is not None:
                from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
                from builtin_interfaces.msg import Duration

                jt = JointTrajectory()
                jt.joint_names = self.ARM_JOINT_NAMES

                for q_rad, t_sec in zip(q_rad_list, t_sec_list):
                    pt = JointTrajectoryPoint()
                    pt.positions = [float(x) for x in q_rad]
                    sec = int(t_sec)
                    nanosec = int((t_sec - sec) * 1e9)
                    pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
                    jt.points.append(pt)

                ok = self.moveit_manager._send_trajectory_action(jt, wait_for_result=True)
                if ok:
                    last_q = q_rad_list[-1]
                    return RobTarget.from_q(last_q, tool)
                else:
                    self.logger.warning("Action server no disponible, usando publicación directa...")

            # 4. Fallback (si estás usando move.launch.py sin MoveIt):
            # Publicar con reloj monotónico exacto
            q_full_now = self.get_current_q(prefer_gripper=True, timeout=0.5)
            gripper_val = float(q_full_now[6]) if q_full_now is not None else 0.0

            start_time = time.perf_counter()
            for q_rad, t_sec in zip(q_rad_list, t_sec_list):
                wait_time = (start_time + t_sec) - time.perf_counter()
                if wait_time > 0:
                    time.sleep(wait_time)

                msg = JointState()
                msg.header.stamp = self.node_joint.get_clock().now().to_msg()
                msg.name = self.JOINT_NAMES_FULL
                msg.position = [float(x) for x in q_rad] + [gripper_val]
                self.node_joint.publisher.publish(msg)

            last_q = q_rad_list[-1]
            return RobTarget.from_q(last_q, tool)
        
        def add_scene_object(self, name: str, pose_init: tuple, size: tuple, 
                         color: tuple = (0.5, 0.5, 0.5, 1.0),
                         shape: int = 1, movable: bool = True, 
                         mesh: str = None, rot_euler: tuple = (0.0, 0.0, 0.0)):
            """
            Agrega un marker a la escena de simulación (RViz).
        
            Permite insertar primitivas geométricas (cubos, esferas) o mallas (STL) que pueden
            ser manipuladas por el robot si 'movable' es True.

            Args:
                name (str): Identificador único del maker.
                pose_init (tuple): Posición inicial (x, y, z) en metros.
                size (tuple): Escala del maker (scale_x, scale_y, scale_z).
                color (tuple, optional): Color RGBA normalizado (0.0 a 1.0). Default: Gris.
                shape (int, optional): Tipo de geometría. Usar clase 'Shapes' del SDK (Shapes.CUBE, por ejemplo).
                movable (bool, optional): Si es True, el gripper podrá "tomar" este maker. Default: True.
                mesh (str, optional): Ruta absoluta al archivo de malla (ej: "file:///ruta/pieza.stl"). Requerido si shape=Shapes.MESH.
                rot_euler (tuple, optional): Orientación inicial (Roll, Pitch, Yaw) en radianes.

            Examples:
                - robot.add_scene_object("caja", (0.2, 0, 0), (0.05, 0.05, 0.05), shape=Shapes.CUBE)

                - robot.add_scene_object(name="mate", pose_init=(280.0e-3, -210e-3, 180e-3),
                  size=(1.0e-3, 1.0e-3, 1.0e-3), color=(154/255, 114/255, 71/255, 1.0), shape=MESH,
                  movable=False,
                  mesh="file:///home/user/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/Mate/Mate.STL", rot_euler=(np.pi/2, 0, -0.48))
            """
            # Pasamos los argumentos explícitamente al nodo interno
            self.node_obj.add_object(
                name=name, pose_init=pose_init, size=size, color=color,
                shape=shape, movable=movable, mesh=mesh, rot_euler=rot_euler
            )

        def load_scene(self, scene_filename: str, subfolder: str = ""):
            """
            Busca el archivo de escena en la misma carpeta que la rutina,
            importa 'setup_scene' y la ejecuta.

            Args:
                scene_filename (str): Nombre del archivo de definición de la escena.
                subfolder (str): Subcarpeta donde se encuentra el archivo.

            Raises:
                FileNotFoundError: Si no se encuentra el archivo.
                AttributeError: Si el archivo no tiene la función 'setup_scene(robot)'.
            """
            self.clear_scene()
            
            print(f"[Sim] Intentando cargar escena: {scene_filename}...")
            
            try:
                # Carga del archivo
                setup_func = self.load_data(
                    subfolder=subfolder, 
                    filename=scene_filename, 
                    var_name="setup_scene",
                    verbose=True
                )
                
            except FileNotFoundError:
                self.logger.warning(f"[Sim] Advertencia: No se encontró el archivo de escena '{scene_filename}'.")
            except AttributeError:
                self.logger.warning(f"[Sim] Advertencia: El archivo '{scene_filename}' no tiene una función 'setup_scene(robot)'.")
            
            try:
                # Ejecutar la función de armado de escena pasando este robot
                setup_func(self)
                self.logger.info(f"Escena '{scene_filename}' cargada correctamente.")
                
            except Exception as e:
                self.logger.error(f"[Sim] Error de ejecución dentro de '{scene_filename}': {e}")
        
        def clear_scene(self):
            """Elimina todos los objetos de la escena actual."""
            self.node_obj.clear_scene()

        def toggle_ros_connection(self, enable: bool):
            """
            Activa o desactiva la comunicación de `ros2_control` con el robot. Permite visualzar movimientos del generador de trayectorias propio (`MoveJ`, `MoveC`) en el paquete de MoveIt.
            Equivalente a: ros2 control switch_controllers --activate/deactivate joint_state_broadcaster
            
            Args:
                enable (bool): 
                `False` libera el robot del control de MoveIt.
                `True` cede el control a MoveIt, rehabilitando MotionPlanning.
            """
            broadcaster_name = "joint_state_broadcaster"
            
            if enable:
                self.logger.info("Re-activando joint_state_broadcaster...")
                return self.moveit_manager._switch_controllers(
                    activate_list=[broadcaster_name],
                    deactivate_list=[],
                    strictness=1
                )
            else:
                self.logger.info("Desactivando joint_state_broadcaster...")
                return self.moveit_manager._switch_controllers(
                    activate_list=[],
                    deactivate_list=[broadcaster_name],
                    strictness=1
                )

        def teach_poses(self, cantidad, ajuste=False, q_list = None):
            """
            Implementación de `teach_poses` en ROS.
            Se debe pasar una lista en grados para lograr compatibilidad con métodos matemáticos. 
            Si los datos obtenidos provienen de la función `teach_poses` del cobot, ya se encuentran en el formato apropiado.

            Raises:
                ValueError: Si la cantidad de vectores articulares no coincide con la requerida.
            """
            self.logger.info("Grabación de poses en ROS: usando lista de vectores articulares...")
            if len (q_list) != cantidad:
                raise ValueError("La cantidad de vectores articulares no coincide con la requerida por el método.")
            return q_list, 0

        def _send_move(self, qtraj_a, robt_name = 'robt', wobj_name = 'wobj', dt=0.1):
            """
            Helper interno para enviar una trayectoria articular al nodo joint_pub.

            Args:
                qtraj_a (lista, np.ndarray): trayectoria articular del brazo.
                robt_name (str): Nombre del robtarget en RViz.
                wobj_name (str): Nombre del workobject en RViz.
                dt (float): Tiempo de espera entre puntos para la publicación.
            """
            self.logger.info(f">>> Move: {robt_name} @ {wobj_name}")

            self.node_joint.publish_trajectory(qtraj_a, joint_names, dt)

            if len(qtraj_a) > 0:
                self.q_current = qtraj_a[-1].copy()

        def _execute_arm_trajectory(self, trajectory_arm_6dof: np.ndarray, robt_name: str, wobj_name: str):
            """
            Helper interno para ejecución de trayectorias en ROS. Recibe valores articulares para el brazo,
            agrega el valor de la pinza actual en cada uno de los puntos y la envía a publicar.

            Args:
                trajectory_arm_6dof (np.ndarray): Trayectoria articular del brazo (Nx6).
                robt_name (str): Nombre del robtarget en RViz.
                wobj_name (str): Nombre del workobject en RViz.
            """
            # Validar límites de los puntos que hacen la trayectoria
            self._check_limits(trajectory_arm_6dof)

            # Obtener valor actual del gripper (índice 6). Si no se inicializó se asume 0
            q_full_now = self.get_current_q(prefer_gripper=True, timeout=0.5)
            # gripper_val = self.q_current[6] if self.q_current is not None else 0.0
            gripper_val = q_full_now[6] if q_full_now is not None else 0.0

            # Agregar la columna de gripper y unir (N, 6) -> (N, 7)
            filas = trajectory_arm_6dof.shape[0]
            gripper_col = np.full((filas, 1), gripper_val)
            # columna_gripper = np.full((filas, 1), gripper_val)
            # trajectory_full = np.hstack([trajectory_arm_6dof, columna_gripper])

            trajectory_full = np.c_[trajectory_arm_6dof, gripper_col]

            # Publicar las poses
            self._send_move(trajectory_full, robt_name, wobj_name)
        
        def _get_sim_skip(self, speed: int) -> int:
            """
            Helper interno para controlar la velocidad del movimiento visualizado en RViz. Controla el subsampling de la trayectoria
            obtenida con el generador joint/cartesiano.
            
            
            Args:
                n_points (int): Total de puntos obtenidos por el generador de trayectoria.
                speed (int): Velocidad comandada (1-100).

            Returns:
                skip (int): Cantidad de puntos a saltar entre cada pose publicada.
            """
            # Protección contra velocidades fuera de rango
            safe_speed = int(np.clip(speed, 1, 100))
                            
            # Incluso en trayectorias cortas, se quiere un mínimo de skip
            BASE_SKIP = 5
            LINEAR_FACTOR = 0.5  # Ajuste de sensibilidad lineal

            variable_skip = round(safe_speed * LINEAR_FACTOR)
            skip = BASE_SKIP + variable_skip    # Min: 5 Máx: 35
            
            return skip
        
        def _parse_to_joint_list(self, target, tool, wobj):
            """
            Helper interno que convierte un RobTarget/SE3/Lista a lista de joints usando isinstance.
            """
            if target is None:
                return None

            if isinstance(target, RobTarget):
                if self.cobot_tb is None:
                    self.logger.error("Falta kinematic_model")
                    return None
                
                pose_calc = wobj * target.pose * tool.inv()
                try:
                    # Usamos el modelo inyectado
                    res = self.cobot_tb.ikine(pose_calc, target.config)
                    q_sol = res[0]
                    return np.array(q_sol).flatten().tolist()
                except Exception as e:
                    self.logger.error(f"Error IK: {e}")
                    return None

            elif isinstance(target, (list, np.ndarray, tuple)):
                return list(target)

        def _publish_goal_tf(self, q: list, tool: SE3, wobj: SE3, frame_name: str, tf_reference: str):
            """
            Publica el robtarget (pose del TCP) correspondiente a q como un frame TF llamado `frame_name`.
            Args:
                q (lista/np.array) : (6 o 7 elementos). Si tiene 7, se usa solo 6 primeros para FK.
            """
            # Tomar los primeros 6 q si hay 7 (gripper)
            q6 = q[:6]

            # Calcular la terna correspondiente al robtarget
            T = self.cobot_tb.fkine(q6) * tool

            # Si se definió un workobject, se lo publica junto con el robtarget asociado
            if wobj is not SE3():
                wobj_name = tf_reference
                self.node_tf.add_wobj(wobj, wobj_name)
            # Si no se definió un workobject, se asume referido a la base y se publica solamente el robtarget
            else:
                wobj_name = 'base'

            self.node_tf.add_robt(wobj.inv() * T, frame_name, wobj_name)

        def shutdown(self):
            """
            Cierra todos los nodos y el executor de ROS2.
            """
            self.executor.shutdown()

            if self.executor_thread.is_alive():
                self.executor_thread.join(timeout=1.0)
                self.node_tf.destroy_node()
                self.node_joint.destroy_node()
                self.node_obj.destroy_node()
                rclpy.shutdown()
                self.logger.info(">>> ROSManager finalizado.")

    class GhostDriver:
        """
        Driver 'Fantasma' para modo Simulación.
        Intercepta todas las llamadas dirigidas a 'self.mc' (hardware) y
        emite una advertencia.
        """
        def __init__(self, logger):
            self.logger = logger

        def __getattr__(self, name):
            """
            Se ejecuta cuando se intenta acceder a un atributo 
            o método que no existe explícitamente.
            """
            def wrapper(*args, **kwargs):
                self.logger.warning(
                    f"Ignorando llamada a hardware: 'robot.mc.{name}(...)'. "
                    "Esta función solo tiene efecto en el cobot real."
                )
                return None
            
            return wrapper
else:
    # Si se intentan usar métodos de ROS en el robot físico, se lanza un error crítico
    class ROSManager(BaseRobotController):
        def __init__(self, *args, **kwargs):
            
            raise RuntimeError(
                "\nERROR CRÍTICO: Intentaste iniciar 'ROSManager' pero ROS2 no está instalado.\n"
                "En este dispositivo (Raspi) solo puedes usar el modo 'real'."
            )
            
        # Métodos vacíos solo para satisfacer a BaseRobotController si fuera estricto
        def MoveJ(self, *args, **kwargs): pass
        def MoveL(self, *args, **kwargs): pass
        def GripperState(self, *args, **kwargs): pass

class MyCobotController(BaseRobotController):
    """
    Interfaz de control para el robot físico MyCobot320. Permite enviar instrucciones de movimiento y controlar la pinza. Utiliza la librería pymycobot para la comunicación con el robot y roboticstoolbox-python para la cinemática y el control avanzado.
    """
    def __init__(self, mode = 'raspi', rotar_base: bool = True, project_path=None, logger=None):
        if logger is None:
            # Creación del logger propio
            logger = logging.getLogger("MyCobotReal")
            logger.setLevel(logging.INFO)
            
            # Handler para consola
            if not logger.handlers:
                ch = logging.StreamHandler()
                # Formato: Hora - Nombre - Nivel - Mensaje
                formatter = logging.Formatter('[%(asctime)s] [%(name)s] [%(levelname)s]: %(message)s')
                ch.setFormatter(formatter)
                logger.addHandler(ch)
            
        super().__init__(project_path=project_path, logger_name='MyCobotController')
        # Conexión con el robot físico
        if mode == 'raspi':
            self.mc = MyCobot320('/dev/ttyAMA0', 115200)
        elif mode == 'TCP/IP':
             host = "10.42.0.1"
             port = 9000
             self.mc = MyCobotSocket(host, port)
        else:
            raise ValueError("Modo de conexión no válido. Intente con 'raspi' o 'TCP/IP'.")

        self._server_ref = None

    def MoveJ(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), wobj_name='MoveJ_wobj', robt_name='MoveJ_robt'):
        """
        Implementación de MoveJ para el robot físico MyCobot320. 
        
        Se calcula la pose objetivo usando la cinemática inversa de roboticstoolbox y se envía el comando mediante la API.
        El robot se detiene una vez llegado al objetivo.
        """
        print(f">>> Move: {robt_name} @ {wobj_name}")

        # Pose global = wobj * robtarget * inv(tool)
        pose_calc = wobj * robtarget.pose * tool.inv()

        # Se aprovecha la cinemática inversa de la toolbox para tener control de la config.
        q_pose = self.cobot_tb.ikine(pose_calc, robtarget.config)[0]

        self.mc.sync_send_angles(np.degrees(q_pose).tolist(), speed)

    def MoveL(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), wobj_name='MoveL_wobj', robt_name='MoveL_robt'):
        """
        Implementación de MoveL para el robot físico MyCobot320. 
        
        Se calcula la pose objetivo usando la cinemática inversa de roboticstoolbox, se traduce a coordenadas xyzrpy
        y se envía el comando mediante la API.
        El robot se detiene una vez llegado al objetivo.
        """
        print(f">>> Move: {robt_name} @ {wobj_name}")
        
        # Cálculo de la pose
        pose_calc = wobj * robtarget.pose * tool.inv()

        # Convertir a formato [x, y, z, rx, ry, rz] en grados.
        pose = list(pose_calc.t) + list(pose_calc.rpy(order='zyx', unit='deg'))
        pose_v2 = self.matrix_to_pose(pose_calc)

        self.mc.sync_send_coords(pose, speed, 1)

    def GripperState(self, apertura: int, spd: int = 30):
        """
        Implementación de `GripperState` para el robot físico MyCobot320.
        """
        # Primero hay que activar el gripper con la API, luego se pide el movimiento
        self.mc.set_gripper_mode(0)
        
        # Si apertura es 0 o 1 se usa la función de abrir/cerrar
        if apertura == 0:
            self.mc.set_gripper_state(1, spd)
        elif apertura == 100:
            self.mc.set_gripper_state(0, spd)
        # Si se pide una posición intermedia la función de la API cambia
        else:
            self.mc.set_gripper_value(apertura, spd, 1)

        # Envío del comando al observer para el modo de transmisión en vivo
        if self._server_ref is not None:
            self._server_ref.set_gripper_state(apertura)

    def MoveJ_q(self, q, spd = 30, unit = 'rad'):
        """
        Implementación de MoveJ_q para el robot físico MyCobot320.
        """
        if unit == 'rad':
            self.mc.sync_send_angles(np.degrees(q).tolist(), spd)
        elif unit == 'deg':
            self.mc.sync_send_angles(q.tolist(), spd)

    def MoveL_gt(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), skip: int = 10):
        """
        Implementación alternativa de MoveL para el robot físico.

        Se genera una trayectoria con el generador cartesiano (interpolador trapezoidal + SLERP) y se envían los puntos secuencialmente.
        """
        
        # Cálculo de la pose
        pose_goal = wobj * robtarget.pose * tool.inv() 

        # Obtención de la pose actual del controlador y conversión a SE3
        q_actual = self.mc.get_angles() # Se piden los ángulos de los encoders
        pose_start = self.cobot_tb.fkine(np.deg2rad(q_actual)) # Con la toolbox se encuentra la pose actual de la brida

        # Generación de la trayectoria con SLERP e interpolador trapezoidal
        self.cobot_tb.genTrCart([pose_start, pose_goal, pose_goal], 0*np.ones(3), conf = robtarget.config)
        qtraj_a = self.cobot_tb.q_ref[::skip]
        q_traj_deg = np.rad2deg(qtraj_a) # El controlador maneja grados
        print(f'Se generó una trayectoria con {len(q_traj_deg)} puntos.')

        self._check_limits(q_traj_deg, 'deg') # Verificación de límites en la trayectoria

        freq_q = 20 # Hz 
        for q in q_traj_deg:
            self.mc.send_angles(q.tolist(), 100)
            time.sleep(1/freq_q)
    
    def teach_poses(self, cantidad, ajuste = False, q_list=None):
        """
        Implementación de `teach_poses` para el robot físico MyCobot320.

        Guía al usuario para recolectar manualmente las poses del robot liberando sus motores y ajustando, de manera opcional,
        con un joystick interactivo para mitigar el efecto del juego mecánico.        
        """
        # Helper interno para grabación de poses
        def conteo_regresivo(segundos: int, mensaje: str):
            """
            Helper interno para mostrar un conteo regresivo en la consola en algunos pasos del método
            de grabación de poses.
            """
            print(mensaje)
            for i in range(segundos, 0, -1):
                print(f" >> Tiempo restante: {i} s   ", end='\r', flush=True)
                time.sleep(1)
            print(" " * 40, end='\r') # Limpia la línea al terminar

        self.logger.info(f"Iniciando rutina de grabación de {cantidad} poses (Modo Ajuste: {ajuste})")

        # Antes de grabar las posiciones se coloca la pieza auxiliar y se cierra la pinza
        print("\n=== Grabación de poses ===")
        print("La pinza se cerrará en 3 segundos para sujetar la pieza auxiliar (palpador).")
        conteo_regresivo(3, "Atención...") 
        self.GripperState(100)
        time.sleep(5)
        self.GripperState(0)

        # Iniciación de listas
        poses_q = [None] * cantidad
        poses_coord = [None] * cantidad

        for n in range (cantidad):
            print(f"\n" + "="*40)
            print(f" GRABACIÓN DE POSE {n+1} / {cantidad}")
            print("="*40)

            input(">> Prepárese para sujetar al robot.\nPresione [ENTER] para liberar los motores dentro de 5 segundos...")
            conteo_regresivo(5, "Liberando motores...")

            for eje in range(1, 7):
                self.mc.release_servo(eje)
            print(f"Motores liberados. Articule al robot a la pose deseada.")

            # La primera pose suele requerir más tiempo
            tiempo_ajuste = 20 if n == 0 else 10
            conteo_regresivo(tiempo_ajuste, f"Bloqueando motores en {tiempo_ajuste} segs.")

            if ajuste:
                print("\nActivando todos los motores para fijar la posición...")
                self.mc.focus_all_servos()
                time.sleep(2)

                q_inicial = self.mc.get_angles()

                # Control con el joystick. Ajustada la pose se presiona esc para continuar
                pose_ajustada = self.joystick_adjust(np.deg2rad(q_inicial),
                                                mover_callback=lambda r: self.MoveJ(r, 20)
                )

                time.sleep(1)
                # Se guardan los ángulos y las coordenadas
                q_ajustado = self.mc.get_angles()
                coord_ajustado = self.mc.get_coords()
                print(f'Pose {n+1} guardada: {q_ajustado}')
                
            
            else:
                # Si no se usa el joystick se toman mediciones antes y después de activar motores
                mediciones_q = []
                mediciones_pose = []
                print("\nTomando 5 mediciones antes de activar motores...")
                for i in range(5):
                    q_medida = self.mc.get_angles()
                    coord_medida = self.mc.get_coords()
                    mediciones_q.append(q_medida)
                    mediciones_pose.append(coord_medida)
                    time.sleep(0.1)

                print("\nActivando todos los motores para fijar la posición...")
                self.mc.focus_all_servos()
                print("Motores activados. El robot está firme.")

                time.sleep(1)  # Espera breve para asegurar que los motores estén activos

                # Tomar 5 mediciones después de activar motores
                print("Tomando 5 mediciones después de activar motores...")
                for i in range(5):
                    q_medida = self.mc.get_angles()
                    coord_medida = self.mc.get_coords()
                    mediciones_q.append(q_medida)
                    mediciones_pose.append(coord_medida)
                    time.sleep(0.1)

                # Calcular el promedio de las mediciones antes y después de activar motores
                q_ajustado = np.mean(np.array(mediciones_q), axis=0).tolist()
                coord_ajustado = np.mean(np.array(mediciones_pose), axis=0).tolist()
                print(f"Pose {n+1} guardada (promedio de 10 mediciones): {q_ajustado}")

            poses_q[n] = q_ajustado
            poses_coord[n] = coord_ajustado

        # Informar los datos
        self.logger.info("\nPoses grabadas:")
        for i, p in enumerate(poses_q):
            self.logger.info(f"Pose {i+1}: {p}")

        self.logger.info("\nCoords grabadas:")
        for i, d in enumerate(poses_coord):
            self.logger.info(f"Q-coords {i+1}: {d}")

        return poses_q, poses_coord
        
    def traj_moveit(self, archivo: str, traj_name: str = "TRAJ", tool = SE3()):
        """
        Implementación de `traj_moveit` en el robot real. Lee la trayectoria pre-procesada
        y la envía respetando los tiempos y velocidades de MoveIt.
        """
        if tool is None:
            tool = SE3()

        traj_data = self.load_data("Trayectorias", archivo, traj_name)
        if not traj_data:
            self.logger.error(f"No se pudo cargar la trayectoria {traj_name} de {archivo}")
            return None

        # Detectar automáticamente si el archivo tiene el formato nuevo (q_deg, speed, t_sec)
        # o el formato viejo (q_rad, t_sec)
        formato_nuevo = (len(traj_data[0]) == 3)

        if formato_nuevo:
            waypoints = traj_data
        else:
            # Si es un archivo viejo en radianes, lo convertimos una sola vez antes de empezar el reloj
            waypoints = []
            for pt, t_sec in traj_data:
                q_deg = [float(x) for x in np.rad2deg(np.asarray(pt)[:6])]
                waypoints.append((q_deg, 25, t_sec))

        # Asegurar que el robot esté en el punto inicial (t=0) antes de largar el cronómetro
        first_q_deg = waypoints[0][0]
        q_actual = self.mc.get_angles()
        # Solo usar sync_send_angles si el robot está a más de 2 grados del inicio
        if not q_actual or np.max(np.abs(np.array(q_actual) - np.array(first_q_deg))) > 2.0:
            self.mc.sync_send_angles(first_q_deg, 10)

        # Activar modo de procesamiento continuo (sin frenar entre puntos intermedios)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)

        self.logger.info(f"Ejecutando {len(waypoints)} waypoints sincronizados...")

        # Si el modo --bridge está activo, pausamos get_angles() para no saturar el puerto serie
        if self._server_ref is not None:
            self._server_ref.pause_serial_reading(True)

        start_time = time.perf_counter()

        try:
            for q_deg, spd, t_sec in waypoints[1:]:
                wait_time = (start_time + t_sec) - time.perf_counter()
                if wait_time > 0:
                    time.sleep(wait_time)

                self.mc.send_angles(q_deg, spd)

                # Informar al servidor TCP en RAM para verlo en vivo en RViz
                if self._server_ref is not None:
                    self._server_ref.set_external_angles(q_deg)
                    
        finally:
            # Al terminar la trayectoria (o si hay error), reanudar la lectura normal de encoders
            if self._server_ref is not None:
                self._server_ref.pause_serial_reading(False)

        # Medir duración real del bucle vs tiempo planificado por MoveIt
        t_real_bucle = time.perf_counter() - start_time
        t_planificado = waypoints[-1][2]

        # Volver a modo secuencial y asegurar ajuste fino del último punto
        time.sleep(0.05)
        self.mc.set_fresh_mode(0)
        time.sleep(0.05)
        last_q_deg = waypoints[-1][0]
        self.mc.send_angles(last_q_deg, 10)

        # Esperar asentamiento mecánico breve para leer encoders finales sin interferir
        time.sleep(0.3)
        q_alcanzado_deg = self.mc.get_angles()

        # Armar el RobTarget de la pose destino (pasando el último punto de grados a radianes)
        last_q_rad = np.deg2rad(last_q_deg)
        last_robt = RobTarget.from_q(last_q_rad, tool)

        # Validación cuantitativa (Tiempo, Error Articular y Error en TCP)
        if q_alcanzado_deg and len(q_alcanzado_deg) == 6:
            err_q_deg = np.abs(np.array(last_q_deg) - np.array(q_alcanzado_deg))
            max_err_q = float(np.max(err_q_deg))

            # Pose esperada vs Pose alcanzada según encoders (en mm)
            pose_esp = self.cobot_tb.fkine(last_q_rad) * tool
            pose_alc = self.cobot_tb.fkine(np.deg2rad(q_alcanzado_deg)) * tool
            err_tcp_mm = float(np.linalg.norm(pose_esp.t - pose_alc.t))

            self.logger.info(
                f"[Métricas MoveIt] Tiempo: {t_real_bucle:.2f}s (Plan: {t_planificado:.2f}s, "
                f"Diff: {t_real_bucle - t_planificado:+.3f}s) | "
                f"Max Err Articular: {max_err_q:.2f}° | Err TCP (encoders): {err_tcp_mm:.2f} mm"
            )

        return last_robt
    
    def load_scene(self, scene_filename: str, subfolder: str = ""):
        """
        Método vacío para mantener compatibilidad con BaseRobotController.
        """
        pass

    def _set_server_observer(self, server):
        """
        Observer que permite reproducir los comandos enviados a la pinza cuando se trabaja con el modo de transmisión en vivo a otra PC con ROS.
        """
        self._server_ref = server