# common_types.py
import numpy as np
import itertools
from spatialmath import SE3
from DHRobotGT import IKineError

class RobTarget:
    """
    Clase que representa poses objetivo para robots. Almacena la posición, orientación y configuración.
    """
    _default_model = None 

    @classmethod
    def set_default_model(cls, model):
        """Asigna el modelo cinemático que usarán todas las instancias de RobTarget por defecto."""
        cls._default_model = model

    def __init__(self, pose, config=None):
        """
        Args:
            pose: Matriz 4x4 (tipo numpy o SE3) que representa la pose cartesiana en mm.
            config: Lista con configuración articular, por ejemplo [1, -1, 1].
        Raises:
            ValueError: Si la pose no es una matriz 4x4 o un objeto SE3.
        """
        # Convertir SE3 a matriz si es necesario
        if isinstance(pose, SE3):
            self.pose = pose
        elif hasattr(pose, "shape") and pose.shape == (4, 4):
            self.pose = SE3(pose)
        else:
            raise ValueError("La pose debe ser una matriz 4x4 o un objeto SE3.")

        if config is not None:
             # Convertir a lista para mejor compatibilidad
            self.config = list(config)
        else:
            self.config = [1, 1, 1]  # Configuración por defecto
        
        self.valid_configs = []

    def find_valid_configs(self, tool: SE3 = SE3(), wobj: SE3 = SE3(), robot_model=None):
        """
        Analiza todas las configuraciones posibles para un RobTarget con una determinada herramienta y workobject.
        Devuelve aquellas que tienen solución analítica, sin verificar colisiones o límites articulares.
        
        Args:
            tool (SE3): Objeto SE3 que representa la herramienta.
            wobj (SE3): Objeto SE3 que representa el workobject.
            robot_model: Modelo cinemático del robot. Si es None, se usa el modelo por defecto.
        Returns:
            Lista de configuraciones válidas.
        Raises:
            ValueError: Si no se definió un modelo de robot.
        """
        # Determinar qué modelo de robot usar
        model_to_use = robot_model if robot_model is not None else self._default_model

        # Chequear que se haya incializado el modelo
        if model_to_use is None:
            raise ValueError("No se ha definido un modelo robot para RobTarget.")
        
        # Calcular pose objetivo según referencias
        T_global = wobj * self.pose * tool.inv()
        configs = list(itertools.product([1, -1], repeat=3))    # Generar todas las posibles configuraciones
        self.valid_configs = []
        
        # Iterar configuraciones. Si tienen solución se agregan a la lista
        for conf in configs:
            try:
                q_sol = model_to_use.ikine(T_global, conf)[0]
                if len(q_sol) > 0:
                    self.valid_configs.append(conf)
            except IKineError:
                continue
            except Exception as e:
                print(f"Error inesperado en cálculo IK: {e}")
                continue
    
        return self.valid_configs

    def offset(self, dx: float=0.0, dy: float=0.0, dz: float=0.0, rx: float=0.0, ry: float=0.0, rz: float=0.0):
        """
        Aplica una rototraslación al RobTarget respecto al workobject.
        
        Args:
            dx, dy, dz (float): Desplazamientos en mm respecto al wobj.
            rx, ry, rz (float): Rotaciones en grados respecto al wobj (orden 'zyx').
        Returns:
            Nuevo RobTarget desplazado y/o rotado respecto al workobject.
        """
        # Vector de desplazamiento y/o rotación
        T_offset = SE3(dx, dy, dz) * SE3.RPY(rx, ry, rz, order='zyx', unit='deg')

        # Aplicar transformación respecto al workobject
        Robt_T = T_offset * self.pose.copy()

        return RobTarget(Robt_T, config=self.config.copy())
    
    def relTool(self, dx: float=0.0, dy: float=0.0, dz: float=0.0, rx: float=0.0, ry: float=0.0, rz: float=0.0):
        """
        Aplica una rototraslación al RobTarget respecto a sí mismo.
        
        Args:
            dx, dy, dz: Desplazamientos en mm respecto a su propia terna.
            rx, ry, rz: Rotaciones en grados respecto a su propia terna (orden 'zyx').
        Returns:
            Nuevo RobTarget desplazado y rotado.
        """
        # Vector de desplazamiento y/o rotación
        T_robt = SE3(dx, dy, dz) * SE3.RPY(rx, ry, rz, order='zyx', unit='deg')

        # Aplicar transformación respecto a la terna RobTarget
        Robt_T = self.pose * T_robt

        return RobTarget(Robt_T, config=self.config.copy())
    
    def __repr__(self):
        return f"RobTarget(pose=SE3({self.pose.t.tolist()}, rpy={self.pose.rpy(unit='deg').tolist()} deg), config={self.config})"
    
    @classmethod
    def from_q(cls, q, tool: SE3 = SE3(), wobj:SE3 = SE3(), robot_model=None):
        """
        Crea un RobTarget a partir de un vector de variables articulares, un workobject y una herramienta recurriendo al PCD.

        Args:
            q (lista o numpy array): Vector de variables articulares.
            tool (SE3): Objeto SE3 que representa la herramienta.
            wobj (SE3): Objeto SE3 que representa el workobject. Por defecto se asume nulo, referenciando al RobTarget a la base.
            robot_model: Modelo cinemático del robot. Si es None, se usa el modelo por defecto.
        Returns:
            Nueva instancia de RobTarget.
        Raises:
            ValueError: Si no se definió un modelo de robot.
        """
        # Determinar qué modelo de robot usar
        model = robot_model if robot_model is not None else cls._default_model
        
        if model is None:
            raise ValueError("No se ha definido un modelo robot para RobTarget.")

        q_arr = np.array(q)
        # Determinar la pose referenciada al workobject
        pose_se3 = wobj.inv() * model.fkine(q_arr) * tool
        
        # Calcular configuración
        config = model.calc_conf(q_arr)
        
        return cls(pose_se3, config)

