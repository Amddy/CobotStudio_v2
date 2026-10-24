"""
SDK CobotStudio
---------------
Capa de abstracción para el desarrollo de rutinas.
Centraliza tipos, constantes y manejo de compatibilidad ROS/pymycobot.
"""
from __future__ import annotations
import numpy as np
import time
from spatialmath import SE3, UnitQuaternion
from RobtManager import RobTarget
from typing import TYPE_CHECKING
from CobotStudio import BaseRobotController

if TYPE_CHECKING:
    from CobotStudio import ROSManager

# En runtime
else:
    try:
        from CobotStudio import ROSManager as _RealROSManager
        from CobotStudio import ROS_OK
        
        if ROS_OK:
            # Si ROS está ok, ROSManager es la clase real
            ROSManager = _RealROSManager
        else:
            # Si ROS falló, forzar el error para ir al except
            raise ImportError("ROS no disponible.")  
    except ImportError:
        # Fallback para sintaxis si la clase no existe
        class ROSManager:
            """
            Clase placeholder para entornos sin ROS.
            Permite mantener el type hinting en rutinas compartidas.
            """
            pass

class Shapes:
    """Constantes para formas de objetos en visualización."""
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    MESH = 10
    ARROW = 0 
    TEXT = 9 

# Exportar símbolos públicos
__all__ = [
    'np', 
    'time', 
    'SE3',
    'UnitQuaternion',
    'RobTarget', 
    'ROSManager',     # La clase real o dummy según el entorno
    'BaseRobotController',
    'Shapes'
]