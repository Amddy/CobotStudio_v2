# CobotStudio

El siguiente documento tiene por objetivo enunciar y describir las funciones principales del entorno así como sus argumentos y casos de uso.

## 1 Funciones globales

Las funciones descriptas a continuación son válidas tanto en el entorno virtual (`ROSManager`) como en el robot real (`MyCobotController`).

### 1.1 Movimiento

#### `MoveJ(robt, speed, tool, wobj, wobj_name, robt_name)`

Realiza un movimiento articular (joint) hacia un objetivo (robtarget). Calcula la cinemática inversa para alcanzar la pose definida por `robt` dentro del sistema de coordenadas `wobj` usando la herramienta `tool`.
- **Parámetros**:
    - **robt** (`RobTarget`): Objetivo de pose y configuración del robot.
    - **speed** (`int` 0-100): Velocidad de movimiento.
    - **tool** (`SE3`): Transformación de la herramienta respecto a la brida (TCP).
    - **wobj** (`SE3`): Objeto de trabajo respecto a la base, referencia para `robt`.
    - **wobj_name** (`str`, optional): Nombre de la referencia en logs/visualización.
    - **robt_name** (`str`, optional): Nombre del punto objetivo en logs/visualización.

#### `MoveL(robt, speed, tool, wobj, wobj_name, robt_name)`

Realiza un movimiento cartesiano (lineal) hacia un objetivo (robtarget). Calcula la cinemática inversa para alcanzar la pose definida por `robt` dentro del sistema de coordenadas `wobj` usando la herramienta `tool`.
- **Parámetros**:
    - **robt** (`RobTarget`): Objetivo de pose y configuración del robot.
    - **speed** (`int` 0-100): Velocidad de movimiento.
    - **tool** (`SE3`): Transformación de la herramienta respecto a la brida (TCP).
    - **wobj** (`SE3`): Objeto de trabajo respecto a la base, referencia para `robt`.
    - **wobj_name** (`str`, optional): Nombre de la referencia en logs/visualización.
    - **robt_name** (`str`, optional): Nombre del punto objetivo en logs/visualización.

#### `GripperState(apertura, spd)`

Modifica el estado de la pinza del robot permitiendo aperturas parciales.
- **Parámetros**:
    - **apertura** (`float` 0-100): Valor de apertura de la pinza (0 cerrado, 100 abierto).
    - **spd** (`int` 0-100): Velocidad de movimiento de la pinza.

#### `MoveJ_q(q, speed, unit)`

Envía al robot al vector de variables articulares determinado.
- **Parámetros**:
    - **q** (`Array(1,6)`): Vector de variables articulares.
    - **speed** (`int` 0-100): Velocidad de movimiento.
    - **unit** (`str` 'deg', 'rad'): Unidad de las variables articulares.

#### `GoHome(speed)`

Envía al robot a su posición home: `q = [0, 0, 0, 0, 0, 0]`.
- **Parámetros**:
    - **speed** (`int` 0-100): Velocidad de movimiento.

#### `traj_moveit(archivo, traj_name, tool)`

Ejecuta una trayectoria guardada en un archivo .py como las que se pueden generar en MoveIt.

- **Parámetros**:
    - **archivo** (`str`): Nombre del archivo .py.
    - **traj_name** (`str`): Nombre de la variable dentro del archivo.
    - **tool** (`SE3`): Herramienta.
- **Retorno**:
    - **last_robt** (`RobTarget`): Robtarget correspondiente al último punto de la trayectoria ejecutada.

### 1.2 Enseñanza de ternas

#### `teach_and_save_wobj(filename, wobj_name, tool, save_q, method, q_test)`

Enseña un workobject referido a la base del robot y lo guarda en `Workobjects/filename.py`, creando el directorio si no existe. Opcionalmente guarda los `q` leídos por el cobot en `Workobjects/filename_q.py`.
- **Parámetros**:
    - **filename** (`str`): Nombre del archivo .py (sin .py opcional).
    - **wobj_name** (`str`): Nombre de la variable dentro del archivo.
    - **tool** (`SE3`): Herramienta usada para enseñar el wobj.
    - **save_q** (`bool`): Si `True`, guarda los q usados en la enseñanza en un archivo separado.
    - **method** (`str`): Método de enseñanza: '3points' o '6points'.
    - **q_test** (`list`, optional): Vectores de variables articulares considerados como puntos de enseñanza. Se utiliza en el robot virtual.
- **Retorno**:
    - **wobj_calculated** (`SE3`): Terna que representa al workobject calculado respecto a la base del robot.

#### `teach_and_save_TCP(filename, tcp_name, save_q, q_test)`

Enseña un TCP con el método de los 4 puntos y lo guarda en carpeta "TCPs" dentro del proyecto.
- **Parámetros**:
    - **filename** (`str`): Nombre del archivo .py.
    - **tcp_name** (`str`): Nombre de la variable.
    - **save_q** (`bool`): Si `True`, guarda los q usados en la enseñanza en un archivo separado.
    - **q_test** (`list`, optional): Vectores de variables articulares considerados como puntos de enseñanza. Se utiliza en el robot virtual.
- **Retorno**:
    - **tcp_calculated** (`SE3`): Objeto SE3 que representa la traslación al TCP enseñado.

### 1.3 Lectura de datos

#### `load_data(subfolder, filename, var_name, verbose)`

Importa dinámicamente una variable específica desde un archivo Python externo. Busca un archivo en la carpeta del proyecto para cargarlo como módulo y extraer la variable (terna, trayectoria, lista, etc.).
- **Parámetros**:
    - **subfolder** (`str`): Subcarpeta. Ejemplo: "Workobjects", "TCPs".
    - **filename** (`str`): Nombre del archivo .py.
    - **var_name** (`str`): Nombre de la variable a cargar.
    - **verbose** (`bool`): Log con información adicional.
- **Retorno**:
    - **val**: Variable leída. Puede ser una lista, array de numpy, o clase personalizada dependiendo del archivo fuente.

#### `load_wobj(archivo, wobj_name, tool, auto_teach, q_teach)`

Intenta cargar un Wobj buscando en la carpeta "Workobjects" del proyecto. Si falla y `auto_teach` es `True`, ofrece al usuario enseñarlo manualmente.
- **Parámetros**:
    - **archivo** (`str`): Nombre del archivo .py (sin .py opcional).
    - **wobj_name** (`str`): Nombre de la variable dentro del archivo.
    - **tool** (`SE3`): Herramienta usada para enseñar (solo si falla la carga).
    - **auto_teach** (`bool`): Si `True`,ofrece la opción de proceder con la enseñanza de la terna.
    - **q_teach** (`list`, optional): Vectores de variables articulares considerados como punto de enseñanza. Se utiliza en el robot virtual.
- **Retorno**:
    - **wobj** (`SE3`): Terna que representa al workobject respecto a la base del robot.

### 1.4 Herramientas

#### `matrix_to_pose(T_matrix)`

Convierte una matriz homogénea SE3 en lista [x, y, z, r, p, y] en grados.
- **Parámetros**:
    - **T_matrix** (`np.ndarray` o `SE3`): Matriz homogénea.
- **Retorno**:
    - **pose** (`list`): Lista con 6 elementos [x, y, z, r, p, y] en grados.

#### `pose_to_matrix(pose)`

Convierte una pose dada como lista `[x, y, z, r, p, y]` en una matriz homogénea SE3.
- **Parámetros**:
    - **pose** (`list`): Lista con 6 elementos [x, y, z, r, p, y] en grados.
- **Retorno**:
    - **T_se3** (`SE3`): Matriz homogénea SE3.

## 1.5 Robtargets

#### `offset(dx, dy, dz, rx, ry, rz)`

Aplica una rototraslación al robtarget respecto al workobject.
        
- **Parámetros**:
    - **dx, dy, dz** (`float`): Desplazamientos en mm respecto al wobj.
    - **rx, ry, rz** (`float`): Rotaciones en grados respecto al wobj (orden 'zyx').

#### `relTool(dx, dy, dz, rx, ry, rz)`

Aplica una rototraslación al RobTarget respecto a sí mismo.
        
- **Parámetros**:
    - **dx, dy, dz** (`float`): Desplazamientos en mm respecto a su propia terna.
    - **rx, ry, rz** (`float`): Rotaciones en grados respecto a su propia terna. (orden 'zyx').


## 2 Funciones de ROS

### 2.1 Construcción de la celda

<a id="add_scene_object"></a>
#### `add_scene_object(name, pose_init, size, color, shape, movable, mesh, rot_euler)`

Agrega un marker a la escena de simulación (RViz). Permite insertar primitivas geométricas (cubos, esferas) o mallas (STL).

- **Parámetros**:
    - **name** (`str`): Identificador único del maker.
    - **pose_init** (`tuple`): Posición inicial (x, y, z) en metros.
    - **size** (`tuple`): Escala del maker (x, y, z).
    - **color** (`tuple`, optional): Color RGBA normalizado (0.0 a 1.0). Default: Gris.
    - **shape** (`int`, optional): Tipo de geometría. Usar clase `Shapes` del SDK (Shapes.CUBE, por ejemplo).
    - **movable** (`bool`, optional): Si es `True`, el gripper podrá "tomar" este maker.
    - **mesh** (`str`, optional): Ruta absoluta al archivo de malla (ej: "file:///ruta/pieza.stl"). Requerido si `shape=Shapes.MESH`.
    - **rot_euler** (`tuple`, optional): Orientación inicial (Roll, Pitch, Yaw) en radianes.


### 2.2 Previsualización de poses

#### `show_tf(terna, nombre, ref)`

Muestra en RViz una terna dada por un objeto SE3 definida respecto a `ref`.

- **Parámetros**:
    - **terna** (`SE3`): Objeto SE3 a mostrar.
    - **nombre** (`str`): Nombre de la terna en RViz.
    - **ref** (`SE3`): Referencia respecto a la cual se define la terna.

#### `view_pose(target, tool, wobj, wobj_name, robt_name)`

Muestra un robtarget en RViz de forma inmediata, sin generar trayectorias.

- **Parámetros**:
    - **target** (`RobTarget` or `list`/`np.ndarray`): Robtarget a mostrar o vector de variables articulares.
    - **tool** (`SE3`): Herramienta aplicada. Si no se especifica, se asume nula (brida).
    - **wobj** (`SE3`): Workobject aplicado.
    - **wobj_name** (`str`): Nombre del workobject para visualizar en RViz.
    - **robt_name** (`str`): Nombre del robtarget para visualizar en RViz.

#### `view_pose_configs(robt, tool, wobj)`

Permite analizar todas las configuraciones posibles para una pose en RViz mediante un menú interactivo. No se analizan colisiones.

- **Parámetros**:
    - **robt** (`RobTarget`): Pose a analizar.
    - **tool** (`SE3`): Herramienta aplicada.
    - **wobj** (`SE3`): Workobject aplicado.

### 2.3 Lectura de datos

#### `get_current_q(prefer_gripper, get_robt, tool, wobj, timeout)`

Devuelve el vector de variables articulares correspondiente a la pose actual ordenado como `np.array`, permitiendo generar un robtarget a partir de la misma. Incluye un mapeo de variables para corregir el orden de joints que a veces altera MoveIt.

- **Parámetros**:
    - **prefer_gripper** (`bool`): Si `True`, devuelve el estado del gripper junto con las 6 variables articulares del brazo.
    - **get_robt** (`bool`): Si `True`, devuelve también el robtarget actual.
    - **tool** (`SE3`): Herramienta aplicada para generar el robtarget (si `get_robt=True`).
    - **wobj** (`SE3`): Workobject aplicado (si `get_robt=True`).
    - **timeout** (`float`): Tiempo máximo de espera en segundos.
- **Retorno**:
    - **ordered_q** (`array`): Vector de variables articulares.
    - **robt** (`RobTarget`, optional): robtarget generado a partir de la pose actual (si `get_robt=True`).

#### `load_scene(scene_filename, subfolder)`

Busca el archivo de escena en la misma carpeta que la rutina, importa `setup_scene` y la ejecuta.

- **Parámetros**:
    - **scene_filename** (`str`): Nombre del archivo de definición de la escena.
    - **subfolder** (`str`): Subcarpeta donde se encuentra el archivo.

## 3 MoveIt

#### `move_goal(target, tool, wobj, publish_tf, tf_frame_name, tf_reference, timeout)`

Mueve el Goal State (robot naranja) de MoveIt a un vector de variables articulares o robtarget determinado. Llama a `/apply_planning_scene` para setear `planning_scene.robot_state`. Devuelve `True` si el servicio respondió `success=True`.

- **Parámetros**:
    - **target** (`RobTarget` or `list`/`array`): Destino representado por un robtarget o vector de variables articulares.
    - **tool** (`SE3`): Herramienta para el robtarget. Se asume nula por defecto (brida).
    - **wobj** (`SE3`): Workobject para el robtarget. Se asume nulo por defecto (base del robot).
    - **publish_tf** (`bool`): Si es `True`, publica la terna del goal mediante TF.
    - **tf_frame_name** (`str`): Nombre del frame TF a publicar si `publish_tf=True`.
    - **tf_reference** (`str`): Nombre del frame de referencia para el TF publicado.
    - **timeout** (`float`): Tiempo máximo de espera por la respuesta del servicio.
- **Retorno**:
    - **bool**: `True` si el servicio respondió.

#### `plan_and_execute(q_goal, q_start, tool, wobj, execute, update_rviz, timeout)`

Orquesta la planificación y ejecución de movimiento.
1. Convierte inputs (RobTargets, listas) a vectores articulares.
2. Llama a MoveIt (`plan_joint_trajectory`).
3. Si `execute=True`, construye la trayectoria y la envía al ActionServer.
4. Actualiza RViz.

- **Parámetros**:
    - **q_goal** (`RobTarget`, `list`, `numpy array`): Destino representado por un robtarget o vector de variables articulares.
    - **start** (`RobTarget`, `list`, `numpy array`, optional): Pose de inicio expresada como robtarget o vector de variables articulares. Si es None, se usa el estado actual.
    - **tool** (`SE3`): Herramienta para el robtarget. Se asume nula por defecto (brida).
    - **wobj** (`SE3`): Workobject para el robtarget. Se asume nulo por defecto (base del robot).
    - **execute** (`bool`): `True` mueve el robot en RViz. `False` planifica la trayectoria sin ejecutarla.
    - **update_rviz** (`bool`): Mover al goal state a la pose final mediante `move_goal`.
    - **timeout** (`float`): Tiempo máximo de espera por la respuesta del servicio.
- **Retorno**:
    - **list**: Lista de waypoints (`q_list`) planificados, o `None` si falló.

#### `save_trajectory(filename, variable_name)`

Guarda la última trayectoria planificada en la carpeta 'Trayectorias' del proyecto actual.

- **Parámetros**:
    - **filename** (`str`): Nombre del archivo.
    - **variable_name** (`str`): Nombre de la variable.

#### `check_pose_configs(robt, tool, wobj, filtrar)`

Recorre todas las configuraciones con solución analítica del PCI, evalúa colisiones mediante MoveIt y muestra un menú interactivo.

- **Parámetros**:
    - **robt** (`RobTarget`): Pose a analizar.
    - **tool** (`SE3`): Herramienta aplicada.
    - **wobj** (`SE3`): Workobject aplicado.
    - **filtrar** (`bool`): `True` para filtrar colisiones. `False` para visualizar todas las configuraciones que satisfacen el PCD.

#### `toggle_ros_connection(self, enable)`

Activa o desactiva la comunicación de `ros2_control` con el robot. Permite visualzar movimientos del generador de trayectorias propio (`MoveJ`, `MoveC`) en el paquete de MoveIt.  
Equivalente a: 
```bash
ros2 control switch_controllers --activate/deactivate joint_state_broadcaster
```

- **Parámetros**:
    - **enable** (`bool`): 
    `False` libera el robot del control de MoveIt.
    `True` cede el control a MoveIt, rehabilitando MotionPlanning.


