import socket
import threading
import time
import json

class CobotServer:
    """
    Server que publica por TCP la última lectura de ángulos del robot en tiempo real.

    El robot físico no permite múltiples conexiones a través del puerto serie. Si la clase `MyCobotController` lo ocupa para enviar comandos, 
    la lectura de variables articulares mediante otro proceso queda inhabilitada. Esta clase se ejecuta en el mismo proceso que el controlador:
    lee los ángulos usando la conexión serie ya abierta (vía callback) y los retransmite por TCP (JSON Lines) a un cliente con ROS para publicarlos.

    Agregado de funcion para recibir las trayectorias de un action server de una pc externa y  ejecutarlas en pymycobot

    Nota:
        La API pymycobot no ofrece la posibilidad de informar el estado del gripper, menos aún su apertura exacta. Se obtiene una mímica de su
        comportamiento a través de un observer que lee los comandos enviados al gripper a través de la rutina en ejecución y luego los agrega
        a las variables articulares informadas por la función `get_angles`de la API.
    """
    def __init__(self, read_angles_callable, mc=None, host: str = '0.0.0.0', port: int = 65432, send_rate_hz: float = 20.0):
        """
        Args:
            read_angles_callable: Función que devuelve lista de ángulos en grados.
            mc: Instancia de MyCobot320 para ejecutar trayectorias recibidas desde el ActionServer.
            host (str): IP de escucha ('0.0.0.0' para todas las interfaces).
            port (int): Puerto TCP.
            send_rate_hz (float): frecuencia de envío por socket.
        """
        self.read_angles = read_angles_callable
        self.mc = mc
        self.host = host
        self.port = port
        self.send_rate_hz = send_rate_hz

        # Último estado del robot (traj_status para usar en con action server)
        self._latest = {"timestamp": None, "angles": None, "gripper": 0.0, "traj_status": "IDLE"}
        self._gripper_val = 0.0 # Variable interna para actualizarse mediante el observer

        self._lock = threading.Lock()               # Proteger lectura y escritura
        self._shutdown = threading.Event()          # Detener hilos
        self._client_connected = threading.Event()  # Chequear si hay un listener
        self._pause_serial = threading.Event()      # Permite pausar get_angles() durante trayectorias continuas
        self._cancel_traj = threading.Event()       # Parada de emergia por moveit

        self._sock_thread = None
        self._reader_thread = None

    def start(self):
        """Inicia los hilos de lectura de hardware y escucha de red."""
        # Leer hardware
        self._reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self._reader_thread.start()

        # Iniciar el listener
        self._sock_thread = threading.Thread(target=self._socket_server_loop, daemon=True)
        self._sock_thread.start()

    def stop(self):
        """Detiene el servidor y cierra conexiones de forma limpia."""
        self._shutdown.set()
        self._cancel_traj.set()

    def wait_for_connection(self, timeout=None):
        """Bloquea la ejecución hasta que un cliente de ROS se conecte. Evita un inicio prematuro de la rutina."""
        print(f"[CobotServer] Esperando conexión de cliente ROS2 en {self.host}:{self.port}...")
        connected = self._client_connected.wait(timeout=timeout)

        if connected:
            print("[CobotServer] ¡Cliente conectado! Iniciando secuencia...")
        else:
            print("[CobotServer] Timeout esperando cliente (o se continuó sin espera).")
        return connected
    
    def set_gripper_state(self, value: float):
        """Recibe un valor para el gripper y lo guarda para enviar. Es llamado por la función que controla al gripper físico."""
        with self._lock:
            self._gripper_val = float(value)

    def set_external_angles(self, angles_deg: list):
        """Actualiza los ángulos en memoria sin interrogar el puerto serie (usado en traj_moveit)."""
        with self._lock:
            self._latest["timestamp"] = time.time()
            self._latest["angles"] = list(map(float, angles_deg))
            self._latest["gripper"] = self._gripper_val

    def pause_serial_reading(self, pause: bool):
        """Pausa o reanuda la consulta periódica a get_angles() para liberar el bus serie."""
        if pause:
            self._pause_serial.set()
        else:
            self._pause_serial.clear()

    def _execute_gripper(self, apertura: float, spd: int = 30):
        """
        Ejecuta el movimiento de la pinza física mediante la API pymycobot y actualiza el observer.

        Pausa temporalmente el hilo de lectura periódica del puerto serie para evitar colisiones
        de tramas en el bus RS-485 half-duplex mientras se envía la instrucción al gripper.

        Args:
            apertura (float): Porcentaje de apertura deseado (0.0 cerrado a 100.0 abierto).
            spd (int, optional): Velocidad de movimiento de la pinza (1 a 100). Defaults to 30.
        """
        if self.mc is None:
            print("[CobotServer] Error: Instancia 'mc' no disponible para mover el gripper.")
            return

        apertura_int = int(max(0, min(100, round(apertura))))
        spd_int = int(max(1, min(100, round(spd))))
        print(f"[CobotServer] Ejecutando Gripper físico -> Apertura: {apertura_int}% | Velocidad: {spd_int}")

        # Pausar lectura periódica de get_angles() para no saturar ni bloquear el puerto serie
        self.pause_serial_reading(True)
        try:
            self.mc.set_gripper_mode(0)
            if apertura_int == 0:
                self.mc.set_gripper_state(1, spd_int)
            elif apertura_int == 100:
                self.mc.set_gripper_state(0, spd_int)
            else:
                self.mc.set_gripper_value(apertura_int, spd_int, 1)

            # Actualizar el estado interno para que se retransmita en el JSON hacia RViz
            self.set_gripper_state(float(apertura_int))
            time.sleep(0.15)
        except Exception as e:
            print(f"[CobotServer] Error ejecutando comando de gripper: {e}")
        finally:
            self.pause_serial_reading(False)

    def _serial_reader_loop(self):
        """
        Ciclo que consulta al robot y actualiza la memoria.
        Corre a una frecuencia mayor que el envío para satisfacer el muestreo.
        """
        # Lee el robot periódicamente al doble de su frecuencia de envío
        poll_interval = 1.0 / max(1.0, self.send_rate_hz * 2.0)

        while not self._shutdown.is_set():
            if self._pause_serial.is_set():
                time.sleep(poll_interval)
                continue
            try:
                # Leer ángulos con la API
                angles = self.read_angles()
                ts = time.time()

                if angles is not None and len(angles) == 6:
                    # Actualizar datos
                    with self._lock:
                        self._latest["timestamp"] = ts
                        self._latest["angles"] = list(map(float, angles))
                        self._latest["gripper"] = self._gripper_val

            except Exception as e:
                print(f"[CobotServer] error leyendo ángulos: {e}")
            
            time.sleep(poll_interval)

    def _execute_trajectory(self, points: list):
        """
        Esta función se activa cuando el ActionServer de la PC manda una trayectoria de MoveIt.
        Soporta formato con velocidad dinámica: [[q1..q6, spd, t_sec], ...]
        y formato simple: [[q1..q6, t_sec], ...]
        """
        if self.mc is None:
            print("[CobotServer] Error: Instancia 'mc' no disponible para ejecutar trayectoria.")
            with self._lock:
                self._latest["traj_status"] = "ABORTED"
            return

        print(f"[CobotServer] ¡Trayectoria recibida de MoveIt ({len(points)} waypoints)! Empezando movimiento...")
        self._cancel_traj.clear()

        # Pausar lectura periódica de get_angles() para no saturar ni bloquear el puerto serie
        self.pause_serial_reading(True)
        with self._lock:
            self._latest["traj_status"] = "EXECUTING"

        try:
            # PREPARAR EL ROBOT PARA MOVIMIENTO CONTINUO
            self.mc.set_fresh_mode(1) # Ignorar frenado entre puntos
            time.sleep(0.04)

            # Reloj monotónico de alta precisión
            start_time = time.perf_counter()

            for pt in points:
                # Si MoveIt solicita cancelar la acción (parada de emergencia)
                if self._cancel_traj.is_set() or self._shutdown.is_set():
                    print("[CobotServer] ¡PARADA DE EMERGENCIA solicitada por MoveIt!")
                    self.mc.stop()
                    self.mc.set_fresh_mode(0)
                    with self._lock:
                        self._latest["traj_status"] = "CANCELED"
                    return

                angles_deg = [float(x) for x in pt[:6]]
                if len(pt) >= 8:
                    spd = int(pt[6])
                    t_sec = float(pt[7])
                else:
                    spd = 30
                    t_sec = float(pt[6])

                # 1. BUCLE DE SINCRONIZACIÓN (Esperar el instante exacto del punto)
                wait_time = (start_time + t_sec) - time.perf_counter()
                if wait_time > 0:
                    time.sleep(wait_time)

                # 2. Enviar comando con la velocidad dinámica correspondiente a este tramo
                self.mc.send_angles(angles_deg, spd)

                # 3. Actualizar en memoria para que RViz en la PC vea el avance en tiempo real
                self.set_external_angles(angles_deg)

            # FINALIZAR MOVIMIENTO Y AJUSTE FINO DEL ÚLTIMO PUNTO
            time.sleep(0.05)
            self.mc.set_fresh_mode(0) # Volver al comportamiento normal
            time.sleep(0.05)
            last_angles_deg = [float(x) for x in points[-1][:6]]
            self.mc.send_angles(last_angles_deg, 10)
            time.sleep(0.25)

            with self._lock:
                self._latest["traj_status"] = "SUCCEEDED"
            print("[CobotServer] Movimiento completado con éxito.")

        except Exception as e:
            print(f"[CobotServer] Error ejecutando trayectoria: {e}")
            with self._lock:
                self._latest["traj_status"] = "ABORTED"
        finally:
            # Reanudar lectura de encoders reales al finalizar
            self.pause_serial_reading(False)


    def _receive_commands_from_client(self, conn, addr):
        """Lee comandos JSON enviados desde CobotClient (Action Server en la PC)."""
        buffer = ""
        while not self._shutdown.is_set():
            try:
                data = conn.recv(65536)
                if not data:
                    break
                buffer += data.decode('utf-8', errors='replace')

                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                        cmd = msg.get("cmd")

                        if cmd == "EXECUTE_TRAJ":
                            points = msg.get("points", [])
                            threading.Thread(
                                target=self._execute_trajectory,
                                args=(points,),
                                daemon=True
                            ).start()
                        elif cmd == "CANCEL_TRAJ":
                            self._cancel_traj.set()
                        elif cmd == "RESET_STATUS":
                            with self._lock:
                                self._latest["traj_status"] = "IDLE"
                        elif cmd == "SET_GRIPPER":
                            val = float(msg.get("value", 0.0))
                            spd = int(msg.get("speed", 30))
                            threading.Thread(
                                target=self._execute_gripper,
                                args=(val, spd),
                                daemon=True
                            ).start()
                    except json.JSONDecodeError:
                        pass
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[CobotServer] Fin de recepción de comandos desde {addr}: {e}")
                break

    def _handle_client(self, conn, addr):
        """Envía el estado actual al cliente conectado en formato JSON Lines."""

        print(f"[CobotServer] cliente conectado {addr}")
        self._client_connected.set()

        # Iniciar hilo receptor de comandos enviados desde el ActionServer de la PC
        recv_thread = threading.Thread(target=self._receive_commands_from_client, args=(conn, addr), daemon=True)
        recv_thread.start()

        period = 1.0 / self.send_rate_hz
        
        try:
            while not self._shutdown.is_set():
                start_t = time.time()
                
                # Obtener datos
                with self._lock:
                    # Solo hay envío si se tienen datos válidos
                    if self._latest["angles"] is None:
                        payload = None
                    else:
                        payload = {
                            "timestamp": self._latest["timestamp"],
                            "angles": self._latest["angles"],
                            "gripper": self._latest["gripper"],
                            "traj_status": self._latest["traj_status"]
                        }

                # Enviar
                if payload:
                    try:
                        # encode('utf-8') + b'\n' es el protocolo "Line Delimited JSON"
                        message = json.dumps(payload, allow_nan=False).encode('utf-8') + b'\n'
                        conn.sendall(message)
                    except (BrokenPipeError, ConnectionResetError):
                        print(f"[CobotServer] Cliente {addr} desconectado.")
                        break
                    except Exception as e:
                        print(f"[CobotServer] Error de transmisión: {e}")
                        break
                
                # Mantener frecuencia constante
                elapsed = time.time() - start_t
                sleep_time = max(0.0, period - elapsed)
                time.sleep(sleep_time)
        
        finally:
            self._client_connected.clear()
            try:
                conn.close()
            except:
                pass

    def _socket_server_loop(self):
        """Espera conexiones entrantes."""

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            try:
                s.bind((self.host, self.port))
                s.listen(1)
                s.settimeout(1.0)
            except Exception as e:
                print(f"[CobotServer] Error fatal al iniciar socket: {e}")
                return

            print(f"[CobotServer] escuchando en {self.host}:{self.port}")

            while not self._shutdown.is_set():
                try:
                    conn, addr = s.accept()
                    # Delegar la conexión a un hilo dedicado
                    t = threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True)
                    t.start()
                except socket.timeout:
                    continue

    def get_latest(self):
        """Helper público para acceder a la última lectura desde otro hilo."""
        with self._lock:
            return dict(self._latest)