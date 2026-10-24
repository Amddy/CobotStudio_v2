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

    Nota:
        La API pymycobot no ofrece la posibilidad de informar el estado del gripper, menos aún su apertura exacta. Se obtiene una mímica de su
        comportamiento a través de un observer que lee los comandos enviados al gripper a través de la rutina en ejecución y luego los agrega
        a las variables articulares informadas por la función `get_angles`de la API.
    """
    def __init__(self, read_angles_callable, host: str = '0.0.0.0', port: int = 65432, send_rate_hz: float = 20.0):
        """
        Args:
            read_angles_callable: Función que devuelve lista de ángulos en grados.
            host (str): IP de escucha ('0.0.0.0' para todas las interfaces).
            port (int): Puerto TCP.
            send_rate_hz (float): frecuencia de envío por socket.
        """
        self.read_angles = read_angles_callable
        self.host = host
        self.port = port
        self.send_rate_hz = send_rate_hz

        # Último estado del robot
        self._latest = {"timestamp": None, "angles": None, "gripper": 0.0}
        self._gripper_val = 0.0 # Variable interna para actualizarse mediante el observer

        self._lock = threading.Lock()               # Proteger lectura y escritura
        self._shutdown = threading.Event()          # Detener hilos
        self._client_connected = threading.Event()  # Chequear si hay un listener
        self._pause_serial = threading.Event()      # Permite pausar get_angles() durante trayectorias continuas

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

                if angles is not None:
                    # Actualizar datos
                    with self._lock:
                        self._latest["timestamp"] = ts
                        self._latest["angles"] = list(map(float, angles))
                        self._latest["gripper"] = self._gripper_val

            except Exception as e:
                print(f"[CobotServer] error leyendo ángulos: {e}")
            
            time.sleep(poll_interval)

    def _handle_client(self, conn, addr):
        """Envía el estado actual al cliente conectado en formato JSON Lines."""

        print(f"[CobotServer] cliente conectado {addr}")
        self._client_connected.set()

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
                            "gripper": self._latest["gripper"]
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

