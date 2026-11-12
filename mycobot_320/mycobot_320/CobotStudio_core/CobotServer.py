import socket
import threading
import time
import json
import numpy as np

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

        # Último estado del robot (traj_status para usar con action server)
        self._latest = {"timestamp": None, "angles": None, "gripper": 0.0, "traj_status": "IDLE", "traj_metrics": None}
        self._gripper_val = 0.0  # Variable interna para actualizarse mediante el observer
        self._current_acc_scale = None # Guarda la última escala de aceleración cargada en SRAM (0x29)

        # IMPORTANTE: Inicializar todos los primitivos de sincronización ANTES de tocar el hardware
        self._lock = threading.Lock()               # Proteger lectura y escritura
        self._shutdown = threading.Event()          # Detener hilos
        self._client_connected = threading.Event()  # Chequear si hay un listener
        self._pause_serial = threading.Event()      # Permite pausar get_angles() durante trayectorias continuas
        self._cancel_traj = threading.Event()       # Parada de emergencia por MoveIt

        self._sock_thread = None
        self._reader_thread = None

        # Configurar rampas de aceleración en SRAM
        if self.mc is not None:
            self._init_servo_ram_dynamics(acc_scale=0.30)

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


    def _init_servo_ram_dynamics(self, acc_scale: float = 0.30, force: bool = False):
        """
        Configura en SRAM (Registro 0x29 = 41) las rampas de aceleración interna de cada
        motor Feetech escaladas al régimen de trabajo de la trayectoria actual.
        Solo escribe en el bus serie si el factor cambió en más de 0.05 respecto al actual.
        """
        if self.mc is None:
            return

        import numpy as np
        scale = float(np.clip(acc_scale, 0.10, 1.00))

        # Evitar re-escribir en el puerto serie si ya está cargada la misma escala
        if not force and self._current_acc_scale is not None:
            if abs(scale - self._current_acc_scale) < 0.05:
                return

        REG_ACCEL_SRAM = 41  # 0x29: 1 unidad = 100 steps/s^2 = 0.1534 rad/s^2
        STEP_RAD_S2 = 0.1534
        MARGIN = 1.25        # +25% sobre la aceleración de MoveIt para evitar lag

        # Aceleraciones máximas al 100% definidas en joint_limits.yaml (rad/s^2)
        max_acc_yaml = {1: 3.00, 2: 2.50, 3: 3.00, 4: 3.50, 5: 4.00, 6: 4.00}

        ram_accels = {}
        for sid, a_max in max_acc_yaml.items():
            a_work = a_max * scale * MARGIN
            reg_val = int(np.clip(round(a_work / STEP_RAD_S2), 6, 40))
            ram_accels[sid] = reg_val

        try:
            self.pause_serial_reading(True)
            time.sleep(0.03)
            for servo_id, acc_val in ram_accels.items():
                self.mc.set_servo_data(servo_id, REG_ACCEL_SRAM, acc_val)
                time.sleep(0.02)
            self._current_acc_scale = scale
            print(f"[CobotServer] Rampas 0x29 en SRAM ajustadas (acc_scale={scale:.2f}): {ram_accels}")
        except Exception as e:
            print(f"[CobotServer] Advertencia: No se pudo actualizar 0x29 en SRAM: {e}")
        finally:
            self.pause_serial_reading(False)
    
    def _execute_gripper(self, apertura: float, spd: int = 30):
        """
        Ejecuta el movimiento de la pinza física, lee la apertura real alcanzada mediante
        get_gripper_value() y detecta si se sujetó un objeto físico entre los dedos.
        """
        if self.mc is None:
            print("[CobotServer] Error: Instancia 'mc' no disponible para mover el gripper.")
            return

        apertura_int = int(max(0, min(100, round(apertura))))
        spd_int = int(max(1, min(100, round(spd))))
        print(f"[CobotServer] Ejecutando Gripper físico -> Comando: {apertura_int}% | Velocidad: {spd_int}")

        # Pausar lectura periódica de get_angles() para acceso exclusivo al bus RS-485
        self.pause_serial_reading(True)
        time.sleep(0.05)

        try:
            self.mc.set_gripper_mode(0)
            if apertura_int == 0:
                self.mc.set_gripper_state(1, spd_int)
            elif apertura_int == 100:
                self.mc.set_gripper_state(0, spd_int)
            else:
                self.mc.set_gripper_value(apertura_int, spd_int, 1)

            # Esperar a que la pinza complete el recorrido mecánico antes de consultar su encoder
            time.sleep(0.45)

            real_aperture = float(apertura_int)
            if hasattr(self.mc, "get_gripper_value"):
                val_leido = self.mc.get_gripper_value()
                if val_leido is not None and 0 <= val_leido <= 100:
                    real_aperture = float(val_leido)

            # Detección de agarre: se pidió cerrar (<= 10%) pero los dedos frenaron antes (>= 12%)
            if apertura_int <= 10 and real_aperture >= 12.0:
                print(
                    f"[CobotServer] ¡AGARRE DETECTADO! Comando: {apertura_int}% -> "
                    f"Pinza retenida en {real_aperture:.0f}% por objeto físico."
                )
            else:
                print(f"[CobotServer] Gripper estabilizado en: {real_aperture:.0f}% (Comando: {apertura_int}%)")

            # Actualizar el observer con la apertura física real para que RViz muestre la posición exacta
            self.set_gripper_state(real_aperture)
        except Exception as e:
            print(f"[CobotServer] Error ejecutando comando de gripper: {e}")
        finally:
            self.pause_serial_reading(False)


    def _serial_reader_loop(self):
        """
        Ciclo que consulta al robot y actualiza la memoria.
        Corre a una frecuencia mayor que el envío para satisfacer el muestreo.
        """
        # Lee el robot periódicamente a su frecuencia de envío
        poll_interval = 1.0 / max(1.0, self.send_rate_hz)

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


    def _execute_trajectory(self, points: list, acc_scale: float = None):
        """
        Ejecuta una trayectoria enviada desde el ActionServer de MoveIt (CobotClient),
        adapta las rampas 0x29 en SRAM si cambió la escala de aceleración y reporta
        las métricas cuantitativas de tiempo y error articular final.
        """
        
        if self.mc is None:
            print("[CobotServer] Error: Instancia 'mc' no disponible para ejecutar trayectoria.")
            with self._lock:
                self._latest["traj_status"] = "ABORTED"
            return

        if not points:
            with self._lock:
                self._latest["traj_status"] = "ABORTED"
            return

        # 1. Ajustar rampas 0x29 en SRAM si CobotClient informó una nueva escala
        if acc_scale is not None:
            self._init_servo_ram_dynamics(acc_scale=float(acc_scale))

        print(f"[CobotServer] ¡Trayectoria recibida de MoveIt ({len(points)} waypoints)! Empezando movimiento...")
        self._cancel_traj.clear()

        # Pausar lectura periódica de get_angles() y esperar que termine cualquier lectura en vuelo
        self.pause_serial_reading(True)
        time.sleep(0.05)
        with self._lock:
            self._latest["traj_status"] = "EXECUTING"
            self._latest["traj_metrics"] = None

        try:
            self.mc.set_fresh_mode(1)  # Modo continuo sin frenado intermedio
            time.sleep(0.04)

            start_time = time.perf_counter()

            for pt in points:
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

                wait_time = (start_time + t_sec) - time.perf_counter()
                if wait_time > 0:
                    time.sleep(wait_time)

                self.mc.send_angles(angles_deg, spd)
                self.set_external_angles(angles_deg)

            # Medir duración real del bucle antes de las pausas de cierre
            t_real_bucle = time.perf_counter() - start_time
            t_planificado = float(points[-1][7]) if len(points[-1]) >= 8 else float(points[-1][6])

            # Dejar asentar suavemente sin re-disparar send_angles en modo 0
            last_angles_deg = [float(x) for x in points[-1][:6]]
            time.sleep(1)
            self.mc.set_fresh_mode(0)
            time.sleep(0.05)

            # 2. Leer encoders reales tras el asentamiento y calcular métricas Online
            q_real_deg = None
            for _ in range(3):
                lectura = self.read_angles()
                if lectura is not None and len(lectura) == 6:
                    q_real_deg = lectura
                    break
                time.sleep(0.03)

            metrics = {
                "t_real": round(t_real_bucle, 3),
                "t_plan": round(t_planificado, 3),
                "t_diff": round(t_real_bucle - t_planificado, 3),
                "max_err_deg": None,
                "err_per_joint_deg": None
            }

            if q_real_deg is not None and len(q_real_deg) == 6:
                self.set_external_angles(q_real_deg)
                err_vec = np.round(np.abs(np.array(last_angles_deg) - np.array(q_real_deg, dtype=float)), 2)
                max_err = float(np.max(err_vec))
                metrics["max_err_deg"] = round(max_err, 2)
                metrics["err_per_joint_deg"] = err_vec.tolist()
                print(
                    f"[CobotServer] [Métricas Online] Tiempo: {t_real_bucle:.2f}s "
                    f"(Plan: {t_planificado:.2f}s, Diff: {t_real_bucle - t_planificado:+.3f}s) | "
                    f"Max Err Articular: {max_err:.2f}° | Err/Eje: {err_vec.tolist()}"
                )
            else:
                print("[CobotServer] Movimiento completado con éxito (sin lectura final de encoders).")

            with self._lock:
                self._latest["traj_metrics"] = metrics
                self._latest["traj_status"] = "SUCCEEDED"

        except Exception as e:
            print(f"[CobotServer] Error ejecutando trayectoria: {e}")
            with self._lock:
                self._latest["traj_status"] = "ABORTED"
        finally:
            try:
                self.mc.set_fresh_mode(0)
            except Exception:
                pass
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
                            acc_scale = msg.get("acc_scale", None)
                            threading.Thread(
                                target=self._execute_trajectory,
                                args=(points, acc_scale),
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

        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
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
                            "traj_status": self._latest["traj_status"],
                            "traj_metrics": self._latest.get("traj_metrics")
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