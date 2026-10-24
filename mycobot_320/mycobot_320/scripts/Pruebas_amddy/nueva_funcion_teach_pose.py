def teach_poses(self, cantidad, ajuste=False, q_list=None, tool=SE3(), spd_joy=6):
        """
        Implementación mejorada de `teach_poses` para el robot físico MyCobot320.

        Corrige:
          1. Liberación real sin damping (release_servo con mode=1 y pausa entre tramas RS485).
          2. Captura de ángulos ANTES de soltar el brazo y fijación activa de consigna para minimizar caída.
          3. Ajuste con joystick a baja velocidad (spd_joy) para evitar saltos bruscos.
        """
        def conteo_regresivo(segundos: int, mensaje: str):
            print(mensaje)
            for i in range(segundos, 0, -1):
                print(f" >> Tiempo restante: {i} s   ", end='\r', flush=True)
                time.sleep(1)
            print(" " * 50, end='\r')

        def leer_angulos_seguro(intentos=5):
            for _ in range(intentos):
                ang = self.mc.get_angles()
                if ang is not None and isinstance(ang, (list, tuple)) and len(ang) == 6:
                    return list(ang)
                time.sleep(0.05)
            raise RuntimeError("Error de lectura serial en get_angles().")

        def leer_coords_seguro(intentos=5):
            for _ in range(intentos):
                crd = self.mc.get_coords()
                if crd is not None and isinstance(crd, (list, tuple)) and len(crd) == 6:
                    return list(crd)
                time.sleep(0.05)
            return [0.0] * 6

        self.logger.info(f"Iniciando rutina de grabación de {cantidad} poses (Modo Ajuste: {ajuste})")

        # Sujeción del palpador
        print("\n=== Grabación de poses ===")
        print("La pinza se cerrará en 3 segundos para sujetar la pieza auxiliar (palpador).")
        conteo_regresivo(3, "Atención...")
        self.GripperState(100)
        time.sleep(5)
        self.GripperState(0)

        poses_q = [None] * cantidad
        poses_coord = [None] * cantidad

        for n in range(cantidad):
            print(f"\n" + "=" * 45)
            print(f" GRABACIÓN DE POSE {n+1} / {cantidad}")
            print("=" * 45)

            input(">> Sujete el brazo del robot con la mano.\nPresione [ENTER] para liberar los motores en 3 segundos...")
            conteo_regresivo(3, "Liberando motores (cancelando damping)...")

            # 1. SOLUCIÓN A MOTORES DUROS:
            # Pasar mode=1 cancela el damping electromagnético y la pausa de 0.08s evita perder paquetes RS485
            for eje in range(1, 7):
                try:
                    self.mc.release_servo(eje, 1)
                except TypeError:
                    self.mc.release_servo(eje)
                time.sleep(0.08)

            print("Motores completamente libres. Lleve la punta del palpador al punto deseado.")

            # En lugar de un temporizador ciego que te apura o te hace esperar de más,
            # puedes usar un tiempo base o confirmar cuando ya estés en posición:
            tiempo_ajuste = 20 if n == 0 else 12
            conteo_regresivo(tiempo_ajuste, f"Mantenga el palpador en el punto. Fijando en {tiempo_ajuste} s...")

            # 2. SOLUCIÓN A LA CAÍDA AL ACTIVAR MOTORES:
            # Leemos los encoders MIENTRAS el usuario aún sostiene el palpador en el punto exacto
            print("\n[!] NO SUELTE EL ROBOT AÚN: Leyendo posición exacta y energizando servos...")
            mediciones_pre_q = []
            mediciones_pre_coord = []
            for _ in range(5):
                mediciones_pre_q.append(leer_angulos_seguro())
                mediciones_pre_coord.append(leer_coords_seguro())
                time.sleep(0.05)

            q_sostenido = np.mean(np.array(mediciones_pre_q), axis=0).tolist()
            coord_sostenido = np.mean(np.array(mediciones_pre_coord), axis=0).tolist()

            # Energizamos cada eje con pausa serial y comandamos explícitamente mantener q_sostenido
            self.mc.focus_all_servos()
            time.sleep(0.1)
            for eje in range(1, 7):
                self.mc.focus_servo(eje)
                time.sleep(0.05)
            self.mc.send_angles(q_sostenido, 15)
            time.sleep(1.5)

            print(">>> Motores bloqueados. Ya puede soltar el robot suavemente. <<<")
            time.sleep(2.0)

            q_tras_soltar = leer_angulos_seguro()
            caida_deg = np.round(np.array(q_tras_soltar) - np.array(q_sostenido), 2)
            print(f"Ángulos en el punto (sostenido): {np.round(q_sostenido, 2).tolist()}")
            print(f"Ángulos tras soltar el brazo:    {np.round(q_tras_soltar, 2).tolist()}")
            print(f"Desviación por juego/gravedad (deg): {caida_deg.tolist()}")

            if ajuste:
                # 3. SOLUCIÓN AL PASO DEL JOYSTICK:
                # Partimos de q_tras_soltar y usamos una velocidad baja (spd_joy=6) pasando la herramienta
                print("\n--- Iniciando ajuste fino con Joystick (ESC para terminar) ---")
                pose_ajustada = self.joystick_adjust(
                    np.deg2rad(q_tras_soltar),
                    mover_callback=lambda r: self.MoveJ(r, spd_joy, tool, SE3())
                )

                time.sleep(1.0)
                q_ajustado = leer_angulos_seguro()
                coord_ajustado = leer_coords_seguro()
                print(f"Pose {n+1} guardada tras ajuste: {q_ajustado}")

            else:
                # Si no usamos joystick, guardamos q_sostenido (cuando el palpador estaba exactamente
                # en el punto real antes de sufrir la caída gravitatoria al soltarlo)
                q_ajustado = [round(val, 2) for val in q_sostenido]
                coord_ajustado = [round(val, 2) for val in coord_sostenido]
                print(f"Pose {n+1} guardada (medición exacta antes de caída): {q_ajustado}")

            poses_q[n] = q_ajustado
            poses_coord[n] = coord_ajustado

        self.logger.info("\nPoses grabadas:")
        for i, p in enumerate(poses_q):
            self.logger.info(f"Pose {i+1}: {p}")

        self.logger.info("\nCoords grabadas:")
        for i, d in enumerate(poses_coord):
            self.logger.info(f"Q-coords {i+1}: {d}")

        return poses_q, poses_coord