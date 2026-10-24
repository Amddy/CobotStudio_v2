# encoding=utf-8
import os
import sys
# sys.path.append(os.getcwd()) # Generalmente no es necesario si ejecutas el script en su carpeta

import matplotlib.pyplot as plt
from pymycobot import MyCobot320

# --- Conexión con el Cobot ---
# NOTA: En Raspberry Pi, el puerto serial normalmente es "/dev/ttyAMA0" o "/dev/ttyS0".
# Asegúrate de usar el puerto correcto para tu dispositivo.
try:
    mc = MyCobot320("/dev/ttyAMA0", 115200)
except Exception as e:
    print(f"Error al conectar con el robot. Asegúrate de que el puerto es el correcto.")
    print(e)
    # Intenta con otro puerto común en Windows para depuración si es necesario, o simplemente sale.
    try:
        mc = MyCobot320("COM12", 115200)
    except Exception as e2:
        print("No se pudo conectar en /dev/ttyAMA0 ni en COM12. Saliendo.")
        print(e2)
        sys.exit(1)


# --- Configuración del Gráfico ---
NUM_SERVOS = 6  # myCobot 320 tiene 6 servos
servo_indices = list(range(1, NUM_SERVOS + 1))
initial_currents = [0] * NUM_SERVOS

# Crea la figura y los ejes para el gráfico
fig, ax = plt.subplots()

# Configura los límites y etiquetas del gráfico para mayor claridad
ax.set_ylim([0, 3500])  # Un poco de margen por si hay picos
ax.set_xlim([1, NUM_SERVOS])
ax.set_title("Corrientes de Servos en Tiempo Real")
ax.set_xlabel("Índice del Servo")
ax.set_ylabel("Valor de Corriente")
ax.set_xticks(servo_indices)
ax.grid(True)

# Crea la línea del gráfico. Los datos de Y se actualizarán en tiempo real.
line, = ax.plot(servo_indices, initial_currents, label='Corriente', color='cornflowerblue', marker='o')
ax.legend(loc='upper right')

# --- Función de Actualización ---
def update_plot(ax_object):
    """
    Esta función se llama periódicamente para obtener los nuevos datos
    y actualizar el gráfico.
    """
    # 1. Obtiene la lista de corrientes de los 6 servos
    currents = mc.get_servo_currents()
    
    # 2. Verifica que los datos recibidos son válidos (una lista con 6 elementos)
    if currents and len(currents) == NUM_SERVOS:
        # 3. Actualiza los datos del eje Y de la línea del gráfico (¡CORRECCIÓN PRINCIPAL!)
        line.set_ydata(currents)
        
        # 4. Solicita un redibujado del lienzo de forma segura (CORRECCIÓN DEL ATTRIBUTEERROR)
        ax_object.figure.canvas.draw_idle()

# --- Temporizador de la Animación ---
# Configura un temporizador que llama a la función `update_plot` cada 100 ms
timer = fig.canvas.new_timer(interval=100)
timer.add_callback(update_plot, ax)
timer.start()

# Muestra el gráfico
plt.show()