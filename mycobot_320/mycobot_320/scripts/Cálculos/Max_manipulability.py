import sys
import os
import numpy as np
from scipy.optimize import minimize
from scipy.optimize import differential_evolution
import time

# Ruta de la carpeta donde está este script
current_dir = os.path.dirname(os.path.abspath(__file__))
# Ruta de la carpeta donde están los imports
parent_dir = os.path.dirname(current_dir)
# Agregamos la carpeta con los imports al sistema de búsqueda de Python
sys.path.append(parent_dir)

from DHRobotGT import myCobot320

cobot_tb = myCobot320(rotar_base=True)

def calcular_w_maximo():
    
    # Función objetivo: Negativo de la manipulabilidad (para minimizar)
    def objetivo_opt(q):
        # El optimizador puede probar valores fuera de límites, 
        # así que a veces conviene clipear o dejar que los bounds actúen.
        return -1.0 * cobot_tb.manipulability(q)

    # Límites de las articulaciones (bounds)
    # cobot_tb.qlim es usualmente una matriz de 2xN. 
    # Scipy necesita una lista de tuplas [(min, max), ...]
    bounds = [(cobot_tb.qlim[0, i], cobot_tb.qlim[1, i]) for i in range(6)]

    # Semilla inicial (Guess):
    # La máxima manipulabilidad suele estar en una pose tipo "L" o arco,
    # no en "todo ceros" (que suele ser singularidad).
    # Probamos una pose "tipo candelabro" o codo a 90 grados.
    q_seed = np.array([0, 0, np.pi/2, 0, -np.pi/2, 0]) 

    # Optimización
    res = minimize(
        objetivo_opt, 
        q_seed, 
        method='L-BFGS-B', # Bueno para problemas con límites (bounds)
        bounds=bounds
    )

    if res.success:
        w_max = -res.fun # Invertimos el signo de nuevo
        q_max = res.x
        print(f"✅ W_max encontrado: {w_max:.2e}")
        print(f"   En la config (rads): {np.round(q_max, 2)}")
        return w_max
    else:
        print("Fallo en la optimización, usando valor por defecto.")
        return 3.5e6 # Valor fallback conservador para mm

# --- CORRECCIÓN: Definir la función de costo AQUÍ AFUERA (Nivel Global) ---
# Al estar afuera, Python sí puede "pickelarla" para enviarla a los workers.
def funcion_costo(q):
    return -1.0 * cobot_tb.manipulability(q)

def encontrar_w_absoluto():
    print("🤖 Iniciando búsqueda GLOBAL de Manipulabilidad Máxima...")
    print("🐢 Ejecutando en MODO SEGURO (Single Core) para evitar errores de memoria...")
    print("☕ Esto tomará unos 2 o 3 minutos.")

    # 2. Definir los límites
    bounds = [(cobot_tb.qlim[0, i], cobot_tb.qlim[1, i]) for i in range(6)]

    inicio = time.time()
    
    # 3. Ejecutar Evolución Diferencial (Sin workers=-1)
    result = differential_evolution(
        funcion_costo, 
        bounds, 
        strategy='best1bin', 
        maxiter=1000,
        popsize=30,       # Población densa para buena búsqueda
        tol=1e-4,
        disp=True         # Verás el progreso paso a paso
        # workers=-1      <--- ELIMINADO: Causa del error PyCapsule
    )
    
    fin = time.time()
    duracion = fin - inicio

    w_max_absoluto = -result.fun
    q_optimo = result.x

    print("\n" + "="*40)
    print(f"✅ BÚSQUEDA FINALIZADA en {duracion:.2f} segundos")
    print("="*40)
    print(f"🏆 Manipulabilidad Máxima (W_max): {w_max_absoluto:.4e}")
    print("-" * 20)
    print("Pose Articular Óptima (radianes):")
    print(np.array2string(q_optimo, precision=4, separator=', '))
    print("-" * 20)
    print("Grados aproximados:")
    print(np.array2string(np.degrees(q_optimo), precision=1, separator=', '))
    
    return w_max_absoluto

if __name__ == "__main__":
    calcular_w_maximo()
    encontrar_w_absoluto()