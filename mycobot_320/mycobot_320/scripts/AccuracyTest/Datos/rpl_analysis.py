#!/usr/bin/env python3
"""
rpl_analysis.py

Funciones para calcular l_js, l_bar, S_l y RPl según tu descripción (ISO-9283 workflow que
mencionaste, implementado exactamente como lo pediste).

Uso:
  results = compute_repetibility_RPl(all_x, all_y, all_z, n_trials=4, debug=True)

Devuelve un dict con:
  {
    "x_bar": ...,
    "y_bar": ...,
    "z_bar": ...,
    "l_js": np.array([...]),   # longitud m = len(all_x)
    "l_bar": ...,
    "S_l": ...,
    "RPl": ...
  }
"""
from typing import List, Dict, Any
import numpy as np


def compute_repetibility_RPl(all_x: List[float],
                             all_y: List[float],
                             all_z: List[float],
                             n_trials: int = 4,
                             debug: bool = False) -> Dict[str, Any]:
    """
    all_x, all_y, all_z: listas (o arrays) con las coordenadas concatenadas de todos los ensayos.
      - Deben tener la misma longitud m.
      - m debe ser igual a 4 * n_trials según tu descripción (4 puntos por ensayo).
    n_trials: número de ensayos (ej. 4).
    debug: si True imprime pasos intermedios (primeros 3 l_j, medias, sumas, etc).

    Retorna diccionario con x_bar,y_bar,z_bar,l_js (np.array), l_bar, S_l, RPl.
    """
    # Convertir a numpy arrays
    ax = np.asarray(all_x, dtype=float)
    ay = np.asarray(all_y, dtype=float)
    az = np.asarray(all_z, dtype=float)

    # Comprobaciones básicas
    if not (len(ax) == len(ay) == len(az)):
        raise ValueError("all_x, all_y y all_z deben tener la misma longitud.")
    m = len(ax)
    expected_m = 4 * n_trials
    if m != expected_m:
        # Aviso pero no aborta — lo dejo como advertencia (puedes cambiar a raise si preferís)
        print(f"Advertencia: longitud m={m} distinta a 4*n_trials={expected_m}. "
              "El cálculo continuará pero verifica que esto sea intencional.")

    # 1) medias (x_bar, y_bar, z_bar) sobre todas las entradas
    x_bar = float(ax.mean())
    y_bar = float(ay.mean())
    z_bar = float(az.mean())

    # 2) calcular l_js: euclídea de cada punto al (x_bar,y_bar,z_bar)
    diffs_sq = (ax - x_bar)**2 + (ay - y_bar)**2 + (az - z_bar)**2
    l_js = np.sqrt(diffs_sq)  # array shape (m,)

    # 3) l_bar = sum(l_js) / n_trials  (tal como pediste, n_trials es 4)
    sum_ljs = float(l_js.sum())
    l_bar = sum_ljs / float(n_trials)

    # 4) S_l = sqrt( sum_j (l_j - l_bar)^2 / (n_trials - 1) )
    #    Note: denominador usa n_trials-1 según tu especificación
    if n_trials - 1 <= 0:
        raise ValueError("n_trials debe ser mayor que 1 para calcular S_l (denominador n-1).")
    var_num = float(np.sum((l_js - l_bar)**2))
    S_l = np.sqrt(var_num / float(n_trials - 1))

    # 5) RPl = l_bar + 3 * S_l
    RPl = l_bar + 3.0 * S_l

    # Debug prints: mostrar pasos y primeros 3 l_j para verificar manualmente
    if debug:
        print("---- DEBUG RPl calculation ----")
        print(f"m (total points) = {m}")
        print(f"n_trials = {n_trials} (se usa para dividir sum(l_js) y en denominador de S_l)")
        print(f"x_bar = {x_bar:.6f}, y_bar = {y_bar:.6f}, z_bar = {z_bar:.6f}")
        print(f"sum(l_js) = {sum_ljs:.6f}")
        print(f"l_bar = sum(l_js)/n_trials = {l_bar:.6f}")
        print(f"Primeros 3 l_j (o menos si no hay):")
        for i in range(min(3, m)):
            print(f"  l_{i+1} = {l_js[i]:.6f}  (calc: sqrt((x-{x_bar:.6f})^2 + (y-{y_bar:.6f})^2 + (z-{z_bar:.6f})^2))")
        print(f"Sum_j (l_j - l_bar)^2 = {var_num:.6f}")
        print(f"S_l = sqrt( sum(...) / (n_trials-1) ) = {S_l:.6f}")
        print(f"RPl = l_bar + 3*S_l = {RPl:.6f}")
        print("---- end debug ----")

    return {
        "x_bar": x_bar,
        "y_bar": y_bar,
        "z_bar": z_bar,
        "l_js": l_js,
        "l_bar": l_bar,
        "S_l": S_l,
        "RPl": RPl
    }


# -------------------------
# Ejemplo de uso rápido:
# -------------------------
if __name__ == "__main__":
    # Ejemplo trivial (sustituí por las listas que obtuviste del procesamiento por archivos)
    # Supongamos all_x/all_y/all_z ya concatenadas (16 valores cada una)
    # A modo de ejemplo te pongo arrays aleatorios; en uso real reemplazalos con tus listas.
    example_all_x = [0.75, -0.5, 0.9, 0.1] * 4  # 16 valores (ejemplo)
    example_all_y = [-231.7, -231.7, -231.1, -231.1] * 4
    example_all_z = [215.0, 215.0, 215.7, 215.7] * 4

    res = compute_repetibility_RPl(example_all_x, example_all_y, example_all_z,
                                   n_trials=4, debug=True)

    # Acceso a resultados:
    print("\nResultado final:")
    print(f"RPl = {res['RPl']:.6f}")
