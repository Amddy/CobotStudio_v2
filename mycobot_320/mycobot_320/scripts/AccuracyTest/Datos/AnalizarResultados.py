#!/usr/bin/env python3
"""
robot_coords_batch.py

Procesamiento simple y modular para:
 - calcular distancias XY entre pares (1-3,1-5,3-7,5-7) por archivo (una fila)
 - extraer x,y,z de los puntos 2,4,6,8 por archivo (guardar solo x y z para esos puntos)
 - iterar sobre varios archivos y devolver:
     * DataFrame (n_files x n_pairs)
     * listas agregadas: all_x, all_y, all_z (contienen las coordenadas de 2,4,6,8 de todos los archivos)
"""

from pathlib import Path
import re
import ast
from typing import List, Tuple, Dict, Any
import numpy as np
import pandas as pd


def parse_coords_robot_from_text(text: str) -> List[List[float]]:
    """Extrae todas las listas coords_robot = [...] del texto (orden de aparición)."""
    pattern = re.compile(r"pose_alc_tb\s*=\s*(\[[^\]]*\])", flags=re.MULTILINE)
    matches = pattern.findall(text)
    coords = []
    for m in matches:
        try:
            vals = ast.literal_eval(m)
            vals = [float(v) for v in vals]
            if len(vals) >= 3:
                coords.append(vals)
        except Exception:
            # ignorar entradas malformadas
            continue
    return coords


def parse_coords_robot_from_file(file_path: Path) -> List[List[float]]:
    """Lee archivo y devuelve lista de coords_robot (cada elemento es lista con al menos [x,y,z,...])."""
    text = file_path.read_text(encoding="utf-8")
    return parse_coords_robot_from_text(text)


def distances_xy_single_row(coords_lists: List[List[float]],
                            pairs: List[Tuple[int, int]] = [(1,3),(1,5),(3,7),(5,7)]
                            ) -> pd.DataFrame:
    """
    Calcula distancias en XY entre pares proporcionados (índices 1-based).
    Devuelve un DataFrame con una sola fila y columnas nombradas 'A-B' por cada par.
    Si un par no existe (por falta de puntos), la celda contendrá np.nan.
    """
    results = {}
    n = len(coords_lists)
    for a, b in pairs:
        ia = a - 1
        ib = b - 1
        colname = f"{a}-{b}"
        if ia < 0 or ib < 0 or ia >= n or ib >= n:
            results[colname] = np.nan
            continue
        pa = coords_lists[ia]
        pb = coords_lists[ib]
        # tomar solo x,y (primeros 2 elementos)
        va = np.array(pa[:2], dtype=float)
        vb = np.array(pb[:2], dtype=float)
        dist = float(np.linalg.norm(va - vb))
        results[colname] = dist
    # devolver DataFrame con una fila
    return pd.DataFrame([results])


def extract_repeat_xyz(coords_lists: List[List[float]],
                       repeat_points: List[int] = [2,4,6,8]
                       ) -> Dict[str, List[float]]:
    """
    Extrae x,y,z de los puntos indicados (1-based) y devuelve diccionario con:
      {"x": [...], "y": [...], "z": [...]}
    Si un punto no existe, se ignora.
    NOTA: guardamos x y z; más adelante podés usar solo x/z si querés.
    """
    xs, ys, zs = [], [], []
    n = len(coords_lists)
    for p in repeat_points:
        idx = p - 1
        if 0 <= idx < n:
            vals = coords_lists[idx]
            xs.append(float(vals[0]))
            ys.append(float(vals[1]))
            zs.append(float(vals[2]))
    return {"x": xs, "y": ys, "z": zs}


def process_files(file_paths: List[Path],
                  pairs: List[Tuple[int, int]] = [(1,3),(1,5),(3,7),(5,7)],
                  repeat_points: List[int] = [2,4,6,8]
                  ) -> Tuple[pd.DataFrame, List[float], List[float], List[float]]:
    """
    Procesa varios archivos y devuelve:
      - df_pairs: DataFrame (n_files x n_pairs) con distancias XY (columnas '1-3','1-5',...)
      - all_x: lista con todas las x (puntos 2,4,6,8 de todos los archivos concatenados)
      - all_y: idem para y
      - all_z: idem para z
    """
    rows = []
    all_x, all_y, all_z = [], [], []

    for fp in file_paths:
        coords = parse_coords_robot_from_file(fp)
        # fila con distancias XY (una sola fila DataFrame)
        row_df = distances_xy_single_row(coords, pairs=pairs)
        # extraer vectores x,y,z de puntos repeat_points
        rep = extract_repeat_xyz(coords, repeat_points)
        # agregar a las listas agregadas
        all_x.extend(rep["x"])
        all_y.extend(rep["y"])
        all_z.extend(rep["z"])
        # queremos que cada fila tenga además una etiqueta de archivo
        row_df.insert(0, "file", fp.name)
        rows.append(row_df)

    if rows:
        df_pairs = pd.concat(rows, ignore_index=True)
    else:
        # columnas por defecto si no hay archivos
        cols = ["file"] + [f"{a}-{b}" for (a,b) in pairs]
        df_pairs = pd.DataFrame(columns=cols)

    return df_pairs, all_x, all_y, all_z


# ------------------------------
# Ejemplo de uso (main)
# ------------------------------
if __name__ == "__main__":
    # Si los archivos están en la misma carpeta que este script:
    BASE_DIR = Path(__file__).resolve().parent

    # Ajustá esta lista a los nombres reales de tus 4 archivos
    file_names = ["ensayo2_v0.txt", "ensayo2_v1.txt", "ensayo2_v2.txt", "ensayo2_v3.txt"]
    file_paths = [BASE_DIR / fn for fn in file_names]

    # Verificamos existencia y avisamos si falta alguno (pero procesamos los que existan)
    existing_files = []
    missing = []
    for fp in file_paths:
        if fp.exists():
            existing_files.append(fp)
        else:
            missing.append(fp.name)
    if missing:
        print("Advertencia: no se encontraron estos archivos (serán ignorados):", missing)
    if not existing_files:
        raise FileNotFoundError("No se encontraron archivos para procesar. Ajustá la lista file_names.")

    # Procesamos los archivos existentes
    df_pairs, all_x, all_y, all_z = process_files(existing_files)

    # Resultado: DataFrame con una fila por archivo y columnas por par
    pd.set_option("display.float_format", lambda x: f"{x:.6f}")
    print("\nDataFrame de distancias (XY) por archivo:")
    print(df_pairs.to_string(index=False))

    # Listas agregadas con coordenadas x,y,z de los puntos 2,4,6,8 de todos los archivos
    print(f"\nCantidad total de valores recogidos por eje (de puntos 2,4,6,8 de todos los archivos):")
    print(" len(all_x) =", len(all_x))
    print(" len(all_y) =", len(all_y))
    print(" len(all_z) =", len(all_z))

    # Ejemplo de inspección rápida
    print("\nPrimeros 20 valores de all_x (si existen):", all_x[:20])
    print("Primeros 20 valores de all_y (si existen):", all_y[:20])
    print("Primeros 20 valores de all_z (si existen):", all_z[:20])

    # Guardar resultados opcionalmente
    out_dir = BASE_DIR / "robot_batch_results"
    out_dir.mkdir(exist_ok=True)
    df_pairs.to_csv(out_dir / "pairs_by_file.csv", index=False)
    # guardar listas como csv simples
    pd.DataFrame({"x": all_x}).to_csv(out_dir / "all_x.csv", index=False)
    pd.DataFrame({"y": all_y}).to_csv(out_dir / "all_y.csv", index=False)
    pd.DataFrame({"z": all_z}).to_csv(out_dir / "all_z.csv", index=False)

    print(f"\nResultados guardados en: {out_dir}")
    from rpl_analysis import compute_repetibility_RPl

    results = compute_repetibility_RPl(all_x, all_y, all_z, n_trials=4, debug=True)
    print("RPl final:", results["RPl"])
