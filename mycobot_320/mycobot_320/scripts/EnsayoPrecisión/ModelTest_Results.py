import pandas as pd
import numpy as np
import os
import re
from spatialmath import SE3, SO3, UnitQuaternion
from scripts.DHRobotGT import myCobot320

# --- Funciones de Preparación y Exportación (sin cambios) ---

def preparar_df_completo(df):
    """Procesa un DataFrame completo, convirtiendo las columnas de string a array."""
    df_procesado = df.copy()
    columns_to_convert = ['q_esp', 'q_alc', 'err_q', 'pose_esp', 'pose_alc', 'err_pose', 'tb_check']
    for col in columns_to_convert:
        if col in df_procesado.columns:
            df_procesado[col] = df_procesado[col].apply(lambda s: np.fromstring(s.strip('[]'), sep=' ') if isinstance(s, str) else s)
    return df_procesado

def preparar_df_reporte_errores(df_completo):
    """
    Toma un DataFrame consolidado y devuelve un DataFrame para el reporte de errores,
    incluyendo el cálculo de la diferencia de rotación (DeltaRot).
    """
    columnas_reporte = ['punto_n', 'version', 'q_esp', 'pose_esp', 'pose_alc', 'err_q', 'err_pose', 'tb_check', 'error_rot_fro']
    df_reporte = df_completo[columnas_reporte].copy()

    def string_to_array(s):
        if not isinstance(s, str):
            return s  # Si ya es un array, lo devuelve tal cual
        try:
            # Convierte el string separado por espacios en un array de NumPy
            return np.fromstring(s, sep=' ')
        except (ValueError, TypeError):
            # Si el string está vacío o malformado, devuelve None para poder filtrarlo
            return None
        
    def pose_to_matrix(pose):
        """Convierte pose dada como lista [x, y, z, rx, ry, rz] en una matriz homogénea SE3."""
        x, y, z, rx, ry, rz = pose
        rx, ry, rz = np.deg2rad([rx, ry, rz])
        T_se3 = SE3(x, y, z) * SE3.RPY([rx, ry, rz], order='zyx')
        return T_se3
    
    # Nos aseguramos de que las columnas de pose sean np.array para los cálculos
    for col in ['q_esp', 'pose_esp', 'pose_alc', 'err_pose', 'err_q', 'tb_check']:
        if col in df_reporte.columns:
            df_reporte[col] = df_reporte[col].apply(string_to_array)
    # Función interna para calcular la diferencia de rotación (DeltaRot)
    def calcular_delta_rot(row):
        # Asegurarnos de que tenemos arrays válidos para trabajar
        if not (isinstance(row['pose_esp'], np.ndarray) and isinstance(row['pose_alc'], np.ndarray)):
            return np.nan
        try:
            # Convertir las poses xyzrpy a matrices SE3
            T_esp = pose_to_matrix(row['pose_esp'])
            T_alc = pose_to_matrix(row['pose_alc'])

            # Extraer las matrices de rotación
            R_esp = SO3(T_esp.R)
            R_alc = SO3(T_alc.R)

            # Despejar DeltaRot: DeltaRot = R_alc * R_esp^-1
            DeltaRot_matrix = R_alc * R_esp.inv()
            q = UnitQuaternion(DeltaRot_matrix)
            delta_rot = q.angvec('deg')[0]
            return np.round(delta_rot, 2)
        except Exception:
            return np.nan

    # Función interna para calcular el error de traslación
    def calcular_err_tras(pose_array):
        if isinstance(pose_array, np.ndarray) and len(pose_array) >= 3:
            delta_tras = np.linalg.norm(pose_array[:3])
            return np.round(delta_tras, 2)
        return np.nan
    
    def calcular_error_jacobiano(row, model):
        # Comprobamos que tenemos los datos necesarios
        if not (isinstance(row['q_esp'], np.ndarray) and isinstance(row['err_q'], np.ndarray)):
            return (np.nan, np.nan) # Devolvemos una tupla de NaNs
        try:
            # Convertir ángulos a radianes para usar la toolbox
            q_esp_rad = np.deg2rad(row['q_esp'])
            err_q_rad = np.deg2rad(row['err_q'])
            
            # Calcular el Jacobiano en la configuración esperada (en la terna base)
            J = model.jacob0(q_esp_rad)
            
            # Calcular el error cartesiano: delta_x = J * delta_q
            delta_x = J @ err_q_rad  # Usamos @ para multiplicación de matrices
            delta_x[3:] = np.rad2deg(delta_x[3:])
            # print(f'delta_x =\n{delta_x}')
            # delta_x es [dx, dy, dz, drx, dry, drz]
            # Calculamos la magnitud del error de traslación (primeros 3 elementos)
            error_traslacional = np.linalg.norm(delta_x[:3])
            
            # Y la magnitud del error de rotación (últimos 3 elementos)
            error_rotacional = np.linalg.norm(delta_x[3:])
            
            return (np.round(delta_x, 2))
            
        except Exception as e:
            print(f"Error en Jacobiano: {e}") # Para depurar
            return (np.nan, np.nan)

    # Aplicamos las funciones para crear las nuevas columnas
    df_reporte['err_tras'] = df_reporte['err_pose'].apply(calcular_err_tras)
    df_reporte['DeltaRot'] = df_reporte.apply(calcular_delta_rot, axis=1)

    cobot_tb = myCobot320(rotar_base=True, metros=False)

    jac_errors = df_reporte.apply(lambda r: calcular_error_jacobiano(r, cobot_tb), axis=1)
    df_reporte['Delta_err_cart'] = jac_errors
    # ###> 4. Definimos el orden final de las columnas, incluyendo DeltaRot
    cols = ['punto_n', 'version', 'err_q', 'err_pose', 'err_tras', 'DeltaRot', 'Delta_err_cart', 'tb_check']

    df_base = df_reporte[cols].copy()

    nombres_nuevos = {
        'err_q': 'Error q [°]',
        'err_pose': 'Error Pose [mm, °]',
        'err_tras': 'Error Trasl. [mm]',
        'DeltaRot': 'Δ Rotación [°]',
        'Delta_err_cart': 'Δ Cartesiano (J) [mm, °]',
        'tb_check': 'Check TB [mm]'
    }
    
    # Aplica el renombrado
    df_final_renombrado = df_base.rename(columns=nombres_nuevos)

    def formatear_array_bonito(val):
        """Convierte np.array o lista en string '[1.23, 4.56]'"""
        if isinstance(val, (np.ndarray, list)):
            # Si hay NaNs dentro, manejarlos para que no rompan el format
            cleaned_val = [x if not np.isnan(x) else 0.0 for x in val] 
            return "[" + ", ".join([f"{x:.2f}" for x in cleaned_val]) + "]"
        return val

    # Lista de columnas que contienen arrays y quieres formatear
    columnas_con_listas = [
        'Error Pose [mm, °]', 
        'Check TB [mm]', 
        'Δ Cartesiano (J) [mm, °]', # Agregué esta también porque suele ser un array
        'Error q [°]'               # Y esta si quieres que se vea igual
    ]

    for col in columnas_con_listas:
        if col in df_final_renombrado.columns:
            df_final_renombrado[col] = df_final_renombrado[col].apply(formatear_array_bonito)
    
    # Devolvemos el df final, asegurándonos de que solo contenga las columnas deseadas
    return df_final_renombrado

def guardar_df_como_html(df, titulo_tabla, ruta_salida_html):
    """Toma CUALQUIER DataFrame y lo guarda en un archivo HTML."""
    print(f"Exportando a HTML: {titulo_tabla}...")
    html_tabla = df.to_html(index=False, justify='center', border=1)
    html_completo = f"""
    <html><head><style>
      body {{ font-family: sans-serif; }} table {{ border-collapse: collapse; margin: 25px 0; font-size: 0.9em; }}
      th, td {{ padding: 12px 15px; border: 1px solid #dddddd; text-align: center; vertical-align: middle; }}
      thead tr {{ background-color: #009879; color: #ffffff; text-align: center; }}
      tbody tr {{ border-bottom: 1px solid #dddddd; }} tbody tr:nth-of-type(even) {{ background-color: #f3f3f3; }}
    </style></head><body>
    <h2>{titulo_tabla}</h2>
    {html_tabla}
    </body></html>
    """
    with open(ruta_salida_html, 'w', encoding='utf-8') as f:
        f.write(html_completo)
    print(f"-> Guardado en: {ruta_salida_html}")

# --- Funciones que envuelven las acciones para el menú ---
def accion_exportar_completas(df_pre, df_post, ruta_resultados):
    print("\n--- Exportando Tablas Completas ---")
    df_html_pre = preparar_df_completo(df_pre)
    guardar_df_como_html(df_html_pre, "Resultados Consolidados Pre-Calibración", os.path.join(ruta_resultados, 'Resultados_Consolidado_Pre.html'))
    df_html_post = preparar_df_completo(df_post)
    guardar_df_como_html(df_html_post, "Resultados Consolidados Post-Calibración", os.path.join(ruta_resultados, 'Resultados_Consolidado_Post.html'))

def accion_exportar_reportes(df_pre, df_post, ruta_resultados):
    print("\n--- Exportando Reportes de Error ---")
    guardar_df_como_html(df_pre, "Reporte de Errores Pre-Calibración", os.path.join(ruta_resultados, 'Reporte_Errores_Pre.html'))
    guardar_df_como_html(df_post, "Reporte de Errores Post-Calibración", os.path.join(ruta_resultados, 'Reporte_Errores_Post.html'))

def accion_analisis_consola(df_pre, df_post):
    print("\n--- Realizando Análisis de Error de Traslación ---")
    promedio_err_tras_pre = df_pre['Error Trasl. [mm]'].dropna().mean()
    promedio_err_tras_post = df_post['Error Trasl. [mm]'].dropna().mean()
    mejora = promedio_err_tras_pre - promedio_err_tras_post
    mejora_porcentual = (mejora / promedio_err_tras_pre) * 100 if promedio_err_tras_pre != 0 else float('inf')
    
    print(f"Promedio de error de traslación (Pre-Calibración):  {promedio_err_tras_pre:.4f}")
    print(f"Promedio de error de traslación (Post-Calibración): {promedio_err_tras_post:.4f}")
    print(f"\nMejora absoluta: {mejora:.4f}")
    print(f"Mejora porcentual: {mejora_porcentual:.2f}%")

# ==============================================================================
# --- Script Principal ---
# ==============================================================================

# 1. Recolección y consolidación de datos (se ejecuta siempre una vez)
directorio_script = os.path.dirname(os.path.abspath(__file__))
ruta_resultados = os.path.join(directorio_script, 'Resultados')
subcarpetas_entrada = ['ModelTestPreCal', 'ModelTestPostCal']
lista_dfs_pre, lista_dfs_post = [], []

print("--- Iniciando recolección y consolidación de datos... ---")
# ... (El bucle de recolección es idéntico) ...
for nombre_subcarpeta in subcarpetas_entrada:
    ruta_subcarpeta_actual = os.path.join(ruta_resultados, nombre_subcarpeta)
    if not os.path.isdir(ruta_subcarpeta_actual): continue
    for nombre_archivo in os.listdir(ruta_subcarpeta_actual):
        if nombre_archivo.startswith('test_coords_log_v') and nombre_archivo.endswith('.csv'):
            tipo_resultado = "Pre" if "Pre" in nombre_subcarpeta else "Post"
            match = re.search(r'_v(\d+)\.csv', nombre_archivo)
            if not match: continue
            try:
                df = pd.read_csv(os.path.join(ruta_subcarpeta_actual, nombre_archivo), sep=';')
                df['version'] = int(match.group(1))
                (lista_dfs_pre if tipo_resultado == "Pre" else lista_dfs_post).append(df)
            except Exception as e:
                print(f"ERROR al recolectar {nombre_archivo}: {e}")

if not lista_dfs_pre or not lista_dfs_post:
    raise SystemExit("No se encontraron suficientes datos para continuar. Proceso abortado.")

df_pre_completo = pd.concat(lista_dfs_pre, ignore_index=True).sort_values(by=['version', 'punto_n'])
df_post_completo = pd.concat(lista_dfs_post, ignore_index=True).sort_values(by=['version', 'punto_n'])

print("--- Datos listos para el análisis. ---")

# 2. Preparar los DataFrames finales para los reportes
df_reporte_pre = preparar_df_reporte_errores(df_pre_completo)
df_reporte_post = preparar_df_reporte_errores(df_post_completo)

# ==============================================================================
# --- Bucle de Menú Interactivo ---
# ==============================================================================
while True:
    print("\n╔═════════════════════════════════════════════════╗")
    print("║                   MENÚ DE ACCIONES                ║")
    print("╠═════════════════════════════════════════════════╣")
    print("║ 1. Exportar Tablas Completas (HTML)               ║")
    print("║ 2. Exportar Reportes de Error (HTML)              ║")
    print("║ 3. Realizar Análisis de Error en Consola          ║")
    print("║ 4. Ejecutar TODAS las acciones anteriores         ║")
    print("║                                                 ║")
    print("║ 0. Salir                                          ║")
    print("╚═════════════════════════════════════════════════╝")
    
    opcion = input("Seleccione una opción: ")

    if opcion == '1':
        accion_exportar_completas(df_pre_completo, df_post_completo, ruta_resultados)
    elif opcion == '2':
        accion_exportar_reportes(df_reporte_pre, df_reporte_post, ruta_resultados)
    elif opcion == '3':
        accion_analisis_consola(df_reporte_pre, df_reporte_post)
    elif opcion == '4':
        accion_exportar_completas(df_pre_completo, df_post_completo, ruta_resultados)
        accion_exportar_reportes(df_reporte_pre, df_reporte_post, ruta_resultados)
        accion_analisis_consola(df_reporte_pre, df_reporte_post)
    elif opcion == '0':
        print("Saliendo del programa.")
        break
    else:
        print("\n¡Opción no válida! Por favor, seleccione un número del menú.")

print("\n--- Proceso finalizado. ---")