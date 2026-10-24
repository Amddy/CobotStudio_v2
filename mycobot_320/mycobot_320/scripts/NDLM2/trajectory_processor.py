import cv2
import numpy as np
import base64

class DrawingProcessor:
    def __init__(self, width_mm=120.0, height_mm=160.0, canvas_w=600, canvas_h=800):
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.canvas_w = canvas_w
        self.canvas_h = canvas_h

    def base64_to_image(self, b64_str: str) -> np.ndarray:
        if "," in b64_str:
            b64_str = b64_str.split(",")[1]
        img_bytes = base64.b64decode(b64_str)
        nparr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
        
        # Extraer trazo considerando fondo blanco o canal alfa
        if img.shape[2] == 4:
            alpha = img[:, :, 3]
            gray = cv2.cvtColor(img[:, :, :3], cv2.COLOR_BGR2GRAY)
            # Trazo negro visible
            binary = np.where((alpha > 50) & (gray < 200), 255, 0).astype(np.uint8)
        else:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
        return binary

    def process(self, b64_str: str, epsilon: float = 1.2):
        binary = self.base64_to_image(b64_str)
        
        # 1. Esqueletizado / Adelgazamiento (Thinning Zhang-Suen)
        thinned = cv2.ximgproc.thinning(binary, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        
        # 2. Extracción de Contornos / Polilíneas
        contours, _ = cv2.findContours(thinned, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        
        strokes_px = []
        for cnt in contours:
            # Simplificación Ramer-Douglas-Peucker
            approx = cv2.approxPolyDP(cnt, epsilon=epsilon, closed=False)
            if len(approx) >= 2:
                stroke = approx.reshape(-1, 2)
                strokes_px.append(stroke)
                
        # 3. Optimización de orden de trazos (Greedy Nearest Neighbor)
        ordered_strokes_px = self._optimize_travel(strokes_px)
        
        # 4. Mapeo a Milímetros en el Plano del Papel
        strokes_mm = []
        for stroke in ordered_strokes_px:
            stroke_mm = []
            for pt in stroke:
                x_px, y_px = pt[0], pt[1]
                # Coordenadas locales al origen del papel (esquina inferior izquierda)
                x_mm = (x_px / self.canvas_w) * self.width_mm
                y_mm = (1.0 - (y_px / self.canvas_h)) * self.height_mm
                stroke_mm.append([round(x_mm, 2), round(y_mm, 2)])
            strokes_mm.append(stroke_mm)
            
        return strokes_mm

    def _optimize_travel(self, strokes):
        if not strokes:
            return []
        unvisited = list(strokes)
        current_pos = np.array([0, 0])
        ordered = []

        while unvisited:
            best_idx = 0
            best_dist = float('inf')
            reverse_stroke = False

            for i, stroke in enumerate(unvisited):
                # Distancia al inicio del trazo
                d_start = np.linalg.norm(stroke[0] - current_pos)
                # Distancia al final del trazo (dibujándolo al revés)
                d_end = np.linalg.norm(stroke[-1] - current_pos)

                if d_start < best_dist:
                    best_dist = d_start
                    best_idx = i
                    reverse_stroke = False
                if d_end < best_dist:
                    best_dist = d_end
                    best_idx = i
                    reverse_stroke = True

            chosen = unvisited.pop(best_idx)
            if reverse_stroke:
                chosen = chosen[::-1]
            ordered.append(chosen)
            current_pos = chosen[-1]

        return ordered