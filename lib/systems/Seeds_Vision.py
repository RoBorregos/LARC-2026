import cv2
import numpy as np

#Configuración 
CAM_INDEX = 1
ROI_W, ROI_H = 300, 300   # tamaño del recuadro central (Region of  Interest)

#En HSV (Hue Saturation Value): V es el brillo (0-255)
V_DARK  = 80# si el brillo promedio es menor que esto = muy oscuro

#En HSV: S es la saturación (Gris/poco color/sombra = bajo, color fuerte = alto)
S_MIN   = 40# saturación mínima para confiar en el color

#En HSV: H(Hue): es el tono en el espectro de colores (0-179 en OpenCV)
#En este espectro, el azul suele estar entre 95 y 130
H_LO    = 95# límite inferior del rango azul 
H_HI    = 130# límite superior del rango azul
#


cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise SystemExit("No se pudo abrir la cámara")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Recorta el ROI (región central del frame)
    H, W = frame.shape[:2]
    cx, cy = W // 2, H // 2
    x1 = max(0, cx - ROI_W // 2)
    y1 = max(0, cy - ROI_H // 2)
    x2 = min(W, cx + ROI_W // 2)
    y2 = min(H, cy + ROI_H // 2)
    roi = frame[y1:y2, x1:x2]

    # ── Convierte a HSV y saca canales ─────────────────────────
    # HSV separa: Hue (color), Saturation (intensidad), Value (brillo)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h_ch = hsv[:, :, 0]   # canal de color (tono)
    s_ch = hsv[:, :, 1]   # canal de saturación
    v_ch = hsv[:, :, 2]   # canal de brillo

    # Promedios de toda la región
    V_mean = float(np.mean(v_ch))
    S_mean = float(np.mean(s_ch))

    # Hue promedio solo en píxeles con suficiente saturación
    # (píxeles grises/oscuros tienen hue poco confiable)
    pixels_con_color = h_ch[s_ch >= S_MIN]
    H_mean = float(np.mean(pixels_con_color)) if pixels_con_color.size else float("nan")

    # ── Clasificación ──────────────────────────────────────────
    if V_mean < V_DARK:
        # Muy oscuro → grano negro/sobremaduro
        label = "SOBREMADURO (oscuro)"
    elif S_mean < S_MIN:
        # Poco color → no se puede determinar
        label = "INCIERTO (poca saturacion)"
    elif H_LO <= H_mean <= H_HI:
        # Tono azulado → sobremaduro
        label = "SOBREMADURO (azul)"
    else:
        # Cualquier otro color → maduro
        label = "MADURO (no azul)"

    # ── Dibuja resultados en pantalla ──────────────────────────
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(frame,
                f"V:{V_mean:.1f}  S:{S_mean:.1f}  H:{H_mean:.1f}",
                (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
    cv2.putText(frame,
                label,
                (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (255, 255, 255), 2)

    cv2.imshow("Seed Vision", frame)

    #Teclas
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):   # salir
        break
    if key == ord('s'):   # guardar recorte
        fname = f"roi_{label.replace(' ', '_').replace('(','').replace(')','')}.png"
        cv2.imwrite(fname, roi)
        print("Guardado:", fname)

cap.release()
cv2.destroyAllWindows()

