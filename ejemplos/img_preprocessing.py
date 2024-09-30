import cv2
import numpy as np

def procesar_imagen(ruta_entrada, ruta_salida):

    # 1. Leer la imagen
    imagen = cv2.imread(ruta_entrada)
    if imagen is None:
        raise FileNotFoundError(f"No se pudo cargar la imagen en {ruta_entrada}")
    
    # Aumentar el brillo y el contraste
    alpha = 1.5  # Contraste [1.0-3.0]
    beta = 50    # Brillo [0-100]
    # ajustada = cv2.convertScaleAbs(imagen, alpha=alpha, beta=beta)

    # 2. Convertir a escala de grises
    gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
    
    # Extra
    # ecualizada = cv2.equalizeHist(gris)

    # 3. Suavizar la imagen
    suavizada = cv2.GaussianBlur(gris, (5, 5), 0)
    
    # 4. Detectar bordes
    bordes = cv2.Canny(suavizada, 80, 230)
    
    # 5. Dibujar bordes sobre la imagen original
    imagen[bordes != 0] = [0, 0, 255]  # Bordes en rojo
    
    # 6. Mostrar la imagen procesada
    cv2.imshow('Imagen Procesada', imagen)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # 7. Guardar la imagen procesada
    cv2.imwrite(ruta_salida, imagen)

if __name__ == "__main__":
    ruta_entrada = 'multimedia/rose.jpg'
    ruta_salida = 'multimedia/rose_proc.jpg'
    procesar_imagen(ruta_entrada, ruta_salida)
