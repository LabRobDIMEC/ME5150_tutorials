import cv2
import numpy as np

def resize_image(image, scale_percent=50):
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    return cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

# Cargar la imagen
#path = path a la imagen, usar el path absoluto
image = cv2.imread(path)

#calibracion de la camara usb
camera_matrix = np.array([[2.89641894e+03, 0.00000000e+00, 6.16793681e+02],
 [0.00000000e+00, 3.24938068e+03, 3.79647575e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=float)

# Definir los coeficientes de distorsión como un array de numpy
distortion_coeffs = np.array([-1.67481381,-0.10916859,0.00417054,0.20023447,2.43448993], dtype=float)

undistor_image = cv2.undistort(image, camera_matrix, distortion_coeffs)
#cv2.imshow('Undistorted Image', undistor_image)
# Convertir a escala de grises
gray = cv2.cvtColor(undistor_image, cv2.COLOR_BGR2GRAY)

#preprocesamiento de la imagen aqui

#encontrar contorno o centro del lente

resized_img_contorno = resize_image(undistor_image)

#imagen original con el borde/centro dibujado
cv2.imshow('Bordes de la caja', resized_img_contorno)

cv2.waitKey(0)
cv2.destroyAllWindows()
