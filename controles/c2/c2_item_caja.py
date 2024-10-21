import cv2
import numpy as np

def resize_image(image, scale_percent=50):
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    return cv2.resize(image, dim, interpolation=cv2.INTER_AREA)


#path = path a la imagen, usar el path absoluto
img = cv2.imread(path)

#calibracion de la camara usb
camera_matrix = np.array([[2.89641894e+03, 0.00000000e+00, 6.16793681e+02],
 [0.00000000e+00, 3.24938068e+03, 3.79647575e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=float)


distortion_coeffs = np.array([-1.67481381,-0.10916859,0.00417054,0.20023447,2.43448993], dtype=float)
#rectificar la imagen
undistor_image = cv2.undistort(img, camera_matrix, distortion_coeffs)

#preprocesamiento de la imagen aqui

#encontrar contorno de la caja

#se redimensiona la imagen para que sea mas pequeña
resized_img_contornos = resize_image(undistor_image)

#imagen original con el borde dibujado
cv2.imshow('Bordes de la caja', resized_img_contornos)

cv2.waitKey(0)
cv2.destroyAllWindows()
