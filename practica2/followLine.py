import WebGUI
import HAL
import Frequency

import numpy as np
import argparse
import cv2
import time

# Estados
FOLLOW, LOST = range(2)

IDEAL_RATE = 50

V_STRAIGHT = 19
V_TURN = 5xd
V_SEARCH = 6
W_SEARCH = 3.5
#V_BACK = 9
KP, KD = 1.0, 0.4 #KD a 0 para empezar, ir aumentando
MIN_AREA = 80 #área mínima en la que damos la línea por localizada
TURN_THRESHOLD = 0.35 #a partir del que consideramos curva #0.4

T_HOLD = 0.2 # tiempo que mantenemos la última orden antes de buscar #0.1

state = FOLLOW
prev_error = 0.0
last_dir = 1 # +1 izquierda, -1 derecha
last_w = 0.0
last_v = V_TURN
lost_time = 0.0
w_filt = 0.0
ALPHA = 0.6 # suavizado de la salida (0..1, más alto = más reactivo)

def get_line_error(img):
	"""Devuelve error en [-1, 1] o None si no ve la línea."""
	#RGB2HSV no es el correcto para este simulador
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	height, width = hsv.shape[:2]
	#recortamos la imagen, para que sólo detecte la parte inferior de esta, la línea, 60%
	img_inf = hsv[int(height*0.4):, :] #líneas y columnas

	#rojo, nos movemos en dos rangos
	m1 = cv2.inRange(img_inf, (0, 100, 100), (10, 255, 255))
	m2 = cv2.inRange(img_inf, (170, 100, 100), (180, 255, 255))
	mask = cv2.bitwise_or(m1, m2)

	M = cv2.moments(mask, binaryImage=True)
	if (M["m00"] < MIN_AREA):
		return None, mask
	cx = M["m10"] / M["m00"]
	sol_cx = (cx - width / 2) / (width / 2)
	return sol_cx, mask

while True:
	Frequency.tick(IDEAL_RATE)

	img = HAL.getImage()
	if img is None:
		HAL.setV(0); 
		HAL.setW(0)
		continue

	error, mask = get_line_error(img)
	WebGUI.showImage(mask) #comprobar la calibración

	if error is not None:
		"""Línea visible"""
		state = FOLLOW

		w_cmd = -(KP * error + KD * (error - prev_error))
		w_filt = ALPHA * w_cmd + (1 - ALPHA) * w_filt # suavizado
		if abs(error) > TURN_THRESHOLD:
			v = V_TURN
		else:
			v = V_STRAIGHT
			
		HAL.setV(v)
		HAL.setW(w_filt)

		last_v, last_w = v, w_filt
		prev_error = error
		if abs(error) > 0.15:
			if error < 0:
				last_dir = -1
			else:
				last_dir = 1

	else:
		"""Línea no visible"""
		if state == FOLLOW:
			state = LOST
			lost_time = time.time()
		
		t = time.time() - lost_time
		if t < T_HOLD:
 			# tiempo corto, seguimos con la última orden, sin frenar
			HAL.setV(V_SEARCH)
			HAL.setW(-max(abs(last_w), W_SEARCH) * last_dir)
		else:
			# modo búsqueda, girar hacia donde la vimos por última vez
			HAL.setV(V_SEARCH)
			HAL.setW(-W_SEARCH * last_dir)
