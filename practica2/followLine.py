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

V_STRAIGHT = 14
V_TURN = 10
V_SEARCH = 8
W_SEARCH = 8
V_BACK = 7
KP, KD = 1.2, 0.4 #KD a 0 para empezar
MIN_AREA = 200 #área mínima en la que damos la línea por localizada
TURN_THRESHOLD = 0.4 #a partir del que consideramos curva

T_PAUSE = 0.2
T_BACK = 0.7

state = FOLLOW
prev_error = 0.0
last_dir = 1 # +1 izquierda, -1 derecha
lost_time = 0.0

def get_line_error(img):
	"""Devuelve error en [-1, 1] o None si no ve la línea."""
	#RGB2HSV no es el correcto para este simulador
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	height, width = hsv.shape[:2]
	#recortamos la imagen, para que sólo detecte la parte inferior de esta, la línea, 45%
	img_inf = hsv[int(height*0.55):, :] #líneas y columnas

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

def search_line(t):
	"""Comportamiento de búsqueda según los segundos t desde que se perdió la línea."""
	if t < T_PAUSE:
		HAL.setV(0)
		HAL.setW(0)
	elif t < T_BACK:
		HAL.setV(-V_BACK)
		HAL.setW(0)
	else:
		HAL.setV(V_SEARCH)
		HAL.setW(W_SEARCH * last_dir)

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
		if state == LOST:
			state = FOLLOW
			prev_error = error

		w = -(KP * error + KD * (error - prev_error))
		if abs(error) > TURN_THRESHOLD:
			v = V_TURN
		else:
			v = V_STRAIGHT
			
		HAL.setV(v)
		HAL.setW(w)

		prev_error = error
		if error < 0:
			last_dir = -1
		else:
			last_dir = 1

	else:
		"""Línea no visible"""
		if state == FOLLOW:
			state = LOST
			lost_time = time.time()
		
		search_line(time.time() - lost_time)
