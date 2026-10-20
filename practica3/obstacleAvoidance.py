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

V_STRAIGHT = 20.8
V_TURN = 5.3
V_SEARCH = 6
W_SEARCH = 3

KP, KD = 1.3, 0.5 
KP_FAST, KD_FAST = 0.3, 0.2 #0.5, 0.8

ACCEL = 0.3
MIN_AREA = 50 #80  #área mínima en la que damos la línea por localizada
ALPHA = 0.8 #0.6 # suavizado de la salida, más alto = más reactivo
SLOW_ALPHA = 0.04 # media lenta, cuanto más bajo, más lenta
SLOW_LO, SLOW_HI = 0.4, 0.7 # err_slow: por debajo, velocidad máxima; por encima, V_TURN
FAST_LO, FAST_HI = 0.5, 0.7 # error instantáneo: seguridad para curvas cerradas
err_slow = 0.0 # global, fuera del bucle

T_HOLD = 0.2 # tiempo que mantenemos la última orden antes de buscar #0.1

state = FOLLOW
prev_error = 0.0
last_dir = 1 # +1 izquierda, -1 derecha
last_w = 0.0
last_v = V_TURN
lost_time = 0.0
w_filt = 0.0

#reducción de la velocidad al tomar una curva dependiendo de cuanto aumente el error
def ramp(x, lo, hi):
	return min(max((x - lo) / (hi - lo), 0.0), 1.0)

def get_line_error(img):
	"""Devuelve error en [-1, 1] o None si no ve la línea."""
	#RGB2HSV no es el correcto para este simulador
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	height, width = hsv.shape[:2]
	#recortamos la imagen, para que sólo detecte la parte inferior de esta, la línea, 40%
	img_inf = hsv[int(height*0.6):, :] #líneas y columnas

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

def obstacle_detection():
    laser = HAL.getLaserData()

    # Comprobando si los datos son válidos; si no, al siguiente ciclo
    if laser is None or len(laser.values) < 180:
        HAL.setV(0.0)
        HAL.setW(0.0)
        continue

    # Distancias mínimas por zonas
    center = min(rd(laser, i) for i in range(60, 121)) # ±30º frontal
    sideL = min(rd(laser, i) for i in range(140, 180)) # lado izquierdo
    sideR = min(rd(laser, i) for i in range(0, 40)) # lado derecho

    # Suma de distancias para decidir hacia dónde girar
    left = sum(rd(laser, i) for i in range(135, 180))
    right = sum(rd(laser, i) for i in range(0, 45))

    obst = center < 0.40 or sideL < 0.22 or sideR < 0.22

    #SACAR DE ESTA FUNCIÓN EL VALOR DEL VECTOR DE REPULSIÓN PARA SACAR ANGLE

    return obst

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
		if state == LOST: # línea recién recuperada
			prev_error = error # evita el pico de D
			w_filt = last_w
		state = FOLLOW

		err_slow = SLOW_ALPHA * error + (1 - SLOW_ALPHA) * err_slow
		k = max(ramp(abs(err_slow), SLOW_LO, SLOW_HI), ramp(abs(error), FAST_LO, FAST_HI))
		v_target = V_STRAIGHT - (V_STRAIGHT - V_TURN) * k
		v = min(v_target, last_v + ACCEL)

		s = 1.0 - k # 1 = velocidad máxima, 0 = V_TURN
		big = ramp(abs(error), 0.5, 0.75) # error grande -> ganancia de curva ya
		s = s * (1.0 - big)
		kp = KP + (KP_FAST - KP) * s
		kd = KD + (KD_FAST - KD) * s

		w_cmd = -(kp * error + kd * (error - prev_error))
		w_filt = ALPHA * w_cmd + (1 - ALPHA) * w_filt # suavizado
		
		print(f"err={error:.2f} slow={err_slow:.2f} k={k:.2f} v={v:.1f} w={w_filt:.2f}")

        #DETECCIÓN DEL OBSTÁCULO, FUNCIÓN QUE SE ACTIVE SEGÚN SE DETECTE UN OBSTÁCULO

        #las variables V y W las sacamos de la función de dete4cción del obstáculo
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
			print("LOST! last_w=%.2f last_dir=%d" % (last_w, last_dir))
		
		t = time.time() - lost_time
		last_v = V_SEARCH
		if t < T_HOLD:
 			# tiempo corto, seguimos con la última orden, sin frenar
			HAL.setV(V_SEARCH)
			HAL.setW(-max(abs(last_w), W_SEARCH) * last_dir)
		else:
			# modo búsqueda, girar hacia donde la vimos por última vez
			HAL.setV(V_SEARCH)
			HAL.setW(-W_SEARCH * last_dir)
			
