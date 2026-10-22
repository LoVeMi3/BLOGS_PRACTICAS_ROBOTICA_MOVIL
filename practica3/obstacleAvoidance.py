import WebGUI
import HAL
import Frequency

import numpy as np
import argparse
import cv2
import time
import math

IDEAL_RATE = 50

V_STRAIGHT = 20.8 #v_max=0.5 # velocidad lineal máxima
V_TURN = 5.3

KP, KD = 1.3, 0.5 
KP_FAST, KD_FAST = 0.3, 0.2 #0.5, 0.8

k_att_lat = 1.0 # ganancia lateral del vector de atracción (siguelíneas)
k_rep = 1.0 # ganancia del vector de repulsión
k_ang = 1.5 # ganancia del ángulo del vector resultante -> velocidad angular

d_influence = 1.0 # distancia a partir de la cual el obstáculo repele (tu 1.0 m)
d_min = 0.05 # evita dividir por cero si el obstáculo está pegado

"""
def rd(laser, i):
	"""Distancia válida del láser (inf si es 0, NaN o inf)."""
	d = laser.values[i]
	if d is None or math.isnan(d) or math.isinf(d) or d <= 0.0:
		return float('inf')
	return d
"""

def obstacle_detection():
	laser = HAL.getLaserData()

	# Comprobando si los datos son válidos; si no, al siguiente ciclo
	if laser is None or len(laser.values) < 180:
		return False, 0.0, 0.0

	# Punto más cercano, con peso menor en los laterales:
	# dividir por el peso hace que un punto lateral "parezca" más lejano
	best_i, best_score = None, float('inf')
	for i in range(0, 180):
		d = rd(laser, i)
		if d == float('inf'):
			continue
		ang = math.radians(i)
		weight = 0.4 + 0.6 * math.sin(ang)   # 0.4 en los lados, 1.0 al frente
		score = d / weight
		if score < best_score:
			best_score, best_i = score, i

	if best_i is None:
		return False, 0.0, 0.0

	d = rd(laser, best_i)
	ang = math.radians(best_i)
	ox = d * math.sin(ang)     # hacia delante
	oy = -d * math.cos(ang)    # positivo = izquierda

	obst = d < d_influence
	return obst, ox, oy

def potential_field_cmd(obstacle=None):
	# Vector de atracción: siempre hacia delantee
	att_x = 1.0
	att_y = k_att_lat

	# Vector de repulsión: apunta en sentido opuesto al obstáculo
	rep_x, rep_y = 0.0, 0.0
	if obstacle is not None:
		ox, oy = obstacle
		d = math.hypot(ox, oy)

		if d < d_influence:
			d = max(d, d_min)
			#crece al acercarse, 0 en d_influence
			mag = min(k_rep * (1.0 / d - 1.0 / d_influence), 5.0)

			#del obstáculo hacia el robot
			rep_x = mag * (-ox / d)
			rep_y = mag * (-oy / d)

			#Si el obstáculo está justo delante, el empuje lateral es nulo
			# y el robot se queda atascado, se rompe la simetría girando a un lado.
			if ox > 0.0 and abs(oy) < 0.05:
				rep_y += mag * 0.5

	# Suma de vectores
	total_x = att_x + rep_x
	total_y = att_y + rep_y

	# Vector resultante -> velocidades
	angle = math.atan2(total_y, total_x) # hacia dónde quiere ir el robot

	w = max(-V_TURN, min(V_TURN, k_ang * angle))

	#avanza más cuanto más alineado esté con el vector resultante
	v = V_STRAIGHT * max(0.0, math.cos(angle)) * min(1.0, math.hypot(total_x, total_y))
	v = max(0.0, min(V_STRAIGHT, v))

	return v, w



while True:
	Frequency.tick(IDEAL_RATE)

	img = HAL.getImage()
	if img is None:
		HAL.setV(0); 
		HAL.setW(0)
		continue

	obst, obs_x, obs_y = obstacle_detection()

	if obst:
		obstacle = (obs_x, obs_y)
	else: 
		obstacle = None
		
	v, w = potential_field_cmd(obstacle)

	HAL.setV(v)
	HAL.setW(w)
			
