import WebGUI
import HAL
import Frequency

import math

IDEAL_RATE = 50

V_STRAIGHT = 3 # velocidad lineal máxima
V_TURN = 1.5

k_rep = 0.3 # ganancia del vector de repulsión
k_ang = 1.5 # ganancia del ángulo del vector resultante -> velocidad angular

d_influence = 2.0 # distancia a partir de la cual el obstáculo repele
d_min = 0.05 # evita dividir por cero si el obstáculo está pegado
REACHED_DIST = 1.0 #umbral pbjetivo alcanzado

def repulsion(laser):
	rx = 0
	ry = 0
	#Distancia válida del láser (inf si es 0, NaN o inf).
	for i, d in enumerate(laser.values[:180]):
		if d is None or len(laser.values) < 180 or math.isnan(d) or math.isinf(d) or d <= 0.0 or d > d_influence:
			continue
		d = max(d, d_min)
		ang = math.radians(i)
		ox = d * math.sin(ang)
		oy = -d * math.cos(ang)
		#ir ajustando k_rep
		mag = k_rep * (((1.0 / d) - (1.0 / d_influence)) / (d * d))
		rx += mag * (-ox / d)
		ry += mag * (-oy / d)

	return rx / 180.0, ry / 180.0 #evitar valores enormes



target = WebGUI.getNextTarget()
while True:
	Frequency.tick(IDEAL_RATE)

	"""
	Leer pose y láser.
	Comprobar si se ha alcanzado el objetivo.
	Calcular la atracción.
	Calcular la repulsión.
	Sumar y convertir a v y w.
	Enviar y depurar.
	"""

	pose_x = HAL.getPose3d().x
	pose_y = HAL.getPose3d().y
	pose_yaw = HAL.getPose3d().yaw
	laser = HAL.getLaserData()
	if laser is None:
		continue
	
	dx = target.getPose().x - pose_x
	dy = target.getPose().y - pose_y
	if math.hypot(dx, dy) < REACHED_DIST:
		target.setReached(True)
		target = WebGUI.getNextTarget()
		continue

	#vector atractivo
	att_x = dx * math.cos(pose_yaw) + dy * math.sin(pose_yaw)
	att_y = -dx * math.sin(pose_yaw) + dy * math.cos(pose_yaw)
	norm = math.hypot(att_x, att_y)
	if norm > 1e-6:
		att_x, att_y = att_x / norm, att_y / norm #crece con la distancia

	#vector repulsión
	rep_x, rep_y = repulsion(laser)

	total_x = att_x + rep_x
	total_y = att_y + rep_y

	angle = math.atan2(total_y, total_x)
	w = max(-V_TURN, min(V_TURN, k_ang * angle))
	v = V_STRAIGHT * max(0.0, math.cos(angle)) #si va "de espaldas", v = 0

	WebGUI.showForces([att_x, att_y], [rep_x, rep_y], [total_x, total_y])
	WebGUI.showLocalTarget([att_x, att_y])

	HAL.setV(v)
	HAL.setW(w)

