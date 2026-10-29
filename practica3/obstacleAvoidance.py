import WebGUI
import HAL
import Frequency

import math

IDEAL_RATE = 50

V_STRAIGHT = 10 # velocidad lineal máxima
V_TURN = 6

k_rep = 0.5 # ganancia del vector de repulsión
k_ang = 1.5 # ganancia del ángulo del vector resultante -> velocidad angular
k_tan = 1.5 # fuerza del empujón lateral al rodear

d_influence = 2.5 # distancia a partir de la cual el obstáculo repele
d_min = 0.05 # evita dividir por cero si el obstáculo está pegado
last_dir = 0.5 # +1 izquierda, -1 derecha
REACHED_DIST = 1.0 #umbral pbjetivo alcanzado

"""
REP_ACTIVE = 0.02    # |rep| a partir del cual consideramos que hay obstáculo (ajustar)
CONFLICT_COS = -0.5  # cos del ángulo att-rep por debajo del cual hay "bloqueo frontal"
RELEASE_CYCLES = 25  # ciclos sin obstáculo para soltar el lado elegido (~0.5 s)
SWITCH_COOLDOWN = 100  # ciclos mínimos antes de poder cambiar de lado (~2 s)
MAX_RANGE = 5.0

escape_dir = 0       # +1 = izquierda, -1 = derecha, 0 = libre
free_cycles = 0
since_switch = 0

def side_clearance(laser):
	#Espacio libre medio a la derecha (0-89) y a la izquierda (90-179).
	vals = [clean(d) for d in laser.values[:180]]
	right = sum(vals[0:90]) / 90.0
	left = sum(vals[90:180]) / 90.0
	return left, right
	"""

def repulsion(laser):
	rx = 0
	ry = 0
	#Distancia válida del láser (inf si es 0, NaN o inf).
	for i, d in enumerate(laser.values[:180]):
		if d is None or math.isnan(d) or math.isinf(d) or d <= 0.0 or d > d_influence:
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
 	
	pose_x = HAL.getPose3d().x
	pose_y = HAL.getPose3d().y
	pose_yaw = HAL.getPose3d().yaw
	laser = HAL.getLaserData()
	if laser is None or len(laser.values) < 180:
		continue
	
	dx = target.getPose().x - pose_x
	dy = target.getPose().y - pose_y
	if math.hypot(dx, dy) < REACHED_DIST:
		target.setReached(True)
		target = WebGUI.getNextTarget()
		"""escape_dir = 0"""
		continue

	#vector atractivo
	att_x = dx * math.cos(pose_yaw) + dy * math.sin(pose_yaw)
	att_y = -dx * math.sin(pose_yaw) + dy * math.cos(pose_yaw)
	norm = math.hypot(att_x, att_y)
	if norm > 1e-6:
		att_x, att_y = att_x / norm, att_y / norm #crece con la distancia

	#vector repulsión
	rep_x, rep_y = repulsion(laser)
	"""rep_norm = math.hypot(rep_x, rep_y)

	# --- Decisión de lado ---
	since_switch += 1
	left_free, right_free = side_clearance(laser)

	if rep_norm > REP_ACTIVE:
		free_cycles = 0
		cos_ar = (att_x * rep_x + att_y * rep_y) / rep_norm  # att ya está normalizado
		blocked = cos_ar < CONFLICT_COS

		if escape_dir == 0 and blocked:
			# Elegimos el lado con más espacio; si hay empate, el que apunta al objetivo
			if abs(left_free - right_free) > 0.2:
				escape_dir = 1 if left_free > right_free else -1
			else:
				escape_dir = 1 if att_y >= 0 else -1
			since_switch = 0

		# Si el lado elegido se ha cerrado y el otro está claramente más libre, cambiamos
		elif escape_dir != 0 and since_switch > SWITCH_COOLDOWN:
			chosen = left_free if escape_dir == 1 else right_free
			other = right_free if escape_dir == 1 else left_free
			if chosen < 0.8 and other > chosen + 1.0:
				escape_dir = -escape_dir
				since_switch = 0
	else:
		free_cycles += 1
		if free_cycles > RELEASE_CYCLES:
			escape_dir = 0

	# Componente lateral hacia el lado elegido (y > 0 = izquierda)
	tan_y = escape_dir * k_tan * rep_norm """

	total_x = att_x + rep_x
	total_y = att_y + rep_y """+ tan_y """

	angle = math.atan2(total_y, total_x)
	w = max(-V_TURN, min(V_TURN, k_ang * angle))
	v = V_STRAIGHT * max(0.0, math.cos(angle)) #si va "de espaldas", v = 0

	WebGUI.showForces([att_x, att_y], [rep_x, rep_y], [total_x, total_y])
	WebGUI.showLocalTarget([att_x, att_y])

	HAL.setV(v)
	HAL.setW(w * last_dir)
