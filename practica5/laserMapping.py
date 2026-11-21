import WebGUI
import HAL
import Frequency

import math
import random

IDEAL_RATE = 50

V_STRAIGHT = 10 # velocidad lineal máxima
V_TURN = 6

# Estados
SPIRAL, FORWARD, BACKUP, TURN = range(4)
state = SPIRAL

ticks = 0
turn_ticks = 0
turn_dir = 1.0  # +1 izquierda, -1 derecha
k_rep = 0.5
d_influence = 2.3
d_min = 0.05

BLANK = 0.4
L_BLANK = log(BLANK/ 1.0 - BLANK) #log(p / (1.0 - p)
TAKEN = 0.6
L_TAKEN = log(TAKEN/ 1.0 - TAKEN)
SUS = 0.5
L_SUS = log(SUS/ 1.0 - SUS)

#logaritmo de ratio de probabilidad - regla de bayes
#exploración - llevar a cuenta por los puntos que ya he pasado:
# estén ocupados o libres, la exploración busca zonas por las que no ha pasado

def rd(laser, i):
    """Lectura del láser saturada a 5 m y con valores no finitos -> 5 m."""
    r = laser.values[i]
    return min(r, 5.0) if math.isfinite(r) else 5.0

def get_state(laser):
	# Distancias mínimas por zonas
    center = min(rd(laser, i) for i in range(60, 121)) # ±30º frontal
    sideL = min(rd(laser, i) for i in range(140, 180)) # lado izquierdo
    sideR = min(rd(laser, i) for i in range(0, 40)) # lado derecho

    # Suma de distancias para decidir hacia dónde girar
    left = sum(rd(laser, i) for i in range(135, 180))
    right = sum(rd(laser, i) for i in range(0, 45))

    obst = center < 0.40 or sideL < 0.22 or sideR < 0.22

    if state == SPIRAL:
        v = 0.3 # velocidad lineal
        r = 0.4 + 0.004 * ticks # radio creciente con el tiempo
        HAL.setV(v)
        HAL.setW(v / r) # velocidad angular
        ticks += 1
        if obst:
            ticks = 0
            state = BACKUP
        elif ticks > 400: # radio muy grande -> recto
            ticks = 0
            state = FORWARD

    elif state == FORWARD:
        HAL.setV(0.4)
        HAL.setW(0.0)
        if obst:
            ticks = 0
            state = BACKUP

    elif state == BACKUP:
        HAL.setV(-0.15)
        HAL.setW(0.0)
        ticks += 1
        if ticks > 10:
            turn_dir = 1.0 if left > right else -1.0
            # Giro aleatorio entre ~57º y ~143º (1.0 a 2.5 rad) con w = 1 rad/s
            angle = 1.0 + random.random() * 1.5
            turn_ticks = int(angle * IDEAL_RATE)
            ticks = 0
            state = TURN

    elif state == TURN:
        HAL.setV(0.0)
        HAL.setW(turn_dir * 1.0)
        ticks += 1
        if ticks > turn_ticks:
            ticks = 0
            # Espiral solo si hay mucho espacio; si no, recto
            if center > 1.5 and sideL > 1.0 and sideR > 1.0:
                state = SPIRAL
            else:
                state = FORWARD

	return state

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

#paso 4?
#waypoints para cada esquina porque me atraviesa las paredes si no
def make_cells_wp(path, step):
	waypoints = []
	waypoints.append(path[0])
	run = 0
	for i in range(1, len(path) - 1):
		d_prev = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
		d_next = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
		run += 1
		if d_prev != d_next or run >= step: #esquina o tramo largo
			waypoints.append(path[i])
			run = 0
	waypoints.append(path[-1])
	return waypoints

rx, ry = WebGUI.worldToGrid([pose.x, pose.y])
robot_cell = (ry, rx)
tx, ty = WebGUI.worldToGrid(target)
target_cell = (ty, tx)
path_cells, dist = planning(map_matrix, target_cell, robot_cell) #BFS + extraer ruta
cells = make_cells_wp(path_cells, STEP)
waypoints = [WebGUI.gridToWorld((c, r)) for (r, c) in cells]

while True:
    Frequency.tick(IDEAL_RATE)

    pose_x = HAL.getPose3d().x
	pose_y = HAL.getPose3d().y
	pose_yaw = HAL.getPose3d().yaw
	laser = HAL.getLaserData()
	if laser is None or len(laser.values) < 180:
		continue

    #vector repulsión
	rep_x, rep_y = repulsion(laser)
