import WebGUI
import HAL
import Frequency

import math

IDEAL_RATE = 50

V_STRAIGHT = 5 # velocidad lineal máxima, 10 y 6
V_TURN = 3

k_ang = 1.5 # ganancia del ángulo del vector resultante -> velocidad angular

REACHED_DIST = 0.5 #umbral pbjetivo alcanzado

#PLANIFICACIÓN - se hace sólo una vez
#mapa de penalizaciones y mapa de sitios a dónde ir
map_matrix = WebGUI.getMap('/resources/exercises/global_navigation/images cityLargenBin.png') #matriz del mapa, qué celdas ocupadas y cuáles no
col = 0 #i
row = 0 #j
for i in xrange(len(map_matrix[0])):
    for j in xrange(len(map_matrix)):
        row = j
		col = i
not_visited_map = [row][col]

target = WebGUI.getTargetPose()
while True:
    Frequency.tick(IDEAL_RATE)

	pose_x = HAL.getPose3d().x
	pose_y = HAL.getPose3d().y
	pose_yaw = HAL.getPose3d().yaw

	dx = target.x - pose_x
	dy = target.y - pose_y
	if math.hypot(dx, dy) < REACHED_DIST:
		target.setReached(True)
		target = WebGUI.getNextTarget()
		continue

	angle = math.atan2(dy, dx)
	w = max(-V_TURN, min(V_TURN, k_ang * angle))
	v = V_STRAIGHT * max(0.0, math.cos(angle)) #si va "de espaldas", v = 0

	HAL.setV(v)
	HAL.setW(w)

	WebGUI.rowColumn(vector)
	WebGUI.worldToGrid(pose)
	WebGUI.gridToWorld(cell)

	WebGUI.showNumpy(numpy)
	WebGUI.showPath(array)

