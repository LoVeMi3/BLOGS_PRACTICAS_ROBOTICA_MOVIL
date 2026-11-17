import WebGUI
import HAL
import Frequency

import math
import numpy as np
from collections import deque
import heapq

IDEAL_RATE = 50
FREE = 255
STEP = 10
DIRECTIONS = ((1, 0), (-1, 0), (0, 1), (0, -1))
SAFE_DIST = 4 # a partir de esta distancia a la pared no hay penalización
PENALTY = 4.0 # cuánto se penaliza estar pegado a la pared

V_STRAIGHT = 10 # velocidad lineal máxima, 10 y 6
V_TURN = 6

k_ang = 1.7 # ganancia del ángulo del vector resultante -> velocidad angular
REACHED_DIST = 1.3 #umbral pbjetivo alcanzado


#distancia en celdas de la pared más cercana
def obstacle_distance(mat_map):
	n_rows, n_cols = len(mat_map), len(mat_map[0])
	obst_dist = [[-1] * n_cols for _ in range(n_rows)]
	queue = deque()
	for r in range(n_rows):
		for c in range(n_cols):
			if mat_map[r][c] != FREE:
				obst_dist[r][c] = 0
				queue.append((r, c))
	
	while queue:
		row, col = queue.popleft()
		for dist_r, dist_c in DIRECTIONS:
			nr = row + dist_r
			nc = col + dist_c
			if nr >= 0 and nr < n_rows and nc >= 0 and nc < n_cols and obst_dist[nr][nc] == -1:
				obst_dist[nr][nc] = obst_dist[row][col] + 1
				queue.append((nr, nc))

	return obst_dist

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

def step_cost(obst_dist_value):
	#penalización que crece cuanto más se acerca a la pared
	return max(0, SAFE_DIST - obst_dist_value) * PENALTY + 1.0

#PLANIFICACIÓN - se hace sólo una vez - BFS, gradiente, ruta) - todo en celdas.
def planning(mat_map, tar_cell, rob_cell):
	n_rows = len(mat_map)
	n_cols = len(mat_map[0])
	r_tar, c_tar = tar_cell
	obst_dist = obstacle_distance(mat_map)

	dist = [[-1.0] * n_cols for _ in range(n_rows)] #celdas no alcanzadas
	dist[r_tar][c_tar] = 0.0 #celda objetivo
	heap = [(0.0, r_tar, c_tar)]

	while(heap):
		od, r, c = heapq.heappop(heap)
		if od > dist[r][c]:
			continue
		for dir_r, dir_c in DIRECTIONS:
			num_r = r + dir_r
			num_c = c + dir_c
			if num_r >= 0 and num_r < n_rows and num_c >= 0 and num_c < n_cols and mat_map[num_r][num_c] == FREE:
				num_d = od + step_cost(obst_dist[num_r][num_c])
				#cuántos pasos hay desde ella hasta el objetivo
				if dist[num_r][num_c] == -1 or num_d < dist[num_r][num_c]:
					dist[num_r][num_c] = num_d #ir sumando
					heapq.heappush(heap, (num_d, num_r, num_c))

	row, col = rob_cell
	if dist[row][col] == -1:
		return [], dist #robot en celda no alcanzable

	#descenso por el gradiente
	path_cells = [rob_cell]
	while dist[row][col] != 0:
		best = None
		for dr, dc in DIRECTIONS:
			nr = row + dr
			nc = col + dc
			if 0 <= nr and nr < n_rows and 0 <= nc and nc < n_cols and dist[nr][nc] != -1:
				if best is None or dist[nr][nc] < dist[best[0]][best[1]]:
					best = (nr, nc)
		row, col = best
		path_cells.append((row, col))

	return path_cells, dist



#esperar a que se haga click en el mapa
target = WebGUI.getTargetPose()
while target is None:
	Frequency.tick(IDEAL_RATE)
	target = WebGUI.getTargetPose()

pose = HAL.getPose3d()

#las tenía al revés, aish, no voy a cambiar todas las funciones de arriba
rx, ry = WebGUI.worldToGrid([pose.x, pose.y])
robot_cell = (ry, rx)
tx, ty = WebGUI.worldToGrid(target)
target_cell = (ty, tx)

#mapa de penalizaciones y mapa de sitios a dónde ir
map_matrix = WebGUI.getMap('/resources/exercises/global_navigation/images/cityLargenBin.png') #matriz del mapa, qué celdas ocupadas y cuáles no

path_cells, dist = planning(map_matrix, target_cell, robot_cell) #BFS + extraer ruta
if not path_cells:
	print("No hay ruta:", robot_cell, target_cell, map_matrix[robot_cell[0]][robot_cell[1]])
	while True: #protección
		Frequency.tick(IDEAL_RATE)

d = np.array(dist, dtype=np.float32)
valid = d >= 0
img = np.zeros(d.shape, dtype=np.uint8)
img[valid] = (255 * d[valid] / d[valid].max()).astype(np.uint8)
WebGUI.showNumpy(img)

cells = make_cells_wp(path_cells, STEP)
waypoints = [WebGUI.gridToWorld((c, r)) for (r, c) in cells]

WebGUI.showPath(waypoints)


#CONTROL - en el bucle while - (mandar v y w al robot) - todo en mundo.
i = 0
while True:
	Frequency.tick(IDEAL_RATE)

	pose_x = HAL.getPose3d().x
	pose_y = HAL.getPose3d().y
	pose_yaw = HAL.getPose3d().yaw

	wx, wy = waypoints[i]
	dx = wx - pose_x
	dy = wy - pose_y
	if math.hypot(dx, dy) < REACHED_DIST:
		if i < len(waypoints) - 1:
			i += 1
		continue

	a = math.atan2(dy, dx) - pose_yaw
	angle = math.atan2(math.sin(a), math.cos(a))
	w = max(-V_TURN, min(V_TURN, k_ang * angle))
	v = V_STRAIGHT * max(0.0, math.cos(angle)) #si va "de espaldas", v = 0

	HAL.setV(v)
	HAL.setW(w)

