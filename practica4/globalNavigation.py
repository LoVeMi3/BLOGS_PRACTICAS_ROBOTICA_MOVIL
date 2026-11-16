import WebGUI
import HAL
import Frequency

import math
import numpy as np
from collections import deque

IDEAL_RATE = 50
FREE = 255
STEP = 10

V_STRAIGHT = 5 # velocidad lineal máxima, 10 y 6
V_TURN = 3

k_ang = 1.5 # ganancia del ángulo del vector resultante -> velocidad angular
REACHED_DIST = 0.5 #umbral pbjetivo alcanzado

#PLANIFICACIÓN - se hace sólo una vez - BFS, gradiente, ruta) - todo en celdas.
#CONTROL - en el bucle while - (mandar v y w al robot) - todo en mundo.

def planning(mat_map, tar_cell, rob_cell):
	n_rows = len(mat_map)
	n_cols = len(mat_map[0])

	dist = [[-1] * n_cols for _ in range(n_rows)] #celdas no alcanzadas
	r_tar, c_tar = tar_cell
	dist[r_tar][c_tar] = 0 #celda objetivo
	queue = deque([(r_tar, c_tar)])

	neighbours_directions = ((1, 0), (-1, 0), (0, 1), (0, -1))

	while(queue):
		r, c = queue.popleft()
		for dir_r, dir_c in neighbours_directions:
			num_r = r + dir_r
			num_c = c + dir_c
			if num_r >= 0 and num_r < n_rows and num_c >= 0 and num_c < n_cols and  dist[num_r][num_c] == -1 and mat_map[num_r][num_c] == FREE:
				dist[num_r][num_c] = dist[r][c] + 1
				queue.append((num_r, num_c))

	row, col = rob_cell
	if dist[row][col] == -1:
		return [], dist #robot en celda no alcanzable

	path_cells = [rob_cell]
	while dist[row][col] != 0:
		best = None
		for dr, dc in neighbours_directions:
			nr = row + dr
			nc = col + dc
			if 0 <= nr and nr < n_rows and 0 <= nc and nc < n_cols and dist[nr][nc] != -1:
				if best is None or dist[nr][nc] < dist[best[0]][best[1]]:
					best = (nr, nc)
		row, col = best
		path_cells.append((row, col))

	return path_cells, dist


pose = HAL.getPose3d()
target = WebGUI.getTargetPose()

robot_cell = WebGUI.worldToGrid(pose)
target_cell = WebGUI.worldToGrid(target)

#mapa de penalizaciones y mapa de sitios a dónde ir
map_matrix = WebGUI.getMap('/resources/exercises/global_navigation/images/cityLargenBin.png') #matriz del mapa, qué celdas ocupadas y cuáles no

path_cells, dist = planning(map_matrix, target_cell, robot_cell) #BFS + extraer ruta
d = np.array(dist, dtype=np.float32)
valid = np.isfinite(d)
img = np.zeros(d.shape, dtype=np.uint8)
img[valid] = (255 * d[valid] / d[valid].max()).astype(np.uint8)
WebGUI.showNumpy(img)
"""
WebGUI.showPath(path_cells)

N = len(path_cells)
path_cells += [target]
waypoints = []
for cell in path_cells[::N + 1]:
	waypoints.append(WebGUI.gridToWorld(cell))
"""
STEP = 10
cells = path_cells[::STEP]
if cells[-1] != path_cells[-1]:
	cells.append(path_cells[-1])
waypoints = [WebGUI.gridToWorld(c) for c in cells]

WebGUI.showPath(waypoints)



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

