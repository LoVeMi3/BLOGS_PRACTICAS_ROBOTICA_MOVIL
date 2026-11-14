import WebGUI
import HAL
import Frequency

import math

IDEAL_RATE = 50

V_STRAIGHT = 5 # velocidad lineal máxima, 10 y 6
V_TURN = 3

k_ang = 1.5 # ganancia del ángulo del vector resultante -> velocidad angular

REACHED_DIST = 0.5 #umbral pbjetivo alcanzado

#PLANIFICACIÓN - se hace sólo una vez - BFS, gradiente, ruta) - todo en celdas.
#CONTROL - en el bucle while - (mandar v y w al robot) - todo en mundo.

"""
HACER UNA FUNCIÓN QUE SEA PLANIFICACIÓN Y DEVUELVA path_cell

from collections import deque

rows = len(map_matrix)
cols = len(map_matrix[0])
FREE = 255   # ¡comprueba qué valor es "carretera" imprimiendo map_matrix!

# --- BFS desde el objetivo ---
dist = [[-1] * cols for _ in range(rows)]
r0, c0 = target_cell
dist[r0][c0] = 0
q = deque([(r0, c0)])

while q:
    r, c = q.popleft()
    for dr, dc in ((1,0), (-1,0), (0,1), (0,-1)):
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols \
           and dist[nr][nc] == -1 and map_matrix[nr][nc] == FREE:
            dist[nr][nc] = dist[r][c] + 1
            q.append((nr, nc))

# --- Extraer ruta bajando por el gradiente ---
path_cells = [robot_cell]
r, c = robot_cell
while dist[r][c] != 0:
    best = None
    for dr, dc in ((1,0), (-1,0), (0,1), (0,-1)):
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and dist[nr][nc] != -1:
            if best is None or dist[nr][nc] < dist[best[0]][best[1]]:
                best = (nr, nc)
    r, c = best
    path_cells.append((r, c))
"""

px = HAL.getPose3d().x
py = HAL.getPose3d().y
tx = WebGUI.getTargetPose().x
ty = WebGUI.getTargetPose().y

robot_cell = WebGUI.worldToGrid(px, py)
target_cell = WebGUI.worldToGrid(tx, ty)

#mapa de penalizaciones y mapa de sitios a dónde ir
map_matrix = WebGUI.getMap('/resources/exercises/global_navigation/images cityLargenBin.png') #matriz del mapa, qué celdas ocupadas y cuáles no

#path_cells = [...] #BFS + extraer ruta
#N = tamano de path_cell
waypoints = []
for cell in path_cells[::N + 1]:
	waypoints.append(WebGUI.gridToWorld(cell))

WebGUI.showNumpy(numpy)
WebGUI.showPath(array)

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


