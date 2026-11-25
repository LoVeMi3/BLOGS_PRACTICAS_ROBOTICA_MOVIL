import WebGUI
import HAL
import Frequency

import math
import random
import numpy as np

IDEAL_RATE = 50

V_STRAIGHT = 10 # velocidad lineal máxima
V_TURN = 6

k_rep = 0.5
d_influence = 0.85
d_min = 0.05

#y es row, x es col
#y es HEIGHT, x es WIDTH

HEIGHT = 970
WIDTH = 1500
#el (0,0) del mundo cae en el centro de la matriz
grid = np.zeros((HEIGHT, WIDTH)) #0 = log-odds, de p = 0.5
grid_visits = np.zeros((HEIGHT, WIDTH)) #cuántas veces se actualiza cada celda

L_MAX = 5.0
BLANK = 0.4
L_BLANK = math.log(BLANK/ (1.0 - BLANK)) #log(p / (1.0 - p)
TAKEN = 0.7
L_TAKEN = math.log(TAKEN/ (1.0 - TAKEN))
SUS = 0.5 #prior

T_FREE = 0.8 #log-odds: ~2 observaciones de libre
T_OCC = 0.8 #log-odds: ~1 observación de ocupado (L_TAKEN ≈ 0.85)
FREE, UNKNOWN, OCCUPIED = -1, 0, 1

STATE = EXPLORING
NUM_LASER_VALUES = 360

#logaritmo de ratio de probabilidad - regla de bayes
#exploración - llevar a cuenta por los puntos que ya he pasado:
# estén ocupados o libres, la exploración busca zonas por las que no ha pasado

"""def rd(laser, i):
    #Lectura del láser saturada a 5 m y con valores no finitos -> 5 m.
    r = laser.values[i]
    return min(r, 5.0) if math.isfinite(r) else 5.0"""

def world_to_cell(x, y):
    mx, my, non_yaw = WebGUI.poseToMap(x, y, 0)
    return int(round(my)), int(round(mx)) #fila, columna

def classify(grid, grid_visits):
    matrix = np.full((HEIGHT, WIDTH), UNKNOWN, dtype=np.int8)
    visited = grid_visits > 0
    matrix[visited & (grid < -T_FREE)] = FREE
    matrix[visited & (grid > T_OCC)] = OCCUPIED
    return matrix

def repulsion(laser):
    rx = 0
    ry = 0
    #Distancia válida del láser (inf si es 0, NaN o inf).
    for i, d in enumerate(laser.values[:NUM_LASER_VALUES]):
        if d is None or math.isnan(d) or math.isinf(d) or d <= 0.0 or d >= d_influence:
            continue
        d = max(d, d_min)
        ang = math.radians(i)
        ox = d * math.sin(ang)
        oy = -d * math.cos(ang)
        #ir ajustando k_rep
        mag = k_rep * (((1.0 / d) - (1.0 / d_influence)) / (d * d))
        rx += mag * (-ox / d)
        ry += mag * (-oy / d)
    
    return rx / NUM_LASER_VALUES, ry / NUM_LASER_VALUES

def update_state(pose, laser):
    #vector repulsión
    row, col = world_to_cell(pose.x, pose.y)
    if (row < 0 or row >= HEIGHT) and (col < 0 or col >= WIDTH):
        return -1
    rep_x, rep_y = repulsion(laser)

#teorema de bresenham para calcular las celdas libres que hay entre
# el punto que estamos y al que nos dirigimos
def get_line_dir(origin, end):
    x0, y0 = origin #col, fila
    x1, y1 = end

    steep = abs(y1 - y0) > abs(x1 - x0)
    if steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1

    swapped = x0 > x1
    if swapped:
        x0, x1 = x1, x0
        y0, y1 = y1, y0

    dx = x1 - x0
    dy = abs(y1 - y0)
    error = dx // 2
    ystep = 1 if y0 < y1 else -1

    y = y0
    points = []
    for x in range(x0, x1 + 1):
        points.append((y, x) if steep else (x, y))
        error -= dy
        if error < 0:
            y += ystep
            error += dx

    if swapped:
        points.reverse()
    return points

#cada haz del láser es una linterna que dice 
#"hasta aquí no hay nada, y justo aquí hay una pared"
def update_map(pose, laser):
    row, col = world_to_cell(pose.x, pose.y)
    if not (0 <= row < HEIGHT and 0 <= col < WIDTH):
        return -1
    punch = False
    for i in range(len(laser.values)):
        #r = rd(laser, i)
        r = laser.values[i]
        if r is None or not math.isfinite(r) or r >= 5.0:
            punch = False
            r = 5.0
        else:
            punch = True
        
        ang = pose.yaw + math.radians(i) - math.pi/2
        target_x = pose.x + r * math.cos(ang)
        target_y = pose.y + r * math.sin(ang)
    
        target_row, target_col = world_to_cell(target_x, target_y)
    
        points = get_line_dir((col, row), (target_col, target_row))
        for px, py in points[:-1]:
            if (0 <= py < HEIGHT and 0 <= px < WIDTH):
                grid[py, px] = max(-L_MAX, min(L_MAX, grid[py, px] + L_BLANK))
                grid_visits[py, px] += 1
        
        end_p_x, end_p_y = points[-1] #if swapped, estamos cogiendo al revés
        if punch and (0 <= end_p_y < HEIGHT and 0 <= end_p_x < WIDTH):
            grid[end_p_y, end_p_x] = max(-L_MAX, min(L_MAX, grid[end_p_y, end_p_x] + L_TAKEN))
            grid_visits[end_p_y, end_p_x] += 1

def show_map():
    img = np.full((HEIGHT, WIDTH), 127, dtype=np.uint8)
    img[grid < -0.5] = 255
    img[grid > 0.5] = 0
    WebGUI.setUserMap(img)

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

map_counter = 0
while True:
    Frequency.tick(IDEAL_RATE)

    pose = HAL.getPose3d()
    laser = HAL.getLaserData()
    if laser is None or len(laser.values) < NUM_LASER_VALUES:
        continue
    
    HAL.setV(0.0)
    HAL.setW(0.3) #probando mapa

    if map_counter % 5 == 0: #~10 Hz
        update_map(pose, laser)
    elif map_counter % 1 == 0: #1 segundo
        show_map()

    if STATE == EXPLORING:
        #si hay que replanificar (evento o timer)
            matrix = classify(grid, grid_visits)
    
    map_counter += 1

