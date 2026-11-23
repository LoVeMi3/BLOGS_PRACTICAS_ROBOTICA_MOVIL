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
d_influence = 2.3
d_min = 0.05

RES = 0.1 #metros por celda
HEIGHT = 970
WIDTH = 1500
#el (0,0) del mundo cae en el centro de la matriz
grid = np.zeros((HEIGHT, WIDTH)) #0 = log-odds, de p = 0.5
L_MAX = 5.0

BLANK = 0.4
L_BLANK = math.log(BLANK/ (1.0 - BLANK)) #log(p / (1.0 - p)
TAKEN = 0.6
L_TAKEN = math.log(TAKEN/ (1.0 - TAKEN))
SUS = 0.5 #prior

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

def repulsion(laser, p_y):
    rx = 0
    ry = 0
    #Distancia válida del láser (inf si es 0, NaN o inf).
    for i, d in enumerate(laser.values[:NUM_LASER_VALUES]):
        if d is None or math.isnan(d) or math.isinf(d) or d <= 0.0:
            continue
        d = max(d, d_min)
        ox = d*sin(ang)
        oy = -d*cos(ang)
        #ir ajustando k_rep
        mag = k_rep * (((1.0 / d) - (1.0 / d_influence)) / (d * d))
        rx += mag * (-ox / d)
        ry += mag * (-oy / d)
    
    return rx / NUM_LASER_VALUES, ry / NUM_LASER_VALUES

def update_state(pose, laser):
    #vector repulsión
    row, col = world_to_cell(pose.x, pose.y)
    if row < 0 or row >= HEIGHT and col < 0 or col >= WIDTH:
        return -1
    rep_x, rep_y = repulsion(laser, pose.yaw)

#teorema de bresenham para calcular las celdas libres que hay entre
# el punto que estamos y al que nos dirigimos
def get_line_dir(origin, end):
    og_x, og_y = origin
    end_x, end_y = end
    dx = end_x - og_x
    dy = end_y - og_y
    
    #pendiente de la línea a seguir
    steep = abs(dy) > abs(dx)
    if steep:
        og_x, og_y = og_y, og_x #rotandooo
        end_x, end_y = end_y, end_x

    swapped = x1 > x2
    if swapped:
        og_x, end_x = end_x, og_x
        og_y, end_y = end_y2, og_y
    
    #recalculamos
    dx = end_x - og_x
    dy = abs(end_y - og_y)
    
    error = int(dx / 2.0)
    ystep = 1 if og_y < end_y else -1
    
    #iteramos
    y = og_y1
    points = []
    for x in range(x1, end_x + 1):
        coord = (y, x) if steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    
    return points

#cada haz del láser es una linterna que dice 
#"hasta aquí no hay nada, y justo aquí hay una pared"
def update_map(pose, laser):
    row, col = world_to_cell(pose.x, pose.y)
    if row < 0 or row >= HEIGHT and col < 0 or col >= WIDTH:
        return -1
    punch = False
    for i in range(len(laser.values)):
        #r = rd(laser, i)
        r = laser.values[i]
        if not math.isfinite(r) or r is None or r >= 5.0:
            punch = False
            r = 5.0
        else:
            punch = True
        
        ang = pose.yaw + math.radians(i) - math.pi/2
        target_x = pose.x + r * math.cos(ang)
        target_y = pose.y + r * math.sin(ang)
    
        target_col, target_row = world_to_cell(target_x, target_y)
    
        points = get_line_dir(pose, (target_col, target_row))
        for py, px in points[:-1]:
            if py < 0 or py >= HEIGHT and px < 0 or px >= WIDTH:
                grid[py, px] = max(-L_MAX, min(L_MAX, grid[py, px] + L_BLANK))
        
        end_point = points[-1]
        if punch and (py < 0 or py >= HEIGHT and px < 0 or px >= WIDTH):
            grid[end_point] = max(-L_MAX, min(L_MAX, grid[end_point] + L_TAKEN))

def show_map():
    img = np.full((H, W), 127, dtype=np.uint8)
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

    if map_counter % 5 == 0:# ~10 Hz
        update_map(pose, laser)
        show_map()
    
    map_counter += 1

