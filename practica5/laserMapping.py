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

def cell_to_world(row, col):
    mx, my = WebGUI.mapToPose(col, row, 0))
    return int(round(my)), int(round(mx))

def norm_angle(a):
    return atan2(sin(a), cos(a))

def sector_min(laser, i0, i1):
    #mínimo de las lecturas válidas entre los índices i0..i1
    vals = [d for d in laser.values[i0:i1+1] if math.isinf(d) and d > 0]
    if vals:
        return min(vals)
    else:
        return 5.0

#marcar como no transitable toda celda a menos que el radio del robot de una celda OCCUPIED
#engordamos la velda ocupada
def inflate(mask, r):
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r: #disco de radio r
                out |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out

def is_frontier(cell, matrix):
    if cell == FREE:
        x, y = cell
        near_x = [R_ROBOT, -R_ROBOT, 0, -R_ROBOT, 0, R_ROBOT]
        near_y = [R_ROBOT, -R_ROBOT, R_ROBOT, 0, -R_ROBOT, 0]
        count = 0
        for i in range(len(near_x)):
            x = start_x + near_x[i]
            y = start_y + near_y[i]
            if matrix[y][x] == UNKNOWN:
                count += 1
        if count > 0: #encontrar algún vecino UNKNOWN
            return True
    else:
        return False

def in_blacklist(cell, blacklist):
    cx, cy = cell
    for bx, by in blacklist:
        if cx == bx and cy == by:
            return True
    return False

def plan(pose, blacklist):
    matrix = classify(grid, grid_visits)
    occupied = (matrix == OCCUPIED)
    inflated = inflate(occupied, R_ROBOT) #radio del robot, cómo se saca?
    transitable = (matrix == FREE) & ~inflated

    start_x, start, y = world_to_cell(pose.x, pose.y)
    if not transitable[start]:
        near_x = [R_ROBOT, -R_ROBOT, 0, -R_ROBOT, 0, R_ROBOT]
        near_y = [R_ROBOT, -R_ROBOT, R_ROBOT, 0, -R_ROBOT, 0]
        for i in range(len(near_x)):
            x = start_x + near_x[i]
            y = start_y + near_y[i]
            if transitable[(x, y)]:
                non_start = False
                start = (x, y)
            else:
                non_start = True
                continue
        if non_start:
            return None
    
    lista = [start]
    inicio = {start: None}
    while lista:
        cell = lista.pop_front()
        if is_frontier(cell, matrix) and not in_blacklist(cell, blacklist):
            target = cell
            break
        for cada vecino n de cell (8 direcciones):
            if grid[n] is not None and transitable[n] is not None and n not in inicio:
                inicio[n] = cell
                lista.push_back(n)
    
    if target is None:
        return None

    path = []
    c = target
    while c is not None:
        path.append(c)
        c = lista[c]
    path.reverse()

    cells_wp = make_cells_wp(path, 10)

    waypoints = [cell_to_world(r, c) for (r, co) in cells_wp]
    tgt = cell_to_world(target)
    return (waypoints, tgt)



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

# ---------- estado ----------
SEED, EXPLORING, FIN = 0, 1, 2
state = SEED
seed_rot = 0.0
last_yaw = None

waypoints = []
wp_idx = 0
target = None
blacklist = []
need_replan = True
last_plan_t = 0.0
no_target = 0

stuck_ref = None
stuck_t = 0.0
tick = 0

while True:
    Frequency.tick(IDEAL_RATE)

    pose = HAL.getPose3d()
    laser = HAL.getLaserData()
    if laser is None or len(laser.values) < NUM_LASER_VALUES:
        continue
    now = time.time()
    
    if tick % 5 == 0: #~10 Hz
        update_map(pose, laser)
    elif tick % 1 == 0: #1 segundo
        show_map()
    tick += 1

    #girar 360 grados para limpiar la zona de origen
    if state == SEED:
        if last_yaw is None:
            last_yaw = pose.yaw
        seed_rot += abs(norm_angle(pose.yaw - last_yaw))
        last_yaw = pose.yaw
        HAL.setV(0.0)
        HAL.setW(SEED_W)
        if seed_rot >= 2 * math.pi:
            state = EXPLORING
            need_replan = True
        continue
    if state == FIN:
        HAL.setV(0.0)
        HAL.setW(0.0)
        continue
    #state == EXPLORING
    if not need_replan and now - last_plan_t > REPLAN_PERIOD:
        need_replan = True
    
    if need_replan:
        HAL.setV(0.0)
        HAL.setW(0.0)
        update_map(pose, laser)
        result = plan(pose, blacklist)
        last_plan_t = time.time()
        need_replan = False

        if result is None:
            no_target += 1
            waypoints = []
            target = None
            if no_target >= 3:
                state = FIN
                show_map()
                continue
            last_plan_t = last_plan_t - REPLAN_PERIOD + 1.5 #reintento en 1.5 s
        else:
            wps, tgt = result #waypoints, target
            if len(wps) < 2: #ya estamos en la frontera
                blacklist.append(tgt)
                need_replan = True
                continue
            no_target = 0
            waypoints = wps
            target = tgt
            wp_idx = 1
            stuck_ref = (pose.x, pose.y)
            stuck_t = last_plan_t
    
    #sin ruta: girar despacio mientras se reintenta
    if not waypoints:
        HAL.setV(0.0)
        HAL.setW(0.5)
        continue
    
    #waypoint alcanzado
    wx, wy = waypoints[wp_idx]
    dist = math.hypot(wx - pose.x, wy - pose.y)
    if dist < WP_TOL:
        wp_idx += 1
        if wp_idx >= len(waypoints): #objetivo alcanzado
            blacklist.append(target)
            waypoints = []
            need_replan = True
            HAL.setV(0.0)
            HAL.setW(0.0)
            continue
        wx, wy = waypoints[wp_idx]
    
    # atasco
    if now - stuck_t > STUCK_TIME:
        moved = math.hypot(pose.x - stuck_ref[0], pose.y - stuck_ref[1])
        if moved < STUCK_DIST:
            blacklist.append(target)
            waypoints = []
            need_replan = True
            continue
        stuck_ref = (pose.x, pose.y)
        stuck_t = now

    # controlador
    err = norm_angle(math.atan2(wy - pose.y, wx - pose.x) - pose.yaw)
    front = sector_min(laser, 70, 110)
    rx, ry = repulsion(laser)

    if front < FRONT_STOP: #seguridad local
        left = sector_min(laser, 120, 170)
        right = sector_min(laser, 10, 60)
        v = 0.0
        w = 0.8 if left >= right else -0.8
        if now - last_plan_t > 1.5:
            need_replan = True
    elif abs(err) > ANG_BIG: #girar en el sitio
        v = 0.0
        w = KP_W * err
    else: #avanzar corrigiendo rumbo
        v = V_STRAIGHT if abs(err) < 0.15 else V_TURN
        v *= max(0.0, 1.0 - abs(err) / ANG_BIG)
        if front < 0.8:
            v *= front / 0.8
        v = max(v, 0.2 * V_TURN)
        w = KP_W * err + K_LAT * ry

    w = max(-W_MAX, min(W_MAX, w))
    HAL.setV(v)
    HAL.setW(w)

