import math
import random
import Frequency
import HAL
import WebGUI

# Estados
SPIRAL, FORWARD, BACKUP, TURN = range(4)

IDEAL_RATE = 50

# Código secuencial (se ejecuta una vez)
state = SPIRAL
ticks = 0
turn_ticks = 0
turn_dir = 1.0  # +1 izquierda, -1 derecha


def rd(laser, i):
    """Lectura del láser saturada a 5 m y con valores no finitos -> 5 m."""
    r = laser.values[i]
    return min(r, 5.0) if math.isfinite(r) else 5.0


while True:
    Frequency.tick(IDEAL_RATE)

    # Código iterativo
    laser = HAL.getLaserData()

    # Comprobando si los datos son válidos; si no, al siguiente ciclo
    if laser is None or len(laser.values) < 180:
        HAL.setV(0.0)
        HAL.setW(0.0)
        continue

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
