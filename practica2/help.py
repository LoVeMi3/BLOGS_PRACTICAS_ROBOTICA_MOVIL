import WebGUI
import HAL
import Frequency

import numpy as np
import cv2
import time

FOLLOW, LOST = range(2)
IDEAL_RATE = 50

# --- Velocidades ---
V_MAX = 18
V_MIN = 8
V_SEARCH = 6
W_SEARCH = 3.0
ACCEL = 0.15          # lo que puede subir v por ciclo (frenar es instantáneo)

# --- Control ---
KP_STRAIGHT, KD_STRAIGHT = 0.7, 0.5
KP_TURN, KD_TURN = 1.8, 0.7
ALPHA = 0.6

# --- Visión ---
MIN_AREA = 60
FAR_TOP = 0.25        # hasta dónde miramos (más bajo = más lejos)
FAR_BOT = 0.60
NEAR_TOP = 0.60
FAR_FULL_BRAKE = 0.45 # |far| a partir del cual frena al máximo
SIGNIFICANT = 0.15

state = FOLLOW
prev_error = 0.0
last_dir = 1
last_w = 0.0
w_filt = 0.0
v_cur = V_SEARCH


def get_errors(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    m1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
    m2 = cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
    mask = cv2.bitwise_or(m1, m2)

    def cx_of(band):
        M = cv2.moments(band, binaryImage=True)
        if M["m00"] < MIN_AREA:
            return None
        return (M["m10"] / M["m00"] - w / 2) / (w / 2)

    near = cx_of(mask[int(h * NEAR_TOP):, :])
    far = cx_of(mask[int(h * FAR_TOP):int(h * FAR_BOT), :])
    return near, far, mask


while True:
    Frequency.tick(IDEAL_RATE)

    img = HAL.getImage()
    if img is None:
        HAL.setV(0)
        HAL.setW(0)
        continue

    near, far, mask = get_errors(img)
    WebGUI.showImage(mask)

    error = near if near is not None else far

    if error is not None:
        state = FOLLOW

        if near is not None and far is not None:
            error = 0.6 * near + 0.4 * far

        # Curvatura: 0 recta, 1 curva fuerte
        if far is None:
            curv = 1.0
        else:
            curv = min(1.0, abs(far) / FAR_FULL_BRAKE)
        curv = max(curv, min(1.0, abs(error) / FAR_FULL_BRAKE))

        v_target = V_MAX - (V_MAX - V_MIN) * curv
        # frena al instante, acelera con rampa
        if v_target < v_cur:
            v_cur = v_target
        else:
            v_cur = min(v_target, v_cur + ACCEL)

        kp = KP_STRAIGHT + (KP_TURN - KP_STRAIGHT) * curv
        kd = KD_STRAIGHT + (KD_TURN - KD_STRAIGHT) * curv

        w_cmd = -(kp * error + kd * (error - prev_error))
        w_filt = ALPHA * w_cmd + (1 - ALPHA) * w_filt

        HAL.setV(v_cur)
        HAL.setW(w_filt)

        prev_error = error
        last_w = w_filt
        if abs(error) > SIGNIFICANT:
            last_dir = -1 if error < 0 else 1

    else:
        state = LOST
        v_cur = V_SEARCH
        HAL.setV(V_SEARCH)
        HAL.setW(-max(abs(last_w), W_SEARCH) * last_dir)
