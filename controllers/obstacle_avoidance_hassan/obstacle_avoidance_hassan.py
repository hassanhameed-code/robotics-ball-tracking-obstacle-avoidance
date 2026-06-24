"""
Webots Robot Controller — Simple Obstacle Avoidance
- Blue box  → turn left  (obstacle)
- Red box   → turn right (obstacle)
- Duck      → turn left  (obstacle)
- Ball      → approach / stop
- Default   → go forward
"""

from controller import Robot
import cv2
import numpy as np

# ── Configuration ────────────────────────────────────────────────────
SPEED     = 7.0
TIME_STEP = 64

# ── HSV colour ranges ────────────────────────────────────────────────
BLUE_LOWER  = np.array([100, 150,  50]);  BLUE_UPPER  = np.array([130, 255, 255])
RED_LOWER1  = np.array([  0, 150,  80]);  RED_UPPER1  = np.array([ 10, 255, 255])
RED_LOWER2  = np.array([170, 150,  80]);  RED_UPPER2  = np.array([180, 255, 255])
DUCK_LOWER  = np.array([ 20, 100, 100]);  DUCK_UPPER  = np.array([ 35, 255, 255])

# FIFA soccer ball — white patches and black patches
# Lowered thresholds to detect ball on green field
BALL_W_LO   = np.array([  0,   0, 200]);  BALL_W_HI   = np.array([180,  30, 255])
BALL_B_LO   = np.array([  0,   0,   0]);  BALL_B_HI   = np.array([180, 255,  40])

# ── Thresholds ───────────────────────────────────────────────────────
BLUE_OBSTACLE_THRESHOLD = 0.38   # Blue needs more coverage before turning left
RED_OBSTACLE_THRESHOLD  = 0.38   # Red needs more coverage before turning right
DUCK_OBSTACLE_THRESHOLD = 0.09  # Duck (yellow) — unchanged
BALL_THRESHOLD          = 0.01   # Lowered — detect ball even when small/far
BALL_CLOSE              = 0.15   # Coverage % to stop at ball


# ── Vision helpers ────────────────────────────────────────────────────
def get_frame(camera, w, h):
    raw = camera.getImage()
    if raw is None:
        return None
    img = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 4))
    return img[:, :, :3]

def coverage(hsv, lo, hi, w, h):
    mask = cv2.inRange(hsv, lo, hi)
    return cv2.countNonZero(mask) / (w * h)

def sense(camera, w, h):
    bgr = get_frame(camera, w, h)
    if bgr is None:
        return 0.0, 0.0, 0.0, 0.0
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    blue = coverage(hsv, BLUE_LOWER, BLUE_UPPER, w, h)
    red  = (coverage(hsv, RED_LOWER1, RED_UPPER1, w, h) +
            coverage(hsv, RED_LOWER2, RED_UPPER2, w, h))
    duck = coverage(hsv, DUCK_LOWER, DUCK_UPPER, w, h)

    wh   = coverage(hsv, BALL_W_LO, BALL_W_HI, w, h)
    bk   = coverage(hsv, BALL_B_LO, BALL_B_HI, w, h)

    # Lowered conditions — detect ball even when partially visible
    ball = (wh + bk) / 2.0 if (wh > 0.02 and bk > 0.01) else 0.0

    return blue, red, duck, ball


# ── Simple control logic ──────────────────────────────────────────────
def obstacle_avoidance(blue_cov, red_cov, duck_cov, ball_cov):
    """
    Simple rule-based obstacle avoidance.
    Motor convention:
      forward    → (+SPEED, +SPEED)
      turn left  → (-SPEED, +SPEED)   pivot
      turn right → (+SPEED, -SPEED)   pivot
      stop       → (0, 0)
    """

    # Priority 1: Handle ball detection
    if ball_cov > BALL_THRESHOLD:
        if ball_cov > BALL_CLOSE:   # Ball is close → stop
            return 0.0, 0.0
        else:                       # Ball is far/medium → approach
            return SPEED, SPEED

    # Priority 2: Avoid obstacles
    if blue_cov > BLUE_OBSTACLE_THRESHOLD:
        # Blue box detected → turn left
        return -SPEED, SPEED

    if red_cov > RED_OBSTACLE_THRESHOLD:
        # Red box detected → turn right
        return SPEED, -SPEED

    if duck_cov > DUCK_OBSTACLE_THRESHOLD:
        # Duck detected → turn left
        return -SPEED * 0.7, SPEED * 0.7

    # Default: go forward
    return SPEED, SPEED


# ── Main ──────────────────────────────────────────────────────────────
def main():
    robot = Robot()

    left_motor  = robot.getDevice("left_motor")
    right_motor = robot.getDevice("right_motor")
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

    left_sensor  = robot.getDevice("left wheel sensor")
    right_sensor = robot.getDevice("right wheel sensor")
    left_sensor.enable(TIME_STEP)
    right_sensor.enable(TIME_STEP)

    camera = robot.getDevice("camera")
    camera.enable(TIME_STEP)
    cam_w = camera.getWidth()
    cam_h = camera.getHeight()

    print("[INFO] Obstacle Avoidance Controller BY: Hassan Dar , starting...")

    ball_stopped = False

    while robot.step(TIME_STEP) != -1:
        blue, red, duck, ball = sense(camera, cam_w, cam_h)
        l_spd, r_spd = obstacle_avoidance(blue, red, duck, ball)
        left_motor.setVelocity(l_spd)
        right_motor.setVelocity(r_spd)

        if l_spd == 0.0 and r_spd == 0.0 and ball > 0.0 and not ball_stopped:
            print("Ball found - yay!")
            ball_stopped = True


if __name__ == "__main__":
    main()