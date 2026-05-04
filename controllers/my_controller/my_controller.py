from controller import Supervisor, Motion
import math

TIME_STEP          = 32
BALL_STOP_DISTANCE = 0.6
OBSTACLE_THRESHOLD = 0.50
CLEAR_THRESHOLD    = 0.65

robot      = Supervisor()
robot_node = robot.getSelf()

# ─── Motions ──────────────────────────────────────────────────────────────────
forwards   = Motion("motions/Forwards.motion")
turn_left  = Motion("motions/TurnLeft40.motion")
turn_right = Motion("motions/TurnRight40.motion")
side_left  = Motion("motions/SideStepLeft.motion")
side_right = Motion("motions/SideStepRight.motion")

current_motion = None

def play(m):
    global current_motion
    if current_motion is m:
        # Let motion finish one cycle before replaying
        if not m.isOver():
            return
    if current_motion is not None:
        current_motion.stop()
    m.play()
    current_motion = m

def stop_all():
    global current_motion
    if current_motion:
        current_motion.stop()
        current_motion = None

# ─── Sonars ───────────────────────────────────────────────────────────────────
sonar_l = robot.getDevice("Sonar/Left")
sonar_r = robot.getDevice("Sonar/Right")
sonar_l.enable(TIME_STEP)
sonar_r.enable(TIME_STEP)

# ─── Supervisor helpers ───────────────────────────────────────────────────────
def get_pos(node):
    return node.getField("translation").getSFVec3f()

def get_ball_node():
    return robot.getFromDef("ball")

# ─── Heading tracker ──────────────────────────────────────────────────────────
# Instead of reading orientation (which doesn't update mid-motion),
# we track heading ourselves by counting turns applied.
# NAO starts facing +X (toward goal) based on your scene.
heading = 0.0          # radians, 0 = facing +X world axis
TURN_STEP = math.radians(40)   # TurnLeft40 / TurnRight40 = 40 degrees per motion

def robot_forward_vec():
    """Unit vector in world XZ for current tracked heading."""
    return math.cos(heading), math.sin(heading)

def angle_to_ball():
    """
    Returns signed angle to turn to face ball.
    Positive = need to turn LEFT, Negative = need to turn RIGHT.
    Uses world positions + tracked heading.
    """
    rpos = get_pos(robot_node)
    bpos = get_pos(get_ball_node())

    dx = bpos[0] - rpos[0]
    dz = bpos[2] - rpos[2]

    # Angle of ball in world frame
    ball_world_angle = math.atan2(dz, dx)

    # Difference from our current heading
    diff = ball_world_angle - heading
    # Normalise to [-π, π]
    diff = (diff + math.pi) % (2 * math.pi) - math.pi
    return diff

def dist_to_ball():
    rpos = get_pos(robot_node)
    bpos = get_pos(get_ball_node())
    dx = bpos[0] - rpos[0]
    dz = bpos[2] - rpos[2]
    return math.sqrt(dx*dx + dz*dz)

# ─── FSM ──────────────────────────────────────────────────────────────────────
STATE_ALIGN = "ALIGN"
STATE_WALK  = "WALK"
STATE_AVOID = "AVOID"
STATE_STOP  = "STOP"

state = STATE_ALIGN

# Track whether a turn motion is in progress and if we've updated heading
turn_in_progress = False
turn_direction   = 0   # +1 = left, -1 = right

print(f"[NAO] Start — heading={math.degrees(heading):.1f}°")

while robot.step(TIME_STEP) != -1:

    sl = sonar_l.getValue()
    sr = sonar_r.getValue()
    obstacle_close = sl < OBSTACLE_THRESHOLD or sr < OBSTACLE_THRESHOLD
    obstacle_clear = sl > CLEAR_THRESHOLD and sr > CLEAR_THRESHOLD

    dist  = dist_to_ball()
    angle = angle_to_ball()

    # ── Detect when a turn motion finishes and update tracked heading ──────────
    if turn_in_progress and current_motion is not None and current_motion.isOver():
        heading += turn_direction * TURN_STEP
        heading  = (heading + math.pi) % (2 * math.pi) - math.pi
        turn_in_progress = False
        print(f"[NAO] Turn done — new heading={math.degrees(heading):.1f}°  angle_to_ball={math.degrees(angle_to_ball()):.1f}°")

    # ── ALIGN ──────────────────────────────────────────────────────────────────
    if state == STATE_ALIGN:

        if dist <= BALL_STOP_DISTANCE:
            state = STATE_STOP

        elif obstacle_close:
            state = STATE_AVOID
            print(f"[NAO] ALIGN → AVOID")

        elif abs(angle) <= math.radians(20) and not turn_in_progress:
            state = STATE_WALK
            print(f"[NAO] ALIGN → WALK  angle={math.degrees(angle):.1f}°")

        elif not turn_in_progress:
            if angle > 0:
                print(f"[NAO] Turn LEFT  angle={math.degrees(angle):.1f}°")
                play(turn_left)
                turn_direction   = +1
                turn_in_progress = True
            else:
                print(f"[NAO] Turn RIGHT angle={math.degrees(angle):.1f}°")
                play(turn_right)
                turn_direction   = -1
                turn_in_progress = True

    # ── WALK ───────────────────────────────────────────────────────────────────
    elif state == STATE_WALK:

        if dist <= BALL_STOP_DISTANCE:
            state = STATE_STOP
            print(f"[NAO] WALK → STOP")

        elif obstacle_close:
            state = STATE_AVOID
            print(f"[NAO] WALK → AVOID  L={sl:.2f} R={sr:.2f}")

        elif abs(angle) > math.radians(25):
            state = STATE_ALIGN
            print(f"[NAO] WALK → ALIGN  angle={math.degrees(angle):.1f}°")

        else:
            play(forwards)

    # ── AVOID ──────────────────────────────────────────────────────────────────
    elif state == STATE_AVOID:

        if dist <= BALL_STOP_DISTANCE:
            state = STATE_STOP

        elif obstacle_clear:
            state = STATE_ALIGN
            print(f"[NAO] AVOID → ALIGN")

        else:
            if sl < sr:
                play(side_right)
                print(f"[NAO] AVOID strafe RIGHT  L={sl:.2f}")
            else:
                play(side_left)
                print(f"[NAO] AVOID strafe LEFT   R={sr:.2f}")

    # ── STOP ───────────────────────────────────────────────────────────────────
    elif state == STATE_STOP:
        stop_all()
        print(f"[NAO] DONE — Ball reached! dist={dist:.2f}m")
        break