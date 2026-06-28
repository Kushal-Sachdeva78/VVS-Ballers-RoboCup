# ============================================================================
#  goal_cam.py  —  OpenMV H7 forward-facing goal vision for RCJ Lightweight
#  VVS Ballers  |  single H7, NO mirror, looking forward  ->  Teensy 4.1
# ----------------------------------------------------------------------------
#  WHAT THIS DOES
#   The IR ring already gives you the BALL, the 4 ultrasonics give you WALLS,
#   and the line sensor gives you OUT. So this camera does the one job nothing
#   else on the robot can do: find the GOAL, tell the Teensy which way to turn
#   to face it, how far it is, and which CORNER is OPEN to shoot at (dodging the
#   keeper). It does NOT aim by the ball — the IR ring stays your PRIMARY ball
#   sensor — but it now ALSO detects the orange ball and draws its bearing in
#   the IDE overlay (the 2026 Lightweight ball is orange AND emits IR, so the
#   camera can corroborate the IR ring). See the UART note at the bottom for
#   sending the ball bearing to the Teensy for camera+IR fusion.
#
#  REFERENCE PATTERNS (RCJ 2025 TDPs/repos this is built from)
#   - Crestwood Lions : bearing = (cx - center) * (FOV / width)        [goal aim]
#   - Munako Aegis     : RGB565 QVGA, LOCKED exposure/gain/WB, blob+atan2  [stability]
#   - Hyperion         : compact header-byte UART frame to the Teensy   [protocol]
#   - AIR + chaBots    : split the goal at the keeper, aim the OPEN side [dodge]
#   - Reset            : compute the far/open corner from the goal blob  [corner math]
#
#  WIRING (OpenMV H7)
#   UART(3): P4 = TX, P5 = RX.  Connect:
#     OpenMV P4 (TX) ---> Teensy RX   (the data you read on the Teensy)
#     OpenMV P5 (RX) <--- Teensy TX   (optional; not required for this code)
#     OpenMV GND     ---- Teensy GND  (REQUIRED, common ground)
#   Power the H7 from your regulated 3V3/5V rail as in your current build.
#
#  WIRE PROTOCOL  (fixed 9-byte frame, little logic on the camera)
#   byte 0 : 0xAA  header
#   byte 1 : 0x55  header
#   byte 2 : flags  bit0 attackGoalSeen | bit1 ownGoalSeen | bit2 keeperSeen
#   byte 3 : attackBearing   int8  deg, +right of robot's forward axis
#   byte 4 : attackDist      uint8 coarse cm (255 = far/unknown)
#   byte 5 : openCornerBear  int8  deg  <-- THE angle to aim your kick at
#   byte 6 : keeperBearing   int8  deg  (valid only if bit2 set)
#   byte 7 : ownGoalBearing  int8  deg  (handy for orientation/defense)
#   byte 8 : checksum        (sum of bytes 2..7) & 0xFF
#   Teensy: wait for 0xAA,0x55 -> read 6 payload bytes + checksum -> verify.
#   NOTE: orange-ball detection below is OVERLAY-ONLY and does NOT touch this
#   frame, so your current Teensy parser keeps working unchanged.
#
#  TO RUN ON BOOT: save this file onto the OpenMV flash as  main.py
# ============================================================================

import sensor
import time
import math
from pyb import UART, LED

# ----------------------------------------------------------------------------
# 1) CONFIG  — tune the CAPITALISED values for your field/lighting
# ----------------------------------------------------------------------------
DEBUG       = True          # True: draw + print FPS in the IDE. False for matches (faster).
USE_KEEPER  = True          # detect a dark robot in front of the goal and aim around it
DETECT_BALL = True          # also find the orange ball + draw its bearing line in the overlay
ATTACK_GOAL = "yellow"      # which goal you attack this side: "yellow" or "blue"

# --- LAB colour thresholds  (L_min,L_max, A_min,A_max, B_min,B_max) ---
# STARTING POINTS ONLY. Re-tune in OpenMV IDE:
#   Tools -> Machine Vision -> Threshold Editor, under your ARENA lighting.
YELLOW = (18, 35, -5, 9, 41, 18)   # yellow goal: high L, strong +B
BLUE   = (20,  75, -25,  35, -70, -10)   # blue goal:   strong -B
DARK   = ( 0,  35, -20,  20, -20,  20)   # opponent/keeper body (dark)
ORANGE = (8, 66, 13, 46, 69, 0)   # orange ball: bright, +A (red) AND strong +B (yellow)
                                        # GOTCHA: the YELLOW goal also has strong +B. If the goal
                                        # gets read as the ball, push A_min UP (orange is redder
                                        # than yellow) until they separate cleanly under arena light.

# --- camera geometry ---
HFOV_DEG    = 70.0          # H7 stock-lens horizontal FOV (~70 deg). Refine by test.
EXPOSURE_US = 8000          # locked exposure. Raise if dark / lower if washed out.

# --- goal distance (Crestwood focal-length method; coarse, optional) ---
GOAL_WIDTH_CM = 45.0        # real goal width — set from the CURRENT rulebook
FOCAL_PX      = 250.0       # CALIBRATE: place robot dist D cm away, read printed
                            # goal width W px, then FOCAL_PX = W * D / GOAL_WIDTH_CM

# --- blob gating / aiming ---
GOAL_MIN_PIXELS = 50        # ignore goal blobs smaller than this (noise)
KEEPER_MIN_PIXELS = 60      # ignore tiny dark blobs
BALL_MIN_PIXELS = 30        # ignore orange blobs smaller than this (noise)
INSET        = 0.18         # aim this fraction of goal-width inside the chosen corner
FACE_MARGIN  = 25           # px: only pick a corner once the goal is ~in front of us
MIN_OPEN_PX  = 14           # an "open" gap beside the keeper must be at least this wide

# --- frame header ---
HDR0, HDR1 = 0xAA, 0x55

# ----------------------------------------------------------------------------
# 2) SENSOR INIT  — lock everything so colour is stable across the match
# ----------------------------------------------------------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)          # 320 x 240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)                # <- critical for stable thresholds (Aegis)
sensor.set_auto_whitebal(False)            # <- critical
try:
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
except Exception:
    sensor.set_auto_exposure(False)        # older firmware fallback

IMG_W  = sensor.width()
IMG_H  = sensor.height()
IMG_CX = IMG_W / 2.0
DEG_PER_PX = HFOV_DEG / IMG_W

ATTACK_THR, OWN_THR = (YELLOW, BLUE) if ATTACK_GOAL == "yellow" else (BLUE, YELLOW)

uart = UART(3, 115200, timeout_char=1000)
clock = time.clock()
led_ok, led_lost = LED(2), LED(1)          # green = goal seen, red = lost

# ----------------------------------------------------------------------------
# 3) HELPERS
# ----------------------------------------------------------------------------
def bearing_px(x):
    """Image x -> bearing in degrees. +ve means the target is to the ROBOT'S RIGHT."""
    return (x - IMG_CX) * DEG_PER_PX

def largest_blob(img, thr, min_px=GOAL_MIN_PIXELS):
    """Biggest blob matching thr, or None. min_px lets the ball use its own gate."""
    blobs = img.find_blobs([thr], pixels_threshold=min_px,
                           area_threshold=min_px, merge=True, margin=10)
    return max(blobs, key=lambda b: b.pixels()) if blobs else None

def find_keeper(img, goal):
    """Largest DARK blob that horizontally overlaps the goal = the keeper in front of it."""
    if goal is None:
        return None
    gxl, gxr = goal.x(), goal.x() + goal.w()
    best, best_px = None, 0
    for b in img.find_blobs([DARK], pixels_threshold=KEEPER_MIN_PIXELS,
                            area_threshold=KEEPER_MIN_PIXELS, merge=True, margin=5):
        bxl, bxr = b.x(), b.x() + b.w()
        if bxr > gxl and bxl < gxr and b.pixels() > best_px:   # overlaps the goal span
            best, best_px = b, b.pixels()
    return best

def goal_distance_cm(width_px):
    if width_px <= 0:
        return 255
    return (GOAL_WIDTH_CM * FOCAL_PX) / width_px

def open_corner_bearing(goal, keeper):
    """Angle to aim the kick: the OPEN corner (the one we're not aligned with, or the
    side the keeper isn't covering). Built from AIR/chaBots/Reset."""
    gxl = goal.x()
    gxr = goal.x() + goal.w()
    gw  = goal.w()

    # Still turning toward the goal? Aim at its centre first to get it in front of us.
    if not (gxl - FACE_MARGIN <= IMG_CX <= gxr + FACE_MARGIN):
        return bearing_px(goal.cx())

    # Keeper present -> shoot the larger open slice of the goal (split-goal, AIR-style).
    if keeper is not None:
        kxl, kxr = keeper.x(), keeper.x() + keeper.w()
        left_open  = max(0, kxl - gxl)
        right_open = max(0, gxr - kxr)
        if right_open >= left_open and right_open > MIN_OPEN_PX:
            return bearing_px((max(kxr, gxl) + gxr) / 2.0)      # centre of right gap
        elif left_open > MIN_OPEN_PX:
            return bearing_px((gxl + min(kxl, gxr)) / 2.0)      # centre of left gap
        # keeper covers nearly the whole mouth -> nudge to the more-open corner
        return bearing_px(gxr - INSET * gw if right_open >= left_open else gxl + INSET * gw)

    # No keeper -> aim at the corner the robot is NOT aligned with (the farther one),
    # inset slightly so the ball still goes in.
    if abs(gxl - IMG_CX) < abs(gxr - IMG_CX):   # aligned with LEFT corner
        return bearing_px(gxr - INSET * gw)     # -> shoot the RIGHT corner
    else:
        return bearing_px(gxl + INSET * gw)     # -> shoot the LEFT corner

def s8(v):
    """Clamp to signed int8 and return as a raw byte (two's complement)."""
    v = int(round(v))
    v = 127 if v > 127 else (-128 if v < -128 else v)
    return v & 0xFF

def u8(v):
    v = int(round(v))
    return 255 if v > 255 else (0 if v < 0 else v)

def send_frame(flags, a_bear, a_dist, open_bear, k_bear, o_bear):
    payload = [flags & 0xFF, s8(a_bear), u8(a_dist), s8(open_bear), s8(k_bear), s8(o_bear)]
    chk = sum(payload) & 0xFF
    uart.write(bytes([HDR0, HDR1] + payload + [chk]))

# ----------------------------------------------------------------------------
# 4) MAIN LOOP
# ----------------------------------------------------------------------------
while True:
    clock.tick()
    img = sensor.snapshot()

    attack = largest_blob(img, ATTACK_THR)
    own    = largest_blob(img, OWN_THR)
    keeper = find_keeper(img, attack) if USE_KEEPER else None
    ball   = largest_blob(img, ORANGE, BALL_MIN_PIXELS) if DETECT_BALL else None

    attack_seen = attack is not None
    own_seen    = own    is not None
    keeper_seen = keeper is not None
    ball_seen   = ball   is not None
    # ball bearing: +ve = ball is to the ROBOT'S RIGHT (same convention as the goal)
    ball_bear   = bearing_px(ball.cx()) if ball_seen else 0

    if attack_seen:
        a_bear    = bearing_px(attack.cx())
        a_dist    = goal_distance_cm(attack.w())
        open_bear = open_corner_bearing(attack, keeper)
        led_ok.on();  led_lost.off()
    else:
        a_bear, a_dist, open_bear = 0, 255, 0
        led_ok.off(); led_lost.on()

    k_bear = bearing_px(keeper.cx()) if keeper_seen else 0
    o_bear = bearing_px(own.cx())    if own_seen    else 0

    flags = (0x01 if attack_seen else 0) | (0x02 if own_seen else 0) | (0x04 if keeper_seen else 0)
    send_frame(flags, a_bear, a_dist, open_bear, k_bear, o_bear)

    # ---- IDE debug overlay (turn DEBUG off for matches) -------------------
    if DEBUG:
        img.draw_cross(int(IMG_CX), int(IMG_H / 2), color=(255, 255, 255))
        if attack_seen:
            img.draw_rectangle(attack.rect(), color=(255, 255, 0))
            # show the aim line at the open-corner bearing
            ax = int(IMG_CX + (open_bear / DEG_PER_PX))
            img.draw_line(int(IMG_CX), IMG_H, ax, attack.cy(), color=(0, 255, 0), thickness=2)
        if own_seen:
            img.draw_rectangle(own.rect(), color=(0, 128, 255))
        if keeper_seen:
            img.draw_rectangle(keeper.rect(), color=(255, 0, 0))
        if ball_seen:
            # box + centroid on the orange ball
            img.draw_rectangle(ball.rect(), color=(255, 128, 0))
            img.draw_cross(ball.cx(), ball.cy(), color=(255, 128, 0))
            # direction line from the robot's forward origin (bottom-centre) to the ball,
            # so its tilt off vertical == the bearing the Teensy would steer to
            img.draw_line(int(IMG_CX), IMG_H, ball.cx(), ball.cy(), color=(255, 128, 0), thickness=2)
            # angle label in degrees (+ve = right of forward)
            img.draw_string(ball.cx() + 4, ball.cy() - 14, "%d deg" % int(round(ball_bear)),
                            color=(255, 128, 0))
        print("fps=%.0f seen=%d aBear=%.1f open=%.1f dist=%.0f keeper=%d ball=%d bBear=%.1f" %
              (clock.fps(), attack_seen, a_bear, open_bear, a_dist, keeper_seen, ball_seen, ball_bear))

# ============================================================================
#  OPTIONAL UPGRADES (left out to keep the base clean)
#   - Jitter: wrap a_bear/open_bear in a OneEuroFilter before sending (TPA-iPES).
#   - Boot toggle of ATTACK_GOAL with the USR button: from pyb import Switch.
#   - Replace separate find_blobs calls with one call + .code() for a small FPS gain
#     (Hyperion) once your thresholds are locked.
#   - Ball over UART (camera+IR fusion): the camera already computes `ball_bear`. To
#     consume it on the Teensy, add a "ball seen" bit to `flags` (e.g. bit3) and an int8
#     ball-bearing byte to `send_frame`, then EXTEND the checksum range and the Teensy
#     parser to match the new length. Left OFF by default so this file stays drop-in
#     compatible with your current 9-byte parser, with the IR ring still primary.
#  FALLBACK WHEN THE GOAL LEAVES FRAME: that's the Teensy's job — hold the last
#  good heading on the BNO055 (Crestwood/Hyperion/Lovbot Legends all do this) and
#  let the IR ring keep you on the ball.
# ============================================================================
