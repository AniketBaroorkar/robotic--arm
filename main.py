"""Apple / banana fixed pickup - vision pipeline and gate logic.

Every gate that can stop the arm reports itself to the UI, so a stalled
pipeline always names the reason instead of failing silently.
"""

import time
from collections import deque

import cv2
import numpy as np
from arduino.app_utils import App, Bridge
from arduino.app_bricks.video_objectdetection import VideoObjectDetection
from arduino.app_bricks.web_ui import WebUI

# ---------------------------------------------------------------- tuning ---
# The brick runs at a permissive floor and Python owns the real threshold,
# so the slider has one source of truth instead of two that drift apart.
BRICK_CONFIDENCE_FLOOR = 0.20
confidence_threshold = 0.40

MINIMUM_COLOR_RATIO = 0.10
REQUIRED_STABLE_DETECTIONS = 3
MAXIMUM_DETECTION_GAP_SECONDS = 1.5
MAXIMUM_POSITION_CHANGE = 60
COMMAND_COOLDOWN_SECONDS = 12.0
PICKUP_TIMEOUT_SECONDS = 45.0

# Colour is advisory until you have seen real ratios in the log. Flip to True
# only once "Colour verified" stops reading unchecked.
REQUIRE_COLOR_MATCH = False

# COCO models call a red apple "sports ball" or "orange" far more often than
# "apple". Mapping them keeps the arm useful; drop them for strictness.
ALLOWED_FRUITS = {"apple": 1, "orange": 1, "sports ball": 1, "banana": 2}

# Fractions of the frame, so changing camera resolution cannot break this.
PICKUP_REGION = {"x_min": 0.05, "x_max": 0.95, "y_min": 0.05, "y_max": 0.95}

FALLBACK_FRAME_WIDTH = 416
FALLBACK_FRAME_HEIGHT = 416

# ----------------------------------------------------------------- state ---
ui = WebUI()
detector = VideoObjectDetection(confidence=BRICK_CONFIDENCE_FLOOR,
                                debounce_sec=0.25, camera_preview=True)

auto_pick_armed = False
robot_busy = False
robot_error = ""
stable_count = 0
previous_x = -1
previous_y = -1
last_detection_time = 0.0
last_command_time = -1e9
busy_since = 0.0
frame_width = FALLBACK_FRAME_WIDTH
frame_height = FALLBACK_FRAME_HEIGHT
frame_source = None
seen_labels = set()


activity_log = deque(maxlen=40)
last_gates = []


def log(text):
    print(text, flush=True)
    entry = {"text": text, "t": time.strftime("%H:%M:%S")}
    activity_log.append(entry)
    ui.send_message("activity", message=entry)


def payload(args, default=None):
    """WebUI hands callbacks (sid, value) or (value,) depending on version."""
    return args[-1] if args else default


# ------------------------------------------------------------ frame grab ---
# on_detect_all() passes ONE argument (the detections dict). There is no frame
# parameter, so the frame has to be pulled off the detector separately.
FRAME_ACCESSORS = ("get_last_frame", "last_frame", "get_frame", "frame",
                   "latest_frame", "get_current_frame", "current_frame",
                   "last_image", "get_last_image")


def to_bgr(value):
    if value is None:
        return None
    if isinstance(value, np.ndarray):
        if value.ndim == 3 and value.shape[2] == 3:
            return value
        if value.ndim == 2:
            return cv2.cvtColor(value, cv2.COLOR_GRAY2BGR)
        return None
    if isinstance(value, (bytes, bytearray, memoryview)):
        try:
            return cv2.imdecode(np.frombuffer(bytes(value), dtype=np.uint8),
                                cv2.IMREAD_COLOR)
        except Exception:
            return None
    return None


def probe_frame_source():
    global frame_source
    log("DETECTOR ATTRIBUTES: " + ", ".join(
        a for a in dir(detector) if not a.startswith("_")))
    for name in FRAME_ACCESSORS:
        if not hasattr(detector, name):
            continue
        try:
            attr = getattr(detector, name)
            if to_bgr(attr() if callable(attr) else attr) is not None:
                frame_source = name
                log(f"Frame source found: detector.{name}")
                return
        except Exception as exc:
            log(f"Frame accessor {name} failed: {exc}")
    log("No frame source. Colour stays unchecked and never blocks the arm.")


def grab_frame():
    if frame_source is None:
        return None
    try:
        attr = getattr(detector, frame_source)
        return to_bgr(attr() if callable(attr) else attr)
    except Exception:
        return None


# ---------------------------------------------------------------- colour ---
def mask_ratio(mask):
    if mask is None or mask.size == 0:
        return 0.0
    return float(cv2.countNonZero(mask)) / float(mask.size)


def verify_colour(image, label, x1, y1, x2, y2):
    """Returns (ok, name, ratio). Unchecked frames pass so they cannot block."""
    if image is None:
        return True, "unchecked", 0.0
    h, w = image.shape[:2]
    ax, ay = max(0, min(x1, w - 1)), max(0, min(y1, h - 1))
    bx, by = max(ax + 1, min(x2, w)), max(ay + 1, min(y2, h))
    crop = image[ay:by, ax:bx]
    if crop.size == 0:
        return True, "unchecked", 0.0

    # Middle 60% only, so background at the box corners cannot outvote the fruit.
    ch, cw = crop.shape[:2]
    crop = crop[int(ch * 0.2):max(int(ch * 0.8), 1), int(cw * 0.2):max(int(cw * 0.8), 1)]
    if crop.size == 0:
        return True, "unchecked", 0.0

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    # Red widened: the old 12-168 hue gap dropped warm-lit apples and S>=80
    # dropped glare-washed highlights.
    red = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 60, 40]), np.array([15, 255, 255])),
        cv2.inRange(hsv, np.array([160, 60, 40]), np.array([179, 255, 255])))
    green = cv2.inRange(hsv, np.array([36, 60, 40]), np.array([90, 255, 255]))
    yellow = cv2.inRange(hsv, np.array([18, 90, 80]), np.array([35, 255, 255]))
    ratios = {"red": mask_ratio(red), "green": mask_ratio(green),
              "yellow": mask_ratio(yellow)}

    if label == "banana":
        return ratios["yellow"] >= MINIMUM_COLOR_RATIO, "yellow", ratios["yellow"]
    name = max(("red", "green"), key=lambda c: ratios[c])
    return ratios[name] >= MINIMUM_COLOR_RATIO, name, ratios[name]


# -------------------------------------------------------------- geometry ---
def region_pixels():
    return {"x_min": PICKUP_REGION["x_min"] * frame_width,
            "x_max": PICKUP_REGION["x_max"] * frame_width,
            "y_min": PICKUP_REGION["y_min"] * frame_height,
            "y_max": PICKUP_REGION["y_max"] * frame_height}


def inside_region(x, y):
    r = region_pixels()
    return r["x_min"] <= x <= r["x_max"] and r["y_min"] <= y <= r["y_max"]


def reset_stability():
    global stable_count, previous_x, previous_y
    stable_count, previous_x, previous_y = 0, -1, -1


# ------------------------------------------------------------- UI output ---
def publish(gates, detection=None):
    global last_gates
    if gates:
        last_gates = gates
    remaining = max(0.0, COMMAND_COOLDOWN_SECONDS - (time.monotonic() - last_command_time))
    ui.send_message("state", message={
        "detection": detection,
        "gates": gates or last_gates,
        "stable": stable_count,
        "required": REQUIRED_STABLE_DETECTIONS,
        "cooldown": round(remaining, 1),
        "armed": auto_pick_armed,
        "busy": robot_busy,
        "error": robot_error,
        "threshold": confidence_threshold,
        "require_colour": REQUIRE_COLOR_MATCH,
        "frame_w": frame_width,
        "frame_h": frame_height,
        "region": region_pixels(),
    })


def gate(name, ok, detail):
    return {"name": name, "ok": bool(ok), "detail": str(detail)}


# ------------------------------------------------------------- controls ---
def on_set_auto_pick(*args):
    global auto_pick_armed
    auto_pick_armed = bool(payload(args, False))
    log(f"Auto-pick {'armed' if auto_pick_armed else 'disarmed'}")
    publish([])


def on_set_confidence(*args):
    global confidence_threshold
    try:
        confidence_threshold = float(payload(args, confidence_threshold))
    except (TypeError, ValueError):
        return
    log(f"Confidence threshold set to {confidence_threshold:.2f}")
    publish([])


def on_send_home(*args):
    global robot_busy, busy_since
    log("Send home requested")
    try:
        Bridge.notify("go_home")
        robot_busy = True
        busy_since = time.monotonic()
    except Exception as exc:
        log(f"Bridge error on go_home: {exc}")


def on_clear_error(*args):
    global robot_busy, robot_error
    robot_busy = False
    robot_error = ""
    reset_stability()
    log("Error cleared, arm marked idle")
    publish([])


def on_hello(*args):
    """Browsers connect after startup, so replay what they missed."""
    ui.send_message("activity_replay", message={"lines": list(activity_log)})
    publish([])


ui.on_message("hello", on_hello)
ui.on_message("set_auto_pick", on_set_auto_pick)
ui.on_message("set_confidence", on_set_confidence)
ui.on_message("send_home", on_send_home)
ui.on_message("clear_error", on_clear_error)


def robot_finished():
    global robot_busy
    robot_busy = False
    reset_stability()
    log("Arm reported finished")


Bridge.provide("robot_finished", robot_finished)


# --------------------------------------------------------------- pipeline ---
def process_detections(detections: dict):
    global robot_busy, robot_error, busy_since, stable_count
    global previous_x, previous_y, last_detection_time, last_command_time
    global frame_width, frame_height

    if not isinstance(detections, dict):
        return

    # Watchdog: without this a lost robot_finished pins the arm busy forever.
    if robot_busy and busy_since and time.monotonic() - busy_since > PICKUP_TIMEOUT_SECONDS:
        robot_busy = False
        robot_error = "Arm never reported finished. Check the sketch and Bridge."
        log(robot_error)

    for raw in detections:
        name = str(raw).strip().lower()
        if name not in seen_labels:
            seen_labels.add(name)
            log(f"Model label seen: '{name}'")

    candidates = []
    for raw, objects in detections.items():
        label = str(raw).strip().lower()
        if label not in ALLOWED_FRUITS or not isinstance(objects, list):
            continue
        for d in objects:
            if not isinstance(d, dict):
                continue
            try:
                conf = float(d.get("confidence", 0.0))
            except (TypeError, ValueError):
                continue
            box = d.get("bounding_box_xyxy")
            if box is not None and len(box) == 4:
                candidates.append((label, conf, box))

    if not candidates:
        if not robot_busy:
            reset_stability()
        publish([gate("Fruit recognised", False, "nothing in view")])
        return

    label, confidence, box = max(candidates, key=lambda t: t[1])
    try:
        x1, y1, x2, y2 = [int(round(float(v))) for v in box]
    except (TypeError, ValueError):
        return
    if x2 <= x1 or y2 <= y1:
        log(f"Ignoring malformed box {box}")
        return

    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

    image = grab_frame()
    if image is not None:
        frame_height, frame_width = image.shape[:2]

    confident = confidence >= confidence_threshold
    colour_ok, colour, ratio = verify_colour(image, label, x1, y1, x2, y2)
    positioned = inside_region(cx, cy)

    detection = {"label": label, "confidence": confidence, "colour": colour,
                 "ratio": ratio, "cx": cx, "cy": cy,
                 "box": [x1, y1, x2, y2]}

    r = region_pixels()
    print(f"{label} {confidence:.2f} box=({x1},{y1},{x2},{y2}) centre=({cx},{cy}) "
          f"frame={frame_width}x{frame_height} "
          f"region=x[{r['x_min']:.0f},{r['x_max']:.0f}] y[{r['y_min']:.0f},{r['y_max']:.0f}] "
          f"colour={colour} {ratio:.2f} inside={positioned}", flush=True)

    colour_detail = "unchecked" if colour == "unchecked" else f"{colour} {ratio * 100:.0f}%"
    colour_gate = colour_ok if REQUIRE_COLOR_MATCH else True

    def gates_now():
        if robot_busy:
            arm_detail, arm_ok = "arm busy", False
        elif not auto_pick_armed:
            arm_detail, arm_ok = "disarmed", False
        else:
            arm_detail, arm_ok = "armed and idle", True
        return [
            gate("Fruit recognised", True, label),
            gate("Confidence over threshold", confident,
                 f"{confidence:.2f} / {confidence_threshold:.2f}"),
            gate("Colour verified", colour_gate,
                 colour_detail + ("" if REQUIRE_COLOR_MATCH else " (advisory)")),
            gate("Centre inside pickup region", positioned,
                 f"{'inside' if positioned else 'outside'} ({cx},{cy})"),
            gate("Held still", stable_count >= REQUIRED_STABLE_DETECTIONS,
                 f"{stable_count} / {REQUIRED_STABLE_DETECTIONS}"),
            gate("Auto-pick armed and arm idle", arm_ok, arm_detail),
        ]

    if robot_busy or not confident:
        if not confident:
            reset_stability()
        publish(gates_now(), detection)
        return

    if REQUIRE_COLOR_MATCH and not colour_ok:
        reset_stability()
        publish(gates_now(), detection)
        return

    if not positioned:
        reset_stability()
        publish(gates_now(), detection)
        return

    now = time.monotonic()
    moved = 0 if previous_x < 0 else abs(cx - previous_x) + abs(cy - previous_y)
    if (previous_x < 0 or moved > MAXIMUM_POSITION_CHANGE
            or now - last_detection_time > MAXIMUM_DETECTION_GAP_SECONDS):
        stable_count = 1
    else:
        stable_count += 1
    previous_x, previous_y, last_detection_time = cx, cy, now

    ready = (stable_count >= REQUIRED_STABLE_DETECTIONS
             and auto_pick_armed
             and now - last_command_time >= COMMAND_COOLDOWN_SECONDS)
    publish(gates_now(), detection)
    if not ready:
        return

    robot_busy = True
    busy_since = now
    stable_count = 0
    last_command_time = now
    try:
        log(f"Picking {label} at ({cx}, {cy})")
        Bridge.notify("pick_fruit", ALLOWED_FRUITS[label], cx, cy)
    except Exception as exc:
        robot_busy = False
        robot_error = f"Bridge error: {exc}"
        log(robot_error)


detector.on_detect_all(process_detections)
probe_frame_source()
log("Pipeline started. Arm the auto-pick toggle when the poses are calibrated.")
App.run()
