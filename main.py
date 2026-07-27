"""
main.py — Fruit Sorting Robot (Arduino App Lab)
================================================
Flow:
 1. Fruit is staged in a WHITE BOX at a fixed, known position.
 2. The camera (video_object_detection brick) identifies WHICH fruit
    it is: apple / banana / orange. Everything else is ignored.
 3. The arm runs a fixed pick sequence: rotate base to the white box,
    bend down using servo 2 onwards (shoulder/elbow/wrist — the base
    only rotates), close gripper, lift.
 4. It rotates to the COLORED box matching the fruit
    (apple → Red, banana → Yellow, orange → Orange), lowers,
    releases, returns home. Repeats forever.
 5. Web page (port 7000): fruit, color, coordinates, confidence,
    counts, log, and E-STOP buttons.
The MCU sketch (sketch/sketch.ino) owns the PCA9685 and does the
smooth servo interpolation; this file only sends target poses over
Bridge RPC and waits for completion.
"""
import threading
import time
from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.video_objectdetection import VideoObjectDetection
# ======================================================================
# CALIBRATION — tune every angle here on YOUR arm before trusting it.
# Servo order everywhere: [base, shoulder, elbow, wrist_pitch,
#                          wrist_rotate, gripper]  (PCA9685 ch 0..5)
# ======================================================================
GRIPPER_OPEN = 30.0      # tune: fully open, fruit fits between jaws
GRIPPER_CLOSED = 80.0    # tune: firm grip on the fruit, don't stall servo
WRIST_ROTATE = 90.0      # kept constant in this simple flow
# Base rotation pointing at the WHITE pickup box:
WHITE_BOX_BASE_ANGLE = 90.0
# "Bend from servo 2 onwards": base holds its angle, these three do the
# bending. Calibrate by jogging the arm down onto a fruit in the white
# box and recording the angles. START SHALLOW and increase gradually.
PICK_SHOULDER = 45.0
PICK_ELBOW = 135.0
PICK_WRIST = 90.0
# Upright "carrying" pose used while rotating between boxes:
CARRY_SHOULDER = 90.0
CARRY_ELBOW = 90.0
CARRY_WRIST = 90.0
# Base rotation for each COLORED destination box. Make these three
# clearly different angles, and different from WHITE_BOX_BASE_ANGLE.
BOX_BASE_ANGLE = {
   "apple": 30.0,    # Red box
   "banana": 60.0,   # Yellow box
   "orange": 150.0,  # Orange box
}
# How far to bend when dropping into a colored box (shallower than the
# pick bend so the gripper clears the box wall):
DROP_SHOULDER = 60.0
DROP_ELBOW = 115.0
DROP_WRIST = 90.0
# Detection behaviour:
FRUIT_COLOR = {"apple": "Red", "banana": "Yellow", "orange": "Orange"}
TARGET_FRUITS = set(FRUIT_COLOR)
CONFIDENCE_MIN = 0.40  # lowered for first-detection debugging; raise to
                       # 0.55+ once detection is confirmed working
STABLE_HITS = 3       # same fruit must be seen this many frames in a row
COOLDOWN_S = 5.0      # wait after a sort before picking again
MOVE_TIMEOUT_S = 12.0
POLL_S = 0.1          # Bridge poll rate (10 Hz — well under the ~20 Hz limit)
GRIP_SETTLE_S = 0.4
HOME = [90.0, 90.0, 90.0, 90.0, 90.0, GRIPPER_OPEN]
# ======================================================================
# BRICKS + STATE
# ======================================================================
ui = WebUI()
detector = VideoObjectDetection(confidence=CONFIDENCE_MIN, debounce_sec=0.0)
_busy = threading.Event()
_last_pick_done = 0.0
_stable_label = None
_stable_count = 0
_counts = {"apple": 0, "banana": 0, "orange": 0}
_state = "idle"
_printed_sample = False
# Heartbeat / camera-watchdog state: lets the web page's LOG panel show
# whether camera frames are actually flowing from the detection brick.
_frames = 0
_last_frame_ts = 0.0
_last_beat_ts = 0.0
_last_labels = []

def log(msg: str) -> None:
   """Print to the App Lab console AND mirror to the web page's log
   panel. The UI send is best-effort: before a browser has connected
   (or while the WebUI brick is still starting) it must never be able
   to crash the app."""
   print(msg, flush=True)
   try:
       ui.send_message("log", {"line": msg})
   except Exception:
       pass

def send_status(target=None, coords=None, conf=None) -> None:
   try:
       ui.send_message("status", {
           "state": _state,
           "target": target,
           "color": FRUIT_COLOR.get(target) if target else None,
           "coords": coords,
           "confidence": conf,
           "counts": _counts,
       })
   except Exception:
       pass  # best-effort, same reasoning as log()

# ======================================================================
# MOTION (via Bridge RPC to sketch.ino)
# ======================================================================
def move_and_wait(base, shoulder, elbow, wrist, wrot, grip) -> None:
   """Send one full pose to the MCU, then poll until it reports the
   smooth interpolation has finished. Raises on E-STOP or timeout."""
   ok = Bridge.call("move_all", float(base), float(shoulder), float(elbow),
                    float(wrist), float(wrot), float(grip))
   if not ok:
       raise RuntimeError("MCU rejected move (E-STOP active?)")
   t0 = time.monotonic()
   while not Bridge.call("is_motion_complete"):
       if time.monotonic() - t0 > MOVE_TIMEOUT_S:
           raise RuntimeError("Motion timeout — check servo power / sketch")
       time.sleep(POLL_S)

def pick_and_place(fruit: str, coords, conf: float) -> None:
   """Full choreographed cycle: white box -> grip -> colored box -> home.
   Runs in its own thread so detection callbacks are never blocked."""
   global _state, _last_pick_done
   try:
       _state = f"picking {fruit}"
       send_status(fruit, coords, conf)
       log(f"[PICK] {fruit} ({FRUIT_COLOR[fruit]}) at {coords}, conf={conf:.2f}")
       b = WHITE_BOX_BASE_ANGLE
       # 1) Face the white box, arm upright, gripper open
       move_and_wait(b, CARRY_SHOULDER, CARRY_ELBOW, CARRY_WRIST, WRIST_ROTATE, GRIPPER_OPEN)
       # 2) Bend down — base holds, servo 2 onwards does the bending
       move_and_wait(b, PICK_SHOULDER, PICK_ELBOW, PICK_WRIST, WRIST_ROTATE, GRIPPER_OPEN)
       # 3) Close gripper
       move_and_wait(b, PICK_SHOULDER, PICK_ELBOW, PICK_WRIST, WRIST_ROTATE, GRIPPER_CLOSED)
       time.sleep(GRIP_SETTLE_S)
       # 4) Lift back to carry height
       move_and_wait(b, CARRY_SHOULDER, CARRY_ELBOW, CARRY_WRIST, WRIST_ROTATE, GRIPPER_CLOSED)
       # 5) Rotate to the matching colored box
       _state = f"placing {fruit}"
       send_status(fruit, coords, conf)
       box = BOX_BASE_ANGLE[fruit]
       move_and_wait(box, CARRY_SHOULDER, CARRY_ELBOW, CARRY_WRIST, WRIST_ROTATE, GRIPPER_CLOSED)
       # 6) Lower into the box
       move_and_wait(box, DROP_SHOULDER, DROP_ELBOW, DROP_WRIST, WRIST_ROTATE, GRIPPER_CLOSED)
       # 7) Release
       move_and_wait(box, DROP_SHOULDER, DROP_ELBOW, DROP_WRIST, WRIST_ROTATE, GRIPPER_OPEN)
       time.sleep(GRIP_SETTLE_S)
       # 8) Lift clear, then home
       move_and_wait(box, CARRY_SHOULDER, CARRY_ELBOW, CARRY_WRIST, WRIST_ROTATE, GRIPPER_OPEN)
       move_and_wait(*HOME)
       _counts[fruit] += 1
       log(f"[DONE] {fruit} -> {FRUIT_COLOR[fruit]} box (total: {_counts[fruit]})")
   except Exception as exc:  # noqa: BLE001 — top-level guard for the cycle
       log(f"[ERROR] pick cycle failed: {exc}")
       try:
           move_and_wait(*HOME)
       except Exception as exc2:  # noqa: BLE001
           log(f"[ERROR] recovery to home also failed: {exc2}")
   finally:
       _state = "idle"
       _last_pick_done = time.monotonic()
       send_status()
       _busy.clear()

# ======================================================================
# DETECTION HANDLING
# ======================================================================
def on_detections(results) -> None:
   """Called by the brick for every processed frame. Payload format:
   {"apple": [{"confidence": 0.9, "bounding_box_xyxy": (x1,y1,x2,y2)}], ...}
   The per-label value is ALWAYS a list. The whole body is guarded so
   an unexpected payload can never silently kill the detection stream."""
   global _stable_label, _stable_count, _printed_sample
   global _frames, _last_frame_ts, _last_beat_ts, _last_labels
   try:
       # Heartbeat: proves the camera + brick are delivering frames,
       # and shows what the model is recognizing (if anything).
       _frames += 1
       now = time.monotonic()
       _last_frame_ts = now
       if results:
           _last_labels = sorted(str(k).lower() for k in results.keys())
       if now - _last_beat_ts > 5.0:
           _last_beat_ts = now
           seen = ", ".join(_last_labels) if _last_labels else "nothing recognized yet"
           log(f"[CAMERA OK] frames: {_frames} | model last saw: {seen}")
       if not _printed_sample and results:
           _printed_sample = True
           log(f"sample_detections: {results}")  # verify your model's labels
       best = None
       ui_items = []
       for label, instances in (results or {}).items():
           lab = str(label).lower()
           if lab not in TARGET_FRUITS:
               continue  # ignore every non-fruit object outright
           for inst in instances:
               conf = float(inst.get("confidence", 0.0))
               x1, y1, x2, y2 = inst.get("bounding_box_xyxy", (0, 0, 0, 0))
               cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
               ui_items.append({
                   "fruit": lab, "color": FRUIT_COLOR[lab],
                   "confidence": round(conf, 2), "cx": cx, "cy": cy,
                   "box": [int(x1), int(y1), int(x2), int(y2)],
               })
               if best is None or conf > best[2]:
                   best = (lab, (cx, cy), conf)
       try:
           ui.send_message("detections", {"items": ui_items})
       except Exception:
           pass
       if best is None:
           _stable_label, _stable_count = None, 0
           return
       label, coords, conf = best
       if label == _stable_label:
           _stable_count += 1
       else:
           _stable_label, _stable_count = label, 1
       # Trigger conditions: stable detection, arm idle, cooldown over
       if _busy.is_set():
           return
       if time.monotonic() - _last_pick_done < COOLDOWN_S:
           return
       if _stable_count < STABLE_HITS:
           return
       _busy.set()
       _stable_label, _stable_count = None, 0
       threading.Thread(target=pick_and_place, args=(label, coords, conf),
                        daemon=True).start()
   except Exception as exc:  # noqa: BLE001
       print(f"[WARN] detection callback error: {exc}", flush=True)

# ======================================================================
# WEB PAGE CONTROLS
# ======================================================================
def on_estop(client, data=None):
   try:
       Bridge.call("estop_trigger")
       log("[E-STOP] triggered from web page")
   except Exception as exc:  # noqa: BLE001
       log(f"[ERROR] E-STOP call failed: {exc}")

def on_estop_reset(client, data=None):
   try:
       Bridge.call("estop_reset")
       log("[E-STOP] reset from web page")
   except Exception as exc:  # noqa: BLE001
       log(f"[ERROR] E-STOP reset failed: {exc}")

def on_go_home(client, data=None):
   if not _busy.is_set():
       def _home():
           try:
               move_and_wait(*HOME)
           except Exception as exc:  # noqa: BLE001
               log(f"[ERROR] manual home failed: {exc}")
       threading.Thread(target=_home, daemon=True).start()
       log("[HOME] manual home requested from web page")

ui.on_message("estop", on_estop)
ui.on_message("estop_reset", on_estop_reset)
ui.on_message("go_home", on_go_home)
detector.on_detect_all(on_detections)

def _camera_watchdog() -> None:
   """If the detection brick never delivers frames (camera missing,
   brick container failed), say so in the LOG panel instead of the
   system just sitting silent."""
   warned_at = 0.0
   while True:
       time.sleep(5.0)
       now = time.monotonic()
       if _frames == 0 or now - _last_frame_ts > 10.0:
           if now - warned_at > 15.0:
               warned_at = now
               log("[CAMERA?] no frames from the detection brick — camera "
                   "not detected or brick failed. Check: lsusb, "
                   "ls /dev/video*, and the App launch log for errors.")

threading.Thread(target=_camera_watchdog, daemon=True).start()
# The MCU sketch homes all six servos to 90 deg by itself at boot, so no
# Bridge call is needed here before App.run() starts the event loop.
log("Fruit sorter started. Waiting for a fruit in the white box...")
send_status()
App.run()  # blocks forever, keeps bricks + callbacks alive
