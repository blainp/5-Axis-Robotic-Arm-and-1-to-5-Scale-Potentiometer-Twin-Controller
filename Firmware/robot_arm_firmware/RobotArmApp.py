import tkinter as tk
import serial
import threading
import time
import queue
from collections import deque

# ─── CONFIG ───────────────────────────────────────────────────────────────────
POT_PORT = "COM5"   # Potentiometer control-arm Arduino
ARM_PORT = "COM4"   # Real robot arm Arduino
BAUD = 115200

LIMIT = 90            # target range is -LIMIT..+LIMIT for the 4 stepper joints
TOLERANCE = 1.5       # degrees - arm stops driving a joint once within this of target
MANUAL_STEP_DEG = 1.0 # degrees per tick a held manual button moves the target

# Potentiometer smoothing (median rejects spike glitches from noisy wiring,
# EMA irons out general jitter, slew-limit stops the target from jumping)
POT_MEDIAN_WINDOW = 3
POT_EMA_ALPHA = 0.25
MAX_TARGET_STEP_DEG = 1.2

CONTROL_TICK_MS = 50   # how often targets get updated from whichever mode is active

# ── Physical wiring: 4 real stepper joints ───────────────────────────────────
# There are 5 motors total on the arm (Base, Shoulder, Aux, Elbow steppers +
# the MG996R wrist-rotation servo) and 5 potentiometers on the control arm,
# one per motor. MAP/MOTOR_MULT only cover the 4 steppers; the wrist is
# handled separately below since it's a servo, not a stepper.
MAP        = [1, 3, 0, 4]   # ui joint index -> physical stepper index on the arm Arduino
MOTOR_MULT = [1, 1, 1, 1]

POT_ANGLE_RANGE = 180   # pot arduino reports 0-180 for the 4 joint pots, so 90 = center

# ── Wrist rotation potentiometer ─────────────────────────────────────────────
# This is potentiometer channel index 4 (the 5th value in each pot-arduino
# line) - previously left unused / miswired into a phantom stepper joint.
# The pot itself sweeps a wider mechanical/electrical range than the servo
# can move, so we linearly rescale it onto the servo's 0-180 range rather
# than clamping (clamping is what made it seem like turning the knob "did
# nothing" past a certain point - it was pinned at 0 or 180 the whole time).
#
# If the wrist doesn't reach its full range, or reaches it too early/late,
# tune WRIST_POT_MIN / WRIST_POT_MAX to match what you observe by printing
# raw pot values - they don't have to be exactly 0-270, just whatever the
# knob's real usable electrical sweep is.
WRIST_POT_MIN   = 0
WRIST_POT_MAX   = 270
WRIST_SERVO_MIN = 0
WRIST_SERVO_MAX = 180


def scale_wrist(raw_pot_deg):
    """Linearly map a wrist-pot reading (WRIST_POT_MIN..WRIST_POT_MAX) onto
    the servo's WRIST_SERVO_MIN..WRIST_SERVO_MAX range, clamped at the ends."""
    span_in = WRIST_POT_MAX - WRIST_POT_MIN
    span_out = WRIST_SERVO_MAX - WRIST_SERVO_MIN
    pct = (raw_pot_deg - WRIST_POT_MIN) / span_in
    pct = max(0.0, min(1.0, pct))
    return WRIST_SERVO_MIN + pct * span_out


class PotReader(threading.Thread):
    """Background reader for the potentiometer arm: median filter (rejects
    single-sample spikes from noisy connections) + EMA low-pass (irons out
    jitter), pushed onto a queue for the main thread. Channels 0-3 are the
    4 stepper joints; channel 4 is the dedicated wrist-rotation pot."""

    def __init__(self, port, baud, data_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.data_queue = data_queue
        self.stop_flag = threading.Event()
        self._history = [deque(maxlen=POT_MEDIAN_WINDOW) for _ in range(5)]
        self._ema = [POT_ANGLE_RANGE / 2] * 4 + [(WRIST_POT_MIN + WRIST_POT_MAX) / 2]

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
        except serial.SerialException as e:
            self.data_queue.put(("error", f"pot port: {e}"))
            return

        time.sleep(2)
        ser.reset_input_buffer()

        while not self.stop_flag.is_set():
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException as e:
                self.data_queue.put(("error", f"pot port: {e}"))
                break

            if not line:
                continue
            parts = line.split(",")
            if len(parts) != 6:
                continue
            try:
                raw = [float(p) for p in parts[:5]]
                switch_state = int(parts[5])
            except ValueError:
                continue

            smoothed = []
            for i in range(5):
                self._history[i].append(raw[i])
                median_val = sorted(self._history[i])[len(self._history[i]) // 2]
                self._ema[i] += POT_EMA_ALPHA * (median_val - self._ema[i])
                smoothed.append(self._ema[i])

            self.data_queue.put(("pot", smoothed, switch_state))

        ser.close()

    def stop(self):
        self.stop_flag.set()


class MasterArmController:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm Master Controller")

        # ── Shared state (4 real stepper joints) ─────────────────────────────
        self.target_angles  = [0.0] * 4   # what we want the arm to move to
        self.current_angles = [0.0] * 4   # what the arm reports it's actually at
        self.motor_cmds     = [0] * 4
        self.extra_cmds     = [90, 0]     # [wrist rotation (absolute 0-180), claw]

        self.mode = "manual"   # starts in manual button control, per your request
        self.manual_dir  = [0] * 4   # -1/0/1 per joint, from held +/- buttons

        # inv_map: physical stepper index -> ui joint index, for reading positions back
        self.inv_map = {}
        for ui_idx, phys_idx in enumerate(MAP):
            self.inv_map[phys_idx] = ui_idx

        # ── Serial: real arm ─────────────────────────────────────────────────
        try:
            self.ser_arm = serial.Serial(ARM_PORT, BAUD, timeout=0.1)
            time.sleep(2)
            self.ser_arm.reset_input_buffer()
            print(f"Connected to arm on {ARM_PORT}")
        except Exception as e:
            print(f"ERROR: could not open arm port {ARM_PORT}: {e}")
            self.ser_arm = None

        # ── Serial: potentiometer arm (background reader) ───────────────────
        self.pot_queue = queue.Queue()
        self.pot_reader = PotReader(POT_PORT, BAUD, self.pot_queue)
        self.pot_reader.start()

        # ── UI ────────────────────────────────────────────────────────────────
        self.build_ui()

        # ── Bindings ──────────────────────────────────────────────────────────
        self.root.bind("<KeyPress-space>", self.emergency_stop)

        # ── Background loops ──────────────────────────────────────────────────
        threading.Thread(target=self.comms_loop, daemon=True).start()
        self.control_loop()
        self.refresh_labels()

    # ── UI construction ───────────────────────────────────────────────────────
    def build_ui(self):
        top = tk.Frame(self.root)
        top.pack(padx=10, pady=8, fill="x")

        self.mode_btn = tk.Button(
            top, text="Mode: MANUAL (click for Potentiometer)",
            bg="#2980b9", fg="white", font=("Arial", 11, "bold"),
            command=self.toggle_mode
        )
        self.mode_btn.pack(side="left")

        self.status_label = tk.Label(top, text="", font=("Arial", 9), fg="#555")
        self.status_label.pack(side="left", padx=15)

        joints = tk.LabelFrame(self.root, text="Joints (hold to move, manual mode only)")
        joints.pack(padx=10, pady=6, fill="x")

        # Only the 4 real stepper joints - the wrist is handled separately below
        self.joint_names = ["Base", "Shoulder", "Aux", "Elbow"]
        self.joint_labels = []
        self.manual_buttons = []

        for i, name in enumerate(self.joint_names):
            row = tk.Frame(joints)
            row.pack(pady=3, fill="x")

            tk.Label(row, text=name, width=12, anchor="w",
                     font=("Arial", 10, "bold")).pack(side="left")

            minus = tk.Button(row, text="-", width=4)
            minus.bind("<ButtonPress-1>",   lambda e, idx=i: self.set_manual_dir(idx, -1))
            minus.bind("<ButtonRelease-1>", lambda e, idx=i: self.set_manual_dir(idx, 0))
            minus.pack(side="left", padx=2)

            plus = tk.Button(row, text="+", width=4)
            plus.bind("<ButtonPress-1>",   lambda e, idx=i: self.set_manual_dir(idx, 1))
            plus.bind("<ButtonRelease-1>", lambda e, idx=i: self.set_manual_dir(idx, 0))
            plus.pack(side="left", padx=2)

            self.manual_buttons.extend([minus, plus])

            lbl = tk.Label(row, text="target: 0.0°   actual: 0.0°",
                           font=("Courier", 9), fg="#333")
            lbl.pack(side="left", padx=10)
            self.joint_labels.append(lbl)

        extra = tk.LabelFrame(self.root, text="Wrist Rotation & Claw")
        extra.pack(padx=10, pady=6, fill="x")

        # Wrist rotation is now always live from its own dedicated potentiometer,
        # regardless of manual/pot mode - it's a direct analog dial, not a
        # button-driven joint, so there's nothing to click here.
        self.wrist_label = tk.Label(extra, text="Wrist rotation: --- ° (live from potentiometer)")
        self.wrist_label.grid(row=0, column=0, columnspan=2, pady=(4, 2))

        tk.Label(extra, text="Claw (manual mode only):").grid(row=2, column=0, columnspan=2, pady=(8, 2))
        self.claw_open_btn = tk.Button(extra, text="Open", width=8)
        self.claw_open_btn.bind("<ButtonPress-1>",   lambda e: self.set_extra(1, 1))
        self.claw_open_btn.bind("<ButtonRelease-1>", lambda e: self.set_extra(1, 0))
        self.claw_open_btn.grid(row=3, column=0, padx=4, pady=2)

        self.claw_close_btn = tk.Button(extra, text="Close", width=8)
        self.claw_close_btn.bind("<ButtonPress-1>",   lambda e: self.set_extra(1, -1))
        self.claw_close_btn.bind("<ButtonRelease-1>", lambda e: self.set_extra(1, 0))
        self.claw_close_btn.grid(row=3, column=1, padx=4, pady=2)

        tk.Label(self.root, text="SPACE = emergency stop (zeroes joint targets + claw)",
                 font=("Arial", 9), fg="#e74c3c").pack(pady=(0, 8))

        self.set_manual_widgets_state("normal")  # starts in manual mode

    def set_manual_widgets_state(self, state):
        for btn in self.manual_buttons:
            btn.config(state=state)
        self.claw_open_btn.config(state=state)
        self.claw_close_btn.config(state=state)

    # ── Mode switching ────────────────────────────────────────────────────────
    def toggle_mode(self):
        if self.mode == "manual":
            self.mode = "pot"
            self.mode_btn.config(text="Mode: POTENTIOMETER (click for Manual)", bg="#8e44ad")
            self.manual_dir = [0] * 4
            self.set_manual_widgets_state("disabled")
        else:
            self.mode = "manual"
            self.mode_btn.config(text="Mode: MANUAL (click for Potentiometer)", bg="#2980b9")
            self.set_manual_widgets_state("normal")

    def emergency_stop(self, event=None):
        self.target_angles = [0.0] * 4
        self.extra_cmds[1] = 0
        self.manual_dir = [0] * 4
        self.status_label.config(text="EMERGENCY STOP - joint targets reset", fg="#e74c3c")
        # Note: wrist rotation is not reset here - it's a live analog passthrough
        # from the potentiometer and will simply reflect wherever the knob is
        # sitting on the very next control tick.

    # ── Manual control ────────────────────────────────────────────────────────
    def set_manual_dir(self, idx, val):
        self.manual_dir[idx] = val

    def set_extra(self, idx, val):
        self.extra_cmds[idx] = val

    # ── Control loop (mode-aware target update) ──────────────────────────────
    def control_loop(self):
        latest_pot = None
        try:
            while True:
                item = self.pot_queue.get_nowait()
                if item[0] == "pot":
                    latest_pot = item
                elif item[0] == "error":
                    self.status_label.config(text=f"POT SERIAL ERROR: {item[1]}", fg="#e74c3c")
        except queue.Empty:
            pass

        # Wrist rotation always tracks its dedicated potentiometer live,
        # independent of manual/pot mode.
        if latest_pot:
            _, smoothed, _switch_state = latest_pot
            self.extra_cmds[0] = int(round(scale_wrist(smoothed[4])))

        if self.mode == "pot" and latest_pot:
            _, smoothed, switch_state = latest_pot
            # 1. Steppers (4 real joints)
            for i in range(4):
                desired = max(-LIMIT, min(LIMIT, smoothed[i] - (POT_ANGLE_RANGE / 2)))
                current = self.target_angles[i]
                step = max(-MAX_TARGET_STEP_DEG, min(MAX_TARGET_STEP_DEG, desired - current))
                self.target_angles[i] = current + step

            # 2. Claw (via switch)
            self.extra_cmds[1] = -1 if switch_state else 1

        elif self.mode == "manual":
            for i in range(4):
                if self.manual_dir[i] != 0:
                    self.target_angles[i] = max(
                        -LIMIT, min(LIMIT, self.target_angles[i] + self.manual_dir[i] * MANUAL_STEP_DEG)
                    )

        self.root.after(CONTROL_TICK_MS, self.control_loop)

    # ── Serial comms with the real arm (closed-loop position control) ───────
    def comms_loop(self):
        while True:
            if self.ser_arm and self.ser_arm.is_open:
                for i in range(4):
                    diff = self.target_angles[i] - self.current_angles[i]
                    if   diff >  TOLERANCE: self.motor_cmds[i] =  1
                    elif diff < -TOLERANCE: self.motor_cmds[i] = -1
                    else:                   self.motor_cmds[i] =  0

                final = [0] * 5   # 5 physical stepper slots on the arm Arduino
                for i in range(4):
                    final[MAP[i]] = int(self.motor_cmds[i] * MOTOR_MULT[i])

                cmd = f"<{','.join(map(str, final))},{self.extra_cmds[0]},{self.extra_cmds[1]}>"

                try:
                    self.ser_arm.write(cmd.encode())
                except Exception as e:
                    print(f"WRITE ERR: {e}")

                try:
                    line = self.ser_arm.readline().decode().strip()
                    if line and "," in line:
                        vals = line.split(",")
                        if len(vals) >= 5:
                            new_angles = [0.0] * 4
                            for phys, ui in self.inv_map.items():
                                new_angles[ui] = float(vals[phys])
                            self.current_angles = new_angles
                except Exception:
                    pass

            time.sleep(0.05)

    # ── UI refresh ────────────────────────────────────────────────────────────
    def refresh_labels(self):
        for i in range(4):
            self.joint_labels[i].config(
                text=f"target: {self.target_angles[i]:6.1f}°   actual: {self.current_angles[i]:6.1f}°"
            )
        self.wrist_label.config(text=f"Wrist rotation: {self.extra_cmds[0]:3d}° (live from potentiometer)")
        if "ERROR" not in self.status_label.cget("text") and "STOP" not in self.status_label.cget("text"):
            self.status_label.config(text=f"Mode: {self.mode}", fg="#555")
        self.root.after(100, self.refresh_labels)

    def on_close(self):
        self.pot_reader.stop()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = MasterArmController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()