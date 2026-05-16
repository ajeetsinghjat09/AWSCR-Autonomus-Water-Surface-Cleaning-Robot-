# =============================================================================
#  WSC-R  —  Autonomous Water Surface Cleaning Robot
#  MicroPython for ESP32 (38-pin)
#  Author : Ajeet Singh Jaat, Kanishk Choubey, Vansh Namdev — SATI Vidisha
#  NOTE   : MPU-6050 / IMU removed — test build (sensors: HC-SR04 + VL53L0X)
# =============================================================================
#
#  PIN MAP
#  ─────────────────────────────────────────────────────
#  VL53L0X    SDA=GPIO17  SCL=GPIO16  (Software I2C,   addr 0x29, 3.3V)
#  HC-SR04    TRIG=GPIO5  ECHO=GPIO18 (5V VCC, voltage divider on ECHO)
#  L298N-A    ENA=GPIO13  IN1=GPIO12  IN2=GPIO14   (Port 775 motor)
#  L298N-B    ENA=GPIO33  IN1=GPIO32  IN2=GPIO23   (Starboard 775 motor)
#  CONVEYOR   ENA=GPIO25  IN1=GPIO26  IN2→GND (forward-only, 12V)
#
#  POWER
#  ─────
#  12V Li-ion → L298N VCC rails + DC-DC buck → 5V → ESP32 VIN / HC-SR04
#  All GNDs star-tied near battery
# =============================================================================

import machine
import time
import math
from machine import Pin, PWM, SoftI2C

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

# Motor PWM frequency (Hz) — L298N works well at 1–5 kHz
PWM_FREQ = 2000
# Max PWM duty (0–1023 on MicroPython ESP32)
MAX_DUTY = 1023

# Speed presets (0.0–1.0 fraction of MAX_DUTY)
SPEED_FULL  = 0.85
SPEED_TURN  = 0.55   # slower side during differential steering turn
SPEED_STOP  = 0.0

# Ultrasonic obstacle thresholds (cm)
OBSTACLE_STOP_CM = 30   # emergency stop
OBSTACLE_TURN_CM = 80   # start evasive turn

# VL53L0X debris detection threshold (mm)
DEBRIS_THRESHOLD_MM = 650

# Boustrophedon track width (m) — matches effective cleaning swath
TRACK_SPACING_M = 0.30

# Conveyor runs only when robot moves forward
CONVEYOR_FORWARD_ONLY = True

# ─────────────────────────────────────────────────────────────────────────────
#  HARDWARE INITIALISATION
# ─────────────────────────────────────────────────────────────────────────────

# ── Software I2C for VL53L0X ─────────────────────────────────────────────────
i2c_tof = SoftI2C(scl=Pin(16), sda=Pin(17), freq=400_000)

# ── HC-SR04 ultrasonic ───────────────────────────────────────────────────────
trig = Pin(5, Pin.OUT)
echo = Pin(18, Pin.IN)

# ── L298N-A — Port 775 motor ─────────────────────────────────────────────────
pwm_a = PWM(Pin(13), freq=PWM_FREQ, duty=0)
in1_a = Pin(12, Pin.OUT)
in2_a = Pin(14, Pin.OUT)

# ── L298N-B — Starboard 775 motor ────────────────────────────────────────────
pwm_b = PWM(Pin(33), freq=PWM_FREQ, duty=0)
in1_b = Pin(32, Pin.OUT)
in2_b = Pin(23, Pin.OUT)

# ── L298N-Conv — Conveyor belt (forward only) ─────────────────────────────────
pwm_conv = PWM(Pin(25), freq=PWM_FREQ, duty=0)
in1_conv = Pin(26, Pin.OUT)
in2_conv = Pin(27, Pin.OUT)   # tie to GND in hardware; drive LOW in SW too

# Fix conveyor direction permanently to FORWARD
in1_conv.value(1)
in2_conv.value(0)


# ─────────────────────────────────────────────────────────────────────────────
#  MOTOR HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def _set_motor(pwm, in1, in2, speed):
    """
    Set one L298N channel.
    speed > 0 → forward
    speed < 0 → reverse
    speed = 0 → brake (both IN low)
    """
    duty = int(abs(speed) * MAX_DUTY)
    duty = max(0, min(MAX_DUTY, duty))
    if speed > 0:
        in1.value(1)
        in2.value(0)
    elif speed < 0:
        in1.value(0)
        in2.value(1)
    else:
        in1.value(0)
        in2.value(0)
    pwm.duty(duty)


def drive(left_speed, right_speed):
    """
    Drive both 775 motors.
    Positive = forward, Negative = reverse, 0 = stop.
    Automatically manages conveyor: on when both moving forward.
    """
    _set_motor(pwm_a, in1_a, in2_a, left_speed)
    _set_motor(pwm_b, in1_b, in2_b, right_speed)
    # Conveyor only runs when both wheels drive forward
    if CONVEYOR_FORWARD_ONLY and left_speed > 0 and right_speed > 0:
        conveyor_on()
    else:
        conveyor_off()


def stop_all():
    """Emergency stop all motors including conveyor."""
    _set_motor(pwm_a, in1_a, in2_a, 0)
    _set_motor(pwm_b, in1_b, in2_b, 0)
    conveyor_off()


def conveyor_on(speed=0.75):
    pwm_conv.duty(int(speed * MAX_DUTY))


def conveyor_off():
    pwm_conv.duty(0)


# ─────────────────────────────────────────────────────────────────────────────
#  HC-SR04 ULTRASONIC
# ─────────────────────────────────────────────────────────────────────────────

def measure_distance_cm(timeout_us=30000):
    """
    Returns distance in cm, or None if no echo within timeout.
    Voltage divider MUST be on Echo line (5V → 3.3V).
    """
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)

    # Wait for echo high
    t0 = time.ticks_us()
    while echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), t0) > timeout_us:
            return None

    # Measure echo duration
    t1 = time.ticks_us()
    while echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), t1) > timeout_us:
            return None
    t2 = time.ticks_us()

    duration = time.ticks_diff(t2, t1)
    distance = (duration * 0.0343) / 2   # sound speed 343 m/s → 0.0343 cm/µs
    return distance


# ─────────────────────────────────────────────────────────────────────────────
#  VL53L0X TIME-OF-FLIGHT  (Software I2C — address 0x29)
# ─────────────────────────────────────────────────────────────────────────────
TOF_ADDR = 0x29

def tof_write(reg, data):
    i2c_tof.writeto_mem(TOF_ADDR, reg, bytes([data]))

def tof_read8(reg):
    return i2c_tof.readfrom_mem(TOF_ADDR, reg, 1)[0]

def tof_read16(reg):
    d = i2c_tof.readfrom_mem(TOF_ADDR, reg, 2)
    return (d[0] << 8) | d[1]

def tof_init():
    """
    Minimal VL53L0X init sequence.
    Full calibration improves accuracy; this is field-ready default.
    """
    try:
        model = tof_read8(0xC0)
        if model != 0xEE:
            print(f"[VL53L0X] Unexpected model {model:#x} — check wiring")
            return False
        # Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks
        tof_write(0x60, 0x00)
        tof_write(0x46, 0x25)
        # Set timing budget to ~33 ms (default)
        tof_write(0x80, 0x01)
        tof_write(0xFF, 0x01)
        tof_write(0x00, 0x00)
        tof_write(0x91, 0x3C)   # stop_variable
        tof_write(0x00, 0x01)
        tof_write(0xFF, 0x00)
        tof_write(0x80, 0x00)
        print("[VL53L0X] Initialised on GPIO17/16 (SoftI2C)")
        return True
    except OSError as e:
        print(f"[VL53L0X] Init failed: {e}")
        return False

def tof_read_mm():
    """
    Single ranging measurement.
    Returns distance in mm, or None on timeout / error.
    """
    try:
        # Start single-shot ranging
        tof_write(0x80, 0x01)
        tof_write(0xFF, 0x01)
        tof_write(0x00, 0x00)
        tof_write(0x91, 0x3C)
        tof_write(0x00, 0x01)
        tof_write(0xFF, 0x00)
        tof_write(0x80, 0x00)
        tof_write(0x00, 0x01)   # sysrange_start

        # Wait for result ready (bit 0 of 0x13)
        t0 = time.ticks_ms()
        while (tof_read8(0x13) & 0x07) == 0:
            if time.ticks_diff(time.ticks_ms(), t0) > 200:
                return None
            time.sleep_ms(1)

        # Read range (mm) and clear interrupt
        dist = tof_read16(0x1E)
        tof_write(0x0B, 0x01)   # clear interrupt
        return dist
    except OSError:
        return None


def debris_detected():
    """
    Detection rule: TRUE if d_ToF < d_threshold AND signal is valid.
    (Pitch correction removed — no IMU available in this build.)
    """
    d = tof_read_mm()
    if d is None or d == 0 or d > 8000:   # sensor reports 8190 on no-return
        return False
    return d < DEBRIS_THRESHOLD_MM


# ─────────────────────────────────────────────────────────────────────────────
#  NAVIGATION — BOUSTROPHEDON STATE MACHINE
# ─────────────────────────────────────────────────────────────────────────────
#
#  States:
#   0 — NORMAL_FORWARD   (CPP track traversal)
#   1 — OBSTACLE_EVADE   (HC-SR04 triggered)
#   2 — TRACK_REVERSAL   (end of Boustrophedon lane)
#   3 — DEBRIS_COLLECT   (VL53L0X triggered, conveyor active)

STATE_FORWARD = 0
STATE_EVADE   = 1
STATE_REVERSE = 2
STATE_DEBRIS  = 3

state_names = {
    STATE_FORWARD: "NORMAL_FORWARD",
    STATE_EVADE:   "OBSTACLE_EVADE",
    STATE_REVERSE: "TRACK_REVERSAL",
    STATE_DEBRIS:  "DEBRIS_COLLECT",
}


class Robot:
    def __init__(self):
        self.state = STATE_FORWARD

        # Track-reversal counter (how many lanes completed)
        self.lane = 0
        # Max lanes before mission complete (set per deployment)
        self.max_lanes = 20

        # Evasion direction alternates to prevent looping
        self._evade_dir = 1   # +1 right, -1 left
        self._evade_step = 0
        self._evade_steps_total = 4   # × loop period

        # Debris dwell: hold collect state for N cycles
        self._debris_dwell = 0
        self.DEBRIS_DWELL_CYCLES = 15   # ~1.5 s at 10 Hz

        # Simple yaw tracking via gyro-less lane counter
        self.heading_deg = 0.0

        print("[Robot] Initialised. Starting in NORMAL_FORWARD.")
        print("[Robot] IMU disabled — running on HC-SR04 + VL53L0X only.")

    # ──────────────────────────────────────────────────────────────────────────
    #  MAIN LOOP  (~10 Hz update rate)
    # ──────────────────────────────────────────────────────────────────────────
    def run(self):
        tof_ok = tof_init()

        print("[Robot] Mission started — Boustrophedon CPP")
        print(f"[Robot] ToF sensor: {'OK' if tof_ok else 'FAILED (check wiring)'}")

        LOOP_MS = 100   # 10 Hz

        while True:
            t_start = time.ticks_ms()

            # ── 1. Read ultrasonic ────────────────────────────────────────────
            dist_cm = measure_distance_cm()

            # ── 2. Read ToF ───────────────────────────────────────────────────
            debris = debris_detected() if tof_ok else False

            # ── 3. State machine ──────────────────────────────────────────────
            self._update_state(dist_cm, debris)
            self._execute_state(dist_cm, debris)

            # ── 4. Telemetry ──────────────────────────────────────────────────
            print(
                f"[{state_names[self.state]:<16}] "
                f"Dist:{str(round(dist_cm, 1)) + ' cm' if dist_cm else '--- cm':>9} | "
                f"Debris:{'YES' if debris else ' no'} | "
                f"Lane:{self.lane}"
            )

            # ── 5. Loop timing ────────────────────────────────────────────────
            elapsed = time.ticks_diff(time.ticks_ms(), t_start)
            if elapsed < LOOP_MS:
                time.sleep_ms(LOOP_MS - elapsed)

    # ──────────────────────────────────────────────────────────────────────────
    #  STATE TRANSITIONS
    # ──────────────────────────────────────────────────────────────────────────
    def _update_state(self, dist_cm, debris):

        # Priority 1: Obstacle (stop distance)
        if dist_cm is not None and dist_cm < OBSTACLE_STOP_CM:
            if self.state != STATE_EVADE:
                print(f"[State] → OBSTACLE_EVADE  (dist={dist_cm:.0f} cm)")
                self._evade_step = 0
                self._evade_dir *= -1   # alternate evasion direction
            self.state = STATE_EVADE
            return

        # Exit evasion when obstacle cleared
        if self.state == STATE_EVADE:
            self._evade_step += 1
            if self._evade_step >= self._evade_steps_total:
                if dist_cm is None or dist_cm > OBSTACLE_TURN_CM:
                    print("[State] → NORMAL_FORWARD  (obstacle cleared)")
                    self.state = STATE_FORWARD
            return

        # Priority 2: Debris detection — activate conveyor, hold a few cycles
        if debris and self.state == STATE_FORWARD:
            print("[State] → DEBRIS_COLLECT")
            self.state = STATE_DEBRIS
            self._debris_dwell = 0
            return

        if self.state == STATE_DEBRIS:
            self._debris_dwell += 1
            if self._debris_dwell > self.DEBRIS_DWELL_CYCLES:
                print("[State] → NORMAL_FORWARD  (debris collected)")
                self.state = STATE_FORWARD
            return

    # ──────────────────────────────────────────────────────────────────────────
    #  STATE ACTIONS
    # ──────────────────────────────────────────────────────────────────────────
    def _execute_state(self, dist_cm, debris):

        if self.state == STATE_FORWARD:
            # Straight-line Boustrophedon traversal
            # No heading correction without IMU — open loop straight drive
            drive(SPEED_FULL, SPEED_FULL)

        elif self.state == STATE_EVADE:
            # Phase 1 (steps 0-1): pivot turn away from obstacle
            # Phase 2 (steps 2-3): advance then resume
            step = self._evade_step
            if step <= 1:
                # In-place pivot: one motor forward, one reverse
                if self._evade_dir > 0:
                    drive( SPEED_TURN, -SPEED_TURN)   # turn right
                else:
                    drive(-SPEED_TURN,  SPEED_TURN)   # turn left
            else:
                # Clear obstacle — move forward slowly
                drive(SPEED_TURN, SPEED_TURN)

        elif self.state == STATE_DEBRIS:
            # Slow approach to ensure debris enters conveyor intake
            drive(0.30, 0.30)       # slow creep, conveyor running
            conveyor_on(0.80)       # max belt speed during ingestion

        elif self.state == STATE_REVERSE:
            # Boustrophedon 180° reversal
            self.lane += 1
            if self.lane >= self.max_lanes:
                print("[Mission] All lanes complete — halting.")
                stop_all()
            else:
                self._do_boustrophedon_turn()

    def _do_boustrophedon_turn(self):
        """
        Execute a 180-degree in-place turn for lane reversal.
        Uses timed rotation (no IMU yaw feedback in this build).
        """
        print(f"[Nav] Boustrophedon turn — starting lane {self.lane}")
        # 90° turn right
        drive(SPEED_TURN, -SPEED_TURN)
        time.sleep_ms(700)   # tune by testing until ~90° achieved
        # Advance one track width
        drive(SPEED_FULL, SPEED_FULL)
        time.sleep_ms(400)
        # Another 90° turn right (total 180°)
        drive(SPEED_TURN, -SPEED_TURN)
        time.sleep_ms(700)
        stop_all()
        time.sleep_ms(200)
        print(f"[Nav] Now on lane {self.lane}")


# ─────────────────────────────────────────────────────────────────────────────
#  UTILITY — SCAN I2C BUS ON STARTUP
# ─────────────────────────────────────────────────────────────────────────────

def scan_buses():
    print("\n── I2C scan (SoftI2C, GPIO17/16) ──────────────")
    devs = i2c_tof.scan()
    for a in devs:
        print(f"   Found: 0x{a:02X}", end="")
        if a == 0x29:
            print("  ← VL53L0X ✓")
        else:
            print()
    if not devs:
        print("   No devices — check wiring / pull-ups!")
    print()


# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print(" WSC-R  Autonomous Water Surface Cleaning Robot")
    print(" MicroPython — ESP32 38-pin")
    print(" [TEST BUILD — MPU-6050 / IMU disabled]")
    print("=" * 60)

    scan_buses()

    # Safety: ensure motors are off before doing anything
    stop_all()
    time.sleep_ms(500)

    robot = Robot()
    robot.run()


# Run immediately when flashed as main.py
main()