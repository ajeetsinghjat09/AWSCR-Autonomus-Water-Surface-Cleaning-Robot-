A low-cost, battery-powered catamaran Unmanned Surface Vehicle (USV) for systematic autonomous floating-debris collection in inland water bodies.
Urban ponds • Irrigation reservoirs • Navigable canals


📄 Based on the published paper:
"Autonomous Water Surface Cleaning Robot with IMU-Assisted Stability Control and Time-of-Flight-Based Proximal Debris Detection"
Ajeet Singh Jaat, Kanishk Choubey, Vansh Namdev — Dept. of Cyber Security & IoT, SATI Vidisha

</div>

🌊 Why WSC-R?
Over 8 million metric tons of plastic enter aquatic environments every year. Manual cleanup is slow, dangerous, and unscalable. Large skimmer vessels can't fit into confined, ecologically sensitive water bodies.
WSC-R bridges that gap — a sub-10 kg, sub-$100 robotic platform that autonomously patrols and cleans water surfaces using a lawnmower coverage pattern, triggered conveyor belt collection, and smart obstacle avoidance — no GPS required.

✨ Key Features
FeatureDetail🗺️ Boustrophedon CPPSystematic lawnmower-pattern area coverage, zero dead zones🎯 ToF Debris DetectionVL53L0X 940 nm sensor detects floating solids via differential optical reflectance🚧 Ultrasonic Obstacle AvoidanceHC-SR04 with alternating evasion logic, priority-encoded state machine⚙️ Differential ThrustTwin 775 DC motors via L298N — zero-radius point turns🏭 Auto Conveyor ControlBelt activates only during forward motion or debris detection events🔋 3.14 hr Endurance180 Wh Li-ion pack with BMS protection📡 Serial TelemetryReal-time state + sensor readout at 10 Hz over UART🧩 Modular ArchitectureClean 4-state FSM — easy to extend with IMU, GPS, or CV pipeline

🏗️ System Architecture
┌──────────────────────────────────────────────────────────┐
│                    ESP32 (38-pin)                        │
│                                                          │
│  Core 0 (Real-time)        Core 1 (Communication)        │
│  ├─ HC-SR04 polling 20Hz   ├─ Serial telemetry           │
│  ├─ VL53L0X ranging 10Hz   └─ Wi-Fi (Phase II)           │
│  ├─ Motor PWM (L298N ×2)                                 │
│  ├─ Conveyor PWM                                         │
│  └─ Navigation FSM                                       │
└────────────┬─────────────────────────────────────────────┘
             │
   ┌─────────┴──────────┐
   │   4-State FSM      │
   ├────────────────────┤
   │ 0 NORMAL_FORWARD   │ ← Boustrophedon track traversal
   │ 1 OBSTACLE_EVADE   │ ← HC-SR04 < 30 cm triggers this
   │ 2 TRACK_REVERSAL   │ ← 180° turn at lane end
   │ 3 DEBRIS_COLLECT   │ ← VL53L0X < 650 mm triggers this
   └────────────────────┘

📌 Pin Map
VL53L0X ToF Sensor
  SDA  ── GPIO 17   (SoftI2C, addr 0x29, 3.3V)
  SCL  ── GPIO 16

HC-SR04 Ultrasonic
  TRIG ── GPIO 5    (5V VCC, voltage divider on ECHO)
  ECHO ── GPIO 18

L298N-A  (Port 775 motor)
  ENA  ── GPIO 13
  IN1  ── GPIO 12
  IN2  ── GPIO 14

L298N-B  (Starboard 775 motor)
  ENA  ── GPIO 33
  IN1  ── GPIO 32
  IN2  ── GPIO 23

L298N-Conv  (Conveyor belt)
  ENA  ── GPIO 25
  IN1  ── GPIO 26
  IN2  ── GND (forward-only, hardwired)

⚠️ Important: Place a voltage divider on the HC-SR04 ECHO line (5V → 3.3V) to protect the ESP32 GPIO.


⚡ Power Architecture
12V Li-ion (180 Wh)
    │
    ├──► L298N-A / L298N-B / L298N-Conv  (12V motor rails)
    │
    └──► DC-DC Buck Converter → 5V
              │
              ├──► ESP32 VIN
              └──► HC-SR04 VCC

All GNDs star-tied near battery negative terminal.
SubsystemPowerBLDC Thrusters ×2 (cruising)40 WConveyor belt motor10 WESP32 + sensors5 WTotal55 WEndurance≈ 3.14 hrs

🔬 Sensor Details
VL53L0X — Time-of-Flight Debris Detection
The sensor is mounted at the bow, 35° downward toward the water surface.

Calm water → specular reflection deflects the 940 nm beam away from the sensor → null / out-of-range return
Floating debris (PET, EPS, HDPE, leaves) → Lambertian diffuse scatter returns measurable photon count → range < 650 mm

Debris Detected = TRUE  if d_ToF < 650 mm AND signal_valid
                  FALSE otherwise
Mean detection-to-conveyor-activation latency: 93 ms
False-positive rate (with 50 mm opaque baffle, direct sunlight): < 2%
HC-SR04 — Obstacle Avoidance
ThresholdActiondist < 30 cmEmergency stop → evasion pivotdist < 80 cmWarn, prepare evasiondist > 80 cmResume forward / CPP
Evasion direction alternates each trigger to prevent looping.

🧭 Navigation — Boustrophedon Coverage Path Planning
The robot traverses the operational zone in parallel lanes spaced 0.3 m apart (= effective cleaning swath width), guaranteeing 100% theoretical coverage.
→ → → → → → → → → →
                    ↓
← ← ← ← ← ← ← ← ← ←
↓
→ → → → → → → → → →
Lane reversal is performed as a timed 180° differential-thrust turn (no IMU in this build). Tune time.sleep_ms(700) in _do_boustrophedon_turn() to match your chassis.

🚀 Getting Started
Requirements

ESP32 38-pin development board
Thonny IDE or mpremote
MicroPython firmware ≥ v1.21 flashed to ESP32

Flash MicroPython
bash# Install esptool
pip install esptool

# Erase and flash (replace PORT with your serial port)
esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash -z 0x1000 esp32-*.bin
Deploy WSC-R
bash# Using mpremote
mpremote connect /dev/ttyUSB0 cp AWSC-R.py :main.py

# Or paste into Thonny and save as main.py on the device
The robot starts automatically on power-up (runs main() at boot).
First Boot Output
============================================================
 WSC-R  Autonomous Water Surface Cleaning Robot
 MicroPython — ESP32 38-pin
 [TEST BUILD — MPU-6050 / IMU disabled]
============================================================

── I2C scan (SoftI2C, GPIO17/16) ──────────────
   Found: 0x29  ← VL53L0X ✓

[VL53L0X] Initialised on GPIO17/16 (SoftI2C)
[Robot] Initialised. Starting in NORMAL_FORWARD.
[Robot] Mission started — Boustrophedon CPP
[NORMAL_FORWARD   ] Dist:  142.3 cm | Debris: no | Lane:0
[NORMAL_FORWARD   ] Dist:  138.7 cm | Debris: no | Lane:0
[DEBRIS_COLLECT   ] Dist:  141.1 cm | Debris:YES | Lane:0

⚙️ Configuration
All tunable parameters are at the top of AWSC-R.py:
pythonPWM_FREQ            = 2000    # L298N PWM frequency (Hz)
SPEED_FULL          = 0.85    # Forward cruise speed (0.0–1.0)
SPEED_TURN          = 0.55    # Slower side during pivot turns

OBSTACLE_STOP_CM    = 30      # Emergency stop threshold
OBSTACLE_TURN_CM    = 80      # Begin evasion threshold

DEBRIS_THRESHOLD_MM = 650     # ToF detection distance

TRACK_SPACING_M     = 0.30    # Boustrophedon lane spacing
And inside the Robot class:
pythonself.max_lanes           = 20    # Total lanes before mission ends
self.DEBRIS_DWELL_CYCLES = 15    # Conveyor hold time after detection (~1.5s)

📊 Empirical Results

Field trials on a 20 m × 15 m freshwater pond — 10 independent missions

MetricResultMean debris collection efficiency83.6%Operational endurance3.14 hoursToF false-positive rate (with baffle)< 2%Detection-to-activation latency93 msHeading error (IMU-equipped build)1.6° mean

🗺️ Roadmap
PhaseFeatures✅ Phase I (this repo)ESP32 + HC-SR04 + VL53L0X + Boustrophedon CPP🔄 Phase IIMPU-6050 IMU re-integration, wave compensation, 50 W solar + MPPT🔜 Phase IIIGNSS absolute localisation, Raspberry Pi Zero 2W + YOLOv5-Nano debris classification🔜 Phase IVAutonomous docking, motorised bin evacuation, multi-week unattended deployment

📁 Project Structure
WSC-R/
├── AWSC-R.py           # Main MicroPython firmware (flash as main.py)
├── README.md           # This file
