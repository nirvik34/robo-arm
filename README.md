# 🦾 6-DOF Robotic Arm Real Physics & Motion Controller

An interactive hardware controller, 3D WebGL digital twin, and automated **Hardcoded Pick & Drop Robotic Arm Suite** powered by **Arduino Uno**, **PCA9685 16-Channel 12-Bit PWM Driver**, and **HC-SR04 Ultrasonic Distance Sensor**.

Features **Real Multi-Link Gravitational Dynamics**, **Feedforward Gravity Droop Compensation**, and **High-Precision 50Hz Live Angle Telemetry** for all 6 arm joints (Channels 0 to 5).

---

## ⚡ Quickstart

### 1. Launch the Controller Dashboard
```bash
./scripts/start_dashboard.sh
```
Open your browser to:
👉 **[http://localhost:8000](http://localhost:8000)**

### 2. Flash / Re-upload Arduino Firmware
- Click **"⚡ Flash Firmware"** in the web dashboard header.
- Or run via terminal:
  ```bash
  ./scripts/upload_firmware.sh
  ```

---

## 🔌 Hardware Wiring & Pinout Guide

### 1. Arduino Uno to PCA9685 (I2C)
| Arduino Uno Pin | PCA9685 Pin | Description |
| :--- | :--- | :--- |
| **A4** (or SDA) | **SDA** | I2C Data Line |
| **A5** (or SCL) | **SCL** | I2C Clock Line |
| **5V** | **VCC** | Logic Power (5V) |
| **GND** | **GND** | Common Ground |

*Note: Ensure PCA9685 `V+` screw terminal is connected to an external 5V–6V 3A+ power supply for the servos.*

### 2. PCA9685 Servo Channel Mapping (Channels 0 to 5)
- **Channel 0**: Base / Waist Rotation (Yaw: 0° – 180°, Center: 90.00°)
- **Channel 1**: Shoulder Joint (Pitch: 0° – 180°, Upright: 90.00°, Reach: 60.00°) — **ACTIVE with Gravity Compensation**
- **Channel 2**: Elbow Joint (Pitch: 0° – 180°, Neutral: 90.00°, Reach: 50.00°)
- **Channel 3**: Wrist Pitch (Pitch: 0° – 180°, Level: 90.00°, Down: 105.00°)
- **Channel 4**: Wrist Roll (Roll: 0° – 180°, Neutral: 90.00°)
- **Channel 5**: Gripper / Claw (Open: 20.00° – Grip: 80.00°)

### 3. Arduino Uno to HC-SR04 Ultrasonic Sensor
| HC-SR04 Pin | Arduino Uno Pin | Description |
| :--- | :--- | :--- |
| **VCC** | **5V** | Sensor Power (+5V) |
| **Trig** | **D8** | Ultrasonic Trigger Pulse Output |
| **Echo** | **D9** | Ultrasonic Echo Pulse Input |
| **GND** | **GND** | Ground |

---

## ⚖️ Real Physics & Gravitational Compensation

### Multi-Link Analytical Dynamics
Standard hobby servos suffer from mechanical compliance and proportional droop under the gravitational torque of the arm and payload. The built-in physics engine computes Newton-Euler moments for every link:
$$\tau_{\text{shoulder}} = \sum m_j g r_{\text{com},j} \sin(\theta_{\text{eff}})$$
- **Joint Torques**: Live torque calculation in Newton-meters ($N\cdot m$) and % of servo stall torque for Shoulder, Elbow, and Wrist.
- **Gravity Droop Prediction**: Computes mechanical deflection:
  $$\Delta \theta_{\text{droop}} = \frac{\tau_g}{K_{\text{servo}}}$$
- **Feedforward Angle Compensation**: Adds dynamic opposing offset to PCA9685 PWM pulses to keep the physical arm rigid and true.
- **Payload Mass Slider**: Adjust payload mass from 0g to 250g in real-time, watching torques and compensated angles adjust dynamically.

---

## 🤖 Hardcoded Autonomous Pick & Drop Sequences

### 8-Stage Precision Waypoint Trajectory:
1. **Step 1: Ultrasonic Verification**: Measures payload distance with HC-SR04 on D8/D9 (< 30 cm).
2. **Step 2: Base Align & Open Claw**: Base rotates to Pick Station (45°), Claw opens (20°).
3. **Step 3: Reach Down**: Shoulder extends (60°), Elbow descends (50°), Wrist levels (105°).
4. **Step 4: Grip Object**: Claw firmly clamps onto payload (80°).
5. **Step 5: Vertical Lift**: Shoulder (95°) and Elbow (100°) lift upright, counteracting gravity sag.
6. **Step 6: Base Transfer**: Base rotates smoothly to Drop Bin (135°).
7. **Step 7: Lower into Drop Bin**: Arm descends into destination station (Shoulder 65°, Elbow 55°).
8. **Step 8: Release & Home**: Claw opens (20°), arm retracts, and returns to safe standby (all 90°).

Click **"▶ Run Hardcoded Pick & Drop"** or **"◀ Run Hardcoded Reverse (Drop to Pick)"** from the dashboard.
