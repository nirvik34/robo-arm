#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

#include <Arduino.h>

/**
 * 6-DOF Robotic Arm - Centralized Hardware & Kinematics Configuration
 * Manipulator: 3D-Printed QR-Code Arm (RoboWings modified 6-DOF)
 * Controller: Arduino Uno + PCA9685 16-Channel 12-Bit PWM Driver (0x40)
 */

#define NUM_JOINTS 6

// ============================================================================
// 1. PHYSICAL LINK DIMENSIONS (Measured in Millimeters)
// ============================================================================
// Base pedestal to Shoulder pitch axis
#define LINK_BASE_MM      65.0f
// Upper arm: Shoulder pitch axis to Elbow pitch axis
#define LINK_SHOULDER_MM  125.0f
// Forearm: Elbow pitch axis to Wrist pitch axis
#define LINK_ELBOW_MM     125.0f
// Wrist pitch axis to Gripper mount
#define LINK_WRIST_MM     60.0f
// Gripper finger length
#define LINK_CLAW_MM      45.0f
// Total tool extension (Wrist axis to End-Effector tip)
#define LINK_TOOL_MM      (LINK_WRIST_MM + LINK_CLAW_MM) // 105.0 mm

// ============================================================================
// 2. CENTRALIZED CALIBRATION & JOINT CONFIGURATION
// ============================================================================
/**
 * Coordinate Mapping Convention:
 * joint_angle = direction * (servo_angle - 90.0) + offsetDeg
 * servo_angle = ((joint_angle - offsetDeg) / direction) + 90.0
 * 
 * - offsetDeg: The physical joint angle when the servo is centered at 1500 us (servo_angle = 90.0).
 * - direction: +1 if increasing servo pulse moves joint in positive direction; -1 if inverted.
 * - minLimitDeg / maxLimitDeg: Physical soft limits to prevent mechanical binding/collisions.
 */
struct JointCalibration {
  uint8_t channel;        // PCA9685 channel index (0..5)
  const char* name;       // Joint identifier
  int8_t direction;       // +1 (normal) or -1 (inverted)
  float offsetDeg;        // Physical angle at 1500us neutral
  float minLimitDeg;      // Mechanical min angle limit
  float maxLimitDeg;      // Mechanical max angle limit
  uint16_t minPulseUs;    // Min pulse width (us, ~0 deg)
  uint16_t maxPulseUs;    // Max pulse width (us, ~180 deg)
  uint16_t centerPulseUs; // Center reference pulse (us, default 1500)
  bool isContinuous;      // True if continuous 360 rotation servo
  float speedDps;         // Calibrated rotation speed (deg/s)
  bool isCalibrated;      // True if physically aligned and verified
};

#include <avr/pgmspace.h>

// Default Factory Calibration Table stored in Flash (PROGMEM)
const JointCalibration DEFAULT_JOINTS[NUM_JOINTS] PROGMEM = {
  // CH 0: Base / Waist Yaw
  {0, "Base Yaw",        +1, 0.0f,   0.0f, 180.0f,  544, 2400, 1500, false, 180.0f, true},

  // CH 1: Shoulder Pitch
  {1, "Shoulder Pitch",  +1, 0.0f,  20.0f, 160.0f,  544, 2400, 1500, false, 120.0f, true},

  // CH 2: Elbow Pitch
  {2, "Elbow Pitch",     +1, 0.0f,  25.0f, 155.0f,  544, 2400, 1500, false, 180.0f, true},

  // CH 3: Wrist Pitch
  {3, "Wrist Pitch",     +1, 0.0f,  20.0f, 160.0f,  544, 2400, 1500, false, 180.0f, true},

  // CH 4: Wrist Roll
  {4, "Wrist Roll",      +1, 0.0f,   0.0f, 180.0f,  544, 2400, 1500, false, 180.0f, true},

  // CH 5: Gripper Claw
  {5, "Gripper Claw",    +1, 0.0f,  15.0f,  85.0f,  544, 2400, 1500, false, 200.0f, true}
};

// ============================================================================
// 3. CENTRALIZED PHYSICAL POSE DEFINITIONS (Stored in PROGMEM)
// ============================================================================
struct ArmPose {
  const char* name;
  float angles[NUM_JOINTS]; // [Base, Shoulder, Elbow, WristPitch, WristRoll, Claw]
};

// POSE 1: HOME
const ArmPose POSE_HOME PROGMEM = {
  "HOME",
  {90.0f, 90.0f, 120.0f, 90.0f, 90.0f, 20.0f}
};

// POSE 2: PICK
const ArmPose POSE_PICK PROGMEM = {
  "PICK",
  {45.0f, 55.0f, 60.0f, 115.0f, 90.0f, 20.0f}
};

// POSE 3: LIFT
const ArmPose POSE_LIFT PROGMEM = {
  "LIFT",
  {45.0f, 85.0f, 95.0f, 90.0f, 90.0f, 80.0f}
};

// POSE 4: DROP
const ArmPose POSE_DROP PROGMEM = {
  "DROP",
  {135.0f, 60.0f, 65.0f, 110.0f, 90.0f, 80.0f}
};

#endif // ARM_CONFIG_H
