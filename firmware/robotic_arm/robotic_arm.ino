/**
 * 6-DOF Robotic Arm Firmware - Precision Kinematics, Calibration & Motion Controller
 * Architecture: Arduino Uno + PCA9685 16-Channel 12-Bit PWM Driver (I2C: 0x40)
 * Manipulator: 3D-Printed QR-Code Arm (RoboWings modified 6-DOF)
 *
 * Channels:
 *   CH 0: Base Yaw (Waist)
 *   CH 1: Shoulder Pitch
 *   CH 2: Elbow Pitch
 *   CH 3: Wrist Pitch
 *   CH 4: Wrist Roll
 *   CH 5: Gripper Claw
 *
 * Sensors & Comms:
 *   HC-SR04 Ultrasonic Distance Sensor (Trig: D8, Echo: D9)
 *   HC-06 Bluetooth Module (RX: D10, TX: D11 @ 9600 baud)
 *   USB Serial High-Speed Link (115200 baud)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "arm_config.h"

#define NUM_SERVOS NUM_JOINTS
#define DEFAULT_BAUD 115200
#define UPDATE_INTERVAL_MS 20    // 50 Hz control loop
#define STATUS_BROADCAST_MS 200  // 5 Hz live telemetry stream

// Ultrasonic Sensor Pins
#define TRIG_PIN 8
#define ECHO_PIN 9
#define SONAR_INTERVAL_MS 150

// HC-06 Bluetooth Pins
#define BT_RX_PIN 10  // To HC-06 TXD
#define BT_TX_PIN 11  // To HC-06 RXD (via resistor divider)
#define BT_BAUD 9600

// EEPROM Magic Identifier (bumped to force fresh factory defaults)
#define EEPROM_MAGIC 0x41524D39UL // 'ARM9'

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

// ============================================================================
// RUNTIME DATA STRUCTURES
// ============================================================================

// Active Joint Calibration Settings (Loaded from DEFAULT_JOINTS or EEPROM)
JointCalibration joints[NUM_JOINTS];

// Live Joint Motion & Trajectory State
struct JointState {
  float currentAngle;       // Current physical joint angle in degrees
  float targetAngle;        // Commanded physical joint angle in degrees
  bool active;              // PWM energized / holding torque active
  bool isMoving;            // True during timed pulse or continuous motion
  unsigned long motionStartMs;
  unsigned long motionEndMs;
  float startAngle;
};

JointState jointState[NUM_JOINTS];

float moveSpeed = 65.0f; // Global trajectory speed in degrees per second
unsigned long lastUpdateMs = 0;
unsigned long lastBroadcastMs = 0;
bool globalAttached = false;
bool pcaDetected = false;

// Ultrasonic Telemetry
float lastDistanceCm = -1.0f;
unsigned long lastSonarMs = 0;

// Autonomous State Machine Parameters
enum RoutineState {
  ROUTINE_IDLE,
  ROUTINE_SONAR_CHECK,
  ROUTINE_APPROACH,
  ROUTINE_REACH,
  ROUTINE_GRIP,
  ROUTINE_LIFT,
  ROUTINE_TRANSFER,
  ROUTINE_DROP_LOWER,
  ROUTINE_DROP_RELEASE,
  ROUTINE_RETRACT,
  ROUTINE_HOME
};

RoutineState currentRoutine = ROUTINE_IDLE;
unsigned long routineStepStartMs = 0;
unsigned long routineDwellUntilMs = 0;
bool routineIsReverse = false;
float routinePickBase = 45.0f;
float routineDropBase = 135.0f;
#define ROUTINE_STEP_TIMEOUT_MS 5000UL // 5.0 seconds failsafe watchdog per step

// Serial Response Flag (echoes to Bluetooth if command came from BT)
bool currentCmdFromBt = false;

// ============================================================================
// COORDINATE MAPPING & PWM CONVERSIONS
// ============================================================================

// Convert Microseconds to PCA9685 12-bit tick (0..4095 at 50Hz)
uint16_t usToTicks(uint16_t us) {
  uint32_t ticks = ((uint32_t)us * 4096UL) / 20000UL;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

/**
 * Maps physical joint angle (degrees) to internal servo shaft angle (0..180 deg)
 * Formula: servo_angle = (joint_angle - offsetDeg) / direction
 */
float jointToServoAngle(int idx, float jointAngle) {
  float shifted = jointAngle - joints[idx].offsetDeg;
  float servoAngle = (joints[idx].direction >= 0) ? shifted : (180.0f - shifted);
  return constrain(servoAngle, 0.0f, 180.0f);
}

/**
 * Converts internal servo shaft angle (0..180 deg) to PCA9685 PWM tick
 */
uint16_t servoAngleToTicks(int idx, float servoAngle) {
  servoAngle = constrain(servoAngle, 0.0f, 180.0f);
  float norm = servoAngle / 180.0f;
  float pulseUs = (float)joints[idx].minPulseUs + norm * (float)(joints[idx].maxPulseUs - joints[idx].minPulseUs);
  return usToTicks((uint16_t)pulseUs);
}

// Convert physical joint angle directly to PCA9685 tick
uint16_t jointAngleToTicks(int idx, float jointAngle) {
  float servoAngle = jointToServoAngle(idx, jointAngle);
  return servoAngleToTicks(idx, servoAngle);
}

// ============================================================================
// HARDWARE ACTUATOR OUTPUTS
// ============================================================================

void stopContinuousServo(int idx) {
  if (idx < 0 || idx >= NUM_JOINTS) return;
  jointState[idx].isMoving = false;
  jointState[idx].motionEndMs = 0;
  if (pcaDetected) {
    if (globalAttached && jointState[idx].active) {
      pwm.setPWM(idx, 0, usToTicks(joints[idx].centerPulseUs));
    } else {
      pwm.setPWM(idx, 0, 4096); // Full off
    }
  }
}

void writeChannelPWM(int idx, float jointAngle) {
  if (!pcaDetected) return;
  if (!globalAttached || !jointState[idx].active) {
    pwm.setPWM(idx, 0, 4096); // PWM full off (relaxed)
    return;
  }

  if (joints[idx].isContinuous) {
    // For continuous servos: when not executing a timed pulse, hold neutral stop
    if (!jointState[idx].isMoving) {
      pwm.setPWM(idx, 0, usToTicks(joints[idx].centerPulseUs));
    }
    return;
  }

  // Standard positional servo
  uint16_t tick = jointAngleToTicks(idx, jointAngle);
  pwm.setPWM(idx, 0, tick);
}

void setChannelActive(int idx, bool active) {
  if (idx < 0 || idx >= NUM_JOINTS) return;
  jointState[idx].active = active;
  if (active) {
    if (pcaDetected) pwm.wakeup();
    if (joints[idx].isContinuous) {
      stopContinuousServo(idx);
    } else {
      writeChannelPWM(idx, jointState[idx].currentAngle);
    }
  } else {
    if (joints[idx].isContinuous) {
      stopContinuousServo(idx);
    }
    if (pcaDetected) pwm.setPWM(idx, 0, 4096);
  }
}

void isolateChannel(int idx) {
  if (idx < 0 || idx >= NUM_JOINTS) return;
  globalAttached = true;
  for (int i = 0; i < NUM_JOINTS; i++) {
    setChannelActive(i, (i == idx));
  }
}

void detachAll() {
  globalAttached = false;
  currentRoutine = ROUTINE_IDLE;
  for (int i = 0; i < NUM_JOINTS; i++) {
    jointState[i].active = false;
    jointState[i].isMoving = false;
    if (pcaDetected) {
      if (joints[i].isContinuous) {
        pwm.setPWM(i, 0, usToTicks(joints[i].centerPulseUs));
      } else {
        pwm.setPWM(i, 0, 4096);
      }
    }
  }
}

void attachAll() {
  globalAttached = true;
  if (pcaDetected) pwm.wakeup();
  for (int i = 0; i < NUM_JOINTS; i++) {
    jointState[i].active = true;
    if (joints[i].isContinuous) {
      stopContinuousServo(i);
    } else {
      writeChannelPWM(i, jointState[i].currentAngle);
    }
  }
}

// ============================================================================
// MOTION EXECUTION & TRAJECTORY PLANNING
// ============================================================================

/**
 * Commands a joint angle smoothly while respecting calibration limits.
 * Does NOT immediately overwrite currentAngle (prevents abrupt jumping).
 */
void commandSetAngle(int ch, float targetJointAngle, bool immediate = false) {
  if (ch < 0 || ch >= NUM_JOINTS) return;
  targetJointAngle = constrain(targetJointAngle, joints[ch].minLimitDeg, joints[ch].maxLimitDeg);

  globalAttached = true;
  jointState[ch].active = true;
  jointState[ch].targetAngle = targetJointAngle;

  if (immediate) {
    jointState[ch].currentAngle = targetJointAngle;
  }

  if (joints[ch].isContinuous) {
    float delta = targetJointAngle - jointState[ch].currentAngle;
    if (fabs(delta) < 0.5f) {
      jointState[ch].targetAngle = targetJointAngle;
      jointState[ch].currentAngle = targetJointAngle;
      stopContinuousServo(ch);
      return;
    }

    bool cw = (delta > 0.0f);
    if (joints[ch].direction < 0) cw = !cw;

    // Calculate run duration in ms based on calibrated speed
    float spd = (joints[ch].speedDps > 10.0f) ? joints[ch].speedDps : 240.0f;
    unsigned long durationMs = (unsigned long)((fabs(delta) / spd) * 1000.0f);
    if (durationMs < 30) durationMs = 30; // Minimum kick to reliably overcome static friction

    jointState[ch].startAngle = jointState[ch].currentAngle;
    jointState[ch].motionStartMs = millis();
    jointState[ch].motionEndMs = jointState[ch].motionStartMs + durationMs;
    jointState[ch].isMoving = true;

    // Direct run pulse firmly outside deadband (+-250 us)
    uint16_t pulseUs = cw ? (joints[ch].centerPulseUs + 250) : (joints[ch].centerPulseUs - 250);
    if (pcaDetected) {
      pwm.setPWM(ch, 0, usToTicks(pulseUs));
    }
  } else {
    if (immediate) {
      writeChannelPWM(ch, jointState[ch].currentAngle);
    }
  }
}

void setAllTargets(float a0, float a1, float a2, float a3, float a4, float a5) {
  globalAttached = true;
  float targets[NUM_JOINTS] = {a0, a1, a2, a3, a4, a5};
  for (int i = 0; i < NUM_JOINTS; i++) {
    commandSetAngle(i, targets[i], false);
  }
}

void applyPose(const ArmPose& pose) {
  float angs[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; i++) {
    angs[i] = pgm_read_float(&(pose.angles[i]));
  }
  setAllTargets(angs[0], angs[1], angs[2], angs[3], angs[4], angs[5]);
}

/**
 * Checks whether all joints have arrived at their target angles within tolerance.
 */
bool areJointsAtTarget(float toleranceDeg = 1.5f) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (joints[i].isContinuous) {
      if (jointState[i].isMoving) return false;
    } else {
      if (fabs(jointState[i].targetAngle - jointState[i].currentAngle) > toleranceDeg) {
        return false;
      }
    }
  }
  return true;
}

// ============================================================================
// PLANAR INVERSE KINEMATICS (Closed-Form Analytical Solution)
// ============================================================================

bool solvePlanarIK(float x_mm, float y_mm, float z_mm, float toolPitchDeg,
                   float &q0_deg, float &q1_deg, float &q2_deg, float &q3_deg) {
  // 1. Base Yaw (CH 0): 90° centered along +X, <90° right, >90° left
  float yawRad = atan2(y_mm, x_mm);
  q0_deg = (yawRad * RAD_TO_DEG) + 90.0f;
  if (q0_deg < joints[0].minLimitDeg || q0_deg > joints[0].maxLimitDeg) {
    return false;
  }

  // 2. Planar projection in arm's vertical motion plane
  float R_tcp = sqrt(x_mm * x_mm + y_mm * y_mm);
  float Z_tcp = z_mm - LINK_BASE_MM; // Height above shoulder pivot

  // 3. Wrist center position (subtract tool length at desired pitch angle)
  float pitchRad = toolPitchDeg * DEG_TO_RAD;
  float R_wrist = R_tcp - LINK_TOOL_MM * cos(pitchRad);
  float Z_wrist = Z_tcp - LINK_TOOL_MM * sin(pitchRad);

  // 4. Distance to wrist pivot
  float D2 = R_wrist * R_wrist + Z_wrist * Z_wrist;
  float D = sqrt(D2);

  float L1 = LINK_SHOULDER_MM;
  float L2 = LINK_ELBOW_MM;

  // Workspace reachability boundary check
  if (D > (L1 + L2) || D < fabs(L1 - L2)) {
    return false;
  }

  // 5. Law of Cosines for Elbow bend angle (Interior angle beta)
  float cosBeta = (L1 * L1 + L2 * L2 - D2) / (2.0f * L1 * L2);
  cosBeta = constrain(cosBeta, -1.0f, 1.0f);
  float betaRad = acos(cosBeta);
  float elbowBendDeg = 180.0f - (betaRad * RAD_TO_DEG);

  // Map to joint coordinate
  q2_deg = 180.0f - elbowBendDeg;

  // 6. Shoulder elevation angle
  float alpha1 = atan2(Z_wrist, R_wrist);
  float cosAlpha2 = (L1 * L1 + D2 - L2 * L2) / (2.0f * L1 * D);
  cosAlpha2 = constrain(cosAlpha2, -1.0f, 1.0f);
  float alpha2 = acos(cosAlpha2);
  float shoulderElevDeg = (alpha1 + alpha2) * RAD_TO_DEG;

  // Map to joint coordinate: 90° = upright vertical, <90° = reach forward
  q1_deg = 90.0f - (shoulderElevDeg - 90.0f);

  // 7. Wrist Pitch to maintain orientation
  float armLinkPitch = shoulderElevDeg - elbowBendDeg;
  q3_deg = 90.0f - (toolPitchDeg - armLinkPitch);

  // Soft limit verification
  if (q1_deg < joints[1].minLimitDeg || q1_deg > joints[1].maxLimitDeg ||
      q2_deg < joints[2].minLimitDeg || q2_deg > joints[2].maxLimitDeg ||
      q3_deg < joints[3].minLimitDeg || q3_deg > joints[3].maxLimitDeg) {
    return false;
  }

  return true;
}

// ============================================================================
// EEPROM CALIBRATION PERSISTENCE
// ============================================================================

struct EepromRecord {
  uint32_t magic;
  int8_t direction[NUM_JOINTS];
  float offsetDeg[NUM_JOINTS];
  float minLimitDeg[NUM_JOINTS];
  float maxLimitDeg[NUM_JOINTS];
  uint16_t minPulseUs[NUM_JOINTS];
  uint16_t maxPulseUs[NUM_JOINTS];
  uint16_t centerPulseUs[NUM_JOINTS];
  bool isContinuous[NUM_JOINTS];
  float speedDps[NUM_JOINTS];
  bool isCalibrated[NUM_JOINTS];
};

void saveCalibrationToEEPROM() {
  EepromRecord rec;
  rec.magic = EEPROM_MAGIC;
  for (int i = 0; i < NUM_JOINTS; i++) {
    rec.direction[i] = joints[i].direction;
    rec.offsetDeg[i] = joints[i].offsetDeg;
    rec.minLimitDeg[i] = joints[i].minLimitDeg;
    rec.maxLimitDeg[i] = joints[i].maxLimitDeg;
    rec.minPulseUs[i] = joints[i].minPulseUs;
    rec.maxPulseUs[i] = joints[i].maxPulseUs;
    rec.centerPulseUs[i] = joints[i].centerPulseUs;
    rec.isContinuous[i] = joints[i].isContinuous;
    rec.speedDps[i] = joints[i].speedDps;
    rec.isCalibrated[i] = joints[i].isCalibrated;
  }
  EEPROM.put(0, rec);
}

bool loadCalibrationFromEEPROM() {
  EepromRecord rec;
  EEPROM.get(0, rec);
  if (rec.magic != EEPROM_MAGIC) {
    return false;
  }
  // Data bounds validation (reject uninitialized or corrupt EEPROM records)
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (rec.minPulseUs[i] < 400 || rec.maxPulseUs[i] > 2600 || rec.maxPulseUs[i] <= rec.minPulseUs[i]) {
      return false;
    }
  }
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints[i].channel = i;
    joints[i].name = (const char*)pgm_read_word(&(DEFAULT_JOINTS[i].name));
    joints[i].direction = (rec.direction[i] >= 0) ? +1 : -1;
    joints[i].offsetDeg = rec.offsetDeg[i];
    joints[i].minLimitDeg = rec.minLimitDeg[i];
    joints[i].maxLimitDeg = rec.maxLimitDeg[i];
    joints[i].minPulseUs = rec.minPulseUs[i];
    joints[i].maxPulseUs = rec.maxPulseUs[i];
    joints[i].centerPulseUs = rec.centerPulseUs[i];
    joints[i].isContinuous = rec.isContinuous[i];
    joints[i].speedDps = rec.speedDps[i];
    joints[i].isCalibrated = rec.isCalibrated[i];
  }
  return true;
}

void resetCalibrationToDefault() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    memcpy_P(&joints[i], &DEFAULT_JOINTS[i], sizeof(JointCalibration));
    joints[i].channel = i;
  }
  saveCalibrationToEEPROM();
}

// ============================================================================
// ULTRASONIC SENSOR TELEMETRY
// ============================================================================

float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 15000UL); // 15 ms max
  if (duration == 0) return -1.0f;
  float dist = ((float)duration * 0.0343f) / 2.0f;
  if (dist < 2.0f || dist > 300.0f) return -1.0f;
  return dist;
}

// ============================================================================
// COMMUNICATION & JSON TELEMETRY (ZERO-MALLOC STREAMING)
// ============================================================================

Print& getOutStream() {
  return currentCmdFromBt ? (Print&)btSerial : (Print&)Serial;
}

void outAck(const __FlashStringHelper* action) {
  Print& p = getOutStream();
  p.print(F("{\"status\":\"ok\",\"action\":\""));
  p.print(action);
  p.println(F("\"}"));
}

void outChVal(const __FlashStringHelper* action, int ch, const __FlashStringHelper* key, float val, int prec = 2) {
  Print& p = getOutStream();
  p.print(F("{\"status\":\"ok\",\"action\":\""));
  p.print(action);
  p.print(F("\",\"channel\":"));
  p.print(ch);
  p.print(F(",\""));
  p.print(key);
  p.print(F("\":"));
  p.print(val, prec);
  p.println('}');
}

void printStatus() {
  Serial.print(F("{\"type\":\"status\",\"attached\":"));
  Serial.print(globalAttached ? F("true") : F("false"));
  Serial.print(F(",\"speed\":"));
  Serial.print(moveSpeed, 1);
  Serial.print(F(",\"pca\":"));
  Serial.print(pcaDetected ? F("true") : F("false"));
  Serial.print(F(",\"cur\":["));
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.print(jointState[i].currentAngle, 2);
    if (i < NUM_JOINTS - 1) Serial.print(',');
  }
  Serial.print(F("],\"tgt\":["));
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.print(jointState[i].targetAngle, 2);
    if (i < NUM_JOINTS - 1) Serial.print(',');
  }
  Serial.print(F("],\"active\":["));
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.print(jointState[i].active ? F("true") : F("false"));
    if (i < NUM_JOINTS - 1) Serial.print(',');
  }
  Serial.print(F("],\"isCont\":["));
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.print(joints[i].isContinuous ? F("true") : F("false"));
    if (i < NUM_JOINTS - 1) Serial.print(',');
  }
  Serial.print(F("],\"calib\":["));
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.print(joints[i].isCalibrated ? F("true") : F("false"));
    if (i < NUM_JOINTS - 1) Serial.print(',');
  }
  Serial.print(F("],\"dist\":"));
  Serial.print(lastDistanceCm, 1);
  Serial.print(F(",\"routine\":"));
  Serial.print((int)currentRoutine);
  Serial.println('}');
}

void printCalibrationTable() {
  Print& p = getOutStream();
  p.print(F("{\"type\":\"calibration\",\"joints\":["));
  for (int i = 0; i < NUM_JOINTS; i++) {
    p.print(F("{\"ch\":"));
    p.print(i);
    p.print(F(",\"name\":\""));
    p.print(joints[i].name);
    p.print(F("\",\"dir\":"));
    p.print(joints[i].direction);
    p.print(F(",\"offset\":"));
    p.print(joints[i].offsetDeg, 1);
    p.print(F(",\"min\":"));
    p.print(joints[i].minLimitDeg, 1);
    p.print(F(",\"max\":"));
    p.print(joints[i].maxLimitDeg, 1);
    p.print(F(",\"minUs\":"));
    p.print(joints[i].minPulseUs);
    p.print(F(",\"maxUs\":"));
    p.print(joints[i].maxPulseUs);
    p.print(F(",\"centerUs\":"));
    p.print(joints[i].centerPulseUs);
    p.print(F(",\"isCont\":"));
    p.print(joints[i].isContinuous ? F("true") : F("false"));
    p.print(F(",\"speedDps\":"));
    p.print(joints[i].speedDps, 1);
    p.print(F(",\"calibrated\":"));
    p.print(joints[i].isCalibrated ? F("true") : F("false"));
    p.print('}');
    if (i < NUM_JOINTS - 1) p.print(',');
  }
  p.println(F("]}"));
}

bool parseChAngle(const char* s, int &ch, float &ang) {
  char* end;
  ch = (int)strtol(s, &end, 10);
  if (end == s) return false;
  const char* p = end;
  ang = (float)strtod(p, &end);
  return end != p;
}

void startPickDropRoutine(bool reverse, float pickB = 45.0f, float dropB = 135.0f) {
  routineIsReverse = reverse;
  routinePickBase = reverse ? dropB : pickB;
  routineDropBase = reverse ? pickB : dropB;
  currentRoutine = ROUTINE_SONAR_CHECK;
  routineStepStartMs = millis();
  routineDwellUntilMs = millis();
  attachAll();

  Print& p = getOutStream();
  p.print(F("{\"status\":\"ok\",\"action\":\"routine_started\",\"reverse\":"));
  p.print(reverse ? F("true") : F("false"));
  p.println('}');
}

// ============================================================================
// COMMAND PARSER & DISPATCHER
// ============================================================================

void processCommand(char* cmd) {
  while (*cmd == ' ') cmd++;
  int len = strlen(cmd);
  while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\r' || cmd[len - 1] == '\n')) {
    cmd[--len] = '\0';
  }
  if (len == 0) return;

  if (strcmp(cmd, "PING") == 0) {
    getOutStream().println(F("PONG"));
  }
  else if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "?") == 0) {
    printStatus();
  }
  else if (strcmp(cmd, "SHOWCALIB") == 0 || strcmp(cmd, "CALIBTABLE") == 0) {
    printCalibrationTable();
  }
  else if (strcmp(cmd, "SAVECALIB") == 0) {
    saveCalibrationToEEPROM();
    outAck(F("saved_to_eeprom"));
  }
  else if (strcmp(cmd, "LOADCALIB") == 0) {
    if (loadCalibrationFromEEPROM()) {
      outAck(F("loaded_from_eeprom"));
    } else {
      outAck(F("load_eeprom_failed"));
    }
  }
  else if (strcmp(cmd, "RESETCALIB") == 0) {
    resetCalibrationToDefault();
    outAck(F("reset_to_default"));
  }
  else if (strcmp(cmd, "DIST") == 0 || strcmp(cmd, "SONAR") == 0) {
    lastDistanceCm = readUltrasonicCm();
    Print& p = getOutStream();
    p.print(F("{\"status\":\"ok\",\"action\":\"sonar\",\"dist\":"));
    p.print(lastDistanceCm, 1);
    p.println('}');
  }
  else if (strcmp(cmd, "ATTACH") == 0) {
    attachAll();
    outAck(F("attached"));
    printStatus();
  }
  else if (strcmp(cmd, "DETACH") == 0 || strcmp(cmd, "STOP") == 0 || strcmp(cmd, "HALT") == 0) {
    detachAll();
    outAck(F("detached"));
    printStatus();
  }
  else if (strncmp(cmd, "ISOLATE ", 8) == 0) {
    int ch = atoi(cmd + 8);
    if (ch >= 0 && ch < NUM_JOINTS) {
      isolateChannel(ch);
      outChVal(F("isolated"), ch, F("channel"), ch, 0);
      printStatus();
    }
  }
  else if (strncmp(cmd, "RELAX ", 6) == 0 || strncmp(cmd, "OFF ", 4) == 0) {
    int ch = atoi(cmd + (cmd[0] == 'R' ? 6 : 4));
    if (ch >= 0 && ch < NUM_JOINTS) {
      setChannelActive(ch, false);
      outChVal(F("relaxed"), ch, F("channel"), ch, 0);
      printStatus();
    }
  }

  // --- Physical Poses ---
  else if (strcmp(cmd, "HOME") == 0 || strcmp(cmd, "STANDBY") == 0) {
    currentRoutine = ROUTINE_IDLE;
    applyPose(POSE_HOME);
    outAck(F("pose_home"));
  }
  else if (strcmp(cmd, "PICK") == 0) {
    currentRoutine = ROUTINE_IDLE;
    applyPose(POSE_PICK);
    outAck(F("pose_pick"));
  }
  else if (strcmp(cmd, "LIFT") == 0) {
    currentRoutine = ROUTINE_IDLE;
    applyPose(POSE_LIFT);
    outAck(F("pose_lift"));
  }
  else if (strcmp(cmd, "DROP") == 0) {
    currentRoutine = ROUTINE_IDLE;
    applyPose(POSE_DROP);
    outAck(F("pose_drop"));
  }
  else if (strcmp(cmd, "GRIP") == 0) {
    commandSetAngle(5, 80.0f, false);
    outAck(F("grip"));
  }
  else if (strcmp(cmd, "CLAW") == 0) {
    float nextVal = (jointState[5].targetAngle > 40.0f) ? 20.0f : 80.0f;
    commandSetAngle(5, nextVal, false);
    outChVal(F("claw"), 5, F("angle"), nextVal, 0);
  }

  // --- Autonomous Routine ---
  else if (strncmp(cmd, "PICKDROP", 8) == 0 || strcmp(cmd, "RUN") == 0) {
    float pickB = 45.0f;
    float dropB = 135.0f;
    if (strlen(cmd) > 8) {
      float p1, p2;
      if (sscanf(cmd + 8, "%f %f", &p1, &p2) == 2) {
        pickB = constrain(p1, joints[0].minLimitDeg, joints[0].maxLimitDeg);
        dropB = constrain(p2, joints[0].minLimitDeg, joints[0].maxLimitDeg);
      }
    }
    startPickDropRoutine(false, pickB, dropB);
  }
  else if (strncmp(cmd, "REVERSE", 7) == 0) {
    float pickB = 45.0f;
    float dropB = 135.0f;
    if (strlen(cmd) > 7) {
      float p1, p2;
      if (sscanf(cmd + 7, "%f %f", &p1, &p2) == 2) {
        pickB = constrain(p1, joints[0].minLimitDeg, joints[0].maxLimitDeg);
        dropB = constrain(p2, joints[0].minLimitDeg, joints[0].maxLimitDeg);
      }
    }
    startPickDropRoutine(true, pickB, dropB);
  }

  // --- Manual Positioning ---
  else if (strncmp(cmd, "SET ", 4) == 0) {
    int ch; float ang;
    if (parseChAngle(cmd + 4, ch, ang) && ch >= 0 && ch < NUM_JOINTS) {
      commandSetAngle(ch, ang, false);
      outChVal(F("set"), ch, F("tgt"), ang, 2);
    }
  }
  else if (strncmp(cmd, "HOLD ", 5) == 0) {
    int ch = atoi(cmd + 5);
    if (ch >= 0 && ch < NUM_JOINTS) {
      globalAttached = true;
      jointState[ch].active = true;
      jointState[ch].targetAngle = jointState[ch].currentAngle;
      if (joints[ch].isContinuous) {
        stopContinuousServo(ch);
      } else {
        writeChannelPWM(ch, jointState[ch].currentAngle);
      }
      outChVal(F("hold"), ch, F("ang"), jointState[ch].currentAngle, 2);
    }
  }

  // --- Single-Joint Calibration & Diagnostic Routines ---
  else if (strncmp(cmd, "CALIBRAW ", 9) == 0) {
    int ch; float pulse;
    if (parseChAngle(cmd + 9, ch, pulse) && ch >= 0 && ch < NUM_JOINTS) {
      uint16_t pulseUs = (uint16_t)constrain(pulse, 500.0f, 2500.0f);
      globalAttached = true;
      jointState[ch].active = true;
      if (pcaDetected) {
        pwm.setPWM(ch, 0, usToTicks(pulseUs));
      }
      float norm = (float)(pulseUs - joints[ch].minPulseUs) / (float)(joints[ch].maxPulseUs - joints[ch].minPulseUs);
      float servoAng = constrain(norm * 180.0f, 0.0f, 180.0f);
      float jointAng = (joints[ch].direction >= 0) ? (servoAng + joints[ch].offsetDeg) : (joints[ch].offsetDeg + (180.0f - servoAng));
      jointState[ch].currentAngle = jointAng;
      jointState[ch].targetAngle = jointAng;
      outChVal(F("calib_raw"), ch, F("pulseUs"), pulseUs, 0);
    }
  }
  else if (strncmp(cmd, "CALIBTEST ", 10) == 0) {
    int ch; float ang;
    if (parseChAngle(cmd + 10, ch, ang) && ch >= 0 && ch < NUM_JOINTS) {
      isolateChannel(ch);
      commandSetAngle(ch, ang, false);
      outChVal(F("calib_test"), ch, F("ang"), ang, 2);
    }
  }
  else if (strncmp(cmd, "CALIBSET ", 9) == 0) {
    int ch, dir, isCont;
    float offset, minL, maxL;
    int parsed = sscanf(cmd + 9, "%d %d %f %f %f %d", &ch, &dir, &offset, &minL, &maxL, &isCont);
    if (parsed >= 5 && ch >= 0 && ch < NUM_JOINTS) {
      joints[ch].direction = (dir >= 0) ? +1 : -1;
      joints[ch].offsetDeg = offset;
      joints[ch].minLimitDeg = minL;
      joints[ch].maxLimitDeg = maxL;
      if (parsed >= 6) joints[ch].isContinuous = (isCont != 0);
      joints[ch].isCalibrated = true;
      outChVal(F("calib_set"), ch, F("offset"), offset, 1);
    }
  }

  // --- Manual Continuous Servo Jogging ---
  else if (strncmp(cmd, "JOG ", 4) == 0) {
    char chStr[8], dirStr[8];
    int parsed = sscanf(cmd + 4, "%s %s", chStr, dirStr);
    if (parsed >= 2) {
      int ch = atoi(chStr);
      if (ch >= 0 && ch < NUM_JOINTS) {
        globalAttached = true;
        jointState[ch].active = true;
        for (int i = 0; dirStr[i]; i++) dirStr[i] = toupper(dirStr[i]);
        if (strcmp(dirStr, "STOP") == 0 || strcmp(dirStr, "0") == 0) {
          stopContinuousServo(ch);
          outAck(F("jog_stop"));
        } else if (strcmp(dirStr, "CW") == 0 || strcmp(dirStr, "+") == 0) {
          jointState[ch].isMoving = true;
          jointState[ch].motionEndMs = 0xFFFFFFFFUL;
          uint16_t pulseUs = joints[ch].centerPulseUs + 250;
          if (pcaDetected) pwm.setPWM(ch, 0, usToTicks(pulseUs));
          outAck(F("jog_cw"));
        } else if (strcmp(dirStr, "CCW") == 0 || strcmp(dirStr, "-") == 0) {
          jointState[ch].isMoving = true;
          jointState[ch].motionEndMs = 0xFFFFFFFFUL;
          uint16_t pulseUs = joints[ch].centerPulseUs - 250;
          if (pcaDetected) pwm.setPWM(ch, 0, usToTicks(pulseUs));
          outAck(F("jog_ccw"));
        }
      }
    }
  }

  // --- Planar Inverse Kinematics Command ---
  else if (strncmp(cmd, "IK ", 3) == 0) {
    float x, y, z;
    float pitch = -60.0f; // Default approach pitch
    char p1[10], p2[10], p3[10], p4[10];
    int parsed = sscanf(cmd + 3, "%s %s %s %s", p1, p2, p3, p4);
    if (parsed >= 3) {
      x = atof(p1); y = atof(p2); z = atof(p3);
      if (parsed >= 4) pitch = atof(p4);

      float q0, q1, q2, q3;
      if (solvePlanarIK(x, y, z, pitch, q0, q1, q2, q3)) {
        commandSetAngle(0, q0, false);
        commandSetAngle(1, q1, false);
        commandSetAngle(2, q2, false);
        commandSetAngle(3, q3, false);
        Print& p = getOutStream();
        p.print(F("{\"status\":\"ok\",\"action\":\"ik_solved\",\"q\":["));
        p.print(q0, 2); p.print(',');
        p.print(q1, 2); p.print(',');
        p.print(q2, 2); p.print(',');
        p.print(q3, 2); p.println(F("]}"));
      } else {
        outAck(F("ik_unreachable"));
      }
    }
  }

  // --- Multi-Joint Motion ---
  else if (strncmp(cmd, "MOVE ", 5) == 0) {
    float a[NUM_JOINTS];
    float spd = -1.0f;
    const char* p = cmd + 5;
    char* end;
    int parsed = 0;
    for (int i = 0; i < NUM_JOINTS + 1; i++) {
      float v = (float)strtod(p, &end);
      if (end == p) break;
      if (i < NUM_JOINTS) a[i] = v; else spd = v;
      parsed++;
      p = end;
    }
    if (parsed >= NUM_JOINTS) {
      if (spd > 0) moveSpeed = spd;
      setAllTargets(a[0], a[1], a[2], a[3], a[4], a[5]);
      outAck(F("move"));
    }
  }
  else if (strncmp(cmd, "SPEED ", 6) == 0) {
    float s = (float)strtod(cmd + 6, NULL);
    if (s >= 5.0f && s <= 360.0f) {
      moveSpeed = s;
      outChVal(F("speed"), 0, F("speed"), moveSpeed, 1);
    }
  }
  else if (strcmp(cmd, "DIAG") == 0 || strcmp(cmd, "TESTALL") == 0) {
    attachAll();
    Print& p = getOutStream();
    p.println(F("{\"status\":\"ok\",\"action\":\"diag_start\"}"));
    for (int ch = 0; ch < NUM_JOINTS; ch++) {
      float baseAng = jointState[ch].currentAngle;
      p.print(F("{\"action\":\"diag_channel\",\"channel\":"));
      p.print(ch);
      p.println(F("}"));
      writeChannelPWM(ch, constrain(baseAng + 15.0f, joints[ch].minLimitDeg, joints[ch].maxLimitDeg));
      delay(250);
      writeChannelPWM(ch, constrain(baseAng - 15.0f, joints[ch].minLimitDeg, joints[ch].maxLimitDeg));
      delay(250);
      writeChannelPWM(ch, baseAng);
      delay(150);
    }
    p.println(F("{\"status\":\"ok\",\"action\":\"diag_complete\"}"));
  }
  else if (strcmp(cmd, "I2CSCAN") == 0 || strcmp(cmd, "SCAN") == 0) {
    Print& p = getOutStream();
    p.print(F("{\"status\":\"ok\",\"action\":\"i2c_scan\",\"devices\":["));
    int cnt = 0;
    for (byte i = 1; i < 127; i++) {
      Wire.beginTransmission(i);
      if (Wire.endTransmission() == 0) {
        if (cnt > 0) p.print(',');
        p.print(F("\"0x"));
        p.print(i, HEX);
        p.print('"');
        cnt++;
      }
    }
    pcaDetected = (cnt > 0);
    p.print(F("],\"count\":"));
    p.print(cnt);
    p.println('}');
  }
  else {
    outAck(F("unknown_command"));
  }
}

// ============================================================================
// HARDWARE INITIALIZATION (SETUP)
// ============================================================================

void setup() {
  Serial.begin(DEFAULT_BAUD);
  btSerial.begin(BT_BAUD);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  // I2C with hardware bus timeout
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  digitalWrite(A4, HIGH); // Enable pullup on SDA
  digitalWrite(A5, HIGH); // Enable pullup on SCL

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);

  // Check PCA9685 presence at 0x40
  Wire.beginTransmission(0x40);
  pcaDetected = (Wire.endTransmission() == 0);

  // Load Calibration from EEPROM if present, else fallback to DEFAULT_JOINTS
  if (!loadCalibrationFromEEPROM()) {
    resetCalibrationToDefault();
  }

  // Initialize live joint states at default safe HOME configuration
  for (int i = 0; i < NUM_JOINTS; i++) {
    float hAng = pgm_read_float(&(POSE_HOME.angles[i]));
    jointState[i].currentAngle = hAng;
    jointState[i].targetAngle = hAng;
    jointState[i].active = true;
    jointState[i].isMoving = false;
    jointState[i].motionStartMs = 0;
    jointState[i].motionEndMs = 0;
  }

  attachAll();
  lastUpdateMs = millis();
  lastSonarMs = millis();
  lastBroadcastMs = millis();

  outAck(F("init_complete"));
  printStatus();
}

// ============================================================================
// MAIN CONTROL LOOP (50 Hz Control, Sensor Readouts, Safety Watchdog)
// ============================================================================

const int BUF_SIZE = 64;
char usbBuf[BUF_SIZE];
int usbIdx = 0;
char btBuf[BUF_SIZE];
int btIdx = 0;

void loop() {
  unsigned long now = millis();

  // 1. Process USB Serial Commands (PC / Dashboard)
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (usbIdx > 0) {
        usbBuf[usbIdx] = '\0';
        currentCmdFromBt = false;
        processCommand(usbBuf);
        usbIdx = 0;
      }
    } else if (usbIdx < BUF_SIZE - 1) {
      usbBuf[usbIdx++] = c;
    }
  }

  // 2. Process Bluetooth Serial Commands (HC-06 Module)
  while (btSerial.available() > 0) {
    char c = btSerial.read();
    if (c == '\r' || c == '\n') {
      if (btIdx > 0) {
        btBuf[btIdx] = '\0';
        currentCmdFromBt = true;
        processCommand(btBuf);
        btIdx = 0;
      }
    } else if (btIdx < BUF_SIZE - 1) {
      btBuf[btIdx++] = c;
    }
  }

  // 3. Periodic Ultrasonic Sensor Reading
  if (now - lastSonarMs >= SONAR_INTERVAL_MS) {
    lastSonarMs = now;
    lastDistanceCm = readUltrasonicCm();
  }

  // 4. Autonomous Pick & Drop State Machine with Target-Arrival & Safety Timeout
  if (currentRoutine != ROUTINE_IDLE) {
    // Failsafe Watchdog: If joints stall or fail to reach target within 5.0 seconds
    if (now - routineStepStartMs > ROUTINE_STEP_TIMEOUT_MS) {
      getOutStream().println(F("⚠️ [ABORT] Routine step timeout! Arm stalled. Halting."));
      currentRoutine = ROUTINE_IDLE;
    }
    // Check if arm has settled at destination pose
    else if (areJointsAtTarget(1.8f) && now >= routineDwellUntilMs) {
      switch (currentRoutine) {
        case ROUTINE_SONAR_CHECK:
          lastDistanceCm = readUltrasonicCm();
          currentRoutine = ROUTINE_APPROACH;
          routineStepStartMs = now;
          routineDwellUntilMs = now + 250;
          // Step 2: Slew base to pick azimuth, open claw, position arm above target
          commandSetAngle(0, routinePickBase, false);
          commandSetAngle(1, 80.0f, false);
          commandSetAngle(2, 90.0f, false);
          commandSetAngle(3, 90.0f, false);
          commandSetAngle(4, 90.0f, false);
          commandSetAngle(5, 20.0f, false); // Claw fully open
          break;

        case ROUTINE_APPROACH:
          currentRoutine = ROUTINE_REACH;
          routineStepStartMs = now;
          routineDwellUntilMs = now + 300;
          // Step 3: Descend into exact calibrated Pick Pose
          commandSetAngle(0, routinePickBase, false);
          commandSetAngle(1, pgm_read_float(&(POSE_PICK.angles[1])), false); // Shoulder forward (55°)
          commandSetAngle(2, pgm_read_float(&(POSE_PICK.angles[2])), false); // Elbow down (60°)
          commandSetAngle(3, pgm_read_float(&(POSE_PICK.angles[3])), false); // Wrist perpendicular (115°)
          commandSetAngle(4, 90.0f, false);
          commandSetAngle(5, 20.0f, false); // Keep open
          break;

        case ROUTINE_REACH:
          currentRoutine = ROUTINE_GRIP;
          routineStepStartMs = now;
          routineDwellUntilMs = now + 500; // Allow 500ms for grip closure
          // Step 4: Firm clamp onto payload puck (80°)
          commandSetAngle(5, 80.0f, false);
          break;

        case ROUTINE_GRIP:
          currentRoutine = ROUTINE_LIFT;
          routineStepStartMs = now;
          routineDwellUntilMs = now + 350;
          // Step 5: Elevate payload vertically (Shoulder 85°, Elbow 95°, Wrist level 90°)
          commandSetAngle(0, routinePickBase, false);
          commandSetAngle(1, pgm_read_float(&(POSE_LIFT.angles[1])), false);
          commandSetAngle(2, pgm_read_float(&(POSE_LIFT.angles[2])), false);
          commandSetAngle(3, pgm_read_float(&(POSE_LIFT.angles[3])), false);
          commandSetAngle(4, 90.0f, false);
          commandSetAngle(5, 80.0f, false); // Claw stays clamped!
          break;

        case ROUTINE_LIFT:
          currentRoutine = ROUTINE_TRANSFER;
          routineStepStartMs = now;
          routineDwellUntilMs = now + 400;
          // Step 6: Slew base to Drop Azimuth while maintaining lift elevation
          commandSetAngle(0, routineDropBase, false);
          commandSetAngle(1, pgm_read_float(&(POSE_LIFT.angles[1])), false);
          commandSetAngle(2, pgm_read_float(&(POSE_LIFT.angles[2])), false);
          commandSetAngle(3, pgm_read_float(&(POSE_LIFT.angles[3])), false);
          commandSetAngle(4, 90.0f, false);
          commandSetAngle(5, 80.0f, false); // Claw stays clamped!
          break;

        case ROUTINE_TRANSFER:
          currentRoutine = ROUTINE_DROP_LOWER;
          routineStepStartMs = now;
          routineDwellUntilMs = now + 350;
          // Step 7: Lower into Drop Pose
          commandSetAngle(0, routineDropBase, false);
          commandSetAngle(1, pgm_read_float(&(POSE_DROP.angles[1])), false);
          commandSetAngle(2, pgm_read_float(&(POSE_DROP.angles[2])), false);
          commandSetAngle(3, pgm_read_float(&(POSE_DROP.angles[3])), false);
          commandSetAngle(4, 90.0f, false);
          commandSetAngle(5, 80.0f, false); // Keep held until settled
          break;

        case ROUTINE_DROP_LOWER:
          currentRoutine = ROUTINE_DROP_RELEASE;
          routineStepStartMs = now;
          routineDwellUntilMs = now + 400; // Allow 400ms for jaws to open
          // Step 8: Open gripper claw to release payload puck (20°)
          commandSetAngle(5, 20.0f, false);
          break;

        case ROUTINE_DROP_RELEASE:
          currentRoutine = ROUTINE_RETRACT;
          routineStepStartMs = now;
          routineDwellUntilMs = now + 300;
          // Retract up before rotating base to prevent catching bin walls
          commandSetAngle(0, routineDropBase, false);
          commandSetAngle(1, 85.0f, false);
          commandSetAngle(2, 95.0f, false);
          commandSetAngle(3, 90.0f, false);
          commandSetAngle(4, 90.0f, false);
          commandSetAngle(5, 20.0f, false);
          break;

        case ROUTINE_RETRACT:
          currentRoutine = ROUTINE_HOME;
          routineStepStartMs = now;
          routineDwellUntilMs = now + 300;
          applyPose(POSE_HOME);
          break;

        case ROUTINE_HOME:
          getOutStream().println(F("✅ Pick & Drop Routine Complete!"));
          currentRoutine = ROUTINE_IDLE;
          break;

        default:
          currentRoutine = ROUTINE_IDLE;
          break;
      }
    }
  }

  // 5. Smooth 50Hz Trajectory Interpolator & Continuous Servo Timing Loop
  if (now - lastUpdateMs >= UPDATE_INTERVAL_MS) {
    float dt = (float)(now - lastUpdateMs) / 1000.0f;
    lastUpdateMs = now;
    float maxStep = moveSpeed * dt;

    if (globalAttached) {
      for (int i = 0; i < NUM_JOINTS; i++) {
        if (jointState[i].active) {
          if (joints[i].isContinuous) {
            // Continuous Servo: Handle timed rotational bursts
            if (jointState[i].isMoving) {
              if (now >= jointState[i].motionEndMs) {
                // Rotation duration reached: stop motor and lock angle
                jointState[i].currentAngle = constrain(jointState[i].targetAngle, 0.0f, 180.0f);
                stopContinuousServo(i);
              } else {
                // Live interpolation of currentAngle for smooth telemetry
                unsigned long elapsed = now - jointState[i].motionStartMs;
                unsigned long total = jointState[i].motionEndMs - jointState[i].motionStartMs;
                if (total > 0 && elapsed <= total) {
                  float progress = (float)elapsed / (float)total;
                  float inter = jointState[i].startAngle + progress * (jointState[i].targetAngle - jointState[i].startAngle);
                  jointState[i].currentAngle = constrain(inter, 0.0f, 180.0f);
                }
              }
            } else {
              // Maintain neutral holding pulse
              if (pcaDetected) {
                pwm.setPWM(i, 0, usToTicks(joints[i].centerPulseUs));
              }
            }
          } else {
            // Positional Servo: Smooth rate-limited trajectory interpolation
            if (jointState[i].currentAngle != jointState[i].targetAngle) {
              float diff = jointState[i].targetAngle - jointState[i].currentAngle;
              if (fabs(diff) <= maxStep) {
                jointState[i].currentAngle = jointState[i].targetAngle;
              } else {
                jointState[i].currentAngle += (diff > 0.0f ? maxStep : -maxStep);
              }
            }
            // Continual PWM refresh to maintain servo holding torque
            writeChannelPWM(i, jointState[i].currentAngle);
          }
        }
      }
    }
  }

  // 6. Periodic Live Telemetry Broadcast (5 Hz)
  if (now - lastBroadcastMs >= STATUS_BROADCAST_MS) {
    lastBroadcastMs = now;
    printStatus();
  }
}
