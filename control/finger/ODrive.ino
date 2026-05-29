#include "Globals.h"

const uint32_t CAN_BAUDRATE = 250000;
const float VEL_LIMIT_TURNS_S = 90.0f;
const float I_SOFT_A = 0.2f;
const float DEGREES_PER_TURN = 14.054f;

float motor_zero_offsets[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
ODriveUserData odrive_data[NUM_MOTORS];
float last_commanded_torque[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Safe homing parameters
const float HOME_DEADBAND_DEG = 1.0f;          // strict zero target
const float HOME_STEP_DEG = 1.0f;              // smaller step for safer, more precise homing
const float HOME_MIN_PROGRESS_DEG = 0.05f;     // allow small progress near zero
const int HOME_STUCK_LIMIT = 2;               // allow more slow steps before declaring stuck
const int HOME_STEP_DELAY_MS = 60;            // give encoder/motor time to settle
const int HOME_MAX_STEPS = 400;                // hard limit
const float HOME_PULL_LIMIT_DEG = -30.0f;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can1;

ODriveCAN odrv0(wrap_can_intf(can1), 0);
ODriveCAN odrv1(wrap_can_intf(can1), 1);
ODriveCAN odrv2(wrap_can_intf(can1), 2);
ODriveCAN odrv3(wrap_can_intf(can1), 3);
ODriveCAN odrv4(wrap_can_intf(can1), 4);
ODriveCAN* odrives[NUM_MOTORS] = { &odrv0, &odrv1, &odrv2, &odrv3, &odrv4 };

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  auto* ud = static_cast<ODriveUserData*>(user_data);
  ud->last_heartbeat = msg;
  ud->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  auto* ud = static_cast<ODriveUserData*>(user_data);
  ud->last_feedback = msg;
  ud->received_feedback = true;
}

void onCanMessage(const CAN_message_t& msg) {
  for (int i = 0; i < NUM_MOTORS; i++) onReceive(msg, *odrives[i]);
}

void pumpODriveCAN() { pumpEvents(can1); }

void setMotorTorque(int motorIdx, float torqueNm) {
  if (motorIdx >= 0 && motorIdx < NUM_MOTORS && odrive_data[motorIdx].received_heartbeat) {
    odrives[motorIdx]->setTorque(torqueNm);
    last_commanded_torque[motorIdx] = torqueNm;
  }
}

void setupODrive() {
  Serial.println("\n--- Initializing CAN Bus ---");
  can1.begin();
  can1.setBaudRate(CAN_BAUDRATE);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(onCanMessage);

  for (int i = 0; i < NUM_MOTORS; i++) {
    odrives[i]->onStatus(onHeartbeat, &odrive_data[i]);
    odrives[i]->onFeedback(onFeedback, &odrive_data[i]);
  }

  Serial.println("Waiting for ODrives to wake up...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    while (!odrive_data[i].received_heartbeat) { pumpODriveCAN(); }
  }

  Serial.println("Configuring Pure Torque Control...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    odrives[i]->clearErrors();
    delay(20);
    pumpODriveCAN();
    odrives[i]->setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);
    delay(20);
    pumpODriveCAN();
    odrives[i]->setLimits(VEL_LIMIT_TURNS_S, I_SOFT_A);
    delay(20);
    pumpODriveCAN();
    odrives[i]->setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  // 5. VERIFY CLOSED LOOP STATE
  Serial.println("Verifying Axis States...");
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    pumpODriveCAN();
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_heartbeat) continue;
    
    uint8_t state = odrive_data[i].last_heartbeat.Axis_State;
    uint32_t err = odrive_data[i].last_heartbeat.Axis_Error;
    
    if (state == AXIS_STATE_CLOSED_LOOP_CONTROL && err == 0) {
      Serial.printf("✅ Node %d entered CLOSED LOOP.\n", i);
    } else {
      Serial.printf("❌ Node %d REJECTED Closed Loop! State: %d | Error: 0x%08X\n", i, state, err);
    }
  }
  
  delay(1000);
  pumpODriveCAN();
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (odrive_data[i].received_feedback) {
      motor_zero_offsets[i] = odrive_data[i].last_feedback.Pos_Estimate;
    }
  }
  Serial.println("--- ODrive Setup Complete --- \n");
}

void disableAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (odrive_data[i].received_heartbeat) odrives[i]->setState(AXIS_STATE_IDLE);
  }
}

void enableAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (odrive_data[i].received_heartbeat) odrives[i]->setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
}

void printMotorPositions() {
  Serial.println("===== Motor Positions =====");
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_feedback) continue;
    float rawTurns = odrive_data[i].last_feedback.Pos_Estimate;
    Serial.printf("Motor %d: raw=%.4f, zero=%.4f, target=%.4f\n", i, rawTurns, motor_zero_offsets[i], currentMotorTarget[i]);
  }
}

void printStatus() {
  Serial.println("===== Motor Positions =====");
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_feedback) continue;
    float rawTurns = odrive_data[i].last_feedback.Pos_Estimate;
    Serial.printf("Motor %d: raw=%.4f, zero=%.4f, target=%.4f\n", i, rawTurns, motor_zero_offsets[i], currentMotorTarget[i]);
  }
  Serial.println("===== Joint Angles =====");
  float* joints = getJointAngles();
  Serial.printf("Splay: %.2f | MCP: %.2f | PIP: %.2f | DIP: %.2f\n", joints[0], joints[1], joints[2], joints[3]);
  Serial.println("===== Estimated Tip Position =====");
  float* tip = getTipPosition(joints);
  Serial.printf("X: %.2f | Y: %.2f | Z: %.2f\n", tip[0], tip[1], tip[2]);
  }

}
bool activeJointsNearZero(float joints[4]) { 
  return 
  // Temporarily bypass Splay (joints[0]) check due to mechanical hardware issues.
  // Only check MCP (joints[1]) and PIP (joints[2]).
  // fabs(joints[0]) < HOME_DEADBAND_DEG &&
         fabs(joints[1]) < HOME_DEADBAND_DEG &&
         fabs(joints[2]) < HOME_DEADBAND_DEG;
}

void setCurrentMotorPositionsAsZero() {
  Serial.println("Setting current COMMANDED motor positions as final safe motor zero...");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!odrive_data[i].received_feedback) {
      Serial.printf("WARNING: Motor %d has no feedback; zero not updated.\n", i);
      continue;
    }

    float commandedTurns = motor_zero_offsets[i] + (currentMotorTarget[i] / DEGREES_PER_TURN);

    motor_zero_offsets[i] = commandedTurns;
    currentMotorTarget[i] = 0.0f;
    odrives[i]->setPosition(commandedTurns, 0.0f, 0.0f);
    Serial.printf("SAFE ZERO: Motor %d zero = %.4f turns\n", i, motor_zero_offsets[i]);
  }
}

bool safeHomeToZero(float startMotorAngles[]) {
  Serial.println("===== SAFE HOMING TO ZERO (PURE SENSOR + STUCK PROTECT) =====");

  float cmd[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    cmd[i] = startMotorAngles[i];
  }
  delay(300);
  pumpODriveCAN();

  // STUCK PROTECTION: Record initial joint angles to track actual physical progress
  float* initialJoints = getJointAngles();
  float lastJoints[4] = {initialJoints[0], initialJoints[1], initialJoints[2], initialJoints[3]};
  int stuckCounter = 0;

  // Set maximum allowed travel (60 degrees relative to CURRENT position)
  // Extensors (M1, M3) tighten (-), Flexors (M2, M4) loosen (+)
  float homeTargets[NUM_MOTORS] = { 
    cmd[0], 
    startMotorAngles[1] - 300.0f,  // M1 Extensor
    startMotorAngles[2] + 300.0f,  // M2 Flexor
    startMotorAngles[3] - 300.0f,  // M3 Extensor
    startMotorAngles[4] + 300.0f   // M4 Flexor
  };

  for (int step = 0; step < HOME_MAX_STEPS; step++) {
    bool anyMotorMoving = false;

    // Execute safe step for active motors (skipping Motor 0 Splay)
    for (int i = 1; i < NUM_MOTORS; i++) { 
      if (cmd[i] > homeTargets[i] + HOME_STEP_DEG) {
        cmd[i] -= HOME_STEP_DEG;
        anyMotorMoving = true;
      } else if (cmd[i] < homeTargets[i] - HOME_STEP_DEG) {
        cmd[i] += HOME_STEP_DEG;
        anyMotorMoving = true;
      } else if (cmd[i] != homeTargets[i]) {
        cmd[i] = homeTargets[i];
        anyMotorMoving = true;
      }
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
      if (!odrive_data[i].received_heartbeat) continue; 
      
      float turns = motor_zero_offsets[i] + (cmd[i] / DEGREES_PER_TURN);
      odrives[i]->setPosition(turns, 0.0f, 0.0f);
    }
    unsigned long t0 = millis();
    while (millis() - t0 < HOME_STEP_DELAY_MS) {
      pumpODriveCAN(); delay(5);
    }

    // Read new joint angles to verify physical movement
    float* jointsPtr = getJointAngles();
    float newJoints[4] = {jointsPtr[0], jointsPtr[1], jointsPtr[2], jointsPtr[3]};

    if (step % 5 == 0) {
      Serial.printf("Step %d | CMDs: M0:%.1f M1:%.1f M2:%.1f M3:%.1f M4:%.1f\n", 
                    step, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
    }

    // CONDITION 1: SUCCESS. Sensors reached zero deadband.
    if (activeJointsNearZero(newJoints)) {
      Serial.println("SUCCESS: Joints reached 0. Perfect Homing!");
      setCurrentMotorPositionsAsZero();
      return true;
    }

    // CONDITION 2: STUCK / SNAP PROTECTION
    if (anyMotorMoving) {
      // Calculate how much the active joints (MCP and PIP) actually moved
      float progress = fabs(newJoints[1] - lastJoints[1]) + fabs(newJoints[2] - lastJoints[2]);
      
      if (progress < HOME_MIN_PROGRESS_DEG) {
        stuckCounter++;
        if (stuckCounter >= 40) { 
          Serial.println("ERROR: STUCK DETECTED! Motors moved 40 deg but joints didn't (Tendon snapped or heavy slack).");
          return false;
        }
      } else {
        stuckCounter = 0; // Reset counter if joints are moving normally
      }
    }

    // Update history for next iteration
    for (int j = 0; j < 4; j++) lastJoints[j] = newJoints[j];

    if (!anyMotorMoving) {
      Serial.println("ERROR: Reached max 300 deg limit but joints didn't reach 0.");
      return false;
    }
  }

  Serial.println("ERROR: Homing timed out.");
  disableAllMotors();
  return false;
}

void runSafeHomeCommand() {
  Serial.println("Manual HOME command received.");
  setMotorTorque(2, 0.01);
  setMotorTorque(4, 0.01);
  float pos2 = 0.0f;
  float pos4 = 0.0f;
  float last_time = millis();
  while (pos2 < 25.62 && pos4 < 25.62) {
    pumpODriveCAN();
    unsigned long now = millis();
    if (now - last_time >= 20) {
      last_time = now;
      pos2 = odrive_data[2].last_feedback.Pos_Estimate - motor_zero_offsets[2];
      pos4 = odrive_data[4].last_feedback.Pos_Estimate - motor_zero_offsets[4];
      if (pos2 >= 25.62) {
        setMotorTorque(2, 0.0);
      }
      if (pos4 >= 25.62) {
        setMotorTorque(4, 0.0);
      }
    }
  }
  setMotorTorque(1, -0.06);
  setMotorTorque(3, -0.06);
  delay(5000);
  setMotorTorque(2, -0.01);
  setMotorTorque(4, -0.01);
  delay(10000);
  for (int i = 0; i < NUM_MOTORS; i++) {
    setMotorTorque(i, 0.0);
  }

  setCurrentMotorPositionsAsZero();
  zeroJoints();
}