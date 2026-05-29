#include "Globals.h"

void handleCommand() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); 
  cmd.toUpperCase(); 

  if (cmd == "STOP") {
    currentMode = MODE_IDLE;
    disableAllMotors();
    Serial.println("ACK: Stopped continuous motion.");
    return;
  }
  
  if (cmd == "HOME") { runSafeHomeCommand(); return; }
  if (cmd == "STREAM ON") { streamTelemetry = true; streamStartTime = millis(); Serial.println("START_DATA"); return; }
  if (cmd == "STREAM OFF") { streamTelemetry = false; Serial.println("END_DATA"); return; }

  // Settings
  if (cmd == "PID ON") {
    feedbackEnabled = true;
    resetPIDs();
    Serial.println("ACK: Joint PID ENABLED.");
    return;
  }
  if (cmd == "PID OFF") {
    feedbackEnabled = false;
    Serial.println("ACK: Joint PID DISABLED.");
    return;
  }
  
  if (cmd == "PRINT"){
    printStatus();
    return;
  }

  if (cmd == "FORCE SENSOR ON") { 
    useForceSensor = true; 
    Serial.println("ACK: Force Feedback set to A101 Physical Sensor."); 
    return; 
  }
  if (cmd == "FORCE SENSOR OFF") { 
    useForceSensor = false; 
    Serial.println("ACK: Force Feedback set to Kinematic Estimation (Sensorless)."); 
    return; 
  }

  // Variables for parsing
  char type[10];
  float v0, v1, v2, v3, v4; 
  int idx;
  
  // ---------------------------------------------------------
  // MOVE COMMANDS
  // ---------------------------------------------------------
  if (cmd.startsWith("MOVE ")) {
    enableAllMotors();
    if (sscanf(cmd.c_str(), "MOVE TIP %f %f %f", &v0, &v1, &v2) == 3) {
      currentMode = MODE_CONTROL_TIP;
      currentTipTarget[0] = v0; currentTipTarget[1] = v1; currentTipTarget[2] = v2;
      Serial.printf("ACK: Moving TIP to %.1f, %.1f, %.1f\n", v0, v1, v2);
    } 
    else if (sscanf(cmd.c_str(), "MOVE JOINT %f %f %f %f", &v0, &v1, &v2, &v3) == 4) {
      currentMode = MODE_CONTROL_JOINT;
      currentJointTarget[0] = v0; currentJointTarget[1] = v1; 
      currentJointTarget[2] = v2; currentJointTarget[3] = v3;
      Serial.printf("ACK: Moving JOINTS to %.1f, %.1f, %.1f, %.1f\n", v0, v1, v2, v3);
    }
    else if (sscanf(cmd.c_str(), "MOVE JOINT %d %f", &idx, &v0) == 2) {
      if (idx >= 0 && idx < 4) { 
        if (v0 <= 0.0f || idx == 0) { // Splay can be positive
          currentMode = MODE_CONTROL_JOINT;
          currentJointTarget[idx] = v0;
          Serial.printf("ACK: Moving JOINT %d to %.1f\n", idx, v0);
        } else {
          Serial.println("ERROR: Joint target must be <= 0 (Negative for flexion).");
        }
      } else {
        Serial.println("ERROR: Invalid joint index.");
      }
    }
    else if (sscanf(cmd.c_str(), "MOVE MOTOR %f %f %f %f %f", &v0, &v1, &v2, &v3, &v4) == 5) {
      currentMode = MODE_CONTROL_MOTOR;
      currentMotorTarget[0] = v0; currentMotorTarget[1] = v1; currentMotorTarget[2] = v2; 
      currentMotorTarget[3] = v3; currentMotorTarget[4] = v4;
      Serial.println("ACK: Moving MOTORS directly.");
    }
    else if (sscanf(cmd.c_str(), "MOVE MOTOR %d %f", &idx, &v0) == 2) {
      if (idx >= 0 && idx < 5) {
        currentMode = MODE_CONTROL_MOTOR;
        currentMotorTarget[idx] = v0; 
        Serial.printf("ACK: Moving MOTOR %d to %.1f\n", idx, v0);
      } else {
        Serial.println("ERROR: Invalid motor index. Use 0-4.");
      }
    }
    else if (sscanf(cmd.c_str(), "MOVE TORQUE MOTOR %f %f %f %f %f", &v0, &v1, &v2, &v3, &v4) == 5) {
      currentMode = MODE_CONTROL_TORQUE;
      setMotorTorque(0, v0); setMotorTorque(1, v1); setMotorTorque(2, v2);
      setMotorTorque(3, v3); setMotorTorque(4, v4);
      Serial.println("ACK: Sending torque to MOTORS directly.");
    }
    else if (sscanf(cmd.c_str(), "MOVE TORQUE MOTOR %d %f", &idx, &v0) == 2) {
      currentMode = MODE_CONTROL_TORQUE;
      if (idx >= 0 && idx < 5) {
        setMotorTorque(idx, v0);
        Serial.printf("ACK: Sending torque to MOTOR %d to %.1f\n", idx, v0);
      } else {
        Serial.println("ERROR: Invalid motor index. Use 0-4.");
      }
    }
  }
  
  // ---------------------------------------------------------
  // SINE COMMANDS (Continuous path)
  // ---------------------------------------------------------
  else if (cmd.startsWith("SINE ")) {
    int axis;
    if (sscanf(cmd.c_str(), "SINE %s %d %f %f %f", type, &axis, &v0, &v1, &v2) == 5) {
      sineAxis = axis; sineAmp = v0; sineFreq = v1; sineOffset = v2;
      motionStartTime = millis();
      
      if (strcmp(type, "TIP") == 0) { currentMode = MODE_SINE_TIP; }
      else if (strcmp(type, "JOINT") == 0) { currentMode = MODE_SINE_JOINT; }
      else if (strcmp(type, "MOTOR") == 0) { currentMode = MODE_SINE_MOTOR; }
      
      Serial.printf("ACK: Started SINE on %s axis %d\n", type, axis);
    }
  }

  // ---------------------------------------------------------
  // TRAJECTORY STREAMING (Continuous path from PC)
  // ---------------------------------------------------------
  else if (cmd.startsWith("TRAJ ")) {
    if (sscanf(cmd.c_str(), "TRAJ TIP %f %f %f", &v0, &v1, &v2) == 3) {
      currentMode = MODE_TRAJ_STREAMING_TIP;
      currentTipTarget[0] = v0; currentTipTarget[1] = v1; currentTipTarget[2] = v2;
    } 
    else if (sscanf(cmd.c_str(), "TRAJ JOINT %f %f %f %f", &v0, &v1, &v2, &v3) == 4) {
      currentMode = MODE_TRAJ_STREAMING_JOINT;
      currentJointTarget[0] = v0; currentJointTarget[1] = v1; currentJointTarget[2] = v2; currentJointTarget[3] = v3;
    }
    else if (sscanf(cmd.c_str(), "TRAJ MOTOR %f %f %f %f %f", &v0, &v1, &v2, &v3, &v4) == 5) {
      currentMode = MODE_TRAJ_STREAMING_MOTOR;
      currentMotorTarget[0] = v0; currentMotorTarget[1] = v1; currentMotorTarget[2] = v2; currentMotorTarget[3] = v3; currentMotorTarget[4] = v4;
    }
  } 
  
  // ---------------------------------------------------------
  // TESTS & CALIBRATION
  // ---------------------------------------------------------
  else if (cmd.startsWith("TEST ")) {
    enableAllMotors();
    if (cmd == "TEST MOTORS") testAllMotorsTogether();
    else if (cmd == "TEST SPLAY") testJointSplay();
    else if (cmd == "TEST MCP") testJointMCP();
    else if (cmd == "TEST PIP") testJointPIP();
    else if (cmd == "TEST DEMO") testDemo();
    else if (cmd == "TEST FORCE") testForceSensor();
    else if (cmd == "TEST ENCODERS") testJointSensors();
    else if (sscanf(cmd.c_str(), "TEST MOTOR %d", &idx) == 1) testSingleMotor(idx);
    else if (cmd == "TEST MAX FORCE FLEXED") testMaxForce(true);
    else if (cmd == "TEST MAX FORCE EXTENDED") testMaxForce(false);
    else if (cmd == "TEST STEP FORCE LOW") testStepForce(1.0f, 3.0f);
    else if (cmd == "TEST STEP FORCE MID") testStepForce(1.0f, 10.0f);
    else if (cmd == "TEST STEP FORCE HIGH") testStepForce(1.0f, 20.0f);
    else if (cmd == "TEST STEP POS") testStepPosition();
    else if (cmd == "TEST TRAJ") testTrajectory();
    else if (cmd == "TEST IMPEDANCE") testImpedance();
  }
  else if (cmd.startsWith("CALIBRATE ENCODER ")) {
    enableAllMotors();
    if (cmd == "CALIBRATE ENCODER SPLAY") calibrateEncoderSplay();
    else if (cmd == "CALIBRATE ENCODER MCP") calibrateEncoderMCP();
    else if (cmd == "CALIBRATE ENCODER PIP") calibrateEncoderPIP();
    else if (cmd == "CALIBRATE ENCODER DIP") calibrateEncoderDIP();
  }

  // ---------------------------------------------------------
  // PID TUNING COMMANDS (Outer Loop)
  // ---------------------------------------------------------
  else if (cmd.startsWith("SET J_KP ")) {
    float val; if (sscanf(cmd.c_str(), "SET J_KP %f", &val) == 1) { 
      for(int i=0; i<3; i++) jointPIDs[i].Kp = val; 
      Serial.printf("ACK: Joint Kp set to %.3f\n", val); 
    }
  }
  else if (cmd.startsWith("SET J_KI ")) {
    float val; if (sscanf(cmd.c_str(), "SET J_KI %f", &val) == 1) { 
      for(int i=0; i<3; i++) jointPIDs[i].Ki = val; 
      Serial.printf("ACK: Joint Ki set to %.3f\n", val); 
    }
  }
  else if (cmd.startsWith("SET J_KD ")) {
    float val; if (sscanf(cmd.c_str(), "SET J_KD %f", &val) == 1) { 
      for(int i=0; i<3; i++) jointPIDs[i].Kd = val; 
      Serial.printf("ACK: Joint Kd set to %.3f\n", val); 
    }
  }
  else if (cmd.startsWith("SET M_KP_S ")) {
    float val; if (sscanf(cmd.c_str(), "SET M_KP_S %f", &val) == 1) { 
      motor_Kp_strong = val; Serial.printf("ACK: motor_Kp_strong set to %.4f\n", val); 
    }
  }
  else if (cmd.startsWith("SET M_KD_S ")) {
    float val; if (sscanf(cmd.c_str(), "SET M_KD_S %f", &val) == 1) { 
      motor_Kd_strong = val; Serial.printf("ACK: motor_Kd_strong set to %.4f\n", val); 
    }
  }
  else if (cmd.startsWith("SET M_KP_W ")) {
    float val; if (sscanf(cmd.c_str(), "SET M_KP_W %f", &val) == 1) { 
      motor_Kp_soft = val; Serial.printf("ACK: motor_Kp_soft set to %.4f\n", val); 
    }
  }
  else if (cmd.startsWith("SET M_KD_W ")) {
    float val; if (sscanf(cmd.c_str(), "SET M_KD_W %f", &val) == 1) { 
      motor_Kd_soft = val; Serial.printf("ACK: motor_Kd_soft set to %.4f\n", val); 
    }
  }
}