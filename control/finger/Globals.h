#pragma once
#include <FlexCAN_T4.h>

#ifdef CAN_ERROR_BUS_OFF
  #undef CAN_ERROR_BUS_OFF
#endif

#define IS_TEENSY_BUILTIN
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"

// Hardware Constants
#define NUM_MOTORS 5
#define NUM_ENC 4
extern const float DEGREES_PER_TURN;

// ODrive Data Structs
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};
extern ODriveUserData odrive_data[NUM_MOTORS];
extern float motor_zero_offsets[NUM_MOTORS];

// Sensor Variables
extern float jointDegs[NUM_ENC];
extern bool streamTelemetry;
extern unsigned long streamStartTime;

// State Machine
enum ControlMode {
  MODE_IDLE, MODE_TEST, MODE_CONTROL_TIP, MODE_CONTROL_JOINT, MODE_CONTROL_MOTOR,
  MODE_SINE_TIP, MODE_SINE_JOINT, MODE_SINE_MOTOR,
  MODE_TRAJ_STREAMING_TIP, MODE_TRAJ_STREAMING_JOINT, MODE_TRAJ_STREAMING_MOTOR,
  MODE_CONTROL_FORCE, MODE_CONTROL_TORQUE,
};
extern ControlMode currentMode;
extern unsigned long motionStartTime;

// Sine/Traj Parameters
extern int sineAxis; 
extern float sineAmp;
extern float sineFreq;
extern float sineOffset;

// Active Targets
extern float currentTipTarget[3];
extern float currentJointTarget[4];
extern float currentMotorTarget[5];
extern float last_commanded_torque[NUM_MOTORS];

// --- INNER LOOP: Motor Torque PID Parameters (Constant) ---
extern float motor_Kp_strong;
extern float motor_Kd_strong;
extern float motor_Kp_soft;
extern float motor_Kd_soft;
extern float motor_pretension;
extern float motor_prev_error[NUM_MOTORS];

extern float currentForceTarget;
extern float force_Kp;
extern float force_Ki;
extern float force_Kd;
extern bool useForceSensor; // Toggles between A101 and Kinematic Estimation

// --- OUTER LOOP: Joint Position PID Control (Tunable) ---
struct PIDController {
  float Kp, Ki, Kd, integral, prevError, outputLimit;
  void reset() { integral = 0.0f; prevError = 0.0f; }
  float compute(float target, float actual, float dt) {
    if (dt <= 0.0f) return 0.0f;
    float error = target - actual;
    integral += error * dt;
    if (integral > outputLimit) integral = outputLimit;
    if (integral < -outputLimit) integral = -outputLimit;
    float derivative = (error - prevError) / dt;
    prevError = error;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    if (output > outputLimit) output = outputLimit;
    if (output < -outputLimit) output = -outputLimit;
    return output;
  }
};
extern PIDController jointPIDs[3];
extern bool feedbackEnabled;

// Function Prototypes
void setupODrive();
void pumpODriveCAN();
void disableAllMotors();
void enableAllMotors();
void setMotorTorque(int motorIdx, float torqueNm);
void runSafeHomeCommand();
void printMotorPositions();
void printStatus();

void setupMA782();
float* getJointAngles();
float* EstimateTipPosition(float* tip_out);
void zeroJoints();
float getForce();

void resetPIDs();
void updateMotion(float dt);
void handleCommand();

void updateForceControl(float dt);

void estimateJointAnglesFromMotors(float* joints_out);
void calculateMotorAngles(float* joints, float* motorAngles_out);
void calculateJointAngles(float* target, float* joints_out);
void getForwardKinematics(float q_splay_deg, float q_mcp_deg, float q_pip_deg, float* tip_out);
float getEstimatedTipForceScalar();
void calculateJacobian(float J_out[3][3]);
void mapJointTorquesToMotorTorques(float* tau_joint, float* tau_motor_out);

// Test Functions
void testSingleMotor(int motorIndex);
void testAllMotorsTogether();
void testJointSplay();
void testJointMCP();
void testJointPIP();
void testDemo();
void testForceSensor();
void testJointSensors();
void calibrateEncoderSplay();
void calibrateEncoderMCP();
void calibrateEncoderPIP();
void calibrateEncoderDIP();