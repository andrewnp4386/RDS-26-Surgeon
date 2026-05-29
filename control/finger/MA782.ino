#include <SPI.h>
#include <math.h>
#include "Globals.h"

// ==============================================================================
// CONFIGURATION
// ==============================================================================
// CS Pins: SPLAY(0), MCP(1), PIP(2), DIP(3)
const int CS_PINS[NUM_ENC] = {10, 25, 36, 37}; 
const char* ENC_NAMES[NUM_ENC] = {"SPLAY", "MCP", "PIP", "DIP"};

// MA782 SPI Settings (1MHz, Mode 0 is standard for MA782)
const uint32_t SPI_HZ = 1000000;
const uint8_t SPI_MODE_USED = SPI_MODE0; 
SPISettings ma782_spi_settings(SPI_HZ, MSBFIRST, SPI_MODE_USED);

// State Variables
uint16_t raw_w[NUM_ENC];
float jointDegs[NUM_ENC]; // Defined as extern in Globals.h

// Calibration Offsets (Update these based on your physical zero positions)
float joint_zero_offsets[NUM_ENC] = {-7.40f, 36.82f, 163.66f, -146.90f};

// ==============================================================================
// INITIALIZATION
// ==============================================================================
void setupMA782() {
  Serial.println("--- Initializing MA782 Encoders ---");
  SPI.begin();
  
  for (int i = 0; i < NUM_ENC; i++) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH); // Deselect (Active Low)
  }

  delay(10); // Give the sensors a brief moment to stabilize

  // Fill buffers with initial actual readings to prevent startup ramp-up
  for (int i = 0; i < NUM_ENC; i++) {
    // 1. Take a single baseline reading for this sensor
    uint16_t initial_raw = spiRead16(CS_PINS[i]);
    float initial_deg = ((float)initial_raw / 65536.0f) * 360.0f;
    
    // 2. Apply offsets and wrap-around exactly like the main loop
    float startDeg = initial_deg - joint_zero_offsets[i];
    if (startDeg > 180.0f) startDeg -= 360.0f;
    if (startDeg < -180.0f) startDeg += 360.0f;
    
    // Initialize the main tracking variables as well
    jointDegs[i] = startDeg;
  }
}

// ==============================================================================
// HARDWARE SPI READ
// ==============================================================================
uint16_t spiRead16(int cs_pin) {
  uint16_t result = 0;
  SPI.beginTransaction(ma782_spi_settings);
  digitalWrite(cs_pin, LOW);
  
  // MA782 requires sending 16 bits to read 16 bits
  result = SPI.transfer16(0x0000); 
  
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();
  return result;
}

// Main function called by the rest of the system
float* getJointAngles() {
  for (int i = 0; i < NUM_ENC; i++) {
    // 1. Read Raw Sensor
    raw_w[i] = spiRead16(CS_PINS[i]);
    
    // 2. Convert 16-bit absolute angle (0-65535) to Degrees (0-360)
    float raw_deg = ((float)raw_w[i] / 65536.0f) * 360.0f;
    
    // 3. Apply Offset and Handle Wrap-around
    float jointDeg = raw_deg - joint_zero_offsets[i];
    if (jointDeg > 180.0f) jointDeg -= 360.0f;
    if (jointDeg < -180.0f) jointDeg += 360.0f;
    
    // 4. Filter and Store
    jointDegs[i] = jointDeg;
  }

  return jointDegs;
}

float* EstimateTipPosition(float* jointAngles) {
  // Simple geometric model based on link lengths and joint angles
  // Link Lengths (in mm)
  float L_SPLAY = 24.0f;
  float L_MCP = 44.0f;
  float L_PIP = 39.0f;
  float L_DIP = 22.0f;
  float Ltotal = L_SPLAY + L_MCP + L_PIP + L_DIP;

  // Convert angles from degrees to radians for calculation
  float theta0 = jointAngles[0] * DEG_TO_RAD; // Splay
  float theta1 = jointAngles[1] * DEG_TO_RAD; // MCP
  float theta2 = jointAngles[2] * DEG_TO_RAD; // PIP
  float theta3 = jointAngles[3] * DEG_TO_RAD; // DIP

  // Calculate (x, y) position of the fingertip in the plane of motion
  float x = Ltotal - (L_MCP * cos(theta1) +  L_PIP * cos(theta1 + theta2) + L_DIP * cos(theta1 + theta2 + theta3))*cos(theta0);
  float y = (L_MCP * cos(theta1) +  L_PIP * cos(theta1 + theta2) + L_DIP * cos(theta1 + theta2 + theta3))*sin(theta0);
  float z = L_MCP * sin(theta1) +  L_PIP * sin(theta1 + theta2) + L_DIP * sin(theta1 + theta2 + theta3);
  static float tipPos[3];
  tipPos[0] = x;
  tipPos[1] = y;
  tipPos[2] = z; 
  return tipPos;
}


void zeroJoints() {
  for (int i = 0; i < NUM_ENC; i++) {
    raw_w[i] = spiRead16(CS_PINS[i]);
    float raw_deg = ((float)raw_w[i] / 65536.0f) * 360.0f;
    joint_zero_offsets[i] = raw_deg;
  }
}