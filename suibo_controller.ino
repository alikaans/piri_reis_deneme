#include <Wire.h>
#include <Servo.h>

// Simulated IMU (replace with Suibo's LSM6DS3 or equivalent)
#define IMU_UPDATE_INTERVAL 100 // ms

// Motor pins for Crab 8 configuration (8 thrusters)
#define NUM_MOTORS 8
const int motorPins[NUM_MOTORS] = {3, 5, 6, 9, 10, 11, 12, 13}; // PWM pins for ESCs
Servo motors[NUM_MOTORS];

// Serial buffer
#define BUFFER_SIZE 128
char serialBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// PID parameters
struct PIDParams {
  float kp, ki, kd;
  float integral, lastError;
};
PIDParams pidYaw = {0.0, 0.0, 0.0, 0.0, 0.0};
PIDParams pidPitch = {0.0, 0.0, 0.0, 0.0, 0.0};
PIDParams pidRoll = {0.0, 0.0, 0.0, 0.0, 0.0};

// IMU data (from gyroscope/accelerometer, no magnetometer)
struct IMUData {
  float yaw, pitch, roll;
};
IMUData imuData = {0.0, 0.0, 0.0};

// Autonomous mode state
bool isAutonomous = false;

// Timing for IMU updates and feedback
unsigned long lastIMUUpdate = 0;
unsigned long lastFeedback = 0;
#define FEEDBACK_INTERVAL 1000 // ms

void setup() {
  // Initialize serial communication (match GUI's baud rate)
  Serial.begin(115200);
  
  // Initialize I2C for IMU (e.g., LSM6DS3)
  Wire.begin();
  initIMU();
  
  // Initialize motors (ESCs for Crab 8)
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].attach(motorPins[i]);
    motors[i].writeMicroseconds(1500); // Neutral (stopped)
  }
  
  // Clear serial buffer
  serialBuffer[0] = '\0';
}

void loop() {
  // Read incoming serial data
  readSerialData();
  
  // Update IMU data periodically
  if (millis() - lastIMUUpdate >= IMU_UPDATE_INTERVAL) {
    updateIMU();
    lastIMUUpdate = millis();
  }
  
  // Apply PID control if autonomous
  if (isAutonomous) {
    applyPIDControl();
  }
  
  // Send sensor feedback periodically
  if (millis() - lastFeedback >= FEEDBACK_INTERVAL) {
    sendSensorFeedback();
    lastFeedback = millis();
  }
}

void initIMU() {
  // Initialize LSM6DS3 or equivalent IMU (simplified, replace with actual library)
  Wire.beginTransmission(0x6A); // LSM6DS3 default I2C address
  Wire.write(0x10); // CTRL1_XL (accelerometer)
  Wire.write(0x60); // 416 Hz, 8g range
  Wire.endTransmission();
  Wire.beginTransmission(0x6A);
  Wire.write(0x11); // CTRL2_G (gyroscope)
  Wire.write(0x60); // 416 Hz, 1000 dps
  Wire.endTransmission();
}

void updateIMU() {
  // Simulate IMU data (replace with actual LSM6DS3 library read)
  imuData.yaw += 0.1;   // Simulated yaw increment from gyro
  imuData.pitch += 0.05; // Simulated pitch increment
  imuData.roll += 0.03;  // Simulated roll increment
  // Example real LSM6DS3 read (uncomment with library):
  /*
  Wire.beginTransmission(0x6A);
  Wire.write(0x22); // OUTX_L_G (gyro) and OUTX_L_XL (accel)
  Wire.endTransmission(false);
  Wire.requestFrom(0x6A, 12);
  if (Wire.available() >= 12) {
    int16_t gx = Wire.read() | (Wire.read() << 8);
    int16_t gy = Wire.read() | (Wire.read() << 8);
    int16_t gz = Wire.read() | (Wire.read() << 8);
    int16_t ax = Wire.read() | (Wire.read() << 8);
    int16_t ay = Wire.read() | (Wire.read() << 8);
    int16_t az = Wire.read() | (Wire.read() << 8);
    imuData.yaw += gx * 0.035; // Example: 1000 dps scale, 100 ms interval
    imuData.pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
    imuData.roll = atan2(ay, az) * 180.0 / PI;
  }
  */
}

void readSerialData() {
  while (Serial.available() && bufferIndex < BUFFER_SIZE - 1) {
    char c = Serial.read();
    serialBuffer[bufferIndex++] = c;
    
    if (c == '\n') {
      serialBuffer[bufferIndex] = '\0';
      processCommand(serialBuffer);
      bufferIndex = 0;
      serialBuffer[0] = '\0';
    }
  }
  
  // Prevent buffer overflow
  if (bufferIndex >= BUFFER_SIZE - 1) {
    bufferIndex = 0;
    serialBuffer[0] = '\0';
  }
}

void processCommand(const char* command) {
  // Parse PID command: PID:yaw,kp,ki,kd;pitch,kp,ki,kd;roll,kp,ki,kd;
  if (strncmp(command, "PID:", 4) == 0) {
    float yawKp, yawKi, yawKd, pitchKp, pitchKi, pitchKd, rollKp, rollKi, rollKd;
    if (sscanf(command, "PID:yaw,%f,%f,%f;pitch,%f,%f,%f;roll,%f,%f,%f;",
               &yawKp, &yawKi, &yawKd, &pitchKp, &pitchKi, &pitchKd, &rollKp, &rollKi, &rollKd) == 9) {
      pidYaw = {yawKp, yawKi, yawKd, 0.0, 0.0};
      pidPitch = {pitchKp, pitchKi, pitchKd, 0.0, 0.0};
      pidRoll = {rollKp, rollKi, rollKd, 0.0, 0.0};
      Serial.println("PID parameters updated");
    }
  }
  // Parse gamepad command: !GP:AXES:a1,a2,a3,a4;BUTTONS:b1,b2,...;
  else if (strncmp(command, "!GP:", 4) == 0) {
    int axes[4];
    int buttons[8];
    char buttonStr[32];
    if (sscanf(command, "!GP:AXES:%d,%d,%d,%d;BUTTONS:%[^;];",
               &axes[0], &axes[1], &axes[2], &axes[3], buttonStr) >= 4) {
      // Parse buttons
      int buttonCount = 0;
      char* token = strtok(buttonStr, ",");
      while (token && buttonCount < 8) {
        buttons[buttonCount++] = atoi(token);
        token = strtok(NULL, ",");
      }
      applyGamepadControl(axes, buttons, buttonCount);
    }
  }
  // Parse autonomous commands
  else if (strcmp(command, "AUTONOM:START\n") == 0) {
    isAutonomous = true;
    Serial.println("Autonomous mode started");
  }
  else if (strcmp(command, "AUTONOM:STOP\n") == 0) {
    isAutonomous = false;
    stopMotors();
    Serial.println("Autonomous mode stopped");
  }
  else if (strcmp(command, "AUTONOM:LOAD_ROUTE:DEFAULT\n") == 0) {
    // Placeholder: Implement route loading (requires Derin Commander API)
    Serial.println("Route loaded: DEFAULT");
  }
}

void applyGamepadControl(int axes[], int buttons[], int buttonCount) {
  // Map axes to motor speeds for Crab 8 (0-255 to 1270-1730 µs PWM)
  int motorSpeeds[NUM_MOTORS];
  
  // Crab 8 mapping:
  // Motors 0-3: Forward/back (X, axes[0])
  // Motors 4-5: Lateral (Y, axes[1])
  // Motors 6-7: Depth (Z, axes[2])
  motorSpeeds[0] = map(axes[0], 0, 255, 1270, 1730); // Forward/back
  motorSpeeds[1] = map(axes[0], 0, 255, 1270, 1730);
  motorSpeeds[2] = map(axes[0], 0, 255, 1270, 1730);
  motorSpeeds[3] = map(axes[0], 0, 255, 1270, 1730);
  motorSpeeds[4] = map(axes[1], 0, 255, 1270, 1730); // Lateral
  motorSpeeds[5] = map(axes[1], 0, 255, 1270, 1730);
  motorSpeeds[6] = map(axes[2], 0, 255, 1270, 1730); // Depth
  motorSpeeds[7] = map(axes[2], 0, 255, 1270, 1730);
  
  // Button 0 stops all motors
  if (buttonCount > 0 && buttons[0]) {
    stopMotors();
    return;
  }
  
  // Apply motor speeds
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(motorSpeeds[i]);
  }
}

void applyPIDControl() {
  // PID control for stabilization
  float targetYaw = 0.0; // Desired orientation
  float targetPitch = 0.0;
  float targetRoll = 0.0;
  
  // Calculate errors
  float yawError = targetYaw - imuData.yaw;
  float pitchError = targetPitch - imuData.pitch;
  float rollError = targetRoll - imuData.roll;
  
  // Update integrals
  pidYaw.integral += yawError;
  pidPitch.integral += pitchError;
  pidRoll.integral += rollError;
  
  // Calculate PID outputs
  float yawOutput = pidYaw.kp * yawError + pidYaw.ki * pidYaw.integral + pidYaw.kd * (yawError - pidYaw.lastError);
  float pitchOutput = pidPitch.kp * pitchError + pidPitch.ki * pidPitch.integral + pidPitch.kd * (pitchError - pidPitch.lastError);
  float rollOutput = pidRoll.kp * rollError + pidRoll.ki * pidRoll.integral + pidRoll.kd * (rollError - pidRoll.lastError);
  
  // Update last errors
  pidYaw.lastError = yawError;
  pidPitch.lastError = pitchError;
  pidRoll.lastError = rollError;
  
  // Apply PID outputs to motors (Crab 8)
  int motorSpeeds[NUM_MOTORS];
  motorSpeeds[0] = map(yawOutput, -100, 100, 1270, 1730); // Forward/back, differential yaw
  motorSpeeds[1] = map(-yawOutput, -100, 100, 1270, 1730);
  motorSpeeds[2] = map(yawOutput, -100, 100, 1270, 1730);
  motorSpeeds[3] = map(-yawOutput, -100, 100, 1270, 1730);
  motorSpeeds[4] = map(rollOutput, -100, 100, 1270, 1730); // Lateral, roll control
  motorSpeeds[5] = map(-rollOutput, -100, 100, 1270, 1730);
  motorSpeeds[6] = map(pitchOutput, -100, 100, 1270, 1730); // Depth
  motorSpeeds[7] = map(pitchOutput, -100, 100, 1270, 1730);
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(motorSpeeds[i]);
  }
}

void stopMotors() {
  // Stop all motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].writeMicroseconds(1500); // Neutral PWM
  }
}

void sendSensorFeedback() {
  // Send IMU data to GUI
  char feedback[64];
  snprintf(feedback, sizeof(feedback), "IMU:%.2f,%.2f,%.2f\n", imuData.yaw, imuData.pitch, imuData.roll);
  Serial.print(feedback);
}