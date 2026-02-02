/**
 * Professional Arduino Motor Controller for ROS2
 *
 * This sketch implements a checksummed serial protocol compatible with
 * the ROS2 arduino_driver_node. Features:
 *
 * - NMEA-style checksum validation ($CMD,args*XX)
 * - Watchdog timeout for safety
 * - Encoder feedback (optional)
 * - Status reporting
 * - Mecanum wheel support
 *
 * Protocol:
 *   RX: $TWIST,linear_mm,angular_mrad*XX\n
 *   RX: $STOP*XX\n
 *   TX: $ODOM,enc1,enc2,enc3,enc4*XX\n
 *   TX: $STATUS,state*XX\n
 *
 * Wiring (example for L298N):
 *   Motor FL: pins 2,3 (dir), 5 (pwm)
 *   Motor FR: pins 4,7 (dir), 6 (pwm)
 *   Motor BL: pins 8,9 (dir), 10 (pwm)
 *   Motor BR: pins 11,12 (dir), 13 (pwm)
 *
 * Author: ROS2 Bowling Project
 * License: MIT
 */

// ============== CONFIGURATION ==============

// Serial settings
#define SERIAL_BAUD 115200
#define COMMAND_TIMEOUT_MS 500
#define STATUS_INTERVAL_MS 1000

// Motor pins (adjust for your setup)
// Front Left
#define FL_DIR1 2
#define FL_DIR2 3
#define FL_PWM  5

// Front Right
#define FR_DIR1 4
#define FR_DIR2 7
#define FR_PWM  6

// Back Left
#define BL_DIR1 8
#define BL_DIR2 9
#define BL_PWM  10

// Back Right
#define BR_DIR1 11
#define BR_DIR2 12
#define BR_PWM  13  // Note: Pin 13 has LED, consider using different pin

// Robot geometry (mm)
#define WHEEL_RADIUS 30.0       // Wheel radius in mm
#define WHEEL_BASE_X 100.0      // Distance from center to wheel (X)
#define WHEEL_BASE_Y 120.0      // Distance from center to wheel (Y)

// Speed limits
#define MAX_PWM 255
#define MIN_PWM 30  // Minimum PWM to overcome friction

// ============== GLOBALS ==============

// Protocol parsing
#define MAX_CMD_LEN 64
char cmdBuffer[MAX_CMD_LEN];
int cmdIndex = 0;
bool cmdReady = false;

// Timing
unsigned long lastCmdTime = 0;
unsigned long lastStatusTime = 0;

// Current velocities
int currentLinear = 0;   // mm/s
int currentAngular = 0;  // mrad/s

// Motor speeds (individual wheel PWM, -255 to 255)
int wheelFL = 0;
int wheelFR = 0;
int wheelBL = 0;
int wheelBR = 0;

// Encoder counts (if using encoders)
volatile long encFL = 0;
volatile long encFR = 0;
volatile long encBL = 0;
volatile long encBR = 0;

// ============== CHECKSUM FUNCTIONS ==============

/**
 * Calculate XOR checksum of a string
 */
byte calculateChecksum(const char* data, int len) {
    byte checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * Send a checksummed response
 * Format: $CMD,args*XX\n
 */
void sendResponse(const char* cmd) {
    int len = strlen(cmd);
    byte checksum = calculateChecksum(cmd, len);

    Serial.print('$');
    Serial.print(cmd);
    Serial.print('*');
    if (checksum < 16) Serial.print('0');
    Serial.println(checksum, HEX);
}

/**
 * Parse and validate incoming command
 * Returns true if checksum is valid
 */
bool parseCommand(char* buffer, char* cmd, int* arg1, int* arg2) {
    // Find checksum separator
    char* checksumPtr = strchr(buffer, '*');
    if (!checksumPtr) return false;

    // Extract and verify checksum
    int dataLen = checksumPtr - buffer;
    byte receivedChecksum = (byte)strtol(checksumPtr + 1, NULL, 16);
    byte expectedChecksum = calculateChecksum(buffer, dataLen);

    if (receivedChecksum != expectedChecksum) {
        return false;
    }

    // Null-terminate data portion
    *checksumPtr = '\0';

    // Parse command and arguments
    char* token = strtok(buffer, ",");
    if (!token) return false;
    strcpy(cmd, token);

    // First argument (optional)
    token = strtok(NULL, ",");
    if (token) {
        *arg1 = atoi(token);

        // Second argument (optional)
        token = strtok(NULL, ",");
        if (token) {
            *arg2 = atoi(token);
        }
    }

    return true;
}

// ============== MOTOR CONTROL ==============

/**
 * Set individual motor speed and direction
 * speed: -255 to 255
 */
void setMotor(int dir1Pin, int dir2Pin, int pwmPin, int speed) {
    speed = constrain(speed, -MAX_PWM, MAX_PWM);

    if (speed > 0) {
        digitalWrite(dir1Pin, HIGH);
        digitalWrite(dir2Pin, LOW);
        analogWrite(pwmPin, speed);
    } else if (speed < 0) {
        digitalWrite(dir1Pin, LOW);
        digitalWrite(dir2Pin, HIGH);
        analogWrite(pwmPin, -speed);
    } else {
        digitalWrite(dir1Pin, LOW);
        digitalWrite(dir2Pin, LOW);
        analogWrite(pwmPin, 0);
    }
}

/**
 * Apply dead zone to avoid motor whine at low speeds
 */
int applyDeadzone(int speed) {
    if (abs(speed) < MIN_PWM) {
        return 0;
    }
    return speed;
}

/**
 * Convert body velocities to mecanum wheel speeds
 * Uses inverse kinematics for mecanum wheels
 *
 * linear: forward velocity in mm/s
 * angular: rotation in mrad/s (positive = CCW)
 */
void mecanumKinematics(int linear, int angular) {
    // Convert angular from mrad/s to rad/s for calculation
    float omega = angular / 1000.0;

    // Mecanum inverse kinematics
    // Each wheel contributes to forward, strafe, and rotation
    // For now, we only use forward (Vx) and rotation (omega)
    // Strafe (Vy) = 0

    float Vx = linear;      // Forward velocity mm/s
    float Vy = 0;           // Strafe velocity mm/s (not used)
    float L = WHEEL_BASE_X + WHEEL_BASE_Y;  // Geometric parameter

    // Wheel velocities in mm/s
    float vFL = Vx - Vy - L * omega;
    float vFR = Vx + Vy + L * omega;
    float vBL = Vx + Vy - L * omega;
    float vBR = Vx - Vy + L * omega;

    // Find max for normalization
    float maxV = max(max(abs(vFL), abs(vFR)), max(abs(vBL), abs(vBR)));

    // Scale to PWM range (assuming max velocity maps to MAX_PWM)
    float scale = MAX_PWM / 200.0;  // 200 mm/s = full speed

    if (maxV * scale > MAX_PWM) {
        scale = MAX_PWM / maxV;
    }

    wheelFL = applyDeadzone((int)(vFL * scale));
    wheelFR = applyDeadzone((int)(vFR * scale));
    wheelBL = applyDeadzone((int)(vBL * scale));
    wheelBR = applyDeadzone((int)(vBR * scale));
}

/**
 * Apply wheel speeds to motors
 */
void updateMotors() {
    setMotor(FL_DIR1, FL_DIR2, FL_PWM, wheelFL);
    setMotor(FR_DIR1, FR_DIR2, FR_PWM, wheelFR);
    setMotor(BL_DIR1, BL_DIR2, BL_PWM, wheelBL);
    setMotor(BR_DIR1, BR_DIR2, BR_PWM, wheelBR);
}

/**
 * Emergency stop - all motors off
 */
void emergencyStop() {
    currentLinear = 0;
    currentAngular = 0;
    wheelFL = wheelFR = wheelBL = wheelBR = 0;
    updateMotors();
}

// ============== COMMAND HANDLERS ==============

void handleTwist(int linear, int angular) {
    currentLinear = linear;
    currentAngular = angular;
    mecanumKinematics(linear, angular);
    updateMotors();
    lastCmdTime = millis();
}

void handleStop() {
    emergencyStop();
    sendResponse("STATUS,STOPPED");
}

void handlePing() {
    sendResponse("PONG");
}

void handleStatus() {
    char response[32];
    snprintf(response, sizeof(response), "STATUS,%s",
             (currentLinear == 0 && currentAngular == 0) ? "IDLE" : "MOVING");
    sendResponse(response);
}

// ============== SERIAL PROCESSING ==============

void processSerial() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '$') {
            // Start of new command
            cmdIndex = 0;
            cmdReady = false;
        } else if (c == '\n' || c == '\r') {
            // End of command
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                cmdReady = true;
            }
        } else if (cmdIndex < MAX_CMD_LEN - 1) {
            cmdBuffer[cmdIndex++] = c;
        }
    }

    if (cmdReady) {
        cmdReady = false;

        char cmd[16];
        int arg1 = 0, arg2 = 0;

        if (parseCommand(cmdBuffer, cmd, &arg1, &arg2)) {
            // Valid checksum - process command
            if (strcmp(cmd, "TWIST") == 0) {
                handleTwist(arg1, arg2);
            } else if (strcmp(cmd, "STOP") == 0) {
                handleStop();
            } else if (strcmp(cmd, "PING") == 0) {
                handlePing();
            } else if (strcmp(cmd, "STATUS") == 0) {
                handleStatus();
            }
        } else {
            // Checksum error
            sendResponse("ERROR,CHECKSUM");
        }
    }
}

// ============== WATCHDOG ==============

void checkWatchdog() {
    if (millis() - lastCmdTime > COMMAND_TIMEOUT_MS) {
        if (currentLinear != 0 || currentAngular != 0) {
            emergencyStop();
            sendResponse("STATUS,TIMEOUT");
        }
    }
}

// ============== STATUS REPORTING ==============

void sendOdometry() {
    char response[48];
    snprintf(response, sizeof(response), "ODOM,%ld,%ld,%ld,%ld",
             encFL, encFR, encBL, encBR);
    sendResponse(response);
}

void periodicStatus() {
    if (millis() - lastStatusTime > STATUS_INTERVAL_MS) {
        lastStatusTime = millis();
        sendOdometry();
    }
}

// ============== SETUP & LOOP ==============

void setup() {
    // Initialize serial
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ; // Wait for serial port
    }

    // Initialize motor pins
    pinMode(FL_DIR1, OUTPUT);
    pinMode(FL_DIR2, OUTPUT);
    pinMode(FL_PWM, OUTPUT);

    pinMode(FR_DIR1, OUTPUT);
    pinMode(FR_DIR2, OUTPUT);
    pinMode(FR_PWM, OUTPUT);

    pinMode(BL_DIR1, OUTPUT);
    pinMode(BL_DIR2, OUTPUT);
    pinMode(BL_PWM, OUTPUT);

    pinMode(BR_DIR1, OUTPUT);
    pinMode(BR_DIR2, OUTPUT);
    pinMode(BR_PWM, OUTPUT);

    // Start with motors stopped
    emergencyStop();

    // Signal ready
    lastCmdTime = millis();
    lastStatusTime = millis();
    sendResponse("STATUS,READY");
}

void loop() {
    processSerial();
    checkWatchdog();
    periodicStatus();
}
