/*
  PROJECT: INTELLIGENT AUTONOMOUS ROBO CAR
  Board: Arduino Nano / ATmega328P

  Modes:
    1) Autonomous exploration
    2) Bluetooth RC control
    3) LDR-triggered sleep (overridden by Bluetooth RC)

  IMPORTANT HARDWARE NOTES
  - HC-05 TXD -> Arduino D2 (SoftwareSerial RX)
  - Arduino D4 (SoftwareSerial TX) -> HC-05 RXD through a 5V-to-3.3V
    level divider. Do not feed 5V directly into HC-05 RXD.
  - HC-05 STATE -> D13 only when USE_BT_STATE is enabled.
  - L298N ENA/ENB jumpers must be removed for PWM speed control.
  - Use a suitable motor supply and join its GND to Arduino GND.
  - A 128x64 SSD1306 framebuffer uses 1024 bytes of the Nano's 2 KB SRAM.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// -----------------------------------------------------------------------------
// 1. FIXED PIN MAP - DO NOT CHANGE
// -----------------------------------------------------------------------------
const uint8_t PIN_ENA       = 5;
const uint8_t PIN_IN1       = 7;
const uint8_t PIN_IN2       = 8;
const uint8_t PIN_IN3       = 9;
const uint8_t PIN_IN4       = 10;
const uint8_t PIN_ENB       = 6;

const uint8_t PIN_TRIG      = 11;
const uint8_t PIN_ECHO      = 12;
const uint8_t PIN_SERVO     = 3;
const uint8_t PIN_IR_LEFT   = A0;
const uint8_t PIN_IR_RIGHT  = A1;
const uint8_t PIN_LDR       = A2;
const uint8_t PIN_BT_RX     = 2;   // Arduino receives HC-05 TXD here
const uint8_t PIN_BT_TX     = 4;   // Arduino sends to HC-05 RXD here
const uint8_t PIN_BT_STATE  = 13;

const uint8_t OLED_ADDRESS  = 0x3C;
const int8_t  OLED_RESET    = -1;
const uint8_t OLED_WIDTH    = 128;
const uint8_t OLED_HEIGHT   = 64;

// -----------------------------------------------------------------------------
// 2. USER CONFIGURATION - CALIBRATE THESE VALUES FOR YOUR CHASSIS
// -----------------------------------------------------------------------------

// Motor direction. Set true if that motor runs backwards for a positive command.
const bool LEFT_MOTOR_INVERTED  = false;
const bool RIGHT_MOTOR_INVERTED = false;

// PWM correction added to each motor's requested magnitude.
// Start at 0. Increase the slower motor's trim a little at a time.
const int8_t LEFT_SPEED_TRIM  = 0;
const int8_t RIGHT_SPEED_TRIM = 0;

const uint8_t AUTONOMOUS_SPEED = 155;  // 0..255
const uint8_t AUTONOMOUS_TURN_SPEED = 165;
const uint8_t RC_INITIAL_SPEED = 190;
const uint8_t RC_MIN_MOVING_PWM = 65;  // speed level 1 maps to this value

// Obstacle distances in centimetres.
const uint16_t OBSTACLE_DISTANCE_CM = 23;
const uint16_t OPEN_PATH_DISTANCE_CM = 29;
const uint16_t MAX_SCAN_DISTANCE_CM = 250;
const unsigned long ULTRASONIC_TIMEOUT_US = 18000UL;
const unsigned long ULTRASONIC_CHECK_INTERVAL_MS = 90UL;
const bool NO_ECHO_MEANS_CLEAR = true;

// IR modules commonly output LOW when an obstacle is detected.
// Change to false if your modules output HIGH on detection.
const bool IR_ACTIVE_LOW = true;

// Servo scan positions and settling time.
const uint8_t SERVO_LEFT_DEG   = 30;
const uint8_t SERVO_CENTER_DEG = 90;
const uint8_t SERVO_RIGHT_DEG  = 150;
const unsigned long SERVO_SETTLE_MS = 240UL;
const unsigned long PRE_SCAN_FACE_MS = 180UL;

// Turn durations depend on wheel size, battery, floor and motor speed.
const unsigned long SIDE_TURN_MS = 330UL;
const unsigned long RECOVERY_180_TURN_MS = 760UL;
const unsigned long RECOVERY_FACE_HOLD_MS = 900UL;
const unsigned long RECOVERY_COOLDOWN_MS = 5000UL;

// Bluetooth connection detection.
// Keep true when HC-05 STATE is wired to D13. If not wired, set false and
// configure the remote app to transmit 'H' about once every second.
const bool USE_BT_STATE = true;
const uint8_t BT_STATE_CONNECTED_LEVEL = HIGH;
const unsigned long BT_HEARTBEAT_TIMEOUT_MS = 3500UL;
const unsigned long BT_BAUD_RATE = 9600UL;

// LDR calibration. For the common divider arrangement, darkness may read low.
const bool LDR_DARK_WHEN_BELOW = true;
const uint16_t LDR_BLACKOUT_THRESHOLD = 90;
const uint16_t LDR_WAKE_THRESHOLD = 135;
const unsigned long LDR_SAMPLE_INTERVAL_MS = 100UL;
const unsigned long BLACKOUT_CONFIRM_MS = 2500UL;
const unsigned long WAKE_CONFIRM_MS = 1200UL;

// OLED timing.
const unsigned long OLED_FRAME_INTERVAL_MS = 85UL;
const unsigned long BLINK_DURATION_MS = 130UL;
const unsigned long WAKE_ANIMATION_MS = 1000UL;
const unsigned long SLEEP_CLOSE_ANIMATION_MS = 900UL;

// -----------------------------------------------------------------------------
// 3. OBJECTS AND GLOBAL STATE
// -----------------------------------------------------------------------------

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
Servo scanServo;
SoftwareSerial bluetooth(PIN_BT_RX, PIN_BT_TX);

bool oledReady = false;
bool servoAttached = false;
bool bootActive = true;

// Operating mode.
enum RobotMode : uint8_t {
  MODE_AUTONOMOUS,
  MODE_RC,
  MODE_SLEEP
};
RobotMode robotMode = MODE_AUTONOMOUS;

// Autonomous navigation state machine.
enum AutoState : uint8_t {
  AUTO_DRIVE,
  AUTO_SCAN_PREP,
  AUTO_SCAN_MOVE_SERVO,
  AUTO_SCAN_SETTLE,
  AUTO_TURNING,
  AUTO_RECOVERY_PAUSE,
  AUTO_RECOVERY_TURN
};
AutoState autoState = AUTO_DRIVE;

// OLED face expressions.
enum FaceExpression : uint8_t {
  FACE_NORMAL,
  FACE_LOOK_LEFT,
  FACE_LOOK_RIGHT,
  FACE_SCANNING,
  FACE_SURPRISED,
  FACE_CONFUSED,
  FACE_EXCITED,
  FACE_SLEEPING,
  FACE_WAKE
};
FaceExpression faceExpression = FACE_NORMAL;

bool irLeftDetected = false;
bool irRightDetected = false;
bool btConnected = false;
bool heartbeatSeen = false;
bool ldrDarkStable = false;
bool ldrTransitionPending = false;
bool wakeSequenceActive = false;
bool oledDirty = true;
bool blinkActive = false;

uint16_t scanDistances[3] = {0, 0, 0};
uint8_t scanIndex = 0;
uint16_t lastLdrValue = 0;
uint8_t rcSpeed = RC_INITIAL_SPEED;
char rcMotionCommand = 'S';

int8_t pupilOffset = 0;
int8_t pupilTarget = 0;

unsigned long lastUltrasonicCheck = 0;
unsigned long autoStateStartedAt = 0;
unsigned long autoTurnDuration = 0;
unsigned long recoveryCooldownUntil = 0;
unsigned long lastLdrSampleAt = 0;
unsigned long ldrTransitionStartedAt = 0;
unsigned long lastBtHeartbeatAt = 0;
unsigned long wakeStartedAt = 0;
unsigned long faceStartedAt = 0;
unsigned long faceDurationMs = 0;
unsigned long blinkStartedAt = 0;
unsigned long nextBlinkAt = 0;
unsigned long lastOLEDFrameAt = 0;
unsigned long lastAnimatedFaceFrameAt = 0;
unsigned long lastPupilStepAt = 0;

// Boot animation state. All animations are timed with millis().
uint8_t bootStage = 0;
unsigned long bootStageStartedAt = 0;
unsigned long lastBootFrameAt = 0;

// -----------------------------------------------------------------------------
// 4. FUNCTION DECLARATIONS
// -----------------------------------------------------------------------------
void setupMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
void setMotorSpeed(int16_t leftCommand, int16_t rightCommand);
void writeOneMotor(uint8_t enablePin, uint8_t in1, uint8_t in2,
                   int16_t command, bool inverted, int8_t trim);

uint16_t readUltrasonic();
void scanEnvironment();
void readIRSensors();
void readLDR();
void checkBluetooth();
void handleBluetoothCommands();
void processRCCommand(char command);
void executeRCMotion();

void autonomousNavigation();
void beginObstacleAvoidance();
void obstacleAvoidance();
void deadEndRecovery();
void sleepMode();
void wakeUpSequence();
void updateWakeSequence();
void updatePowerMode();

void bootAnimation();
void drawBootFrame();
void finishBootAnimation();
void setFace(FaceExpression expression, unsigned long durationMs = 0);
void updateRobotFace();
void updateOLED();
void drawEye(int16_t centerX, int16_t centerY, uint8_t openHeight,
             int8_t pupilShift, bool roundEye);
void drawNormalFace();

bool isValidBluetoothCommand(char command);
bool isLdrDarkReading(uint16_t value, uint16_t threshold);
void attachScannerServo();

// -----------------------------------------------------------------------------
// 5. SETUP
// -----------------------------------------------------------------------------
void setup() {
  setupMotors();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);
  pinMode(PIN_LDR, INPUT);

  if (USE_BT_STATE) {
    pinMode(PIN_BT_STATE, INPUT);
  }

  bluetooth.begin(BT_BAUD_RATE);

  attachScannerServo();
  scanServo.write(SERVO_CENTER_DEG);

  Wire.begin();
  Wire.setClock(400000L);
  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (oledReady) {
    display.clearDisplay();
    display.display();
  }

  rcSpeed = RC_INITIAL_SPEED;
  rcMotionCommand = 'S';
  faceExpression = FACE_NORMAL;
  faceStartedAt = millis();
  nextBlinkAt = millis() + 2200UL;

  // Initial LDR read seeds the hysteresis system; no motion is allowed yet.
  lastLdrValue = analogRead(PIN_LDR);
  lastLdrSampleAt = millis();

  // Keep the bridge disabled for the entire boot animation.
  stopMotors();
  bootActive = true;
  bootStage = 0;
  bootStageStartedAt = millis();
  lastBootFrameAt = 0;
  drawBootFrame();
}

// -----------------------------------------------------------------------------
// 6. MAIN LOOP
// -----------------------------------------------------------------------------
void loop() {
  readIRSensors();
  readLDR();

  // Check STATE before parsing commands so an already-connected HC-05 gets
  // RC priority immediately. In heartbeat mode, the parser refreshes and
  // checks the connection as soon as a valid packet arrives.
  checkBluetooth();
  handleBluetoothCommands();
  checkBluetooth();

  if (bootActive) {
    stopMotors();
    bootAnimation();
    return;
  }

  updatePowerMode();

  if (robotMode == MODE_SLEEP) {
    sleepMode();
  } else if (robotMode == MODE_RC) {
    updateRobotFace();
    updateOLED();
  } else {
    updateWakeSequence();
    if (!wakeSequenceActive) {
      autonomousNavigation();
    }
    updateRobotFace();
    updateOLED();
  }
}

// -----------------------------------------------------------------------------
// 7. MOTOR CONTROL
// -----------------------------------------------------------------------------
void setupMotors() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  stopMotors();
}

void writeOneMotor(uint8_t enablePin, uint8_t in1, uint8_t in2,
                   int16_t command, bool inverted, int8_t trim) {
  int16_t adjusted = command;
  if (inverted) adjusted = -adjusted;

  if (adjusted == 0) {
    analogWrite(enablePin, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    return;
  }

  int16_t magnitude = (adjusted > 0) ? adjusted : -adjusted;
  magnitude += trim;
  if (magnitude < 0) magnitude = 0;
  if (magnitude > 255) magnitude = 255;

  if (adjusted > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enablePin, (uint8_t)magnitude);
}

void setMotorSpeed(int16_t leftCommand, int16_t rightCommand) {
  // A hard software interlock: no movement during boot or sleep.
  if (bootActive || robotMode == MODE_SLEEP) {
    leftCommand = 0;
    rightCommand = 0;
  }

  if (leftCommand > 255) leftCommand = 255;
  if (leftCommand < -255) leftCommand = -255;
  if (rightCommand > 255) rightCommand = 255;
  if (rightCommand < -255) rightCommand = -255;

  writeOneMotor(PIN_ENA, PIN_IN1, PIN_IN2,
                leftCommand, LEFT_MOTOR_INVERTED, LEFT_SPEED_TRIM);
  writeOneMotor(PIN_ENB, PIN_IN3, PIN_IN4,
                rightCommand, RIGHT_MOTOR_INVERTED, RIGHT_SPEED_TRIM);
}

void moveForward() {
  uint8_t speed = (robotMode == MODE_RC) ? rcSpeed : AUTONOMOUS_SPEED;
  setMotorSpeed(speed, speed);
}

void moveBackward() {
  uint8_t speed = (robotMode == MODE_RC) ? rcSpeed : AUTONOMOUS_SPEED;
  setMotorSpeed(-((int16_t)speed), -((int16_t)speed));
}

void turnLeft() {
  uint8_t speed = (robotMode == MODE_RC) ? rcSpeed : AUTONOMOUS_TURN_SPEED;
  setMotorSpeed(-((int16_t)speed), speed);
}

void turnRight() {
  uint8_t speed = (robotMode == MODE_RC) ? rcSpeed : AUTONOMOUS_TURN_SPEED;
  setMotorSpeed(speed, -((int16_t)speed));
}

void stopMotors() {
  // L298N enable LOW + both direction inputs LOW = motor outputs disabled.
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

// -----------------------------------------------------------------------------
// 8. SENSOR INPUTS
// -----------------------------------------------------------------------------
void readIRSensors() {
  const uint8_t leftLevel = digitalRead(PIN_IR_LEFT);
  const uint8_t rightLevel = digitalRead(PIN_IR_RIGHT);

  if (IR_ACTIVE_LOW) {
    irLeftDetected = (leftLevel == LOW);
    irRightDetected = (rightLevel == LOW);
  } else {
    irLeftDetected = (leftLevel == HIGH);
    irRightDetected = (rightLevel == HIGH);
  }
}

uint16_t readUltrasonic() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  const unsigned long duration = pulseIn(PIN_ECHO, HIGH,
                                         ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    return NO_ECHO_MEANS_CLEAR ? MAX_SCAN_DISTANCE_CM : 0;
  }

  uint16_t distance = (uint16_t)(duration / 58UL);
  if (distance == 0) distance = 1;
  if (distance > MAX_SCAN_DISTANCE_CM) distance = MAX_SCAN_DISTANCE_CM;
  return distance;
}

bool isLdrDarkReading(uint16_t value, uint16_t threshold) {
  return LDR_DARK_WHEN_BELOW ? (value < threshold) : (value > threshold);
}

void readLDR() {
  const unsigned long now = millis();
  if (now - lastLdrSampleAt < LDR_SAMPLE_INTERVAL_MS) return;
  lastLdrSampleAt = now;
  lastLdrValue = analogRead(PIN_LDR);

  // Use a second threshold while already dark to provide hysteresis.
  const bool wantsDark = ldrDarkStable
    ? isLdrDarkReading(lastLdrValue, LDR_WAKE_THRESHOLD)
    : isLdrDarkReading(lastLdrValue, LDR_BLACKOUT_THRESHOLD);

  if (wantsDark == ldrDarkStable) {
    ldrTransitionPending = false;
    return;
  }

  if (!ldrTransitionPending) {
    ldrTransitionPending = true;
    ldrTransitionStartedAt = now;
    return;
  }

  const unsigned long requiredTime = wantsDark
    ? BLACKOUT_CONFIRM_MS : WAKE_CONFIRM_MS;

  if (now - ldrTransitionStartedAt >= requiredTime) {
    ldrDarkStable = wantsDark;
    ldrTransitionPending = false;
  }
}

// -----------------------------------------------------------------------------
// 9. BLUETOOTH CONNECTION AND COMMAND HANDLING
// -----------------------------------------------------------------------------
bool isValidBluetoothCommand(char command) {
  return command == 'F' || command == 'B' || command == 'L' ||
         command == 'R' || command == 'S' || command == 'H' ||
         (command >= '0' && command <= '9');
}

void checkBluetooth() {
  bool connectedNow = false;

  if (USE_BT_STATE) {
    connectedNow = (digitalRead(PIN_BT_STATE) == BT_STATE_CONNECTED_LEVEL);
  } else {
    connectedNow = heartbeatSeen &&
      (millis() - lastBtHeartbeatAt <= BT_HEARTBEAT_TIMEOUT_MS);
  }

  if (connectedNow == btConnected) return;

  btConnected = connectedNow;
  if (btConnected) {
    // RC has priority over autonomous navigation and LDR sleep.
    stopMotors();
    rcMotionCommand = 'S';
    if (robotMode != MODE_RC) {
      robotMode = MODE_RC;
      autoState = AUTO_DRIVE;
      attachScannerServo();
      scanServo.write(SERVO_CENTER_DEG);
      setFace(FACE_EXCITED);
    }
  } else {
    stopMotors();
    rcMotionCommand = 'S';
    if (ldrDarkStable) {
      if (robotMode != MODE_SLEEP) {
        robotMode = MODE_SLEEP;
        if (servoAttached) {
          scanServo.detach();
          servoAttached = false;
        }
        setFace(FACE_SLEEPING);
      }
    } else {
      robotMode = MODE_AUTONOMOUS;
      attachScannerServo();
      scanServo.write(SERVO_CENTER_DEG);
      autoState = AUTO_DRIVE;
      lastUltrasonicCheck = 0;
      setFace(FACE_NORMAL);
    }
  }
  oledDirty = true;
}

void handleBluetoothCommands() {
  while (bluetooth.available() > 0) {
    char command = (char)bluetooth.read();
    if (command >= 'a' && command <= 'z') command -= 32;
    if (!isValidBluetoothCommand(command)) continue;

    // In fallback mode, incoming valid packets refresh the heartbeat.
    if (!USE_BT_STATE) {
      heartbeatSeen = true;
      lastBtHeartbeatAt = millis();
      checkBluetooth();
    }

    if (command == 'H') continue;  // heartbeat only; never moves the car

    if (btConnected && robotMode == MODE_RC && !bootActive) {
      processRCCommand(command);
    }
  }
}

void processRCCommand(char command) {
  if (command >= '0' && command <= '9') {
    const uint8_t level = (uint8_t)(command - '0');
    if (level == 0) {
      rcSpeed = 0;
    } else {
      rcSpeed = (uint8_t)(RC_MIN_MOVING_PWM +
        (((255UL - RC_MIN_MOVING_PWM) * (level - 1)) / 8UL));
    }
    executeRCMotion();
    oledDirty = true;
    return;
  }

  rcMotionCommand = command;
  executeRCMotion();
}

void executeRCMotion() {
  if (!btConnected || robotMode != MODE_RC || bootActive) {
    stopMotors();
    return;
  }

  switch (rcMotionCommand) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    default:  stopMotors(); break;
  }
}

// -----------------------------------------------------------------------------
// 10. AUTONOMOUS EXPLORATION AND OBSTACLE AVOIDANCE
// -----------------------------------------------------------------------------
void beginObstacleAvoidance() {
  if (robotMode != MODE_AUTONOMOUS || autoState != AUTO_DRIVE) return;

  stopMotors();
  scanServo.write(SERVO_CENTER_DEG);
  scanIndex = 0;
  autoState = AUTO_SCAN_PREP;
  autoStateStartedAt = millis();

  if (irLeftDetected && irRightDetected) {
    setFace(FACE_SURPRISED);
  } else if (irLeftDetected) {
    setFace(FACE_LOOK_LEFT);
  } else if (irRightDetected) {
    setFace(FACE_LOOK_RIGHT);
  } else {
    setFace(FACE_SURPRISED);
  }
}

void autonomousNavigation() {
  if (robotMode != MODE_AUTONOMOUS || wakeSequenceActive) return;

  const unsigned long now = millis();

  switch (autoState) {
    case AUTO_DRIVE: {
      if (irLeftDetected || irRightDetected) {
        beginObstacleAvoidance();
        return;
      }

      if (now - lastUltrasonicCheck >= ULTRASONIC_CHECK_INTERVAL_MS) {
        lastUltrasonicCheck = now;
        const uint16_t distance = readUltrasonic();
        if (distance < OBSTACLE_DISTANCE_CM) {
          beginObstacleAvoidance();
          return;
        }
      }

      moveForward();
      break;
    }

    case AUTO_SCAN_PREP:
    case AUTO_SCAN_MOVE_SERVO:
    case AUTO_SCAN_SETTLE:
      scanEnvironment();
      break;

    case AUTO_TURNING:
      if (now - autoStateStartedAt >= autoTurnDuration) {
        stopMotors();
        scanServo.write(SERVO_CENTER_DEG);
        autoState = AUTO_DRIVE;
        lastUltrasonicCheck = 0;
        setFace(FACE_NORMAL);
      }
      break;

    case AUTO_RECOVERY_PAUSE:
    case AUTO_RECOVERY_TURN:
      deadEndRecovery();
      break;
  }
}

void scanEnvironment() {
  const unsigned long now = millis();

  if (autoState == AUTO_SCAN_PREP) {
    if (now - autoStateStartedAt < PRE_SCAN_FACE_MS) return;
    setFace(FACE_SCANNING);
    autoState = AUTO_SCAN_MOVE_SERVO;
  }

  if (autoState == AUTO_SCAN_MOVE_SERVO) {
    uint8_t angle = SERVO_CENTER_DEG;
    if (scanIndex == 0) angle = SERVO_LEFT_DEG;
    else if (scanIndex == 1) angle = SERVO_CENTER_DEG;
    else angle = SERVO_RIGHT_DEG;

    scanServo.write(angle);
    autoStateStartedAt = now;
    autoState = AUTO_SCAN_SETTLE;
    return;
  }

  if (autoState == AUTO_SCAN_SETTLE) {
    if (now - autoStateStartedAt < SERVO_SETTLE_MS) return;

    uint16_t distance = readUltrasonic();
    if (scanIndex == 0 && irLeftDetected) distance = 0;
    if (scanIndex == 2 && irRightDetected) distance = 0;
    scanDistances[scanIndex] = distance;

    scanIndex++;
    if (scanIndex < 3) {
      autoState = AUTO_SCAN_MOVE_SERVO;
    } else {
      scanServo.write(SERVO_CENTER_DEG);
      obstacleAvoidance();
    }
  }
}

void obstacleAvoidance() {
  // Pick the direction with the greatest measured clearance.
  // Prefer straight ahead when two or more directions have equal clearance.
  uint8_t bestIndex = 1;
  uint16_t bestDistance = scanDistances[1];

  if (scanDistances[0] > bestDistance) {
    bestDistance = scanDistances[0];
    bestIndex = 0;
  }
  if (scanDistances[2] > bestDistance) {
    bestDistance = scanDistances[2];
    bestIndex = 2;
  }

  if (bestDistance >= OPEN_PATH_DISTANCE_CM) {
    if (bestIndex == 1) {
      stopMotors();
      autoState = AUTO_DRIVE;
      lastUltrasonicCheck = 0;
      setFace(FACE_NORMAL);
    } else {
      if (bestIndex == 0) turnLeft();
      else turnRight();
      autoStateStartedAt = millis();
      autoTurnDuration = SIDE_TURN_MS;
      autoState = AUTO_TURNING;
    }
    return;
  }

  // If a recovery just happened, avoid repeatedly performing a full 180 turn.
  if ((long)(millis() - recoveryCooldownUntil) < 0) {
    if (bestIndex == 0) turnLeft();
    else if (bestIndex == 2) turnRight();
    else moveBackward();
    autoStateStartedAt = millis();
    autoTurnDuration = SIDE_TURN_MS / 2UL;
    autoState = AUTO_TURNING;
    return;
  }

  deadEndRecovery();
}

void deadEndRecovery() {
  const unsigned long now = millis();

  if (autoState != AUTO_RECOVERY_PAUSE &&
      autoState != AUTO_RECOVERY_TURN) {
    stopMotors();
    setFace(FACE_CONFUSED);
    autoState = AUTO_RECOVERY_PAUSE;
    autoStateStartedAt = now;
    recoveryCooldownUntil = now + RECOVERY_COOLDOWN_MS;
    return;
  }

  if (autoState == AUTO_RECOVERY_PAUSE) {
    if (now - autoStateStartedAt < RECOVERY_FACE_HOLD_MS) return;
    scanServo.write(SERVO_CENTER_DEG);
    turnLeft();
    autoState = AUTO_RECOVERY_TURN;
    autoStateStartedAt = now;
    return;
  }

  if (autoState == AUTO_RECOVERY_TURN) {
    if (now - autoStateStartedAt < RECOVERY_180_TURN_MS) return;
    stopMotors();
    scanServo.write(SERVO_CENTER_DEG);
    autoState = AUTO_DRIVE;
    lastUltrasonicCheck = 0;
    setFace(FACE_NORMAL);
  }
}

// -----------------------------------------------------------------------------
// 11. LDR SLEEP AND WAKE MANAGEMENT
// -----------------------------------------------------------------------------
void updatePowerMode() {
  // Bluetooth always takes priority over ambient-light sleep detection.
  if (btConnected) {
    if (robotMode != MODE_RC) {
      robotMode = MODE_RC;
      stopMotors();
      setFace(FACE_EXCITED);
    }
    return;
  }

  if (ldrDarkStable) {
    if (robotMode != MODE_SLEEP) {
      robotMode = MODE_SLEEP;
      stopMotors();
      if (servoAttached) {
        scanServo.detach();
        servoAttached = false;
      }
      wakeSequenceActive = false;
      setFace(FACE_SLEEPING);
    }
  } else if (robotMode == MODE_SLEEP) {
    wakeUpSequence();
  } else if (robotMode == MODE_RC) {
    robotMode = MODE_AUTONOMOUS;
    stopMotors();
    attachScannerServo();
    scanServo.write(SERVO_CENTER_DEG);
    autoState = AUTO_DRIVE;
    setFace(FACE_NORMAL);
  }
}

void sleepMode() {
  stopMotors();
  updateRobotFace();
  updateOLED();
}

void wakeUpSequence() {
  stopMotors();
  robotMode = MODE_AUTONOMOUS;
  attachScannerServo();
  scanServo.write(SERVO_CENTER_DEG);
  autoState = AUTO_DRIVE;
  lastUltrasonicCheck = 0;
  wakeSequenceActive = true;
  wakeStartedAt = millis();
  setFace(FACE_WAKE);
}

void updateWakeSequence() {
  if (!wakeSequenceActive) return;
  if (millis() - wakeStartedAt >= WAKE_ANIMATION_MS) {
    wakeSequenceActive = false;
    setFace(FACE_NORMAL);
    lastUltrasonicCheck = 0;
  }
}

void attachScannerServo() {
  if (!servoAttached) {
    scanServo.attach(PIN_SERVO);
    servoAttached = true;
    scanServo.write(SERVO_CENTER_DEG);
  }
}

// -----------------------------------------------------------------------------
// 12. OLED PERSONALITY AND ANIMATION
// -----------------------------------------------------------------------------
void setFace(FaceExpression expression, unsigned long durationMs) {
  faceExpression = expression;
  faceStartedAt = millis();
  faceDurationMs = durationMs;
  pupilTarget = 0;
  blinkActive = false;
  oledDirty = true;
}

void updateRobotFace() {
  const unsigned long now = millis();

  if (faceDurationMs > 0 && now - faceStartedAt >= faceDurationMs) {
    faceExpression = FACE_NORMAL;
    faceDurationMs = 0;
    faceStartedAt = now;
    oledDirty = true;
  }

  switch (faceExpression) {
    case FACE_LOOK_LEFT:
      pupilTarget = -7;
      break;
    case FACE_LOOK_RIGHT:
      pupilTarget = 7;
      break;
    case FACE_SCANNING:
      pupilTarget = ((now / 520UL) & 1U) ? 7 : -7;
      break;
    case FACE_CONFUSED:
      pupilTarget = -4;
      break;
    default:
      pupilTarget = 0;
      break;
  }

  // Time-based pupil easing: one pixel step every 35 ms.
  if (now - lastPupilStepAt >= 35UL) {
    lastPupilStepAt = now;
    if (pupilOffset < pupilTarget) {
      pupilOffset++;
      oledDirty = true;
    } else if (pupilOffset > pupilTarget) {
      pupilOffset--;
      oledDirty = true;
    }
  }

  if (faceExpression == FACE_NORMAL) {
    if (!blinkActive && now >= nextBlinkAt) {
      blinkActive = true;
      blinkStartedAt = now;
      oledDirty = true;
    } else if (blinkActive && now - blinkStartedAt >= BLINK_DURATION_MS) {
      blinkActive = false;
      nextBlinkAt = now + 2600UL + ((now / 17UL) % 1300UL);
      oledDirty = true;
    }
  } else {
    blinkActive = false;
  }

  if (faceExpression == FACE_SCANNING &&
      now - lastAnimatedFaceFrameAt >= 120UL) {
    lastAnimatedFaceFrameAt = now;
    oledDirty = true;
  }
  if (faceExpression == FACE_SLEEPING) {
    const unsigned long sleepElapsed = now - faceStartedAt;
    const unsigned long refreshInterval =
      (sleepElapsed < SLEEP_CLOSE_ANIMATION_MS) ? OLED_FRAME_INTERVAL_MS : 450UL;
    if (now - lastAnimatedFaceFrameAt >= refreshInterval) {
      lastAnimatedFaceFrameAt = now;
      oledDirty = true;
    }
  }
  if (faceExpression == FACE_WAKE) oledDirty = true;
}

void drawEye(int16_t centerX, int16_t centerY, uint8_t openHeight,
             int8_t pupilShift, bool roundEye) {
  const int16_t eyeWidth = 36;
  if (openHeight < 5) {
    display.drawFastHLine(centerX - 16, centerY, 32, SSD1306_WHITE);
    display.drawFastHLine(centerX - 13, centerY + 2, 26, SSD1306_WHITE);
    return;
  }

  int16_t top = centerY - (openHeight / 2);
  uint8_t radius = roundEye ? 15 : 8;
  if (openHeight < 15) radius = 5;

  display.fillRoundRect(centerX - (eyeWidth / 2), top,
                        eyeWidth, openHeight, radius, SSD1306_WHITE);

  if (openHeight >= 12) {
    int16_t pupilX = centerX + pupilShift;
    int16_t pupilY = centerY + 2;
    display.fillCircle(pupilX, pupilY, 5, SSD1306_BLACK);
    display.drawPixel(pupilX - 1, pupilY - 2, SSD1306_WHITE);
  }
}

void drawNormalFace() {
  const unsigned long now = millis();
  uint8_t eyeOpen = 29;

  if (faceExpression == FACE_SLEEPING) {
    const unsigned long elapsed = now - faceStartedAt;
    if (elapsed >= SLEEP_CLOSE_ANIMATION_MS) {
      eyeOpen = 3;
    } else {
      eyeOpen = (uint8_t)(29UL - (26UL * elapsed) / SLEEP_CLOSE_ANIMATION_MS);
    }
  } else if (faceExpression == FACE_WAKE) {
    const unsigned long elapsed = now - faceStartedAt;
    if (elapsed >= WAKE_ANIMATION_MS) eyeOpen = 29;
    else eyeOpen = (uint8_t)(3UL + (26UL * elapsed) / WAKE_ANIMATION_MS);
  } else if (faceExpression == FACE_SURPRISED) {
    eyeOpen = 35;
  } else if (faceExpression == FACE_EXCITED) {
    eyeOpen = 31;
  } else if (blinkActive && faceExpression == FACE_NORMAL) {
    eyeOpen = 3;
  }

  const bool roundEye = (faceExpression == FACE_SURPRISED ||
                         faceExpression == FACE_EXCITED);

  drawEye(35, 28, eyeOpen, pupilOffset, roundEye);
  drawEye(93, 28, eyeOpen, pupilOffset, roundEye);

  if (faceExpression == FACE_CONFUSED) {
    display.drawLine(17, 7, 49, 12, SSD1306_WHITE);
    display.drawLine(78, 11, 109, 6, SSD1306_WHITE);
  } else if (faceExpression == FACE_EXCITED) {
    display.drawLine(17, 8, 50, 3, SSD1306_WHITE);
    display.drawLine(78, 3, 111, 8, SSD1306_WHITE);
  } else if (faceExpression == FACE_SURPRISED) {
    display.drawCircle(35, 28, 20, SSD1306_WHITE);
    display.drawCircle(93, 28, 20, SSD1306_WHITE);
  }

  if (faceExpression == FACE_SLEEPING) {
    uint8_t zOffset = (uint8_t)((now / 350UL) % 5UL);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(103, 5 - zOffset);
    display.print(F("Zz"));
  }
}

void updateOLED() {
  if (!oledReady) return;

  const unsigned long now = millis();
  const bool animated = (faceExpression == FACE_SCANNING ||
                         faceExpression == FACE_SLEEPING ||
                         faceExpression == FACE_WAKE || blinkActive);
  if (!oledDirty && !animated) return;
  if (now - lastOLEDFrameAt < OLED_FRAME_INTERVAL_MS) return;

  display.clearDisplay();
  drawNormalFace();
  display.drawFastHLine(0, 51, 128, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 55);

  if (robotMode == MODE_RC) {
    display.print(F("RC MODE"));
    display.setCursor(78, 55);
    display.print(F("SPD "));
    display.print((uint8_t)((rcSpeed * 100UL) / 255UL));
  } else if (robotMode == MODE_SLEEP) {
    display.print(F("SLEEP MODE"));
  } else if (autoState == AUTO_SCAN_PREP ||
             autoState == AUTO_SCAN_MOVE_SERVO ||
             autoState == AUTO_SCAN_SETTLE) {
    display.print(F("SCANNING..."));
  } else if (autoState == AUTO_RECOVERY_PAUSE ||
             autoState == AUTO_RECOVERY_TURN) {
    display.print(F("DEAD-END RECOVERY"));
  } else {
    display.print(F("AUTO EXPLORING"));
  }

  display.display();
  lastOLEDFrameAt = now;
  oledDirty = false;
}

// -----------------------------------------------------------------------------
// 13. PREMIUM NON-BLOCKING BOOT ANIMATION
// -----------------------------------------------------------------------------
void drawBootFrame() {
  if (!oledReady) return;
  display.clearDisplay();

  const unsigned long elapsed = millis() - bootStageStartedAt;
  const uint8_t scanX = (uint8_t)((elapsed / 5UL) % 128UL);

  switch (bootStage) {
    case 0:  // Stage 1: completely black
      break;

    case 1:  // Stage 2: small scanning light
      display.drawFastHLine(15, 32, 98, SSD1306_WHITE);
      display.fillCircle(scanX, 32, 2, SSD1306_WHITE);
      display.drawPixel(scanX, 28, SSD1306_WHITE);
      break;

    case 2:  // Stage 3: futuristic scan sweep
      for (uint8_t x = 8; x < 128; x += 16) {
        display.drawFastVLine(x, 9, 46, SSD1306_WHITE);
      }
      display.drawFastHLine(0, 31, 128, SSD1306_WHITE);
      display.fillRect(scanX, 8, 3, 48, SSD1306_WHITE);
      display.drawRect(2, 2, 124, 60, SSD1306_WHITE);
      break;

    case 3: { // Stage 4: robot eyes gradually open
      uint8_t opening = (elapsed >= 650UL)
        ? 30 : (uint8_t)(3UL + (27UL * elapsed) / 650UL);
      drawEye(35, 30, opening, 0, false);
      drawEye(93, 30, opening, 0, false);
      display.drawFastHLine(10, 54, 108, SSD1306_WHITE);
      break;
    }

    case 4: { // Stage 5: eyes look left, then right
      int8_t glance = ((elapsed / 300UL) & 1U) ? 7 : -7;
      drawEye(35, 30, 30, glance, false);
      drawEye(93, 30, 30, glance, false);
      display.drawFastHLine(10, 54, 108, SSD1306_WHITE);
      break;
    }

    case 5: { // Stage 6: initializing
      display.drawRect(3, 3, 122, 58, SSD1306_WHITE);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(41, 12);
      display.print(F("SYSTEM"));
      display.setCursor(22, 27);
      display.print(F("INITIALIZING..."));
      uint8_t progress = (uint8_t)((elapsed * 100UL) / 800UL);
      if (progress > 100) progress = 100;
      display.drawRect(17, 45, 94, 9, SSD1306_WHITE);
      display.fillRect(19, 47, (uint8_t)(90UL * progress / 100UL), 5,
                       SSD1306_WHITE);
      break;
    }

    case 6:  // Stage 7: online
      display.drawRoundRect(5, 5, 118, 54, 7, SSD1306_WHITE);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(29, 14);
      display.print(F("ROBOT SYSTEM"));
      display.setTextSize(2);
      display.setCursor(34, 30);
      display.print(F("ONLINE"));
      display.drawCircle(110, 14, 5, SSD1306_WHITE);
      display.drawLine(107, 14, 109, 16, SSD1306_WHITE);
      display.drawLine(109, 16, 113, 11, SSD1306_WHITE);
      break;

    case 7:  // Stage 8: creator credit
      display.drawRoundRect(3, 3, 122, 58, 7, SSD1306_WHITE);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(39, 12);
      display.print(F("POWERED BY"));
      display.setTextSize(2);
      display.setCursor(4, 28);
      display.print(F("SURAJ"));
      display.setCursor(64, 28);
      display.print(F("SINGH"));
      display.drawFastHLine(20, 52, 88, SSD1306_WHITE);
      break;
  }

  display.display();
}

void bootAnimation() {
  if (!bootActive) return;

  static const uint16_t stageDurationMs[8] = {
    300, 550, 800, 650, 700, 800, 700, 1300
  };

  const unsigned long now = millis();
  if (bootStage < 8 && now - bootStageStartedAt >= stageDurationMs[bootStage]) {
    bootStageStartedAt += stageDurationMs[bootStage];
    bootStage++;
    if (bootStage >= 8) {
      finishBootAnimation();
      return;
    }
    lastBootFrameAt = 0;
  }

  if (now - lastBootFrameAt >= 45UL) {
    lastBootFrameAt = now;
    drawBootFrame();
  }
}

void finishBootAnimation() {
  bootActive = false;
  stopMotors();

  if (btConnected) {
    robotMode = MODE_RC;
    rcMotionCommand = 'S';
    setFace(FACE_EXCITED);
  } else if (ldrDarkStable) {
    robotMode = MODE_SLEEP;
    if (servoAttached) {
      scanServo.detach();
      servoAttached = false;
    }
    setFace(FACE_SLEEPING);
  } else {
    robotMode = MODE_AUTONOMOUS;
    autoState = AUTO_DRIVE;
    attachScannerServo();
    scanServo.write(SERVO_CENTER_DEG);
    setFace(FACE_NORMAL);
  }
  oledDirty = true;
}
