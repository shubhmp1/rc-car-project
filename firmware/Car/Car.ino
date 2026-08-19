#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <VL53L1X.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define ENApin   33   // PWM speed signal to L298N ENA pin
#define IN1pin   32   // L298N IN1 - direction control
#define IN2pin   4    // L298N IN2 - opposite of IN1
#define servoPin 26   // servo signal wire (yellow)

#define RF_CE  27     // controls radio mode (transmit vs listen)
#define RF_CSN 25     // SPI chip select
#define RF_SCK 14     // SPI clock
#define RF_MO  13     // MOSI, ESP32 sends data TO radio
#define RF_MI  35     // MISO, radio sends data TO ESP32 (input-only pin, perfect for MISO)

// ============================================================
// SENSOR PIN DEFINITIONS
// existing car pins above are NOT changed
// these pins are only added for semi-auto sensor readings
//
// TCRT5000 IR sensors:
//   left sensor OUT  -> GPIO34
//   right sensor OUT -> GPIO39
//
// VL53L0X ToF sensor:
//   SDA -> GPIO21
//   SCL -> GPIO22
// ============================================================
#define IR_LEFT_PIN   34
#define IR_RIGHT_PIN  16

#define TOF_SDA       21
#define TOF_SCL       22

// ============================================================
// PWM CONFIGURATION
// motor uses 5000Hz, servo uses 50Hz (standard for positional servos)
// 8 bit resolution = 0 to 255 duty cycle values
// ============================================================
int PWMChannel    = 0;
int servoChannel  = 2;
int pwmFreq       = 5000;
int servoFreq     = 50;
int pwmResolution = 8;

// ============================================================
// DEADZONE
// same value as remote so behavior is consistent
// ============================================================
float deadzone = 0.025;

// ============================================================
// CURRENT MODE
// tracks the mode sent from the remote in each packet
// sensors only take measurements when currentMode == 2
// ============================================================
int currentMode = 1;
int prevCarMode = 1;   // NEW - tracks previous mode so we can detect leaving mode 2

// ============================================================
// SEMI-AUTO (MODE 2) TUNING CONSTANTS
// adjust these on hardware - see notes at the bottom of the reply
// ============================================================
const int   IR_BLACK_VALUE        = HIGH;   // flip to LOW if steering nudges the wrong way
const int   TOF_SLOWDOWN_START_MM = 400;    // start slowing down under this distance
const int   TOF_HARD_STOP_MM      = 100;    // hard stop at/under this distance
const int   AUTO_STEER_NUDGE      = 2;      // servo duty nudge per cycle when off-line
const float AUTO_SPEED_STEP       = 0.02f;  // how fast auto speed ramps up/down per cycle

// Dynamic ToF braking:
// 100% speed needs 1700 mm to stop, plus a 50 mm safety buffer.
const float TOF_MAX_STOP_DISTANCE_MM = 1700.0f;
const float TOF_SAFETY_BUFFER_MM    = 50.0f;
const int   TOF_CONFIRM_READS        = 3;

// CHANGED - two consecutive black readings from one sensor trigger
// a full steering correction that is held for 0.75 seconds.
const int AUTO_BLACK_CONFIRM_READS = 2;
const unsigned long AUTO_FULL_CORRECTION_MS = 750;

// Servo duty 19 is centered. 26 is about full right; 24 is about 35 degrees right.
const int AUTO_STEER_RIGHT_CORRECTION_DUTY = 24;

// ============================================================
// SEMI-AUTO (MODE 2) RUNTIME STATE
// ============================================================
bool  autoStoppedLine  = false;  // kept for telemetry compatibility; Mode 2 never stops for line reads
float autoCurrentSpeed = 0.0f;   // 0.0-1.0 ramped speed magnitude while in auto mode
int   autoServoDuty    = 19;     // current auto-mode steering position, starts centered


// Dynamic ToF braking state.
// Braking only starts after 3 unsafe reads and releases after 3 safe reads.
bool autoTofBraking = false;
int  autoTofUnsafeStreak = 0;
int  autoTofSafeStreak   = 0;





// CHANGED - remembers the full correction direction and its minimum hold time.
unsigned long autoFullCorrectionUntil = 0;
int autoFullCorrectionDuty = 19;


const bool ENABLE_TOF_SAFETY = true;  // false = ignore ToF for steering testing

// ============================================================
// AUTO STEERING RANGE
// full steering lock (matches mode 1's max range) - snaps here
// once a sensor confirms black 3 cycles in a row
// ============================================================
#define AUTO_STEER_CENTER_DUTY 19
#define AUTO_STEER_HALF_LEFT   13   // full left (was 16 for half-angle)
#define AUTO_STEER_HALF_RIGHT  26   // full right (was 23 for half-angle)





// ============================================================
// AUTO STEERING STREAK COUNTERS
// counts consecutive cycles each individual sensor has seen black.
// after two reads, steering uses the full correction angle for 0.75s.
// ============================================================
int autoLeftBlackStreak  = 0;
int autoRightBlackStreak = 0;

// ============================================================
// PACKET STRUCTURE
// must exactly match the remote control struct
// if these dont match, received data will be garbage
// armed / targetSpeed = NEW, appended at the end so nothing shifts
// ============================================================
struct ControlPacket {
  float normalizedY;   // motor speed  (-1.0 = full forward, 1.0 = full reverse)
  float normalizedX;   // steering     (-1.0 = full left,    1.0 = full right)
  int   mode;          // current mode sent from remote
  bool  armed;          // NEW - true once semi-auto cruise driving is armed
  float targetSpeed;    // NEW - 0.0-1.0 locked cruise speed while armed
};

// ============================================================
// SENSOR TELEMETRY STRUCTURE
// sent back to the remote using nRF24 ACK payloads
// this must exactly match the remote SensorTelemetry struct
// new fields appended at the end, same as ControlPacket
// ============================================================
struct SensorTelemetry {
  int   irLeft;
  int   irRight;
  int   tofMm;
  bool  tofValid;
  float currentSpeed;     // NEW - car's current ramped auto-drive speed (0.0-1.0)
  bool  leftOnLine;        // NEW - true if left IR sensor currently sees black
  bool  rightOnLine;        // NEW - true if right IR sensor currently sees black
  bool  slowing;           // NEW - true while ToF is proportionally slowing the car
  bool  stoppedObstacle;   // NEW - true while hard-stopped due to ToF distance
  bool  stoppedLine;       // NEW - true while latched-stopped due to both IR seeing black
  float tofThresholdMm;    // NEW - current computed stop-braking threshold, in mm
  int   tofStreak;         // NEW - consecutive reads counted toward the active streak
  bool  tofBraking;        // NEW - true if currently in the ToF braking state
};

// RF radio object
RF24 radio(RF_CE, RF_CSN);

// VL53L0X ToF sensor object
VL53L1X tof;

// address must match remote exactly
const byte address[6] = "00001";

ControlPacket packet;
SensorTelemetry sensorData;

int duty      = 0;
int servoDuty = 0;

// counter for debugging
int rxCount = 0;

// ============================================================
// SENSOR TIMING
// sensor readings are refreshed every 50ms
// latest values are queued into nRF24 ACK payloads
// so the remote can display them in mode 2
// ============================================================
unsigned long lastSensorUpdate = 0;
const unsigned long sensorInterval = 50;

bool tofReady = false;

// ============================================================
// STOP CAR
// sets motor output to neutral
// used whenever packet values are in the deadzone
// ============================================================
void stopCar() {
  digitalWrite(IN1pin, LOW);
  digitalWrite(IN2pin, LOW);
  ledcWriteChannel(PWMChannel, 0);
}

// ============================================================
// UPDATE AUTO DRIVE
// runs the Mode 2 IR steering correction + ToF sensor logic
// called every time a packet is received while currentMode == 2
// reads IR fresh every call (ToF is refreshed on its own timer by
// updateSensors()), drives the motor/servo directly, and fills in
// the telemetry fields the remote uses for the driving screen
// ============================================================
void updateAutoDrive() {
  bool leftBlack  = (digitalRead(IR_LEFT_PIN)  == IR_BLACK_VALUE);
  bool rightBlack = (digitalRead(IR_RIGHT_PIN) == IR_BLACK_VALUE);
  unsigned long now = millis();

  // CHANGED - Mode 2 never stops because of black line readings.
  autoStoppedLine = false;

  // CHANGED - one single-sensor black read triggers correction.
  // The correction stays active for a full second even after the sensor clears.
  if (packet.armed) {
    if (now < autoFullCorrectionUntil) {
      // Keep the last full correction angle for one second.
      autoServoDuty = autoFullCorrectionDuty;
    } else if (leftBlack && !rightBlack) {
      // Keep your existing left-side full correction: about 45 degrees.
      autoServoDuty = AUTO_STEER_HALF_LEFT;
      autoFullCorrectionDuty = AUTO_STEER_HALF_LEFT;
      autoFullCorrectionUntil = now + AUTO_FULL_CORRECTION_MS;
    } else if (rightBlack && !leftBlack) {
      // CHANGED - right-side correction is about 35 degrees, not 45.
      autoServoDuty = AUTO_STEER_RIGHT_CORRECTION_DUTY;
      autoFullCorrectionDuty = AUTO_STEER_RIGHT_CORRECTION_DUTY;
      autoFullCorrectionUntil = now + AUTO_FULL_CORRECTION_MS;
    } else if (!leftBlack && !rightBlack) {
      // No line detected and the one-second correction has finished:
      // gently return to center.
      if (autoServoDuty > AUTO_STEER_CENTER_DUTY) autoServoDuty--;
      else if (autoServoDuty < AUTO_STEER_CENTER_DUTY) autoServoDuty++;
    }
  } else {
    autoServoDuty = AUTO_STEER_CENTER_DUTY;
    autoFullCorrectionUntil = 0;
    autoFullCorrectionDuty = AUTO_STEER_CENTER_DUTY;
  }

  autoServoDuty = constrain(autoServoDuty, AUTO_STEER_HALF_LEFT, AUTO_STEER_HALF_RIGHT);
  // ToF: figure out how much the obstacle should scale down our allowed speed
  // ============================================================
  // DYNAMIC ToF BRAKING
  //
  // At full speed, stopping distance is 1700 mm.
  // At any lower current speed, stopping distance scales linearly:
  //
  // stoppingDistance = 1700 mm * currentSpeedMagnitude
  //
  // A 50 mm safety buffer is added. Three unsafe readings begin
  // braking; three safe readings allow acceleration again.
  //
  // NEW - tofThresholdMm and tofStreak are reported back to the
  // remote every cycle so the driving screen can show the live
  // trigger distance and how many consecutive reads are in the
  // currently-active streak.
  // ============================================================
  bool  stoppedObstacle = false;
  bool  slowing         = false;
  float allowedSpeed    = 0.0f;
  float tofThresholdMm  = 0.0f;
  int   tofStreak       = 0;

  if (packet.armed) {
    float targetSpeed = constrain(packet.targetSpeed, -1.0f, 1.0f);
    float targetMagnitude = fabsf(targetSpeed);
    float targetDirection = targetSpeed >= 0.0f ? 1.0f : -1.0f;

    // Normally we are allowed to work toward the selected target speed.
    float allowedMagnitude = targetMagnitude;

    if (ENABLE_TOF_SAFETY && sensorData.tofValid) {
      float currentMagnitude = fabsf(autoCurrentSpeed);

      // Example:
      // at 100%: 1700 + 50 = 1750 mm
      // at 50%:   850 + 50 = 900 mm
      tofThresholdMm =
        (TOF_MAX_STOP_DISTANCE_MM * currentMagnitude) +
        TOF_SAFETY_BUFFER_MM;

      bool obstacleTooClose = sensorData.tofMm <= tofThresholdMm;

      if (!autoTofBraking) {
        // Need three consecutive unsafe readings before braking.
        if (obstacleTooClose) {
          autoTofUnsafeStreak++;
          autoTofSafeStreak = 0;

          if (autoTofUnsafeStreak >= TOF_CONFIRM_READS) {
            autoTofBraking = true;
            autoTofUnsafeStreak = 0;
          }
        } else {
          autoTofUnsafeStreak = 0;
        }
      } else {
        // Already braking: need three consecutive safe readings
        // before accelerating toward the target again.
        if (obstacleTooClose) {
          autoTofSafeStreak = 0;
        } else {
          autoTofSafeStreak++;

          if (autoTofSafeStreak >= TOF_CONFIRM_READS) {
            autoTofBraking = false;
            autoTofSafeStreak = 0;
          }
        }
      }

      // NEW - report whichever streak is currently "live": the
      // unsafe streak while approaching the trigger, or the safe
      // streak while braking and waiting to release.
      tofStreak = autoTofBraking ? autoTofSafeStreak : autoTofUnsafeStreak;

      if (autoTofBraking) {
        // Work out the maximum safe speed from the current distance.
        // Example: 900 mm distance:
        // (900 - 50) / 1700 = 0.50, so maximum safe speed is 50%.
        float safeMagnitude =
          (sensorData.tofMm - TOF_SAFETY_BUFFER_MM) /
          TOF_MAX_STOP_DISTANCE_MM;

        safeMagnitude = constrain(safeMagnitude, 0.0f, 1.0f);

        // Never exceed the selected target, but slow if required.
        allowedMagnitude = fmin(targetMagnitude, safeMagnitude);
        slowing = allowedMagnitude < targetMagnitude;

        if (allowedMagnitude <= 0.01f) {
          stoppedObstacle = true;
        }
      }
    } else if (!ENABLE_TOF_SAFETY) {
      // ToF intentionally disabled: clear old braking state.
      autoTofBraking = false;
      autoTofUnsafeStreak = 0;
      autoTofSafeStreak = 0;
    }

    allowedSpeed = targetDirection * allowedMagnitude;
  } else {
    // Reset ToF braking when Mode 2 is disarmed.
    autoTofBraking = false;
    autoTofUnsafeStreak = 0;
    autoTofSafeStreak = 0;
  }

      // ============================================================
  // SPEED RAMPING
  // Steps autoCurrentSpeed toward allowedSpeed by AUTO_SPEED_STEP
  // each cycle instead of jumping instantly. This was missing -
  // without it, autoCurrentSpeed never left 0, so the ToF threshold
  // (which scales off actual current speed) never left 50mm
  // (just the safety buffer), and the car never actually reached
  // its target speed either.
  // ============================================================
  if (autoCurrentSpeed < allowedSpeed) {
    autoCurrentSpeed = fmin(autoCurrentSpeed + AUTO_SPEED_STEP, allowedSpeed);
  } else if (autoCurrentSpeed > allowedSpeed) {
    autoCurrentSpeed = fmax(autoCurrentSpeed - AUTO_SPEED_STEP, allowedSpeed);
  }

  // CHANGED - match Mode 1 convention:
  // negative = physical forward, positive = physical reverse. 
  if (autoCurrentSpeed < -0.01f) {
    digitalWrite(IN1pin, HIGH );
    digitalWrite(IN2pin, LOW);                   
    ledcWriteChannel(PWMChannel, (int)(-autoCurrentSpeed * 255));
  } else if (autoCurrentSpeed > 0.01f) {
    digitalWrite(IN1pin, LOW);
    digitalWrite(IN2pin, HIGH);
    ledcWriteChannel(PWMChannel, (int)(autoCurrentSpeed * 255));
  } else {
    stopCar();
  }

  ledcWriteChannel(servoChannel, autoServoDuty);

  // fill in the telemetry fields the remote's driving screen reads
  sensorData.currentSpeed    = autoCurrentSpeed;
  sensorData.leftOnLine      = rightBlack;
  sensorData.rightOnLine     = leftBlack;
  sensorData.slowing         = slowing;
  sensorData.stoppedObstacle = stoppedObstacle;
  sensorData.stoppedLine     = false;  // Mode 2 never stops for the line
  sensorData.tofThresholdMm  = tofThresholdMm;   // NEW
  sensorData.tofStreak       = tofStreak;        // NEW
  sensorData.tofBraking      = autoTofBraking;   // NEW
}

// ============================================================
// UPDATE SENSOR READINGS
// reads both IR sensors and the VL53L0X distance
// then loads the newest values into the nRF24 ACK payload
// only takes measurements when currentMode == 2
// ============================================================
void updateSensors() {
  if (currentMode == 2) {
    sensorData.irLeft  = digitalRead(IR_LEFT_PIN);
    sensorData.irRight = digitalRead(IR_RIGHT_PIN);

    if (tofReady) {
      int distance = tof.read();

      if (!tof.timeoutOccurred()) {
        sensorData.tofMm = distance;
        sensorData.tofValid = true;
      } else {
        sensorData.tofMm = -1;
        sensorData.tofValid = false;
      }
    } else {
      sensorData.tofMm = -1;
      sensorData.tofValid = false;
    }
  }

    // --- DEBUG: live ToF distance measurement ---
  // prints the VL53L0X reading every time updateSensors() runs
  //Serial.printf("TOF Distance: %d mm    Valid: %s\n",
                //sensorData.tofMm,
                //sensorData.tofValid ? "YES" : "NO");

  radio.writeAckPayload(1, &sensorData, sizeof(sensorData));
}

void setup() {
  Serial.begin(115200);

  // IN1 and IN2 are direction outputs to L298N
  // forward  = IN1 HIGH, IN2 LOW
  // reverse  = IN1 LOW,  IN2 HIGH
  // stop     = IN1 LOW,  IN2 LOW
  pinMode(IN1pin, OUTPUT);
  pinMode(IN2pin, OUTPUT);

  // IR sensors are digital inputs
  // sensor output depends on reflection and the module potentiometer setting
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);

  // attach motor PWM to ENA on L298N
  // ledcAttachChannel(pin, frequency, resolution, channel)
  ledcAttachChannel(ENApin, pwmFreq, pwmResolution, PWMChannel);

  // attach servo PWM to its channel
  ledcAttachChannel(servoPin, servoFreq, pwmResolution, servoChannel);

  // start I2C bus for VL53L0X
  Wire.begin(TOF_SDA, TOF_SCL);

  // initialize VL53L1X
  // if it fails, the car still drives, but telemetry shows NO READ
  tof.setTimeout(500);
  if (tof.init()) {
    tof.setDistanceMode(VL53L1X::Long);
    tof.setMeasurementTimingBudget(50000);
    tof.startContinuous(50);
    tofReady = true;
    Serial.println("VL53L1X ready");
  } else {
    tofReady = false;
    Serial.println("VL53L1X NOT FOUND - check wiring");
  }

  // start SPI bus with remapped pins
  // SPI.begin(SCK, MISO, MOSI)
  SPI.begin(RF_SCK, RF_MI, RF_MO);

  // initialize RF module
  if (!radio.begin()) {
    Serial.println("RF NOT FOUND - check wiring");
    while (1);
  }

  // must match remote data rate exactly
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);

  // ACK payload lets the car send sensor readings back
  // every time the remote sends a normal control packet
  radio.enableAckPayload();

  // pipe 1 listens on the same address as remote writes to
  radio.openReadingPipe(1, address);
  radio.startListening();  // receive only

  sensorData.irLeft          = 0;
  sensorData.irRight         = 0;
  sensorData.tofMm           = -1;
  sensorData.tofValid        = false;
  sensorData.currentSpeed    = 0.0f;
  sensorData.leftOnLine      = false;
  sensorData.rightOnLine     = false;
  sensorData.slowing         = false;
  sensorData.stoppedObstacle = false;
  sensorData.stoppedLine     = false;
  sensorData.tofThresholdMm  = 0.0f;   // NEW
  sensorData.tofStreak       = 0;      // NEW
  sensorData.tofBraking      = false;  // NEW
  updateSensors();

  Serial.println("Car ready");
}

void loop() {

  unsigned long currentTime = millis();

  // update sensor readings on a timed interval
  // this keeps telemetry fresh without blocking motor control
  if (currentTime - lastSensorUpdate >= sensorInterval) {
    lastSensorUpdate = currentTime;
    updateSensors();
  }

  // check if a packet arrived from the remote
  if (radio.available()) {

    // read packet bytes into our struct
    radio.read(&packet, sizeof(packet));
    rxCount++;

    // update current mode from the packet
    currentMode = packet.mode;

    // if we've just left mode 2, reset all auto-drive state so a
    // fresh entry into mode 2 later starts from a clean slate
    if (packet.mode != prevCarMode) {
      if (packet.mode != 2) {
        autoStoppedLine  = false;
        autoCurrentSpeed = 0.0f;
        autoServoDuty    = 19;
        autoLeftBlackStreak  = 0;
        autoRightBlackStreak = 0;

      // reset the 0.75-second full-correction timer too
        autoFullCorrectionUntil = 0;
        autoFullCorrectionDuty = AUTO_STEER_CENTER_DUTY;


        sensorData.currentSpeed    = 0.0f;
        sensorData.leftOnLine      = false;
        sensorData.rightOnLine     = false;
        sensorData.slowing         = false;
        sensorData.stoppedObstacle = false;
        sensorData.stoppedLine     = false;
        sensorData.tofThresholdMm  = 0.0f;   // NEW
        sensorData.tofStreak       = 0;      // NEW
        sensorData.tofBraking      = false;  // NEW
      }
      prevCarMode = packet.mode;
    }

    // queue newest sensor readings for the next ACK response
    radio.writeAckPayload(1, &sensorData, sizeof(sensorData));

    if (currentMode == 2) {
      // ============================================================
      // SEMI-AUTO MODE 2
      // fully autonomous - ignores packet.normalizedY/X, drives itself
      // off IR line sensors + ToF distance instead
      // ============================================================
      updateAutoDrive();
    }
    else {
      // ============================================================
      // MOTOR CONTROL
      // convert normalizedY into PWM duty cycle and direction signal
      // cubic curve gives exponential feel - small inputs = small response
      // large inputs = large response, much more natural to drive
      // ENA = PWM speed, IN1+IN2 = direction
      // ============================================================
      duty = abs(packet.normalizedY * packet.normalizedY) * 255;

      if (packet.normalizedY < -deadzone) {
        // forward - IN1 HIGH, IN2 LOW, PWM on
        digitalWrite(IN1pin, HIGH);
        digitalWrite(IN2pin, LOW);
        ledcWriteChannel(PWMChannel, duty);
      }
      else if (packet.normalizedY > deadzone) {
        // reverse - IN1 LOW, IN2 HIGH, PWM on
        digitalWrite(IN1pin, LOW);
        digitalWrite(IN2pin, HIGH);
        ledcWriteChannel(PWMChannel, duty);
      }
      else {
        // deadzone - stop motor completely
        stopCar();
      }

      // ============================================================
      // SERVO CONTROL
      // convert normalizedX into servo PWM duty cycle
      // 8 bit servo duty range: 13 = 0deg (right), 26 = 180deg (left)
      // center = ~19-20 = 90deg
      // ============================================================
      if (abs(packet.normalizedX) <= deadzone) {
        // centered - hold middle position
        servoDuty = 19;
      }
      else {
        // map -1.0 to 1.0 into servo duty range 13 to 26
        servoDuty = (int)(13 + (packet.normalizedX + 1.0) * 6.5);
        servoDuty = constrain(servoDuty, 13, 26);
      }

      ledcWriteChannel(servoChannel, servoDuty);
    }

    // --- DEBUG: raw received packet values ---
    //Serial.printf("Received  Motor: %.3f    Steering: %.3f\n", packet.normalizedY, packet.normalizedX);

    // --- DEBUG: sensor telemetry ---
    //Serial.printf("IR L: %d    IR R: %d    TOF: %d mm    Valid: %s\n",
                  //sensorData.irLeft,
                  //sensorData.irRight,
                  //sensorData.tofMm,
                  //sensorData.tofValid ? "YES" : "NO");

    // --- DEBUG: total packets received ---
    //Serial.printf("Packets received: %d\n", rxCount);
  }
}