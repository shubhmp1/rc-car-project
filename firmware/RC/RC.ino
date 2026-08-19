#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TFT_eSPI.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define joystickPinX 35   // joystick X axis analog input
#define joystickPinY 34   // joystick Y axis analog input
#define joystickSW   32   // joystick button (SW) - triggers animation replay
                          // wire: SW -> pin 32, GND -> GND
                          // uses internal pull-up so no resistor needed

#define RF_CE  27         // controls radio mode (transmit vs listen)
#define RF_CSN 25         // SPI chip select, tells ESP32 to talk to radio
#define RF_SCK 14         // SPI clock, synchronizes communication timing
#define RF_MO  13         // MOSI, ESP32 sends data TO radio
#define RF_MI  26         // MISO, radio sends data back TO ESP32

// ============================================================
// MODE BUTTON + LED PIN DEFINITIONS
// based on the schematic:
//
// SW1:
//   button input = GPIO19, uses internal pull-up
//   LED output   = GPIO15 through resistor to LED to GND
//
// SW2:
//   button input = GPIO21, uses internal pull-up
//   LED output   = GPIO33 through resistor to LED to GND
//
// SW3:
//   button input = GPIO22, uses internal pull-up
//   LED output   = GPIO12 through resistor to LED to GND
// ============================================================
#define MODE1_BUTTON 19   // SW1 - RC mode button
#define MODE2_BUTTON 21   // SW2 - semi-auto mode button
#define MODE3_BUTTON 22   // SW3 - AI mode button

#define MODE1_LED    15   // LED for RC mode
#define MODE2_LED    33   // LED for semi-auto mode
#define MODE3_LED    12   // LED for AI mode

// ============================================================
// MODE DEFINITIONS
// MODE_RC is the default mode when the remote starts
// MODE_SEMI_AUTO now drives the car (see below)
// MODE_AI does nothing for now except light its LED
// only MODE_RC runs the normal joystick driving code
// ============================================================
#define MODE_RC        1
#define MODE_SEMI_AUTO 2
#define MODE_AI        3

int currentMode = MODE_RC;   // remote starts in normal RC car mode

// ============================================================
// MODE BUTTON DEBOUNCE
// these buttons are wired to GND and use ESP32 internal pull-ups
// HIGH = not pressed
// LOW  = pressed
// each button has its own stable state, last raw reading, and timer
// ============================================================
const int modeButtonPins[3] = { MODE1_BUTTON, MODE2_BUTTON, MODE3_BUTTON };
const int modeLedPins[3]    = { MODE1_LED,    MODE2_LED,    MODE3_LED };

bool modeLastReading[3] = { HIGH, HIGH, HIGH };
bool modeButtonState[3] = { HIGH, HIGH, HIGH };
bool modeLastState[3]   = { HIGH, HIGH, HIGH };
unsigned long modeLastDebounceTime[3] = { 0, 0, 0 };

const unsigned long modeDebounceDelay = 50;  // 50ms debounce window

// ============================================================
// JOYSTICK CALIBRATION
// center values are the resting position of the joystick
// ============================================================
int joystickCenterY = 1925;
int joystickCenterX = 1875;

// ============================================================
// DEADZONE
// joystick must move this far from center before anything happens
// prevents drift from imperfect joystick centering
// ============================================================
float deadzone = 0.025;

// ============================================================
// SLEW RATE
// instead of jumping instantly to target speed, we step toward it
// this prevents mechanical stress and feels more natural to drive
// sameDirectionStep = how fast we accelerate/decelerate same direction
// oppDirectionStep  = how fast we transition through zero when reversing
// ============================================================
float currentNormalizedY = 0;    // current motor speed, starts at rest
float sameDirectionStep = 0.015; // 0 to full speed in ~200ms
float oppDirectionStep  = 0.008; // reversal through zero in ~375ms

// ============================================================
// TIMING
// we dont recalculate every single loop iteration
// motor slew runs every 3ms, steering + RF send runs every 50ms
// semi-auto screen updates + RF send every 50ms too
// ============================================================
unsigned long lastMotorUpdate = 0;
unsigned long lastServoUpdate = 0;
unsigned long lastAutoUpdate  = 0;
const unsigned long motorInterval = 3;
const unsigned long servoInterval = 50;
const unsigned long autoInterval  = 50;

// ============================================================
// PACKET STRUCTURE
// this is exactly what gets sent over RF to the car every 50ms
// normalizedY = motor speed  (-1.0 = full reverse, 1.0 = full forward)
// normalizedX = steering     (-1.0 = full left,    1.0 = full right)
// mode        = current remote mode, so the car knows when to run sensors
// armed       = NEW - true once semi-auto cruise driving has been armed
// targetSpeed = NEW - 0.0-1.0 locked cruise speed while armed
// ============================================================
struct ControlPacket {
  float normalizedY;
  float normalizedX;
  int   mode;
  bool  armed;
  float targetSpeed;
};

// ============================================================
// SENSOR TELEMETRY STRUCTURE
// this comes BACK from the car using nRF24 ACK payloads
// must match the car-side SensorTelemetry struct exactly
//
// irLeft / irRight:
//   digital values from the two TCRT5000 IR reflection sensors
//
// tofMm:
//   VL53L0X distance measurement in millimeters
//
// tofValid:
//   true  = ToF reading is valid
//   false = ToF reading failed / sensor not found / timeout
//
// currentSpeed / leftOnLine / rightOnLine / slowing / stoppedObstacle / stoppedLine:
//   semi-auto driving status reported back by the car each cycle
//
// tofThresholdMm / tofStreak / tofBraking:
//   NEW - live ToF braking debug info reported back by the car each
//   cycle: the current computed stop-trigger distance in mm, how
//   many consecutive reads are in the currently-active streak, and
//   whether the car is currently in the braking state.
// ============================================================
struct SensorTelemetry {
  int   irLeft;
  int   irRight;
  int   tofMm;
  bool  tofValid;
  float currentSpeed;
  bool  leftOnLine;
  bool  rightOnLine;
  bool  slowing;
  bool  stoppedObstacle;
  bool  stoppedLine;
  float tofThresholdMm;
  int   tofStreak;
  bool  tofBraking;
};

SensorTelemetry sensorData = { 0, 0, -1, false, 0.0f, false, false, false, false, false, 0.0f, 0, false };

// RF radio object using CE and CSN pins
RF24 radio(RF_CE, RF_CSN);

// SPI for Display
TFT_eSPI tft = TFT_eSPI();

// address - both radios must match exactly
const byte address[6] = "00001";

float nextNormalizedY = 0;
float normalizedX     = 0;
ControlPacket packet;

// counters for RF reliability debugging
int txCount   = 0;
int failCount = 0;

// ============================================================
// STARTUP ANIMATION FLAG
// startupDone = false while the intro animation is playing
// when false, normalizedY is forced to 0 so the car cant move
// ============================================================
bool startupDone = false;

// ============================================================
// BUTTON DEBOUNCE
// tracks the raw button reading, the stable debounced button state,
// and the last time the raw reading changed
// prevents a single press registering as multiple presses
// HIGH = not pressed because the button uses INPUT_PULLUP
// LOW  = pressed because the switch connects the pin to GND
// ============================================================
bool          lastButtonReading  = HIGH;  // last raw reading from digitalRead()
bool          buttonState        = HIGH;  // stable debounced button state
bool          lastButtonState    = HIGH;  // previous stable debounced state
unsigned long lastDebounceTime   = 0;
const unsigned long debounceDelay = 50;   // 50ms debounce window

// ============================================================
// ANIMATION COOLDOWN
// prevents spamming the button to replay animation back to back
// must wait animationCooldown ms after animation finishes
// before another button press is accepted
// ============================================================
unsigned long lastAnimationEnd  = 0;
const unsigned long animationCooldown = 1500;  // 1.5 seconds between replays

// ============================================================
// DISPLAY STATE TRACKING
// we store the last drawn value for each box
// this way we only redraw a box when its value actually changes
// prevents flickering from constant redraws every 50ms
// ============================================================
bool  lastConnected = false;   // last known connection state (true = connected)
int   lastSpeedPct  = -1;      // last drawn speed percentage (-1 forces first draw)
float lastNormY     = 0;       // last drawn normalizedY (used for direction changes)
float lastNormX     = 0;       // last drawn normalizedX (used for steering changes)

// ============================================================
// SEMI-AUTO (MODE 2) TUNING - SPEED PICKER
// autoPreviewSpeed is no longer read as an absolute joystick
// position. Instead it's built up incrementally: holding the
// stick forward adds AUTO_PREVIEW_STEP to it every auto-mode
// tick (every 50ms, see autoInterval), holding it back subtracts
// the same amount. AUTO_PREVIEW_DEADZONE is how far (in raw ADC
// counts) the stick has to move off center before it counts as
// "held forward/back" so a slightly-off-center resting stick
// doesn't silently drift the value.
// ============================================================
#define AUTO_PREVIEW_STEP     0.02f
#define AUTO_PREVIEW_DEADZONE 300

// ============================================================
// SEMI-AUTO (MODE 2) STATE
// autoArmed     = true once cruise speed is locked in and the car is driving itself
//                 (NOTE: nothing currently sets this true - see handleSemiAutoToggle -
//                 arming/driving is left in place but disabled while the speed
//                 picker above is being tested)
// autoDisarming = true while we're waiting for the car to confirm it has stopped
//                 after a disarm request (SW2 pressed again, or leaving mode 2)
// autoPreviewSpeed   = live speed preview shown before confirming, now -1.0 to 1.0
//                      (-100% to 100%), built incrementally from joystick Y
// autoTargetSpeed    = the -1.0 to 1.0 cruise speed locked in once confirmed
// autoSpeedConfirmed = NEW - true once SW2 has locked in autoPreviewSpeed into
//                      autoTargetSpeed. While true, the joystick no longer changes
//                      autoPreviewSpeed until SW2 is pressed again to unlock it.
// ============================================================
bool  autoArmed          = false;
bool  autoDisarming      = false;
float autoPreviewSpeed   = 0.0f;
float autoTargetSpeed    = 0.0f;
bool  autoSpeedConfirmed = false;

// display state tracking for the two semi-auto screens, kept separate
// from the RC HUD tracking above so neither screen interferes with the other
bool   autoPickScreenDrawn   = false;
bool   autoDriveScreenDrawn  = false;
int    lastPreviewPct        = -1;
bool   lastAutoConfirmedDraw = false;
int    lastTargetPct         = -1;
int    lastCurrentPct        = -1;
bool   lastLeftOnLine        = false;
bool   lastRightOnLine       = false;
String lastAutoStatus        = "";

// ============================================================
// NEW - ToF DEBUG LINE DISPLAY STATE TRACKING
// tracks the last drawn threshold/distance/streak/braking values
// on the driving screen so we only redraw when something changes
// ============================================================
int  lastTofThreshold = -1;
int  lastTofDistance  = -999999;
int  lastTofStreak    = -1;
bool lastTofBraking   = false;

// ============================================================
// DISPLAY LAYOUT
// screen is 320x240 pixels in landscape (rotation 1)
// top box = 60px, middle box = 100px, bottom box = 80px
// bottom third is split into 2 equal vertical halves
// ============================================================
#define SCREEN_W  320
#define SCREEN_H  240

#define BOX1_H    60     // top box height - connection status
#define BOX2_H    100    // middle box height - speed (taller for gauge)
#define BOX3_H    80     // bottom box height - direction + steering
#define BOX_H     BOX1_H // kept for any legacy reference

#define BOX1_Y    0
#define BOX2_Y    BOX1_H
#define BOX3_Y    (BOX1_H + BOX2_H)

#define HALF_W    (SCREEN_W / 2)   // 160px per half

// ============================================================
// SPEED BEAM LAYOUT (mode 2 pick-speed screen)
// vertical bar showing autoPreviewSpeed from -100% (bottom) to
// +100% (top), 0% sits at the vertical center of the bar
// ============================================================
#define BEAM_X        130
#define BEAM_W        60
#define BEAM_TOP      100
#define BEAM_BOTTOM   190
#define BEAM_CENTER_Y ((BEAM_TOP + BEAM_BOTTOM) / 2)
#define BEAM_HALF_H   ((BEAM_BOTTOM - BEAM_TOP) / 2)

// ============================================================
// ANIMATION BORDER INSET
// how many pixels from each screen edge the border sits at rest
// a small inset gives the border breathing room from the very edge
// ============================================================
#define BORDER_INSET 4   // 4px gap from each screen edge

// ============================================================
// TRANSITION SPEED
// bigger step + smaller delay makes the border transition faster
// TRANSITION_STEP  = how many pixels the border moves each frame
// TRANSITION_DELAY = pause per frame in milliseconds
// TRANSITION_PAUSE = tiny pause when border reaches the center
// ============================================================
#define TRANSITION_STEP  6
#define TRANSITION_DELAY 4
#define TRANSITION_PAUSE 40

// ============================================================
// HUD REVEAL SPEED
// HUD reveal is separate because it redraws the HUD while clipping it
// a bigger step and smaller delay makes the HUD reveal feel closer
// to the speed of the regular border transitions
// ============================================================
#define HUD_REVEAL_STEP  10
#define HUD_REVEAL_DELAY 2

// ============================================================
// CENTER TEXT HELPER
// TFT_eSPI default font characters are about 6px wide per text size
// this lets labels be centered horizontally like the speed percentage
// instead of using fixed cursor positions that can look slightly off
// ============================================================
void drawCenteredText(String text, int centerX, int y, int textSize, uint16_t color) {
  int strWidth = text.length() * 6 * textSize;
  tft.setTextSize(textSize);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(centerX - (strWidth / 2), y);
  tft.print(text);
}

// ============================================================
// SPEED COLOR GRADIENT
// converts speed percentage into a smooth green -> yellow -> red color
// 0%   = green
// 50%  = yellow
// 100% = red
// this gives the speedometer a proper gradient instead of 3 hard zones
// ============================================================
uint16_t speedGradientColor(int speedPct) {
  speedPct = constrain(speedPct, 0, 100);

  int r;
  int g;
  int b = 0;

  if (speedPct <= 50) {
    r = map(speedPct, 0, 50, 0, 255);
    g = 255;
  } else {
    r = 255;
    g = map(speedPct, 50, 100, 255, 0);
  }

  return tft.color565(r, g, b);
}

// ============================================================
// MODE LED HELPERS
// during startup/replay animation, no mode LED should be on
// after animation finishes, updateModeLEDs() restores current mode LED
// ============================================================
void turnOffModeLEDs() {
  digitalWrite(MODE1_LED, LOW);
  digitalWrite(MODE2_LED, LOW);
  digitalWrite(MODE3_LED, LOW);
}

// ============================================================
// UPDATE MODE LEDS
// only one mode LED should be on at a time
// LED 1 = RC mode
// LED 2 = semi-auto mode
// LED 3 = AI mode
// ============================================================
void updateModeLEDs() {
  digitalWrite(MODE1_LED, currentMode == MODE_RC        ? HIGH : LOW);
  digitalWrite(MODE2_LED, currentMode == MODE_SEMI_AUTO ? HIGH : LOW);
  digitalWrite(MODE3_LED, currentMode == MODE_AI        ? HIGH : LOW);
}

// ============================================================
// SEND NEUTRAL PACKET
// forces motor, steering, AND semi-auto arming to neutral before animation
// this matters if the joystick is not centered when the button is pressed
// because the animation blocks normal RF sending for a short time
// sending a few neutral packets gives the car a good chance to stop first
// also used when switching modes so the car stops before leaving RC mode
// (armed/targetSpeed are zeroed too so the car can't keep auto-driving
// on a stale "armed" packet after we've moved on)
// ============================================================
void sendNeutralPacket() {
  currentNormalizedY = 0.0f;
  nextNormalizedY    = 0.0f;
  normalizedX        = 0.0f;

  packet.normalizedY = 0.0f;
  packet.normalizedX = 0.0f;
  packet.mode        = currentMode;
  packet.armed       = false;
  packet.targetSpeed = 0.0f;

  for (int i = 0; i < 3; i++) {
    bool ok = radio.write(&packet, sizeof(packet));
    if (ok) txCount++; else failCount++;

    if (ok && radio.available()) {
      radio.read(&sensorData, sizeof(sensorData));
    }

    delay(5);
  }
}

// ============================================================
// WAIT FOR AUTO STOP
// used when leaving mode 2 (or replaying the animation) while the
// car is still armed/driving. Keeps telling the car "mode 2, disarmed"
// and waits for its telemetry to confirm currentSpeed has actually
// reached zero - this respects the car's own momentum-based ramp down
// instead of just cutting it off and switching screens instantly.
// Times out after 3 seconds as a safety net in case of lost signal.
// ============================================================
void waitForAutoStop() {
  unsigned long stopStart = millis();

  while (millis() - stopStart < 3000) {
    packet.normalizedY = 0.0f;
    packet.normalizedX = 0.0f;
    packet.mode        = MODE_SEMI_AUTO;
    packet.armed        = false;
    packet.targetSpeed  = 0.0f;

    bool ok = radio.write(&packet, sizeof(packet));
    if (ok) txCount++; else failCount++;

    if (ok && radio.available()) {
      radio.read(&sensorData, sizeof(sensorData));

      if (sensorData.currentSpeed <= 0.02f) {
        break;
      }
    }

    // keep the driving screen showing "STOPPING..." while we wait
    updateAutoDrivingScreen(ok);
    delay(50);
  }

  autoArmed     = false;
  autoDisarming = false;
}

// ============================================================
// DRAW STATIC BORDERS
// draws all white box outlines once at startup
// these never change so we dont redraw them in the loop
// ============================================================
void drawBorders() {
  tft.drawRect(0, BOX1_Y, SCREEN_W, BOX1_H, TFT_WHITE);
  tft.drawRect(0, BOX2_Y, SCREEN_W, BOX2_H, TFT_WHITE);
  tft.drawRect(0, BOX3_Y, SCREEN_W, BOX3_H, TFT_WHITE);
  tft.drawLine(HALF_W, BOX3_Y, HALF_W, BOX3_Y + BOX3_H, TFT_WHITE);
}

// ============================================================
// DRAW TOP BOX - connection status
// called whenever the RF ok/fail result changes
// green = last packet was acknowledged by the car
// red   = last packet failed, car is out of range or off
// (generic helper - reused as-is by both the RC HUD and the
// semi-auto pick-speed screen, nothing here was changed)
// ============================================================
void drawConnectionBox(bool connected) {
  tft.fillRect(1, BOX1_Y + 1, SCREEN_W - 2, BOX1_H - 2, TFT_BLACK);

  if (connected) {
    drawCenteredText("CONNECTED", SCREEN_W / 2, BOX1_Y + 18, 3, TFT_GREEN);
  } else {
    drawCenteredText("NO SIGNAL", SCREEN_W / 2, BOX1_Y + 18, 3, TFT_RED);
  }
}

// ============================================================
// DRAW MIDDLE BOX - rectangular speedometer
// barY pushed down to BOX2_Y+42 so SPEED label is fully visible
// barH reduced 30->28 and % offset tightened to stay inside box
// (RC mode only - untouched)
// ============================================================
void drawSpeedBox(float normalizedY) {
  int speedPct = (int)(abs(normalizedY * normalizedY) * 100.0f);

  tft.fillRect(1, BOX2_Y + 1, SCREEN_W - 2, BOX2_H - 2, TFT_BLACK);

  drawCenteredText("SPEED", SCREEN_W / 2, BOX2_Y + 4, 1, TFT_WHITE);

  int barX = 16;
  int barY = BOX2_Y + 42;
  int barW = SCREEN_W - 32;
  int barH = 28;

  int tickY = barY - 10;
  for (int t = 0; t <= 4; t++) {
    int tx = barX + (barW * t) / 4;
    tft.drawLine(tx, tickY, tx, barY - 2, TFT_DARKGREY);
    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    String lbl = String(t * 25) + "%";
    tft.setCursor(tx - (lbl.length() * 3), tickY - 9);
    tft.print(lbl);
  }

  tft.drawRect(barX, barY, barW, barH, TFT_WHITE);

  if (speedPct > 0) {
    int fillW = (barW - 2) * speedPct / 100;
    uint16_t barColor = speedGradientColor(speedPct);
    tft.fillRect(barX + 1, barY + 1, fillW, barH - 2, barColor);
  }

  int needleX = barX + 1 + (barW - 2) * speedPct / 100;
  needleX = constrain(needleX, barX + 1, barX + barW - 2);
  tft.drawLine(needleX, barY - 1, needleX, barY + barH, TFT_WHITE);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(3);
  String pctStr = String(speedPct) + "%";
  int strWidth = pctStr.length() * 18;
  tft.setCursor((SCREEN_W - strWidth) / 2, barY + barH + 1);
  tft.print(pctStr);
}

// ============================================================
// DRAW BOTTOM LEFT BOX - direction
// green = FORWARD, red = REVERSE, white = STOPPED
// (RC mode only - untouched)
// ============================================================
void drawDirectionBox(float normalizedY) {
  tft.fillRect(1, BOX3_Y + 1, HALF_W - 2, BOX3_H - 2, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(30, BOX3_Y + 8);
  tft.print("DIRECTION");

  String dir;
  uint16_t dirColor;

  if (normalizedY > 0.02f) {
    dir      = "FORWARD";
    dirColor = TFT_GREEN;
  } else if (normalizedY < -0.02f) {
    dir      = "REVERSE";
    dirColor = TFT_RED;
  } else {
    dir      = "STOPPED";
    dirColor = TFT_WHITE;
  }

  tft.setTextSize(3);
  tft.setTextColor(dirColor, TFT_BLACK);
  int strWidth = dir.length() * 18;
  tft.setCursor((HALF_W - strWidth) / 2, BOX3_Y + 28);
  tft.print(dir);
}

// ============================================================
// DRAW BOTTOM RIGHT BOX - steering angle
// maps normalizedX (-1.0 to 1.0) to degrees (-45 to +45)
// (RC mode only - untouched)
// ============================================================
void drawSteeringBox(float normalizedX) {
  tft.fillRect(HALF_W + 1, BOX3_Y + 1, HALF_W - 2, BOX3_H - 2, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(HALF_W + 28, BOX3_Y + 8);
  tft.print("STEERING");

  int angle = (int)(normalizedX * 45.0f);

  String angleStr;
  if (angle > 0)       angleStr = "R " + String(angle) + (char)247;
  else if (angle < 0)  angleStr = "L " + String(abs(angle)) + (char)247;
  else                 angleStr = "CENTER";

  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  int strWidth = angleStr.length() * 18;
  tft.setCursor(HALF_W + (HALF_W - strWidth) / 2, BOX3_Y + 28);
  tft.print(angleStr);
}

// ============================================================
// DRAW AUTO GAUGE
// generic percentage gauge - originally used by the semi-auto
// pick-speed screen, now superseded there by drawSpeedBeam() below.
// left defined/untouched in case it's useful again elsewhere for a
// simple labeled 0-100% bar+percentage.
// ============================================================
void drawAutoGauge(int boxY, int boxH, String label, int pct) {
  tft.fillRect(1, boxY + 1, SCREEN_W - 2, boxH - 2, TFT_BLACK);
  drawCenteredText(label, SCREEN_W / 2, boxY + 6, 1, TFT_WHITE);

  int barX = 16;
  int barY = boxY + 28;
  int barW = SCREEN_W - 32;
  int barH = 20;

  tft.drawRect(barX, barY, barW, barH, TFT_WHITE);

  pct = constrain(pct, 0, 100);
  if (pct > 0) {
    int fillW = (barW - 2) * pct / 100;
    tft.fillRect(barX + 1, barY + 1, fillW, barH - 2, speedGradientColor(pct));
  }

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  String pctStr = String(pct) + "%";
  int strWidth = pctStr.length() * 12;
  tft.setCursor((SCREEN_W - strWidth) / 2, barY + barH + 8);
  tft.print(pctStr);
}

// ============================================================
// DRAW SPEED BEAM
// draws the vertical beam/bar for the mode 2 speed picker.
// Fills upward (green) from the center line for positive pct,
// downward (red) from the center line for negative pct. 0% shows
// an empty bar. The border turns yellow once the value has been
// confirmed via SW2, white while still adjustable.
// ============================================================
void drawSpeedBeam(int pct, bool confirmed) {
  pct = constrain(pct, -100, 100);

  // clear the beam column plus a little margin around it (labels included)
  tft.fillRect(BEAM_X - 45, BEAM_TOP - 14, (BEAM_W + 90), (BEAM_BOTTOM - BEAM_TOP) + 28, TFT_BLACK);

  uint16_t borderColor = confirmed ? TFT_YELLOW : TFT_WHITE;
  tft.drawRect(BEAM_X, BEAM_TOP, BEAM_W, BEAM_BOTTOM - BEAM_TOP, borderColor);

  if (pct > 0) {
    int fillH = (BEAM_HALF_H * pct) / 100;
    tft.fillRect(BEAM_X + 1, BEAM_CENTER_Y - fillH, BEAM_W - 2, fillH, TFT_GREEN);
  } else if (pct < 0) {
    int fillH = (BEAM_HALF_H * (-pct)) / 100;
    tft.fillRect(BEAM_X + 1, BEAM_CENTER_Y + 1, BEAM_W - 2, fillH, TFT_RED);
  }

  // center (0%) line, drawn last so it stays crisp over any fill
  tft.drawLine(BEAM_X - 6, BEAM_CENTER_Y, BEAM_X + BEAM_W + 6, BEAM_CENTER_Y, TFT_WHITE);

  // scale labels to the left of the beam
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(BEAM_X - 42, BEAM_TOP - 4);
  tft.print("+100%");
  tft.setCursor(BEAM_X - 30, BEAM_CENTER_Y - 4);
  tft.print("0%");
  tft.setCursor(BEAM_X - 42, BEAM_BOTTOM - 4);
  tft.print("-100%");
}

// ============================================================
// DRAW AUTO STATUS BOX
// top box on the driving screen - big centered status text,
// color-coded the same way the RC HUD color-codes direction
// ============================================================
void drawAutoStatusBox(String status, uint16_t color) {
  tft.fillRect(1, BOX1_Y + 1, SCREEN_W - 2, BOX1_H - 2, TFT_BLACK);
  drawCenteredText(status, SCREEN_W / 2, BOX1_Y + 18, 3, color);
}

// ============================================================
// DRAW TARGET / CURRENT SPEED BOXES
// left/right halves of the middle box on the driving screen,
// styled the same way as the RC HUD's direction/steering split
// ============================================================
void drawTargetSpeedBox(int pct) {
  int displayPct = pct;

  tft.fillRect(1, BOX2_Y + 1, HALF_W - 2, BOX2_H - 2, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(40, BOX2_Y + 14);
  tft.print("TARGET SPEED");

  String pctStr = String(displayPct) + "%";
  tft.setTextSize(3);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  int strWidth = pctStr.length() * 18;
  tft.setCursor((HALF_W - strWidth) / 2, BOX2_Y + 50);
  tft.print(pctStr);
}

void drawCurrentSpeedBox(int pct) {
  int displayPct = pct;

  tft.fillRect(HALF_W + 1, BOX2_Y + 1, HALF_W - 2, BOX2_H - 2, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(HALF_W + 32, BOX2_Y + 14);
  tft.print("CURRENT SPEED");

  String pctStr = String(displayPct) + "%";
  tft.setTextSize(3);
  tft.setTextColor(speedGradientColor(abs(displayPct)), TFT_BLACK);
  int strWidth = pctStr.length() * 18;
  tft.setCursor(HALF_W + (HALF_W - strWidth) / 2, BOX2_Y + 50);
  tft.print(pctStr);
}

// ============================================================
// NEW - DRAW TOF DEBUG LINE
// small status line inside the middle box, below the TARGET/
// CURRENT SPEED numbers, showing the live ToF trigger threshold,
// the live distance reading, and how many consecutive reads are
// in the currently-active streak. White while counting toward a
// brake (unsafe streak), yellow while braking and counting toward
// release (safe streak).
// ============================================================
void drawTofDebugLine(int thresholdMm, int distanceMm, int streak, bool braking) {
  tft.fillRect(1, BOX2_Y + 78, SCREEN_W - 2, BOX2_H - 80, TFT_BLACK);

  String label = braking ? "BRAKING STREAK " : "UNSAFE STREAK ";
  String distStr = (distanceMm < 0) ? "---" : String(distanceMm);
  String line = "THR:" + String(thresholdMm) + "mm  DIST:" + distStr +
                "mm  " + label + String(streak) + "/3";

  uint16_t col = braking ? TFT_YELLOW : TFT_WHITE;
  drawCenteredText(line, SCREEN_W / 2, BOX2_Y + 80, 1, col);
}

// ============================================================
// DRAW LEFT / RIGHT LINE SENSOR BOXES
// left/right halves of the bottom box on the driving screen
// yellow "BLACK" = that sensor currently sees the line
// green  "CLEAR" = that sensor sees clear ground
// ============================================================
void drawLeftLineBox(bool onLine) {
  tft.fillRect(1, BOX3_Y + 1, HALF_W - 2, BOX3_H - 2, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(38, BOX3_Y + 8);
  tft.print("LEFT SENSOR");

  String txt = onLine ? "BLACK" : "CLEAR";
  uint16_t col = onLine ? TFT_YELLOW : TFT_GREEN;
  tft.setTextSize(3);
  tft.setTextColor(col, TFT_BLACK);
  int strWidth = txt.length() * 18;
  tft.setCursor((HALF_W - strWidth) / 2, BOX3_Y + 28);
  tft.print(txt);
}

void drawRightLineBox(bool onLine) {
  tft.fillRect(HALF_W + 1, BOX3_Y + 1, HALF_W - 2, BOX3_H - 2, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(HALF_W + 34, BOX3_Y + 8);
  tft.print("RIGHT SENSOR");

  String txt = onLine ? "BLACK" : "CLEAR";
  uint16_t col = onLine ? TFT_YELLOW : TFT_GREEN;
  tft.setTextSize(3);
  tft.setTextColor(col, TFT_BLACK);
  int strWidth = txt.length() * 18;
  tft.setCursor(HALF_W + (HALF_W - strWidth) / 2, BOX3_Y + 28);
  tft.print(txt);
}

// ============================================================
// DRAW AUTO PICK-SPEED SCREEN
// shown on entering mode 2. Shows a vertical beam. Joystick
// forward/back adjusts the value incrementally from -100% to
// 100% (see handleAutoMode), SW2 confirms it into autoTargetSpeed
// without arming the car - press SW2 again to unlock it and adjust again.
//
// box1 = connection status, box2 area is merged with box3's space
// into one big content box (no left/right divider needed here
// since this screen doesn't split into two halves like the RC HUD)
// ============================================================
void drawAutoPickSpeedScreen() {
  tft.fillScreen(TFT_BLACK);

  tft.drawRect(0, BOX1_Y, SCREEN_W, BOX1_H, TFT_WHITE);
  tft.drawRect(0, BOX2_Y, SCREEN_W, SCREEN_H - BOX2_Y, TFT_WHITE);

  drawConnectionBox(lastConnected);

  int pct = (int)(autoPreviewSpeed * 100.0f);

  int displayPct = pct;
  uint16_t numColor = displayPct > 0 ? TFT_GREEN :
                      (displayPct < 0 ? TFT_RED : TFT_WHITE);

  drawCenteredText(String(displayPct) + "%", SCREEN_W / 2, BOX2_Y + 6, 3, numColor);

  drawSpeedBeam(pct, autoSpeedConfirmed);

  drawCenteredText(autoSpeedConfirmed ? "SPEED CONFIRMED" : "PUSH UP: +      PULL DOWN: -", SCREEN_W / 2, 200, 1, TFT_WHITE);
  drawCenteredText(autoSpeedConfirmed ? "PRESS SW2 TO EDIT" : "PRESS SW2 TO CONFIRM", SCREEN_W / 2, 216, 1, TFT_WHITE);

  autoPickScreenDrawn   = true;
  autoDriveScreenDrawn  = false;
  lastPreviewPct         = pct;
  lastAutoConfirmedDraw  = autoSpeedConfirmed;
}

// ============================================================
// UPDATE AUTO PICK-SPEED SCREEN
// only redraws the number/beam/instructions when the
// value or confirmed state actually changes
// ============================================================
void updateAutoPickSpeedScreen(bool connected) {
  if (!autoPickScreenDrawn) {
    drawAutoPickSpeedScreen();
  }

  if (connected != lastConnected) {
    drawConnectionBox(connected);
    lastConnected = connected;
  }

  int pct = (int)(autoPreviewSpeed * 100.0f);

  if (pct != lastPreviewPct || autoSpeedConfirmed != lastAutoConfirmedDraw) {
    int displayPct = pct;
    uint16_t numColor = displayPct > 0 ? TFT_GREEN :
                        (displayPct < 0 ? TFT_RED : TFT_WHITE);

    tft.fillRect(0, BOX2_Y + 1, SCREEN_W - 2, 30, TFT_BLACK); // clear old percentage text
    drawCenteredText(String(displayPct) + "%", SCREEN_W / 2, BOX2_Y + 6, 3, numColor);

    drawSpeedBeam(pct, autoSpeedConfirmed);

    tft.fillRect(0, 198, SCREEN_W, 36, TFT_BLACK); // clear old instruction lines
    drawCenteredText(autoSpeedConfirmed ? "SPEED CONFIRMED" : "PUSH UP: +      PULL DOWN: -", SCREEN_W / 2, 200, 1, TFT_WHITE);
    drawCenteredText(autoSpeedConfirmed ? "PRESS SW2 TO EDIT" : "PRESS SW2 TO CONFIRM", SCREEN_W / 2, 216, 1, TFT_WHITE);

    lastPreviewPct        = pct;
    lastAutoConfirmedDraw = autoSpeedConfirmed;
  }
}

// ============================================================
// DRAW AUTO DRIVING SCREEN
// shown once armed - status up top, target/current speed in the
// middle, left/right line sensor state on the bottom
// (currently unused - see handleSemiAutoToggle - left in place
// so arming/driving can be reconnected later)
// ============================================================
void drawAutoDrivingScreen() {
  tft.fillScreen(TFT_BLACK);
  drawBorders();

  autoDriveScreenDrawn = true;
  autoPickScreenDrawn  = false;

  lastTargetPct   = -1;
  lastCurrentPct  = -1;
  lastLeftOnLine  = false;
  lastRightOnLine = false;
  lastAutoStatus  = "";

  // NEW - force the ToF debug line to redraw on next update
  lastTofThreshold = -1;
  lastTofDistance  = -999999;
  lastTofStreak    = -1;
  lastTofBraking   = false;
}

// ============================================================
// UPDATE AUTO DRIVING SCREEN
// works out the current status text/color from telemetry
// (a full stop always wins over a slowdown, and disarming wins
// over everything since we're actively trying to stop) then only
// redraws the boxes whose values actually changed
// (currently unused - see handleSemiAutoToggle - left in place
// so arming/driving can be reconnected later)
// ============================================================
void updateAutoDrivingScreen(bool connected) {
  if (!autoDriveScreenDrawn) {
    drawAutoDrivingScreen();
  }

  String status;
  uint16_t statusColor;

  if (autoDisarming) {
    status      = "STOPPING...";
    statusColor = TFT_ORANGE;
  } else if (sensorData.stoppedLine) {
    status      = "STOPPED - LINE";
    statusColor = TFT_RED;
  } else if (sensorData.stoppedObstacle) {
    status      = "STOPPED - OBSTACLE";
    statusColor = TFT_RED;
  } else if (sensorData.slowing) {
    status      = "SLOWING DOWN";
    statusColor = TFT_YELLOW;
  } else {
    status      = "STEERING ONLY";
    statusColor = TFT_GREEN;
  }

  if (status != lastAutoStatus) {
    drawAutoStatusBox(status, statusColor);
    lastAutoStatus = status;
  }

  int targetPct  = (int)(autoTargetSpeed * 100.0f);
  int currentPct = (int)(sensorData.currentSpeed * 100.0f);

  if (targetPct != lastTargetPct) {
    drawTargetSpeedBox(targetPct);
    lastTargetPct = targetPct;
  }

  if (currentPct != lastCurrentPct) {
    drawCurrentSpeedBox(currentPct);
    lastCurrentPct = currentPct;
  }

  if (sensorData.leftOnLine != lastLeftOnLine) {
    drawLeftLineBox(sensorData.leftOnLine);
    lastLeftOnLine = sensorData.leftOnLine;
  }

  if (sensorData.rightOnLine != lastRightOnLine) {
    drawRightLineBox(sensorData.rightOnLine);
    lastRightOnLine = sensorData.rightOnLine;
  }

  // NEW - live ToF threshold / distance / streak debug line
  int thresholdMm = (int)sensorData.tofThresholdMm;
  int distanceMm  = sensorData.tofValid ? sensorData.tofMm : -1;
  int streak      = sensorData.tofStreak;
  bool braking    = sensorData.tofBraking;

  if (thresholdMm != lastTofThreshold || distanceMm != lastTofDistance ||
      streak != lastTofStreak || braking != lastTofBraking) {
    drawTofDebugLine(thresholdMm, distanceMm, streak, braking);
    lastTofThreshold = thresholdMm;
    lastTofDistance  = distanceMm;
    lastTofStreak    = streak;
    lastTofBraking   = braking;
  }
}

// ============================================================
// DRAW MODE PLACEHOLDER SCREEN
// used for AI mode for now
// mode 3 is not active driving yet, it only shows this screen
// (untouched)
// ============================================================
void drawModePlaceholder(String title) {
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, SCREEN_W, SCREEN_H, TFT_WHITE);
  drawCenteredText(title, SCREEN_W / 2, 92, 3, TFT_WHITE);
  drawCenteredText("PLACEHOLDER", SCREEN_W / 2, 130, 2, TFT_DARKGREY);
}

// ============================================================
// DRAW HUD SNAPSHOT
// draws the HUD in its neutral state without changing display tracking
// used during the HUD reveal transition so the HUD is already behind
// the growing border and becomes visible as the border opens outward
// (RC mode only - untouched)
// ============================================================
void drawHUDSnapshot() {
  tft.fillScreen(TFT_BLACK);
  drawBorders();
  drawConnectionBox(lastConnected);
  drawSpeedBox(0);
  drawDirectionBox(0);
  drawSteeringBox(0);
}

// ============================================================
// DRAW ANIMATION BORDER
// draws a single white border rect at a given inset from screen edges
// inset=0 means the border is flush against all four screen edges
// inset grows until the border collapses to a point at screen center
// used every frame of the border-shrink and border-grow transitions
// ============================================================
void drawAnimBorder(int inset) {
  int x = inset;
  int y = inset;
  int w = SCREEN_W - inset * 2;
  int h = SCREEN_H - inset * 2;

  if (w >= 2 && h >= 2) {
    tft.drawRect(x, y, w, h, TFT_WHITE);
  }
}

// ============================================================
// FILL OUTSIDE BORDER
// fills only the area outside the current border rectangle
// this lets the existing HUD or animation remain visible inside
// while the border moves inward and visually engulfs it
// ============================================================
void fillOutsideBorder(int inset, uint16_t bgColor) {
  int x = inset;
  int y = inset;
  int w = SCREEN_W - inset * 2;
  int h = SCREEN_H - inset * 2;

  tft.fillRect(0, 0, SCREEN_W, y, bgColor);
  tft.fillRect(0, y + h, SCREEN_W, SCREEN_H - (y + h), bgColor);
  tft.fillRect(0, y, x, h, bgColor);
  tft.fillRect(x + w, y, SCREEN_W - (x + w), h, bgColor);
}

// ============================================================
// BORDER SHRINK - wipes the border inward from screen edges to center
// ============================================================
void borderShrink(uint16_t bgColor) {
  int maxInset = SCREEN_H / 2;

  for (int inset = BORDER_INSET; inset <= maxInset; inset += TRANSITION_STEP) {
    fillOutsideBorder(inset, bgColor);
    drawAnimBorder(inset);
    delay(TRANSITION_DELAY);
  }

  tft.fillScreen(bgColor);
}

// ============================================================
// BORDER GROW - expands the border outward from center to screen edges
// ============================================================
void borderGrow(uint16_t bgColor) {
  int maxInset = SCREEN_H / 2;

  for (int inset = maxInset; inset >= BORDER_INSET; inset -= TRANSITION_STEP) {
    tft.fillScreen(bgColor);
    drawAnimBorder(inset);
    delay(TRANSITION_DELAY);
  }

  tft.fillScreen(bgColor);
  drawAnimBorder(BORDER_INSET);
}

// ============================================================
// BORDER GROW REVEAL HUD
// ============================================================
void borderGrowRevealHUD() {
  int maxInset = SCREEN_H / 2;

  for (int inset = maxInset; inset >= BORDER_INSET; inset -= HUD_REVEAL_STEP) {
    int x = inset;
    int y = inset;
    int w = SCREEN_W - inset * 2;
    int h = SCREEN_H - inset * 2;

    tft.fillScreen(TFT_BLACK);

    if (w > 0 && h > 0) {
      tft.setViewport(x, y, w, h, false);
      drawHUDSnapshot();
      tft.resetViewport();
    }

    drawAnimBorder(inset);
    delay(HUD_REVEAL_DELAY);
  }

  drawHUDSnapshot();
  drawAnimBorder(BORDER_INSET);
}

// ============================================================
// TRANSITION: HUD -> ANIMATION
// ============================================================
void transitionToAnimation() {
  borderShrink(TFT_BLACK);
  delay(TRANSITION_PAUSE);
  borderGrow(TFT_BLACK);
}

// ============================================================
// TRANSITION: ANIMATION -> HUD
// ============================================================
void transitionToHUD() {
  borderShrink(TFT_BLACK);
  delay(TRANSITION_PAUSE);
  borderGrowRevealHUD();
}

// ============================================================
// STARTUP ANIMATION - "Westwood" + "V1"
// ============================================================
void playStartupAnimation() {
  tft.fillRect(
    BORDER_INSET + 1,
    BORDER_INSET + 1,
    SCREEN_W - (BORDER_INSET + 1) * 2,
    SCREEN_H - (BORDER_INSET + 1) * 2,
    TFT_BLACK
  );

  const char* title = "WestWood";
  int titleLen      = strlen(title);
  int titleCharW    = 6 * 4;
  int titleTotalW   = titleLen * titleCharW;
  int titleX        = (SCREEN_W - titleTotalW) / 2;
  int titleY        = 72;

  tft.setTextSize(4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  for (int i = 0; i < titleLen; i++) {
    tft.setCursor(titleX + i * titleCharW, titleY);
    tft.print(title[i]);
    delay(70);
  }

  delay(150);

  int v1CharW  = 6 * 5;
  int v1TotalW = 2 * v1CharW;
  int v1X      = (SCREEN_W - v1TotalW) / 2;
  int v1Y      = titleY + (8 * 4) + 14;

  tft.setTextSize(5);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(v1X, v1Y);
  tft.print("V1");

  delay(1200);
}

// ============================================================
// REDRAW FULL HUD
// ============================================================
void drawFullHUD() {
  tft.fillScreen(TFT_BLACK);
  drawBorders();
  drawConnectionBox(lastConnected);
  drawSpeedBox(0);
  drawDirectionBox(0);
  drawSteeringBox(0);

  lastSpeedPct = -1;
  lastNormY    = 0;
  lastNormX    = 0;

  autoPickScreenDrawn  = false;
  autoDriveScreenDrawn = false;
}

// ============================================================
// SET MODE
// called when one of the three mode buttons is pressed
// before changing modes, the car is forced to neutral:
//   motor speed = 0
//   steering    = 0
//   RF packet   = neutral sent a few times
// if we're leaving mode 2 while armed, we first wait for the car
// to actually finish stopping (respecting its momentum) before
// switching to the new mode's screen
// ============================================================
void setMode(int newMode) {
  if (newMode == currentMode) {
    updateModeLEDs();
    return;
  }

  if (currentMode == MODE_SEMI_AUTO && (autoArmed || autoDisarming)) {
  waitForAutoStop();
}

  startupDone        = false;
  currentNormalizedY = 0.0f;
  nextNormalizedY    = 0.0f;
  normalizedX        = 0.0f;

  sendNeutralPacket();

  currentMode = newMode;
  updateModeLEDs();

  lastSpeedPct = -1;
  lastNormY    = 0.0f;
  lastNormX    = 0.0f;

  // reset semi-auto state whenever we leave, or freshly enter, mode 2
  autoArmed          = false;
  autoDisarming       = false;
  autoTargetSpeed     = 0.0f;
  autoPreviewSpeed     = 0.0f;
  autoSpeedConfirmed   = false;

  if (currentMode == MODE_RC) {
    drawFullHUD();
  } else if (currentMode == MODE_SEMI_AUTO) {
    drawAutoPickSpeedScreen();
  } else if (currentMode == MODE_AI) {
    drawModePlaceholder("AI MODE");
  }

  startupDone = true;

  Serial.print("Mode changed to: ");
  Serial.println(currentMode);
}

// ============================================================
// HANDLE SEMI-AUTO TOGGLE
// called when SW2 is pressed again while already in mode 2.
//
//   not confirmed -> lock the previewed speed into autoTargetSpeed and
//                    enable IR steering correction while keeping the motor stopped
//   confirmed     -> disable correction and unlock joystick speed editing
// ============================================================
void handleSemiAutoToggle() {
  if (!autoSpeedConfirmed) {
    // First SW2 press: lock speed and start Mode 2 driving
    autoTargetSpeed    = autoPreviewSpeed;
    autoSpeedConfirmed = true;
    autoArmed          = true;
    autoDisarming      = false;
  } else {
    // Second SW2 press: tell the car to ramp down to zero first
    autoArmed          = false;
    autoDisarming      = true;
    autoSpeedConfirmed = false;
  }
}

// ============================================================
// ============================================================
// READ MODE BUTTONS
// each mode button uses the same debounce style as the joystick button
// buttons are active LOW because they are wired to GND with INPUT_PULLUP
//
// SW1 press -> MODE_RC
// SW2 press -> MODE_SEMI_AUTO (or arm/disarm toggle if already in mode 2)
// SW3 press -> MODE_AI
// ============================================================
void readModeButtons(unsigned long currentTime) {
  for (int i = 0; i < 3; i++) {
    bool reading = digitalRead(modeButtonPins[i]);

    if (reading != modeLastReading[i]) {
      modeLastDebounceTime[i] = currentTime;
    }

    if ((currentTime - modeLastDebounceTime[i]) > modeDebounceDelay) {
      if (reading != modeButtonState[i]) {
        modeLastState[i] = modeButtonState[i];
        modeButtonState[i] = reading;

        if (modeButtonState[i] == LOW && modeLastState[i] == HIGH) {
          int pressedMode = i + 1;

          if (pressedMode == MODE_SEMI_AUTO && currentMode == MODE_SEMI_AUTO) {
            handleSemiAutoToggle();
          } else {
            setMode(pressedMode);
          }
        }
      }
    }

    modeLastReading[i] = reading;
  }
}

// ============================================================
// HANDLE AUTO MODE
// runs every 50ms while in mode 2.
// Builds the speed preview incrementally from the joystick Y axis
// (instead of reading it as an absolute position) unless the value
// has already been confirmed, sends the (still unarmed) packet,
// reads telemetry back, and drives whichever of the two mode-2
// screens is currently active.
// ============================================================
void handleAutoMode(unsigned long currentTime) {
  if (currentTime - lastAutoUpdate < autoInterval) {
    return;
  }

  lastAutoUpdate = currentTime;

  // ============================================================
  // SPEED PICKER - increment/decrement style
  // holding the stick forward adds AUTO_PREVIEW_STEP to
  // autoPreviewSpeed each tick, holding it back subtracts it.
  // range is -1.0 to 1.0 (-100% to 100%), starts at 0.
  // once confirmed (SW2 pressed), the joystick stops changing it
  // until SW2 is pressed again to unlock it for editing.
  // ============================================================
  if (!autoSpeedConfirmed) {
        // your physical UP/DOWN joystick axis is GPIO 35.
    int yValue = analogRead(joystickPinX);

    if (yValue < joystickCenterX - AUTO_PREVIEW_DEADZONE) {
      // physical UP = increase speed / positive
      autoPreviewSpeed += AUTO_PREVIEW_STEP;
    } else if (yValue > joystickCenterX + AUTO_PREVIEW_DEADZONE) {
      // physical DOWN = decrease speed / negative
      autoPreviewSpeed -= AUTO_PREVIEW_STEP;
    }

    autoPreviewSpeed = constrain(autoPreviewSpeed, -1.0f, 1.0f);
  }

  packet.normalizedY = 0.0f;
  packet.normalizedX = 0.0f;
  packet.mode        = currentMode;
  packet.armed       = autoArmed;
  // car uses negative internal speed for physical forward,
  // while the Mode 2 screen keeps positive = forward.
  packet.targetSpeed = autoArmed ? autoTargetSpeed : 0.0f;

  bool ok = radio.write(&packet, sizeof(packet));

  if (ok) txCount++; else failCount++;

  if (ok && radio.available()) {
    radio.read(&sensorData, sizeof(sensorData));
  }

  // if we requested a disarm via SW2 (not via leaving the mode,
  // that path uses waitForAutoStop() instead), watch telemetry
  // for confirmation the car has actually come to a stop
  // (currently unreachable since autoDisarming is never set true -
  // see handleSemiAutoToggle - left in place for later)
  if (autoDisarming) {
    if (sensorData.currentSpeed <= 0.02f) {
      autoDisarming = false;
      drawAutoPickSpeedScreen();
    }
  }

  if (autoArmed || autoDisarming) {
    updateAutoDrivingScreen(ok);
  } else {
    updateAutoPickSpeedScreen(ok);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(joystickPinX, INPUT);
  pinMode(joystickPinY, INPUT);
  pinMode(joystickSW,   INPUT_PULLUP);

  pinMode(MODE1_BUTTON, INPUT_PULLUP);
  pinMode(MODE2_BUTTON, INPUT_PULLUP);
  pinMode(MODE3_BUTTON, INPUT_PULLUP);

  pinMode(MODE1_LED, OUTPUT);
  pinMode(MODE2_LED, OUTPUT);
  pinMode(MODE3_LED, OUTPUT);

  // no mode LED is on during startup animation
  turnOffModeLEDs();

  SPI.begin(RF_SCK, RF_MI, RF_MO);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  borderGrow(TFT_BLACK);
  playStartupAnimation();
  transitionToHUD();
  drawFullHUD();

  if (!radio.begin()) {
    Serial.println("RF NOT FOUND - check wiring");
    while (1);
  }

  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.enableAckPayload();
  radio.openWritingPipe(address);
  radio.stopListening();

  startupDone = true;
  updateModeLEDs();

  Serial.println("Remote ready");
}

void loop() {
  // Your physical joystick axes are swapped.
  int yValue = analogRead(joystickPinX);  // UP / DOWN = motor
  int xValue = analogRead(joystickPinY);  // LEFT / RIGHT = steering

  unsigned long currentTime = millis();

  readModeButtons(currentTime);

  // ============================================================
  // JOYSTICK BUTTON - replay animation
  // during animation, all mode LEDs turn off
  // after animation, the current mode LED turns back on
  // if mode 2 is armed and driving, the car is brought to a full
  // stop first (same as leaving the mode) before the animation plays
  // ============================================================
  bool buttonReading = digitalRead(joystickSW);

  if (buttonReading != lastButtonReading) {
    lastDebounceTime = currentTime;
  }

  if ((currentTime - lastDebounceTime) > debounceDelay) {
    if (buttonReading != buttonState) {
      lastButtonState = buttonState;
      buttonState = buttonReading;

      if (buttonState == LOW && lastButtonState == HIGH) {
        if ((currentTime - lastAnimationEnd) >= animationCooldown) {
          startupDone        = false;
          currentNormalizedY = 0;
          nextNormalizedY    = 0;
          normalizedX        = 0;

          turnOffModeLEDs();

          if (currentMode == MODE_SEMI_AUTO && autoArmed) {
            waitForAutoStop();
          }

          sendNeutralPacket();

          autoArmed          = false;
          autoDisarming       = false;
          autoTargetSpeed      = 0.0f;
          autoPreviewSpeed      = 0.0f;
          autoSpeedConfirmed    = false;

          transitionToAnimation();
          playStartupAnimation();
          transitionToHUD();

          if (currentMode == MODE_RC) {
            drawFullHUD();
          } else if (currentMode == MODE_SEMI_AUTO) {
            drawAutoPickSpeedScreen();
          } else if (currentMode == MODE_AI) {
            drawModePlaceholder("AI MODE");
          }

          lastAnimationEnd = millis();

          currentNormalizedY = 0;
          nextNormalizedY    = 0;
          normalizedX        = 0;

          startupDone = true;
          updateModeLEDs();
        }
      }
    }
  }

  lastButtonReading = buttonReading;

  // ============================================================
  // MODE GATE
  // RC mode = normal joystick control
  // Auto mode = semi-auto cruise driving (pick-speed / driving screens)
  // AI mode = placeholder only for now
  // ============================================================
  if (currentMode == MODE_SEMI_AUTO) {
    currentNormalizedY = 0.0f;
    nextNormalizedY    = 0.0f;
    normalizedX        = 0.0f;
    handleAutoMode(currentTime);
    return;
  }

  if (currentMode == MODE_AI) {
    currentNormalizedY = 0.0f;
    nextNormalizedY    = 0.0f;
    normalizedX        = 0.0f;
    return;
  }

  // ============================================================
  // MOTOR SLEW RATE CALCULATION
  // runs every 3ms
  // slowly ramps currentNormalizedY toward nextNormalizedY
  // ============================================================
  if (currentTime - lastMotorUpdate >= motorInterval) {
    lastMotorUpdate = currentTime;

    if (!startupDone) {
      currentNormalizedY = 0;
      return;
    }

    if (yValue < joystickCenterY) {
      nextNormalizedY = (yValue - joystickCenterY) / float(joystickCenterY);
    }
    else if (yValue > joystickCenterY) {
      nextNormalizedY = (yValue - joystickCenterY) / float(4095 - joystickCenterY);
    }
    else {
      nextNormalizedY = 0.0f;
    }

    // Physical UP should be forward.
    nextNormalizedY *= -1.0f;

    if (abs(nextNormalizedY - currentNormalizedY) > 0.005) {
      if (nextNormalizedY * currentNormalizedY >= 0) {
        if (nextNormalizedY > currentNormalizedY) {
          currentNormalizedY = fmin(currentNormalizedY + sameDirectionStep, nextNormalizedY);
        }
        else if (nextNormalizedY < currentNormalizedY) {
          currentNormalizedY = fmax(currentNormalizedY - sameDirectionStep, nextNormalizedY);
        }
      }
      else if (nextNormalizedY * currentNormalizedY < 0) {
        if (nextNormalizedY > 0) {
          currentNormalizedY = fmin(currentNormalizedY + oppDirectionStep, 0);
        }
        else if (nextNormalizedY < 0) {
          currentNormalizedY = fmax(currentNormalizedY - oppDirectionStep, 0);
        }
      }
    }

    currentNormalizedY = constrain(currentNormalizedY, -1.0, 1.0);
  }

  // ============================================================
  // STEERING NORMALIZATION + RF SEND
  // runs every 50ms
  // normalizes X joystick then sends full packet to car
  // ============================================================
  if (currentTime - lastServoUpdate >= servoInterval) {
    lastServoUpdate = currentTime;

    if (abs(xValue - joystickCenterX) <= (deadzone * 4095)) {
      normalizedX = 0.0f;
    }
    else if (xValue < joystickCenterX) {
      normalizedX = (xValue - joystickCenterX) / float(joystickCenterX);
    }
    else {
      normalizedX = (xValue - joystickCenterX) / float(4095 - joystickCenterX);
    }

    normalizedX *= -1.0f;  // fixes left/right steering direction
    normalizedX = constrain(normalizedX, -1.0, 1.0);

    packet.normalizedY = currentNormalizedY;
    packet.normalizedX = normalizedX;
    packet.mode         = currentMode;
    packet.armed        = false;
    packet.targetSpeed  = 0.0f;

    bool ok = radio.write(&packet, sizeof(packet));

    if (ok) txCount++; else failCount++;

    if (ok && radio.available()) {
      radio.read(&sensorData, sizeof(sensorData));
    }

    Serial.printf("Y: %.3f    X: %.3f\n", currentNormalizedY, normalizedX);

    bool connected = ok;

    if (connected != lastConnected) {
      drawConnectionBox(connected);
      lastConnected = connected;
    }

    if (!connected) {
      if (lastSpeedPct != 0 || lastNormY != 0.0f) {
        drawSpeedBox(0);
        drawDirectionBox(0);
        lastSpeedPct = 0;
        lastNormY    = 0.0f;
      }

      if (lastNormX != 0.0f) {
        drawSteeringBox(0);
        lastNormX = 0.0f;
      }
    } else {
      int speedPct = (int)(abs(currentNormalizedY * currentNormalizedY) * 100.0f);

      if (speedPct != lastSpeedPct || abs(currentNormalizedY - lastNormY) > 0.01f) {
        drawSpeedBox(currentNormalizedY);
        drawDirectionBox(currentNormalizedY);
        lastSpeedPct = speedPct;
        lastNormY    = currentNormalizedY;
      }

      if (abs(normalizedX - lastNormX) > 0.01f) {
        drawSteeringBox(normalizedX);
        lastNormX = normalizedX;
      }
    }
  }
}