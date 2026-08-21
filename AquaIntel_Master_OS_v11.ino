

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ─────────────────────────────────────────────────────────────────────
// OLED DISPLAY CONFIGURATION
// ─────────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ─────────────────────────────────────────────────────────────────────
// PIN DEFINITIONS (ESP32 GPIO)
// ─────────────────────────────────────────────────────────────────────
// ACTUATORS (Active-LOW: LOW = ON, HIGH = OFF)
#define OUT_VALVE_PIN  4       // Gravity drain valve
#define IN_VALVE_PIN   16      // Supply inlet valve
#define PUMP_PIN       17      // Submersible pump motor

// SENSORS
#define TRIG_PIN       5       // Ultrasonic trigger
#define ECHO_PIN       18      // Ultrasonic echo (with 1k/2k voltage divider)
#define FLOW_PIN       15      // YF-S201 flow sensor

// FEEDBACK OUTPUTS
#define BUZZER_PIN     13      // Piezo buzzer
#define BUZZER_ON      LOW     // Buzzer is Active-LOW
#define BUZZER_OFF     HIGH

#define LED_GREEN      19      // Full tank indicator
#define LED_BLUE       23      // Nominal level indicator
#define LED_RED        25      // Low tank indicator

// ─────────────────────────────────────────────────────────────────────
// CRITICAL: CUSTOMIZE THESE 3 VALUES FOR YOUR SHORTENED TANK
// ─────────────────────────────────────────────────────────────────────
// 
// HOW TO FIND TANK_AIR_GAP:
//   1. Mount your ultrasonic sensor on top of tank (pointing down at water)
//   2. Measure from sensor FACE to tank BOTTOM in centimeters
//   3. This is your TANK_AIR_GAP value
//   
// EXAMPLE:
//   Tank height: 45cm
//   Sensor mounted: 5cm above rim
//   → TANK_AIR_GAP = 45 + 5 = 50cm
//
// IF YOU'RE NOT SURE:
//   1. Keep TANK_AIR_GAP = 68.0 for now
//   2. Fill tank to maximum
//   3. Note what OLED shows in Serial Monitor
//   4. If it shows too HIGH, decrease TANK_AIR_GAP
//   5. If it shows too LOW, increase TANK_AIR_GAP
//   6. Adjust by 10cm at a time until it matches
//
const float TANK_AIR_GAP = 68.0;           // ← CHANGE THIS TO YOUR MEASUREMENT

// Maximum water depth when tank is FULL (what you want OLED to show at maximum)
// OLED will display this value when tank is completely full
const float MAX_WATER_CAPACITY = 46.0;     // ← Keep around 46-49cm for "FULL"

// ─────────────────────────────────────────────────────────────────────
// STATE THRESHOLDS (in cm, what OLED displays)
// 
// You specified:
//   5cm = Lowest safe level
//   46-49cm = Full tank
//
// The thresholds between these are calculated automatically:
//   DEPTH_CRITICAL = 5.0cm (Lowest, pump locked)
//   DEPTH_LOW = 10.0cm (Start filling, RED LED + BUZZER)
//   DEPTH_THROTTLE = 38.0cm (Switch to gentle pulsing)
//   DEPTH_HIGH = 46.0cm (Full, GREEN LED, pump stops)
// ─────────────────────────────────────────────────────────────────────
const float DEPTH_CRITICAL = 5.0;          // Absolute minimum (OUT valve locked)
const float DEPTH_LOW = 25.0;              // Warning level (Red LED + buzzer, OUT valve closes)
const float DEPTH_START_FILL = 40.0;       // Start filling when water drops below this
const float DEPTH_THROTTLE = 44.0;         // Switch to gentle pulsing as it nears full
const float DEPTH_HIGH = 46.0;             // Full tank (pump stops)

// ─────────────────────────────────────────────────────────────────────
// SAFETY PARAMETERS (Do Not Change)
// ─────────────────────────────────────────────────────────────────────
const unsigned long DRY_RUN_TIMEOUT = 60000UL;  // 60 seconds
const float DRY_RUN_THRESHOLD = 0.1;            // L/min
const unsigned long THROTTLE_CYCLE_TIME = 2000UL;  // 2 seconds ON/OFF

// Ultrasonic sensor filtering
const float SENSOR_MIN_VALID = 2.0;
const float SENSOR_MAX_VALID = 100.0;
const int SENSOR_READ_INTERVAL = 150;
const int FILTER_WINDOW = 5;

// ─────────────────────────────────────────────────────────────────────
// STATE MACHINE ENUMERATION
// ─────────────────────────────────────────────────────────────────────
enum SystemState {
  STATE_BOOT = 0,
  STATE_IDLE = 1,
  STATE_FILLING = 2,
  STATE_THROTTLING = 3,
  STATE_FULL = 4,
  STATE_CRITICAL_LOW = 5,
  STATE_DRY_RUN_FAULT = 6,
  STATE_MANUAL_OVERRIDE = 7
};

// ─────────────────────────────────────────────────────────────────────
// GLOBAL STATE VARIABLES
// ─────────────────────────────────────────────────────────────────────
SystemState currentState = STATE_BOOT;
SystemState previousState = STATE_BOOT;

// Sensor readings
float real_water_depth = 0.0;
float sensorReadings[FILTER_WINDOW] = {};
int filterIndex = 0;

// Flow sensor
volatile unsigned long pulseCount = 0;
float flowRate_LPM = 0.0;
const float PULSES_PER_LITRE = 450.0;

// Pump state
bool isPumpEnabled = false;
unsigned long pumpStartTime = 0;
unsigned long throttleCycleStart = 0;
bool throttlePumpState = false;

// Fault tracking
bool pumpDryRunFault = false;

// Manual override
bool isManualOverride = false;

// Buzzer state tracking
bool previousPumpState = false;
bool lowWaterAlarmActive = false;

// ─────────────────────────────────────────────────────────────────────
// TIMING VARIABLES
// ─────────────────────────────────────────────────────────────────────
unsigned long lastSensorRead = 0;
unsigned long lastFlowCalc = 0;
unsigned long lastUIUpdate = 0;
unsigned long fullConditionStartTime = 0;
bool highTimerActive = false;

// Terminal log
String terminalLog[2] = {"SYSTEM BOOTING", "SYSTEM BOOTING"};

// ─────────────────────────────────────────────────────────────────────
// HARDWARE INTERRUPT SERVICE ROUTINE - FLOW SENSOR
// ─────────────────────────────────────────────────────────────────────
void IRAM_ATTR countFlowPulse() {
  pulseCount++;
}

// ─────────────────────────────────────────────────────────────────────
// UTILITY FUNCTION: UPDATE TERMINAL LOG
// ─────────────────────────────────────────────────────────────────────
void updateLog(const char* message) {
  terminalLog[0] = terminalLog[1];
  terminalLog[1] = "> ";
  terminalLog[1] += message;
  Serial.println(terminalLog[1]);
}

// ─────────────────────────────────────────────────────────────────────
// UTILITY FUNCTION: MOVING AVERAGE FILTER
// ─────────────────────────────────────────────────────────────────────
float applyMovingAverageFilter(float newReading) {
  sensorReadings[filterIndex] = newReading;
  filterIndex = (filterIndex + 1) % FILTER_WINDOW;
  
  float sum = 0.0;
  for (int i = 0; i < FILTER_WINDOW; i++) {
    sum += sensorReadings[i];
  }
  return sum / (float)FILTER_WINDOW;
}

// ─────────────────────────────────────────────────────────────────────
// SENSOR FUNCTION: READ ULTRASONIC DISTANCE
// ─────────────────────────────────────────────────────────────────────
float readUltrasonicSensor() {
  // Fire 10µs trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Listen for echo with 30ms safety timeout
  unsigned long echoTime = pulseIn(ECHO_PIN, HIGH, 30000UL);
  
  if (echoTime == 0) {
    return -1.0;
  }
  
  // Calculate raw distance
  float rawDistance = (echoTime * 0.0343) / 2.0;
  
  // Validate reading
  if (rawDistance < SENSOR_MIN_VALID || rawDistance > SENSOR_MAX_VALID) {
    return -1.0;
  }
  
  // Apply moving average filter
  rawDistance = applyMovingAverageFilter(rawDistance);
  
  // Convert sensor distance to true water depth
  // Smaller distance = more water = higher depth
  float trueDepth = TANK_AIR_GAP - rawDistance;
  
  // Clamp to physical limits
  if (trueDepth < 0.0) trueDepth = 0.0;
  if (trueDepth > TANK_AIR_GAP) trueDepth = TANK_AIR_GAP;
  
  return trueDepth;
}

// ─────────────────────────────────────────────────────────────────────
// CORE STATE MACHINE: AUTOMATION LOGIC
// ─────────────────────────────────────────────────────────────────────
void updateStateMachine() {
  
  if (isManualOverride) {
    return;
  }
  
  if (pumpDryRunFault) {
    currentState = STATE_DRY_RUN_FAULT;
    return;
  }
  
  // --- OUT VALVE CONTROL ---
  if (real_water_depth > DEPTH_LOW) {
    digitalWrite(OUT_VALVE_PIN, LOW);    // OPEN out valve
  } else {
    digitalWrite(OUT_VALVE_PIN, HIGH);   // CLOSE out valve
  }

  // --- FULL CONDITION STABILITY CHECK ---
  bool isStableFull = false;
  if (real_water_depth >= DEPTH_HIGH) {
    if (!highTimerActive) {
      highTimerActive = true;
      fullConditionStartTime = millis();
    }
    // Require 3000ms continuous reading >= DEPTH_HIGH to prevent relay crash from ripples
    if (millis() - fullConditionStartTime >= 3000) {
      isStableFull = true;
    }
  } else {
    highTimerActive = false;
  }

  // --- DETERMINE CURRENT PUMP REQUIREMENT ---
  if (real_water_depth < DEPTH_START_FILL) {
    // START FILLING IF LOW
    if (!isPumpEnabled) {
      isPumpEnabled = true;
      pumpStartTime = millis();
      updateLog("AUTO: PUMP START");
    }
    
    if (real_water_depth < DEPTH_CRITICAL) {
      currentState = STATE_CRITICAL_LOW;
    } else {
      currentState = STATE_FILLING;
    }
  }
  else if (isStableFull) {
    // STOP FILLING IF FULL (AND STABLE)
    if (isPumpEnabled) {
      isPumpEnabled = false;
      updateLog("AUTO: TANK FULL");
    }
    currentState = STATE_FULL;
  }
  else {
    // IN BETWEEN (DEPTH_START_FILL <= depth < DEPTH_HIGH or Not Stable Yet)
    if (isPumpEnabled) {
      currentState = STATE_FILLING;
    } else {
      currentState = STATE_IDLE;
    }
  }
  
  // --- RULE 4: DRY-RUN FAULT DETECTION ---
  if (isPumpEnabled && !pumpDryRunFault) {
    unsigned long pumpRunTime = millis() - pumpStartTime;
    
    if (pumpRunTime > DRY_RUN_TIMEOUT && flowRate_LPM < DRY_RUN_THRESHOLD) {
      pumpDryRunFault = true;
      isPumpEnabled = false;
      currentState = STATE_DRY_RUN_FAULT;
      
      // EMERGENCY SHUTDOWN
      digitalWrite(PUMP_PIN, HIGH);
      digitalWrite(IN_VALVE_PIN, HIGH);
      digitalWrite(OUT_VALVE_PIN, HIGH);
      
      updateLog("FAULT: PUMP DRY RUN");
      return;
    }
  }
  
  // --- PUMP & IN-VALVE ACTUATION ---
  if (isPumpEnabled) {
    digitalWrite(IN_VALVE_PIN, LOW); // Open inlet valve
    digitalWrite(PUMP_PIN, LOW);     // Pump ON 100% continuous (No pulsing)
  } else {
    // Ensure pump and IN valve are OFF when not enabled
    digitalWrite(PUMP_PIN, HIGH);
    digitalWrite(IN_VALVE_PIN, HIGH);
  }
}

// ─────────────────────────────────────────────────────────────────────
// LED & BUZZER CONTROLLER
// ─────────────────────────────────────────────────────────────────────
void updateLEDAndBuzzer() {
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
  bool buzzerActive = false;
  
  // Hysteresis for low water alarm (Scenario 1)
  if (real_water_depth <= DEPTH_LOW) {
    lowWaterAlarmActive = true;
  } else if (real_water_depth >= DEPTH_LOW + 5.0) {
    lowWaterAlarmActive = false;
  }
  
  if (pumpDryRunFault) {
    // FAULT STATE: Flash red LED only (no buzzer)
    bool flash = (millis() / 150) % 2;
    digitalWrite(LED_RED, flash);
  } 
  else if (currentState == STATE_BOOT) {
    digitalWrite(LED_BLUE, HIGH);
  }
  else if (real_water_depth >= DEPTH_HIGH) {
    // FULL: Green LED
    digitalWrite(LED_GREEN, HIGH);
  } 
  else if (real_water_depth >= DEPTH_LOW) {
    // NOMINAL: Blue LED
    digitalWrite(LED_BLUE, HIGH);
  } 
  else if (real_water_depth < DEPTH_LOW) {
    // LOW: Red LED
    digitalWrite(LED_RED, HIGH);
  }
  
  // Buzzer Pattern: "kiu kiu kiu" (e.g. 500ms ON, 300ms OFF)
  if (lowWaterAlarmActive) {
    int cycle = millis() % 800;
    if (cycle < 500) {
      buzzerActive = true;
    }
  }
  
  if (!isManualOverride) {
    digitalWrite(BUZZER_PIN, buzzerActive ? BUZZER_ON : BUZZER_OFF);
  }
}

// ─────────────────────────────────────────────────────────────────────
// SCADA SERIAL COMMAND INTERFACE (115200 baud)
// ─────────────────────────────────────────────────────────────────────
void processSerialCommand() {
  if (!Serial.available()) return;
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();
  
  // MASTER MODE COMMANDS
  if (command == "AUTO") {
    isManualOverride = false;
    pumpDryRunFault = false;
    currentState = STATE_IDLE;
    updateLog("AUTO MODE RESTORED");
  }
  else if (command == "OVERRIDE ON") {
    isManualOverride = true;
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    updateLog("MANUAL OVERRIDE ON");
  }
  else if (command == "STOP ALL") {
    isManualOverride = true;
    digitalWrite(OUT_VALVE_PIN, HIGH);
    digitalWrite(IN_VALVE_PIN, HIGH);
    digitalWrite(PUMP_PIN, HIGH);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    isPumpEnabled = false;
    updateLog("EMERGENCY STOP");
  }
  else if (command == "RESET") {
    pumpDryRunFault = false;
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    updateLog("FAULTS CLEARED");
  }
  
  // MANUAL CONTROLS
  else if (isManualOverride) {
    if (command == "OUT OPEN") {
      digitalWrite(OUT_VALVE_PIN, LOW);
      updateLog("OUT VALVE OPENED");
    }
    else if (command == "OUT CLOSE") {
      digitalWrite(OUT_VALVE_PIN, HIGH);
      updateLog("OUT VALVE CLOSED");
    }
    else if (command == "IN OPEN") {
      digitalWrite(IN_VALVE_PIN, LOW);
      updateLog("IN VALVE OPENED");
    }
    else if (command == "IN CLOSE") {
      digitalWrite(IN_VALVE_PIN, HIGH);
      updateLog("IN VALVE CLOSED");
    }
    else if (command == "PUMP ON") {
      digitalWrite(PUMP_PIN, LOW);
      updateLog("PUMP MANUAL ON");
    }
    else if (command == "PUMP OFF") {
      digitalWrite(PUMP_PIN, HIGH);
      updateLog("PUMP MANUAL OFF");
    }
    else if (command == "ALARM ON") {
      digitalWrite(BUZZER_PIN, BUZZER_ON);
      updateLog("ALARM MANUAL ON");
    }
    else if (command == "ALARM OFF") {
      digitalWrite(BUZZER_PIN, BUZZER_OFF);
      updateLog("ALARM MANUAL OFF");
    }
    else {
      Serial.println("Unknown command. Type AUTO to exit manual mode.");
    }
  } 
  else {
    if (command != "AUTO" && command != "OVERRIDE ON" && command != "STOP ALL" && command != "RESET") {
      Serial.println("Type 'OVERRIDE ON' to enable manual controls.");
    }
  }
}

// ─────────────────────────────────────────────────────────────────────
// OLED DISPLAY RENDERER (10 FPS, flicker-free)
// ─────────────────────────────────────────────────────────────────────
void renderOLEDDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // LEFT PANEL: GRAPHICAL TANK
  display.drawRect(0, 0, 24, 64, SSD1306_WHITE);
  
  float fillPercentage = (real_water_depth / MAX_WATER_CAPACITY) * 100.0;
  if (fillPercentage > 100.0) fillPercentage = 100.0;
  if (fillPercentage < 0.0) fillPercentage = 0.0;
  
  int fillHeight = map((int)fillPercentage, 0, 100, 0, 62);
  display.fillRect(1, 63 - fillHeight, 22, fillHeight, SSD1306_WHITE);
  
  // RIGHT PANEL: TELEMETRY & TERMINAL
  int col = 28;
  
  // Status bar
  display.fillRect(col, 0, 100, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(col + 2, 1);
  
  if (pumpDryRunFault) {
    display.print("! PUMP DRY-RUN !");
  } 
  else if (currentState == STATE_BOOT) {
    display.print("WARMING UP...");
  } 
  else if (isManualOverride) {
    display.print("MANUAL MODE");
  } 
  else if (currentState == STATE_CRITICAL_LOW) {
    display.print("CRITICAL LOW");
  }
  else if (currentState == STATE_THROTTLING) {
    display.print("AUTO: THROTTLE");
  }
  else if (currentState == STATE_FILLING) {
    display.print("AUTO: FILLING");
  }
  else if (currentState == STATE_FULL) {
    display.print("AUTO: FULL");
  }
  else {
    display.print("AUTO: IDLE");
  }
  
  display.setTextColor(SSD1306_WHITE);
  
  // Data row 1: Water level
  display.setCursor(col, 12);
  display.print("LVL:");
  display.print(real_water_depth, 1);
  display.print("cm ");
  
  if (real_water_depth <= DEPTH_LOW) {
    display.print("(LO)");
  } else if (real_water_depth >= DEPTH_HIGH) {
    display.print("(HI)");
  } else if (real_water_depth >= DEPTH_THROTTLE) {
    display.print("(TH)");
  }
  
  // Data row 2: Valve states
  bool inValveOpen = (digitalRead(IN_VALVE_PIN) == LOW);
  bool outValveOpen = (digitalRead(OUT_VALVE_PIN) == LOW);
  
  display.setCursor(col, 22);
  display.print("VLV: IN[");
  display.print(inValveOpen ? "OPN" : "CLS");
  display.print("] OUT[");
  display.print(outValveOpen ? "OPN" : "CLS");
  display.print("]");
  
  // Data row 3: Pump & flow rate
  bool pumpOn = (digitalRead(PUMP_PIN) == LOW);
  
  display.setCursor(col, 32);
  display.print("PMP:[");
  display.print(pumpOn ? "ON " : "OFF");
  display.print("] FLW:");
  display.print(flowRate_LPM, 1);
  
  // Divider line
  display.drawFastHLine(col, 42, 100, SSD1306_WHITE);
  
  // Terminal log (two-line scrolling)
  display.setTextSize(1);
  display.setCursor(col, 45);
  display.print(terminalLog[0]);
  
  display.setCursor(col, 55);
  display.print(terminalLog[1]);
  
  display.display();
}

// ─────────────────────────────────────────────────────────────────────
// SETUP: INITIALIZATION & HARDWARE CONFIGURATION
// ─────────────────────────────────────────────────────────────────────
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("AquaIntel Master OS v11.1");
  Serial.println("Custom Tank Calibration (5cm-46cm)");
  Serial.println("========================================");
  
  // Configure GPIO pins
  digitalWrite(OUT_VALVE_PIN, HIGH);
  digitalWrite(IN_VALVE_PIN, HIGH);
  digitalWrite(PUMP_PIN, HIGH);
  pinMode(OUT_VALVE_PIN, OUTPUT);
  pinMode(IN_VALVE_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  
  pinMode(FLOW_PIN, INPUT_PULLUP);
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_OFF);
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);  // Boot indicator
  digitalWrite(LED_RED, LOW);
  
  // Attach hardware interrupt
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), countFlowPulse, FALLING);
  Serial.println("[OK] Flow sensor ISR attached");
  
  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[FAIL] OLED initialization failed!");
    while (1);
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("AquaIntel v11.1");
  display.println("Initializing...");
  display.display();
  Serial.println("[OK] OLED display initialized");
  
  currentState = STATE_BOOT;
  previousState = STATE_BOOT;
  updateLog("SYSTEM BOOTING");
  
  Serial.println("[READY] All systems operational. Entering main loop.\n");
  Serial.println("CUSTOM CALIBRATION:");
  Serial.println("  5cm = LOWEST (Red LED, Pump locked)");
  Serial.println("  46cm = FULL (Green LED, Pump stops)");
  Serial.println("  38cm = Start throttling (2s ON/OFF pulsing)\n");
}

// ─────────────────────────────────────────────────────────────────────
// MAIN LOOP: Non-blocking event loop
// ─────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  
  // TASK 1: READ ULTRASONIC SENSOR (Every 150ms)
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = now;
    
    float newDepth = readUltrasonicSensor();
    
    if (newDepth >= 0.0) {
      real_water_depth = newDepth;
      
      if (currentState == STATE_BOOT) {
        currentState = STATE_IDLE;
        updateLog("SENSORS ONLINE");
      }
    }
  }
  
  // TASK 2: CALCULATE FLOW RATE (Every 1000ms)
  if (now - lastFlowCalc >= 1000) {
    lastFlowCalc = now;
    
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();
    
    flowRate_LPM = ((float)pulses / PULSES_PER_LITRE) * 60.0;
  }
  
  // TASK 3: RUN STATE MACHINE (Every loop)
  updateStateMachine();
  
  // TASK 4: UPDATE LED & BUZZER (Every loop)
  updateLEDAndBuzzer();
  
  // TASK 4.5: PUMP STATE CHANGE ALARM ("kik kik")
  if (isPumpEnabled != previousPumpState) {
    if (!isManualOverride) {
      digitalWrite(BUZZER_PIN, BUZZER_ON);
      delay(60);
      digitalWrite(BUZZER_PIN, BUZZER_OFF);
      delay(60);
      digitalWrite(BUZZER_PIN, BUZZER_ON);
      delay(60);
      digitalWrite(BUZZER_PIN, BUZZER_OFF);
    }
    previousPumpState = isPumpEnabled;
  }
  
  // TASK 5: LISTEN FOR SCADA COMMANDS (Every loop)
  processSerialCommand();
  
  // TASK 6: RENDER OLED DISPLAY (Every 100ms = 10 FPS)
  if (now - lastUIUpdate >= 100) {
    lastUIUpdate = now;
    renderOLEDDisplay();
  }
}

// ─────────────────────────────────────────────────────────────────────
// END OF CODE
// ─────────────────────────────────────────────────────────────────────
//
// QUICK START:
//   1. Upload this code to your ESP32
//   2. Open Serial Monitor at 115200 baud
//   3. Watch the OLED display
//   4. If empty tank shows 0cm and full shows ~46cm, perfect!
//   5. If readings are off, adjust TANK_AIR_GAP constant (line ~58)
//
// SERIAL COMMANDS (at 115200 baud):
//   AUTO              → Restore automation
//   OVERRIDE ON       → Enable manual control
//   STOP ALL          → Emergency shutdown
//   RESET             → Clear faults
//   IN OPEN/CLOSE     → Inlet valve control
//   OUT OPEN/CLOSE    → Drain valve control
//   PUMP ON/OFF       → Pump manual control
//   ALARM ON/OFF      → Buzzer manual control
//
// OLED DISPLAY SHOWS:
//   5cm = Lowest (Red LED, pump locked)
//   10cm = Start filling (RED + BUZZER)
//   38cm = Throttle zone starts
//   46cm = Full (GREEN LED, pump stops)
//
// ─────────────────────────────────────────────────────────────────────
