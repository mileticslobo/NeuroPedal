#include "arduino_secrets.h"

#include <WiFiS3.h>
#include <math.h>
#include <cstring>

// Motor driver pins
constexpr uint8_t RPWM = 5;
constexpr uint8_t LPWM = 6;
constexpr uint8_t R_EN = 7;
constexpr uint8_t L_EN = 8;

// Single external current sensor (ACS758) setup:
// Wire ACS758 VOUT -> A0 (through 10k; add 100nF from A0 to GND).
constexpr uint8_t CS_PIN = A0;

// Hardware constants for ACS758 current sensor (adjust to your specific part)
constexpr float ADC_VREF = 5.0f;
constexpr uint8_t ADC_RESOLUTION = 12;
constexpr uint16_t ADC_MAX = (1u << ADC_RESOLUTION) - 1u;
constexpr float SENSOR_SENSITIVITY_V_PER_A = 0.04f; // 40 mV per ampere for ACS758-050U. Change if you use another variant.
constexpr float FILTER_ALPHA = 0.25f;               // Weight for exponential smoothing of current readings

// Wi-Fi configuration
constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 2500;
constexpr unsigned long STATUS_LOG_INTERVAL_MS = 5000;
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;
constexpr unsigned long WIFI_RETRY_INTERVAL_MS = 15000;

const char *SSID = SECRET_SSID;
const char *PASS = SECRET_OPTIONAL_PASS;

enum class MotorDirection : uint8_t {
  Stopped = 0,
  Forward = 1,
  Reverse = 2
};

enum class RecoveryStep : uint8_t {
  Idle,
  RestartSlow,
  ReverseSlow,
  StopConfirmed
};

struct CurrentOffsets {
  uint16_t offsetRaw = ADC_MAX / 2;
};

struct SystemState {
  bool motorEnabled = false;
  bool timerRunning = false;
  bool spasmDetected = false;
  bool currentsInitialised = false;

  MotorDirection requestedDirection = MotorDirection::Forward;
  MotorDirection appliedDirection = MotorDirection::Stopped;

  RecoveryStep recoveryStep = RecoveryStep::Idle;

  int targetSpeedPercent = 40;  // 0-100
  int recoverySpeedPercent = 30;
  float thresholdAmp = 5.0f;    // Spasm threshold in amperes
  int durationMinutes = 5;

  unsigned long startMillis = 0;
  unsigned long lastSecondTick = 0;
  unsigned long lastCurrentSample = 0;
  unsigned long lastSpasmCheck = 0;
  unsigned long recoveryStartMillis = 0;
  unsigned long runDurationSeconds = 5 * 60;

  float filteredCurrent = 0.0f;
  float lastUsedCurrent = 0.0f;
  int remainingSeconds = 0;
  unsigned long spasmBlankUntil = 0;
};

WiFiServer server(80);

SystemState state;
CurrentOffsets currentOffsets;

// Forward declarations
void configureHardware();
void waitForSerial();
void printBootDiagnostics();
void printSerialHelp();
void handleSerialCommands();
void printPeriodicStatus(unsigned long now);
void connectWiFi();
void ensureWiFi();
void handleMotor(unsigned long now);
void handleTimer(unsigned long now);
void handleSpasm(unsigned long now);
void handleRecovery(unsigned long now);
void updateCurrentReadings();
bool calibrateCurrentOffsets();
float readSensorAmps(uint16_t offsetRaw);
float getCurrentForDirection(MotorDirection direction);
void applyMotorOutputs(MotorDirection direction, int percent);
void stopMotor();
void handleNetwork();
void handleHttpRequest(WiFiClient &client, const String &requestLine);
void sendHtml(WiFiClient &client, const __FlashStringHelper *content);
void sendHtml(WiFiClient &client, const char *content);
void sendJson(WiFiClient &client, const String &json);
String buildStatusJson();
void processControlQuery(const String &query, WiFiClient &client);
bool extractQueryValue(const String &query, const char *key, String &value);
void startMotorSession(MotorDirection direction);
void stopMotorSession(const char *reason);
const char *directionName(MotorDirection direction);
int speedPercentToPwm(int percent);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>cs-motomed-controller Controller</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 16px; background: #f4f4f4; }
    h1 { font-size: 1.4rem; }
    section { background: #fff; padding: 14px; margin-bottom: 16px; border-radius: 8px; box-shadow: 0 1px 3px rgba(0,0,0,0.12); }
    label { display: block; margin-top: 10px; font-weight: bold; }
    input, select, button { width: 100%; padding: 10px; margin-top: 6px; box-sizing: border-box; border-radius: 4px; border: 1px solid #ccc; }
    button { background: #0069d9; color: #fff; border: none; cursor: pointer; }
    button.stop { background: #c82333; }
    .grid { display: grid; gap: 10px; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); margin-top: 12px; }
    .status { font-family: monospace; background: #222; color: #0f0; padding: 10px; border-radius: 4px; overflow-x: auto; }
  </style>
</head>
<body>
  <section>
    <h1>cs-motomed-controller Controller</h1>
    <div class="grid">
      <button id="startButton">Start</button>
      <button id="stopButton" class="stop">Stop</button>
      <button id="calibrateButton">Calibrate Sensors</button>
    </div>
    <label for="speedSlider">Speed (%)</label>
    <input type="range" id="speedSlider" min="0" max="100" value="40" />
    <label for="directionSelect">Direction</label>
    <select id="directionSelect">
      <option value="forward">Forward</option>
      <option value="reverse">Reverse</option>
    </select>
    <label for="durationInput">Duration (minutes)</label>
    <input type="number" id="durationInput" value="5" min="0" max="120" />
    <label for="thresholdInput">Spasm Threshold (A)</label>
    <input type="number" id="thresholdInput" value="5" min="0.5" max="40" step="0.5" />
  </section>
  <section>
    <h2>Status</h2>
    <pre class="status" id="statusBox">Connecting...</pre>
  </section>
  <script>
    const statusBox = document.getElementById('statusBox');
    const speedSlider = document.getElementById('speedSlider');
    const directionSelect = document.getElementById('directionSelect');
    const durationInput = document.getElementById('durationInput');
    const thresholdInput = document.getElementById('thresholdInput');

    async function callControl(params) {
      const url = `/control?${params}`;
      try {
        const res = await fetch(url);
        const text = await res.text();
        statusBox.textContent = text;
      } catch (err) {
        statusBox.textContent = `Error: ${err}`;
      }
    }

    document.getElementById('startButton').addEventListener('click', () => {
      const params = new URLSearchParams({
        action: 'start',
        speed: speedSlider.value,
        direction: directionSelect.value,
        duration: durationInput.value,
        threshold: thresholdInput.value
      });
      callControl(params.toString());
    });

    document.getElementById('stopButton').addEventListener('click', () => {
      callControl('action=stop');
    });

    document.getElementById('calibrateButton').addEventListener('click', () => {
      callControl('action=calibrate');
    });

    speedSlider.addEventListener('change', () => {
      callControl(`speed=${speedSlider.value}`);
    });

    directionSelect.addEventListener('change', () => {
      callControl(`direction=${directionSelect.value}`);
    });

    durationInput.addEventListener('change', () => {
      callControl(`duration=${durationInput.value}`);
    });

    thresholdInput.addEventListener('change', () => {
      callControl(`threshold=${thresholdInput.value}`);
    });

    async function refreshStatus() {
      try {
        const res = await fetch('/status');
        statusBox.textContent = await res.text();
      } catch (err) {
        statusBox.textContent = `Error: ${err}`;
      }
    }

    setInterval(refreshStatus, 1500);
    refreshStatus();
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  waitForSerial();
  Serial.println();
  Serial.println("cs-motomed-controller booting.");
  configureHardware();
  printBootDiagnostics();
  printSerialHelp();
  connectWiFi();
  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  unsigned long now = millis();

  handleSerialCommands();
  ensureWiFi();
  handleMotor(now);
  handleTimer(now);
  handleSpasm(now);
  handleRecovery(now);
  handleNetwork();
  printPeriodicStatus(now);
}

void configureHardware() {
  analogReadResolution(ADC_RESOLUTION);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  stopMotor();
}

void waitForSerial() {
  unsigned long start = millis();
  while (!Serial && (millis() - start) < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }
}

void printBootDiagnostics() {
  Serial.print("Motor pins: RPWM=D");
  Serial.print(RPWM);
  Serial.print(" LPWM=D");
  Serial.print(LPWM);
  Serial.print(" R_EN=D");
  Serial.print(R_EN);
  Serial.print(" L_EN=D");
  Serial.println(L_EN);

  Serial.print("Current sensor: A");
  Serial.print(CS_PIN - A0);
  Serial.print(" ADC bits=");
  Serial.print(ADC_RESOLUTION);
  Serial.print(" initialOffset=");
  Serial.println(currentOffsets.offsetRaw);

  Serial.print("Initial speed=");
  Serial.print(state.targetSpeedPercent);
  Serial.print("% pwm=");
  Serial.println(speedPercentToPwm(state.targetSpeedPercent));
}

void printSerialHelp() {
  Serial.println("Serial commands: f=start forward, r=start reverse, s=stop, +=speed up, -=speed down, c=calibrate, ?=help");
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    if (command >= 'A' && command <= 'Z') {
      command = command - 'A' + 'a';
    }

    switch (command) {
      case 'f':
        startMotorSession(MotorDirection::Forward);
        break;

      case 'r':
        startMotorSession(MotorDirection::Reverse);
        break;

      case 's':
        stopMotorSession("serial stop");
        break;

      case '+':
        state.targetSpeedPercent = constrain(state.targetSpeedPercent + 5, 0, 100);
        Serial.print("Speed set to ");
        Serial.print(state.targetSpeedPercent);
        Serial.print("% pwm=");
        Serial.println(speedPercentToPwm(state.targetSpeedPercent));
        if (state.motorEnabled && !state.spasmDetected) {
          applyMotorOutputs(state.requestedDirection, state.targetSpeedPercent);
        }
        break;

      case '-':
        state.targetSpeedPercent = constrain(state.targetSpeedPercent - 5, 0, 100);
        Serial.print("Speed set to ");
        Serial.print(state.targetSpeedPercent);
        Serial.print("% pwm=");
        Serial.println(speedPercentToPwm(state.targetSpeedPercent));
        if (state.motorEnabled && !state.spasmDetected) {
          applyMotorOutputs(state.requestedDirection, state.targetSpeedPercent);
        }
        break;

      case 'c':
        if (state.motorEnabled) {
          Serial.println("Calibration refused: stop motor first.");
        } else {
          stopMotor();
          state.appliedDirection = MotorDirection::Stopped;
          delay(50);
          calibrateCurrentOffsets();
        }
        break;

      case '?':
        printSerialHelp();
        break;

      case '\n':
      case '\r':
      case ' ':
      case '\t':
        break;

      default:
        Serial.print("Unknown serial command: ");
        Serial.println(command);
        printSerialHelp();
        break;
    }
  }
}

void printPeriodicStatus(unsigned long now) {
  static unsigned long lastLog = 0;
  if (now - lastLog < STATUS_LOG_INTERVAL_MS) {
    return;
  }

  lastLog = now;
  Serial.print("Status: wifi=");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(WiFi.localIP());
  } else {
    Serial.print("disconnected");
  }
  Serial.print(" motor=");
  Serial.print(state.motorEnabled ? "enabled" : "disabled");
  Serial.print(" requested=");
  Serial.print(directionName(state.requestedDirection));
  Serial.print(" applied=");
  Serial.print(directionName(state.appliedDirection));
  Serial.print(" speed=");
  Serial.print(state.targetSpeedPercent);
  Serial.print("% pwm=");
  Serial.print(speedPercentToPwm(state.targetSpeedPercent));
  Serial.print(" current=");
  Serial.print(state.filteredCurrent, 2);
  Serial.print("A spasm=");
  Serial.println(state.spasmDetected ? "yes" : "no");
}

void connectWiFi() {
  if (!SSID || strlen(SSID) == 0) {
    Serial.println("Wi-Fi SSID is empty. Update arduino_secrets.h");
    return;
  }

  Serial.print("Connecting to Wi-Fi network: ");
  Serial.println(SSID);

  WiFi.begin(SSID, PASS);
  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected. IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("Wi-Fi connection failed.");
  }
}

void ensureWiFi() {
  static unsigned long lastAttempt = 0;
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  unsigned long now = millis();
  if (now - lastAttempt >= WIFI_RETRY_INTERVAL_MS) {
    lastAttempt = now;
    connectWiFi();
  }
}

void handleMotor(unsigned long now) {
  const unsigned long CURRENT_SAMPLE_INTERVAL_MS = 40;
  if (now - state.lastCurrentSample >= CURRENT_SAMPLE_INTERVAL_MS) {
    state.lastCurrentSample = now;
    updateCurrentReadings();
  }

  if (state.spasmDetected || !state.motorEnabled) {
    return;
  }

  if (state.appliedDirection != state.requestedDirection) {
    applyMotorOutputs(state.requestedDirection, state.targetSpeedPercent);
    state.appliedDirection = state.requestedDirection;
  } else {
    applyMotorOutputs(state.requestedDirection, state.targetSpeedPercent);
  }
}

void handleTimer(unsigned long now) {
  if (!state.timerRunning || !state.motorEnabled) {
    return;
  }

  if (now - state.lastSecondTick >= 1000) {
    state.lastSecondTick = now;
    unsigned long elapsedSeconds = (now - state.startMillis) / 1000;
    long remaining = (long)state.runDurationSeconds - (long)elapsedSeconds;

    if (remaining <= 0) {
      stopMotorSession("timer complete");
    } else {
      state.remainingSeconds = (int)remaining;
    }
  }
}

void handleSpasm(unsigned long now) {
  const unsigned long SPASM_CHECK_INTERVAL_MS = 80;
  if (!state.motorEnabled) {
    state.lastUsedCurrent = 0.0f;
    return;
  }

  if (state.spasmDetected) {
    return;
  }

  if (now - state.lastSpasmCheck < SPASM_CHECK_INTERVAL_MS) {
    return;
  }

  state.lastSpasmCheck = now;

  float currentUsed = getCurrentForDirection(state.requestedDirection);
  state.lastUsedCurrent = currentUsed;

  if (now < state.spasmBlankUntil) {
    return;
  }

  if (currentUsed >= state.thresholdAmp) {
    Serial.print("Spasm detected. Current: ");
    Serial.println(currentUsed);
    state.spasmDetected = true;
    state.recoveryStep = RecoveryStep::RestartSlow;
    state.recoveryStartMillis = now;
    stopMotor();
    state.appliedDirection = MotorDirection::Stopped;
  }
}

void handleRecovery(unsigned long now) {
  if (!state.spasmDetected) {
    return;
  }

  const unsigned long RECOVERY_DELAY_MS = 1200;
  if (now - state.recoveryStartMillis < RECOVERY_DELAY_MS) {
    return;
  }

  // Refresh currents just before making a decision.
  updateCurrentReadings();
  float currentUsed = getCurrentForDirection(state.requestedDirection);
  state.lastUsedCurrent = currentUsed;

  switch (state.recoveryStep) {
    case RecoveryStep::RestartSlow:
      Serial.println("Attempting slow restart...");
      applyMotorOutputs(state.requestedDirection, state.recoverySpeedPercent);
      state.appliedDirection = state.requestedDirection;
      state.recoveryStep = RecoveryStep::ReverseSlow;
      break;

    case RecoveryStep::ReverseSlow:
      if (currentUsed >= state.thresholdAmp) {
        Serial.println("Second spasm. Trying opposite direction.");
        MotorDirection opposite = (state.requestedDirection == MotorDirection::Forward)
                                      ? MotorDirection::Reverse
                                      : MotorDirection::Forward;
        applyMotorOutputs(opposite, state.recoverySpeedPercent);
        state.appliedDirection = opposite;
        state.recoveryStep = RecoveryStep::StopConfirmed;
      } else {
        Serial.println("Recovery successful. Restoring target speed.");
        state.spasmDetected = false;
        state.recoveryStep = RecoveryStep::Idle;
        applyMotorOutputs(state.requestedDirection, state.targetSpeedPercent);
        state.appliedDirection = state.requestedDirection;
      }
      break;

    case RecoveryStep::StopConfirmed:
      if (currentUsed >= state.thresholdAmp) {
        stopMotorSession("persistent spasm");
      } else {
        Serial.println("Recovery after direction reversal successful.");
        applyMotorOutputs(state.requestedDirection, state.targetSpeedPercent);
        state.appliedDirection = state.requestedDirection;
      }
      state.spasmDetected = false;
      state.recoveryStep = RecoveryStep::Idle;
      break;

    case RecoveryStep::Idle:
    default:
      state.spasmDetected = false;
      break;
  }

  state.recoveryStartMillis = now;
}

void updateCurrentReadings() {
  float i = readSensorAmps(currentOffsets.offsetRaw);

  if (!state.currentsInitialised) {
    state.filteredCurrent = i;
    state.currentsInitialised = true;
  } else {
    state.filteredCurrent += FILTER_ALPHA * (i - state.filteredCurrent);
  }
}

bool calibrateCurrentOffsets() {
  const int samples = 200;
  uint16_t buf[samples];
  uint32_t sum = 0;

  for (int i = 0; i < samples; ++i) {
    buf[i] = analogRead(CS_PIN);
    sum += buf[i];
    delay(2);
  }

  uint16_t mean = sum / samples;

  uint32_t sumSq = 0;
  for (int i = 0; i < samples; ++i) {
    int32_t d = (int32_t)buf[i] - (int32_t)mean;
    sumSq += (uint32_t)(d * d);
  }
  float stdev = sqrtf((float)sumSq / samples);

  // Bidirectional variants idle near Vcc/2; unidirectional variants idle
  // near 0.6 V. Accept anything well inside the ADC range, and trust the
  // std-dev check to catch "motor still moving" or broken-supply cases.
  bool plausible = (mean > 200 && mean < (ADC_MAX - 200)) && (stdev < 12.0f);
  if (!plausible) {
    Serial.print("Calibration rejected. mean=");
    Serial.print(mean);
    Serial.print(" stdev=");
    Serial.println(stdev);
    return false;
  }

  currentOffsets.offsetRaw = mean;
  state.currentsInitialised = false;
  state.filteredCurrent = 0.0f;
  state.lastUsedCurrent = 0.0f;
  state.spasmBlankUntil = millis() + 300;

  Serial.print("Calibrated. offset=");
  Serial.print(mean);
  Serial.print(" stdev=");
  Serial.println(stdev);
  return true;
}

float readSensorAmps(uint16_t offsetRaw) {
  int raw = analogRead(CS_PIN);
  int delta = raw - (int)offsetRaw;
  float voltageDelta = (delta * ADC_VREF) / (float)ADC_MAX;
  float amps = voltageDelta / SENSOR_SENSITIVITY_V_PER_A;
  return fabsf(amps);
}

float getCurrentForDirection(MotorDirection /*direction*/) {
  return state.filteredCurrent;
}

void applyMotorOutputs(MotorDirection direction, int percent) {
  percent = constrain(percent, 0, 100);
  int pwm = speedPercentToPwm(percent);

  switch (direction) {
    case MotorDirection::Forward:
      analogWrite(RPWM, pwm);
      analogWrite(LPWM, 0);
      break;

    case MotorDirection::Reverse:
      analogWrite(RPWM, 0);
      analogWrite(LPWM, pwm);
      break;

    case MotorDirection::Stopped:
    default:
      stopMotor();
      break;
  }
}

void stopMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void handleNetwork() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  String requestLine = client.readStringUntil('\r');
  client.read(); // consume '\n'

  while (client.connected()) {
    String headerLine = client.readStringUntil('\r');
    client.read();
    if (headerLine.length() == 0) {
      break;
    }
  }

  handleHttpRequest(client, requestLine);
  client.stop();
}

void handleHttpRequest(WiFiClient &client, const String &requestLine) {
  if (requestLine.length() == 0) {
    return;
  }

  Serial.print("HTTP: ");
  Serial.println(requestLine);

  if (!requestLine.startsWith("GET ")) {
    client.println("HTTP/1.1 405 Method Not Allowed\r");
    client.println();
    return;
  }

  int pathStart = 4;
  int pathEnd = requestLine.indexOf(' ', pathStart);
  if (pathEnd == -1) {
    pathEnd = requestLine.length();
  }

  String fullPath = requestLine.substring(pathStart, pathEnd);

  if (fullPath.startsWith("/status")) {
    sendJson(client, buildStatusJson());
    return;
  }

  if (fullPath.startsWith("/control")) {
    int qIndex = fullPath.indexOf('?');
    String query = (qIndex >= 0) ? fullPath.substring(qIndex + 1) : "";
    processControlQuery(query, client);
    return;
  }

  sendHtml(client, INDEX_HTML);
}

void sendHtml(WiFiClient &client, const __FlashStringHelper *content) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Connection: close"));
  client.println();
  client.print(content);
}

void sendHtml(WiFiClient &client, const char *content) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Connection: close"));
  client.println();
  client.print(content);
}

void sendJson(WiFiClient &client, const String &json) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Connection: close"));
  client.println();
  client.print(json);
}

String buildStatusJson() {
  String json = "{";
  json += "\"motorEnabled\":" + String(state.motorEnabled ? "true" : "false");
  json += ",\"direction\":";
  switch (state.requestedDirection) {
    case MotorDirection::Forward:
      json += "\"forward\"";
      break;
    case MotorDirection::Reverse:
      json += "\"reverse\"";
      break;
    default:
      json += "\"stopped\"";
      break;
  }
  json += ",\"speedPercent\":" + String(state.targetSpeedPercent);
  json += ",\"remainingSeconds\":" + String(state.remainingSeconds);
  json += ",\"thresholdAmp\":" + String(state.thresholdAmp, 2);
  json += ",\"current\":" + String(state.filteredCurrent, 2);
  json += ",\"lastCurrent\":" + String(state.lastUsedCurrent, 2);
  json += ",\"spasmDetected\":" + String(state.spasmDetected ? "true" : "false");
  json += ",\"wifi\":\"";
  if (WiFi.status() == WL_CONNECTED) {
    json += WiFi.localIP().toString();
  } else {
    json += "disconnected";
  }
  json += "\"}";
  return json;
}

void processControlQuery(const String &query, WiFiClient &client) {
  if (query.length() == 0) {
    sendJson(client, buildStatusJson());
    return;
  }

  String value;

  if (extractQueryValue(query, "speed", value)) {
    int speedPercent = constrain(value.toInt(), 0, 100);
    state.targetSpeedPercent = speedPercent;
    if (!state.spasmDetected && state.motorEnabled) {
      applyMotorOutputs(state.requestedDirection, state.targetSpeedPercent);
    }
  }

  if (extractQueryValue(query, "direction", value)) {
    if (value == "forward") {
      state.requestedDirection = MotorDirection::Forward;
    } else if (value == "reverse") {
      state.requestedDirection = MotorDirection::Reverse;
    }
  }

  if (extractQueryValue(query, "duration", value)) {
    int minutes = constrain(value.toInt(), 0, 180);
    state.durationMinutes = minutes;
    state.runDurationSeconds = (unsigned long)minutes * 60UL;
    if (state.timerRunning) {
      state.remainingSeconds = (int)state.runDurationSeconds;
    }
  }

  if (extractQueryValue(query, "threshold", value)) {
    float threshold = value.toFloat();
    if (threshold >= 0.1f) {
      state.thresholdAmp = threshold;
    }
  }

  if (extractQueryValue(query, "action", value)) {
    if (value == "start") {
      startMotorSession(state.requestedDirection);
    } else if (value == "stop") {
      stopMotorSession("HTTP stop");
    } else if (value == "calibrate") {
      if (state.motorEnabled) {
        // Refuse: regen current from a coasting motor poisons the offset,
        // and auto-restarting after calibration was the spike source.
        sendJson(client, String("{\"error\":\"stop motor before calibrating\"}"));
        return;
      }
      stopMotor();
      state.appliedDirection = MotorDirection::Stopped;
      delay(50);
      calibrateCurrentOffsets();
    }
  }

  sendJson(client, buildStatusJson());
}

bool extractQueryValue(const String &query, const char *key, String &value) {
  String token = String(key) + '=';
  int start = query.indexOf(token);
  if (start == -1) {
    return false;
  }

  start += token.length();
  int end = query.indexOf('&', start);
  if (end == -1) {
    end = query.length();
  }

  value = query.substring(start, end);
  value.replace("%20", " ");
  return true;
}

void startMotorSession(MotorDirection direction) {
  Serial.print("Starting motor: direction=");
  Serial.print(directionName(direction));
  Serial.print(" speed=");
  Serial.print(state.targetSpeedPercent);
  Serial.print("% pwm=");
  Serial.println(speedPercentToPwm(state.targetSpeedPercent));

  state.motorEnabled = true;
  state.spasmDetected = false;
  state.requestedDirection = direction;
  state.appliedDirection = MotorDirection::Stopped;
  state.lastUsedCurrent = 0.0f;

  unsigned long now = millis();
  state.startMillis = now;
  state.lastSecondTick = now;
  state.remainingSeconds = (int)state.runDurationSeconds;
  state.timerRunning = state.runDurationSeconds > 0;
  state.spasmBlankUntil = now + 300;

  applyMotorOutputs(direction, state.targetSpeedPercent);
  state.appliedDirection = direction;
}

void stopMotorSession(const char *reason) {
  state.motorEnabled = false;
  state.timerRunning = false;
  state.remainingSeconds = 0;
  stopMotor();
  state.appliedDirection = MotorDirection::Stopped;

  Serial.print("Motor stopped");
  if (reason && strlen(reason) > 0) {
    Serial.print(": ");
    Serial.print(reason);
  }
  Serial.println();
}

const char *directionName(MotorDirection direction) {
  switch (direction) {
    case MotorDirection::Forward:
      return "forward";
    case MotorDirection::Reverse:
      return "reverse";
    case MotorDirection::Stopped:
    default:
      return "stopped";
  }
}

int speedPercentToPwm(int percent) {
  percent = constrain(percent, 0, 100);
  return map(percent, 0, 100, 0, 255);
}
