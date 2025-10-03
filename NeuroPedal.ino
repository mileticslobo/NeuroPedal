#include "arduino_secrets.h"

#include <WiFiS3.h>
#include <math.h>
#include <cstring>

// Motor driver pins
constexpr uint8_t RPWM = 5;
constexpr uint8_t LPWM = 6;
constexpr uint8_t R_EN = 7;
constexpr uint8_t L_EN = 8;

// Current sensor pins
constexpr uint8_t R_IS = A0;
constexpr uint8_t L_IS = A1;

// Hardware constants for ACS758 current sensor (adjust to your specific part)
constexpr float ADC_VREF = 5.0f;
constexpr uint8_t ADC_RESOLUTION = 12;
constexpr uint16_t ADC_MAX = (1u << ADC_RESOLUTION) - 1u;
constexpr float SENSOR_SENSITIVITY_V_PER_A = 0.04f; // 40 mV per ampere for ACS758-050U. Change if you use another variant.
constexpr float FILTER_ALPHA = 0.25f;               // Weight for exponential smoothing of current readings

// Wi-Fi configuration
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
  uint16_t forwardRaw = ADC_MAX / 2;
  uint16_t reverseRaw = ADC_MAX / 2;
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

  float filteredForwardCurrent = 0.0f;
  float filteredReverseCurrent = 0.0f;
  float lastUsedCurrent = 0.0f;
  int remainingSeconds = 0;
};

WiFiServer server(80);

SystemState state;
CurrentOffsets currentOffsets;

// Forward declarations
void configureHardware();
void connectWiFi();
void ensureWiFi();
void handleMotor(unsigned long now);
void handleTimer(unsigned long now);
void handleSpasm(unsigned long now);
void handleRecovery(unsigned long now);
void updateCurrentReadings();
void calibrateCurrentOffsets();
float readSensorAmps(uint8_t pin, uint16_t offsetRaw);
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

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>NeuroPedal Controller</title>
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
    <h1>NeuroPedal Controller</h1>
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
  configureHardware();
  connectWiFi();
  server.begin();
}

void loop() {
  unsigned long now = millis();

  ensureWiFi();
  handleMotor(now);
  handleTimer(now);
  handleSpasm(now);
  handleRecovery(now);
  handleNetwork();
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
      Serial.println("Timer complete. Stopping motor.");
      state.motorEnabled = false;
      state.timerRunning = false;
      state.remainingSeconds = 0;
      stopMotor();
      state.appliedDirection = MotorDirection::Stopped;
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
        Serial.println("Persistent spasm. Stopping motor.");
        stopMotor();
        state.appliedDirection = MotorDirection::Stopped;
        state.motorEnabled = false;
        state.timerRunning = false;
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
  float forward = readSensorAmps(R_IS, currentOffsets.forwardRaw);
  float reverse = readSensorAmps(L_IS, currentOffsets.reverseRaw);

  if (!state.currentsInitialised) {
    state.filteredForwardCurrent = forward;
    state.filteredReverseCurrent = reverse;
    state.currentsInitialised = true;
  } else {
    state.filteredForwardCurrent += FILTER_ALPHA * (forward - state.filteredForwardCurrent);
    state.filteredReverseCurrent += FILTER_ALPHA * (reverse - state.filteredReverseCurrent);
  }
}

void calibrateCurrentOffsets() {
  const int samples = 200;
  uint32_t forwardSum = 0;
  uint32_t reverseSum = 0;

  for (int i = 0; i < samples; ++i) {
    forwardSum += analogRead(R_IS);
    reverseSum += analogRead(L_IS);
    delay(2);
  }

  currentOffsets.forwardRaw = forwardSum / samples;
  currentOffsets.reverseRaw = reverseSum / samples;
  state.currentsInitialised = false;

  Serial.print("Current sensors calibrated. Offsets -> Forward: ");
  Serial.print(currentOffsets.forwardRaw);
  Serial.print(" Reverse: ");
  Serial.println(currentOffsets.reverseRaw);
}

float readSensorAmps(uint8_t pin, uint16_t offsetRaw) {
  int raw = analogRead(pin);
  int delta = raw - (int)offsetRaw;
  float voltageDelta = (delta * ADC_VREF) / (float)ADC_MAX;
  float amps = voltageDelta / SENSOR_SENSITIVITY_V_PER_A;
  return fabsf(amps);
}

float getCurrentForDirection(MotorDirection direction) {
  if (direction == MotorDirection::Forward) {
    return state.filteredForwardCurrent;
  }
  if (direction == MotorDirection::Reverse) {
    return state.filteredReverseCurrent;
  }
  return max(state.filteredForwardCurrent, state.filteredReverseCurrent);
}

void applyMotorOutputs(MotorDirection direction, int percent) {
  percent = constrain(percent, 0, 100);
  int pwm = map(percent, 0, 100, 0, 255);

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
  json += ",\"currentForward\":" + String(state.filteredForwardCurrent, 2);
  json += ",\"currentReverse\":" + String(state.filteredReverseCurrent, 2);
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
      state.motorEnabled = false;
      state.timerRunning = false;
      state.remainingSeconds = 0;
      stopMotor();
      state.appliedDirection = MotorDirection::Stopped;
    } else if (value == "calibrate") {
      bool wasEnabled = state.motorEnabled;
      if (wasEnabled) {
        state.motorEnabled = false;
        stopMotor();
        state.appliedDirection = MotorDirection::Stopped;
        delay(200);
      }
      calibrateCurrentOffsets();
      if (wasEnabled) {
        startMotorSession(state.requestedDirection);
      }
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

  applyMotorOutputs(direction, state.targetSpeedPercent);
  state.appliedDirection = direction;
}
