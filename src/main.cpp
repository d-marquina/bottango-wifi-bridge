/**
 * Bottango WiFi Bridge
 * ────────────────────
 * Classic ESP32 (WROOM-32 DevKit) acting as a bridge between a web HMI
 * (WiFi) and a Raspberry Pi Pico running the Bottango MicroPython driver
 * (the driver itself is NOT modified; the Pico only enables
 * ENABLE_UART_BRIDGE in its main.py, which mirrors the console over UART).
 *
 *   Browser ──WiFi/WebSocket──► ESP32 ──UART2──► RPi Pico ──► servos
 *
 * Features:
 *   - Own Access Point (no captive portal: browse to http://192.168.4.1).
 *   - Upload the AnimationCommands.json exported by Bottango FROM the web
 *     page; it is persisted in LittleFS (/project.json) — upload once.
 *   - Dynamic animation list (name + computed duration).
 *   - Rename / delete animations from the web page (changes are persisted).
 *   - "Mode A" playback by index: handshake + effector setup (once per
 *     session) + tSYN,0 + curves, honouring the driver's flow control
 *     (wait for OK after each command, exactly like Bottango does).
 *   - Stop via xC (STOP would trigger machine.reset() on the Pico).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <vector>

// ── Configuration ─────────────────────────────────────────────────────────────

static const char *AP_SSID = "Animatronico";
static const char *AP_PASS = "bottango123";   // min. 8 characters (WPA2)

// UART2 to the Pico (UART0 is the DevKit's USB console).
static const int UART_TX_PIN = 17;  // → Pico GP1 (RX)
static const int UART_RX_PIN = 16;  // ← Pico GP0 (TX)
static const uint32_t UART_BAUD = 115200;

static const char *PROJECT_FILE = "/project.json";
static const uint32_t REPLY_TIMEOUT_MS = 3000;

HardwareSerial picoSerial(2);

// ── Loaded project (parsed from /project.json) ───────────────────────────────

struct AnimEntry {
  String name;
  String commands;      // playback lines (sSY / sC) separated by '\n'
  String loopCommands;  // loop lines (kept so rewriting the file is lossless)
  uint32_t durationMs;
};

static String controllerName;
static String setupCommands;              // rSVI2C/rSVPin/... lines, '\n'-separated
static std::vector<AnimEntry> animations;

// ── Player (non-blocking state machine) ───────────────────────────────────────

struct QueuedCmd {
  String cmd;
  String expect;   // expected reply prefix ("OK" or "btngoHSK")
};

enum PlayerState { IDLE, SENDING, PLAYING };

static PlayerState playerState = IDLE;
static std::vector<QueuedCmd> cmdQueue;
static size_t qIndex = 0;
static bool waitingReply = false;
static String expectedPrefix;
static uint32_t replyDeadline = 0;
static uint32_t playEndsAt = 0;
static uint32_t pendingDurationMs = 0;
static bool setupDone = false;    // handshake + setup already sent this session

static String uartLineBuffer;

// ── Network / servers ─────────────────────────────────────────────────────────

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ── Helpers ───────────────────────────────────────────────────────────────────

static String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') continue;
    else out += c;
  }
  return out;
}

static void wsLog(const String &msg) {
  Serial.println(msg);
  ws.textAll("{\"type\":\"log\",\"msg\":\"" + jsonEscape(msg) + "\"}");
}

static void wsState(const char *state) {
  ws.textAll(String("{\"type\":\"state\",\"state\":\"") + state + "\"}");
}

static String buildAnimListJson() {
  String out = "{\"type\":\"anims\",\"controller\":\"" +
               jsonEscape(controllerName) + "\",\"list\":[";
  for (size_t i = 0; i < animations.size(); i++) {
    if (i) out += ',';
    out += "{\"name\":\"" + jsonEscape(animations[i].name) + "\",\"ms\":" +
           String(animations[i].durationMs) + "}";
  }
  out += "]}";
  return out;
}

// ── Duration computation ──────────────────────────────────────────────────────
// Duration = max(startOffset + duration) across every curve of every line.
// Supports both "sC,..." and the batched "sSY,sC,<e1>;<e2>;..." form.

static void scanCurveFields(const String &entry, uint32_t &maxEnd) {
  // entry = "id,start,dur,rest..." → we need fields 1 and 2
  int c1 = entry.indexOf(',');
  if (c1 < 0) return;
  int c2 = entry.indexOf(',', c1 + 1);
  if (c2 < 0) return;
  int c3 = entry.indexOf(',', c2 + 1);
  long start = entry.substring(c1 + 1, c2).toInt();
  long dur   = (c3 < 0 ? entry.substring(c2 + 1) : entry.substring(c2 + 1, c3)).toInt();
  if (start >= 0 && dur > 0 && (uint32_t)(start + dur) > maxEnd)
    maxEnd = start + dur;
}

static uint32_t computeDurationMs(const String &commands) {
  uint32_t maxEnd = 0;
  int lineStart = 0;
  while (lineStart < (int)commands.length()) {
    int lineEnd = commands.indexOf('\n', lineStart);
    if (lineEnd < 0) lineEnd = commands.length();
    String line = commands.substring(lineStart, lineEnd);
    line.trim();
    lineStart = lineEnd + 1;
    if (!line.length()) continue;

    if (line.startsWith("sC,")) {
      scanCurveFields(line.substring(3), maxEnd);
    } else if (line.startsWith("sSY,sC,")) {
      String batch = line.substring(7);
      int p = 0;
      while (p < (int)batch.length()) {
        int semi = batch.indexOf(';', p);
        if (semi < 0) semi = batch.length();
        String entry = batch.substring(p, semi);
        p = semi + 1;
        if (entry.length()) scanCurveFields(entry, maxEnd);
      }
    }
  }
  return maxEnd;
}

// ── Project load / save (LittleFS) ────────────────────────────────────────────

static bool loadProject() {
  controllerName = "";
  setupCommands = "";
  animations.clear();

  File f = LittleFS.open(PROJECT_FILE, "r");
  if (!f) return false;

  JsonDocument doc;   // ArduinoJson 7: elastic capacity
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    wsLog(String("ERROR: invalid JSON: ") + err.c_str());
    return false;
  }

  JsonArray controllers = doc.as<JsonArray>();
  if (controllers.isNull() || controllers.size() == 0) {
    wsLog("ERROR: JSON contains no controllers");
    return false;
  }

  JsonObject first = controllers[0];
  controllerName = (const char *)(first["Controller Name"] | "");
  setupCommands  = (const char *)(first["Setup"]["Controller Setup Commands"] | "");

  for (JsonObject a : first["Animations"].as<JsonArray>()) {
    AnimEntry e;
    e.name         = (const char *)(a["Animation Name"] | "Unnamed");
    e.commands     = (const char *)(a["Animation Commands"] | "");
    e.loopCommands = (const char *)(a["Animation Loop Commands"] | "");
    e.durationMs   = computeDurationMs(e.commands);
    animations.push_back(e);
  }

  Serial.printf("Project: %s (%u animations)\n",
                controllerName.c_str(), animations.size());
  return true;
}

// Rewrites /project.json from the in-memory state (after rename/delete),
// preserving the exact Bottango export schema so it stays re-loadable.
static bool saveProject() {
  JsonDocument doc;
  JsonArray controllers = doc.to<JsonArray>();
  JsonObject c = controllers.add<JsonObject>();
  c["Controller Name"] = controllerName;
  c["Setup"]["Controller Setup Commands"] = setupCommands;
  JsonArray arr = c["Animations"].to<JsonArray>();
  for (const AnimEntry &e : animations) {
    JsonObject a = arr.add<JsonObject>();
    a["Animation Name"] = e.name;
    a["Animation Commands"] = e.commands;
    a["Animation Loop Commands"] = e.loopCommands;
  }

  File f = LittleFS.open(PROJECT_FILE, "w");
  if (!f) {
    wsLog("ERROR: could not write project file");
    return false;
  }
  serializeJson(doc, f);
  f.close();
  return true;
}

// ── Command sending with flow control ─────────────────────────────────────────

static void sendCurrent() {
  const QueuedCmd &qc = cmdQueue[qIndex];
  wsLog(">> " + qc.cmd);
  picoSerial.print(qc.cmd);
  picoSerial.print('\n');                    // the driver requires exactly '\n'
  expectedPrefix = qc.expect;
  waitingReply = true;
  replyDeadline = millis() + REPLY_TIMEOUT_MS;
}

static void queueLines(const String &block, const char *expect) {
  int lineStart = 0;
  while (lineStart < (int)block.length()) {
    int lineEnd = block.indexOf('\n', lineStart);
    if (lineEnd < 0) lineEnd = block.length();
    String line = block.substring(lineStart, lineEnd);
    line.trim();
    lineStart = lineEnd + 1;
    if (line.length()) cmdQueue.push_back({line, expect});
  }
}

static void startPlayback(size_t animIndex) {
  if (playerState != IDLE || animIndex >= animations.size()) return;

  cmdQueue.clear();
  qIndex = 0;

  if (!setupDone) {
    cmdQueue.push_back({"hRQ,0", "btngoHSK"});
    queueLines(setupCommands, "OK");
  }
  cmdQueue.push_back({"tSYN,0", "OK"});
  queueLines(animations[animIndex].commands, "OK");

  pendingDurationMs = animations[animIndex].durationMs;
  playerState = SENDING;
  wsState("playing");
  wsLog("▶ " + animations[animIndex].name +
        " (" + String(pendingDurationMs) + " ms)");
  sendCurrent();
}

static void stopPlayback() {
  playerState = IDLE;
  waitingReply = false;
  cmdQueue.clear();
  wsLog(">> xC");
  picoSerial.print("xC\n");   // clears curves WITHOUT resetting the Pico
  wsState("idle");
  wsLog("Detenido (xC)");
}

static void abortPlayback(const String &reason) {
  wsLog("ERROR: " + reason);
  playerState = IDLE;
  waitingReply = false;
  cmdQueue.clear();
  wsState("idle");
}

static void advanceSequence() {
  qIndex++;
  if (qIndex < cmdQueue.size()) {
    sendCurrent();
    return;
  }
  // Queue drained: mark setup as done; the Pico now plays on its own clock.
  setupDone = true;
  playerState = PLAYING;
  playEndsAt = millis() + pendingDurationMs;
  wsLog("Curvas encoladas; la Pico reproduce sola");
}

static void handlePicoLine(const String &line) {
  wsLog("<< " + line);
  if (!waitingReply) return;
  if (line.startsWith(expectedPrefix)) {
    waitingReply = false;
    advanceSequence();
  }
  // LOG,... and other lines are shown but do not consume the wait.
}

static void pollPicoSerial() {
  while (picoSerial.available()) {
    char c = (char)picoSerial.read();
    if (c == '\n') {
      uartLineBuffer.replace("\r", "");
      if (uartLineBuffer.length()) handlePicoLine(uartLineBuffer);
      uartLineBuffer = "";
    } else {
      uartLineBuffer += c;
      if (uartLineBuffer.length() > 300) uartLineBuffer = "";  // guard rail
    }
  }
}

static void pollPlayer() {
  if (waitingReply && millis() > replyDeadline) {
    abortPlayback("timeout esperando \"" + expectedPrefix + "\"");
    return;
  }
  if (playerState == PLAYING && millis() > playEndsAt) {
    playerState = IDLE;
    wsState("idle");
    wsLog("Animación terminada");
  }
}

// ── Animation management (rename / delete) ────────────────────────────────────

static void renameAnimation(size_t idx, const String &newName) {
  if (idx >= animations.size() || !newName.length()) return;
  animations[idx].name = newName;
  if (saveProject()) {
    ws.textAll(buildAnimListJson());
    wsLog("Renombrada → " + newName);
  }
}

static void deleteAnimation(size_t idx) {
  if (idx >= animations.size()) return;
  if (playerState != IDLE) {
    wsLog("ERROR: detén la reproducción antes de borrar");
    return;
  }
  String gone = animations[idx].name;
  animations.erase(animations.begin() + idx);
  if (saveProject()) {
    ws.textAll(buildAnimListJson());
    wsLog("Borrada: " + gone);
  }
}

// ── WebSocket ─────────────────────────────────────────────────────────────────
// Client → server messages:
//   "play:<idx>"            start playback of animation <idx>
//   "stop"                  stop (xC)
//   "ren:<idx>:<new name>"  rename animation <idx>
//   "del:<idx>"             delete animation <idx>

static void onWsEvent(AsyncWebSocket *serverPtr, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    client->text(buildAnimListJson());
    client->text(String("{\"type\":\"state\",\"state\":\"") +
                 (playerState == IDLE ? "idle" : "playing") + "\"}");
  } else if (type == WS_EVT_DATA) {
    String msg;
    for (size_t i = 0; i < len; i++) msg += (char)data[i];

    if (msg.startsWith("play:")) {
      startPlayback(msg.substring(5).toInt());
    } else if (msg == "stop") {
      stopPlayback();
    } else if (msg.startsWith("ren:")) {
      int sep = msg.indexOf(':', 4);        // name may itself contain ':'
      if (sep > 4) {
        renameAnimation(msg.substring(4, sep).toInt(), msg.substring(sep + 1));
      }
    } else if (msg.startsWith("del:")) {
      deleteAnimation(msg.substring(4).toInt());
    }
  }
}

// ── Project upload (POST /upload) ─────────────────────────────────────────────

static void handleUploadChunk(AsyncWebServerRequest *req, String filename,
                              size_t index, uint8_t *data, size_t len, bool final) {
  static File uploadFile;
  if (index == 0) {
    uploadFile = LittleFS.open(PROJECT_FILE, "w");
  }
  if (uploadFile) uploadFile.write(data, len);
  if (final && uploadFile) {
    uploadFile.close();
    if (loadProject()) {
      setupDone = false;          // effectors may have changed → re-register
      ws.textAll(buildAnimListJson());
      wsLog("Proyecto actualizado: " + controllerName + " (" +
            String(animations.size()) + " animaciones)");
    }
  }
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  picoSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS: mount failed");
  }

  loadProject();   // if a persisted project.json exists, it is ready to go

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP ready. IP: ");
  Serial.println(WiFi.softAPIP());   // 192.168.4.1

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/upload", HTTP_POST,
            [](AsyncWebServerRequest *req) { req->send(200, "text/plain", "OK"); },
            handleUploadChunk);

  server.onNotFound([](AsyncWebServerRequest *req) {
    req->redirect("http://192.168.4.1/");
  });

  server.begin();
  Serial.println("Web server + WebSocket up");
}

void loop() {
  pollPicoSerial();
  pollPlayer();
  ws.cleanupClients();
}
