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
 *   - STREAMING playback (no curve-count limit): curves are sent just
 *     ahead of their start time instead of all up-front, respecting the
 *     driver's 8-slot circular curve buffer per effector. Flow control is
 *     honoured throughout (wait for OK after each command).
 *   - Stop via xC (STOP would trigger machine.reset() on the Pico).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include <algorithm>

// ── Configuration ─────────────────────────────────────────────────────────────

static const char *AP_SSID = "Animatronico";
static const char *AP_PASS = "bottango123";   // min. 8 characters (WPA2)

// UART2 to the Pico (UART0 is the DevKit's USB console).
static const int UART_TX_PIN = 17;  // → Pico GP1 (RX)
static const int UART_RX_PIN = 16;  // ← Pico GP0 (TX)
static const uint32_t UART_BAUD = 115200;

static const char *PROJECT_FILE = "/project.json";
static const uint32_t REPLY_TIMEOUT_MS = 3000;

// Streaming: send each curve this many ms before its start time. Large
// enough to survive WiFi/UART hiccups, small enough to keep few curves
// in flight per effector.
static const uint32_t STREAM_LOOKAHEAD_MS = 1000;

// The driver's per-effector circular curve buffer size (MAX_NUM_CURVES in
// abstract_effector.py). Sending curve N+1 overwrites the oldest of the
// last N — only safe once that one has already finished playing.
static const size_t DRIVER_CURVE_SLOTS = 8;

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

// ── Player state ──────────────────────────────────────────────────────────────

struct QueuedCmd {
  String cmd;
  String expect;   // expected reply prefix ("OK" or "btngoHSK")
};

// One schedulable line of the animation (a full sC, or a whole sSY batch).
struct StreamItem {
  uint32_t startMs;   // when the earliest curve in the line starts
  String line;        // raw protocol line to send
};

enum PlayerState { IDLE, SENDING, STREAMING };

static PlayerState playerState = IDLE;

// SENDING phase: handshake/setup/tSYN queue (strictly sequential).
static std::vector<QueuedCmd> cmdQueue;
static size_t qIndex = 0;

// STREAMING phase: time-scheduled curve lines.
static std::vector<StreamItem> streamItems;
static size_t streamIdx = 0;
static uint32_t animStartMs = 0;        // millis() when tSYN,0 was acknowledged
static uint32_t animDurationMs = 0;
// Per-effector ledger of END times of every curve already sent, in send
// order. Used to respect the driver's 8-slot buffer (see canSendToEffector).
static std::map<String, std::vector<uint32_t>> effectorLedger;

static bool waitingReply = false;
static String expectedPrefix;
static uint32_t replyDeadline = 0;
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

// ── Curve-line parsing ────────────────────────────────────────────────────────
// sC entry fields (after the "sC," prefix): id,start,dur,startY,cp1x,cp1y,...
// sSY batches several entries separated by ';' after the "sSY,sC," prefix.

// Extracts id / start / duration from one curve entry. Returns false if the
// entry is malformed (e.g. the trailing hash fragment of an sSY batch).
static bool parseCurveEntry(const String &entry, String &id,
                            uint32_t &start, uint32_t &dur) {
  int c1 = entry.indexOf(',');
  if (c1 <= 0) return false;
  int c2 = entry.indexOf(',', c1 + 1);
  if (c2 < 0) return false;
  int c3 = entry.indexOf(',', c2 + 1);
  id = entry.substring(0, c1);
  long s = entry.substring(c1 + 1, c2).toInt();
  long d = (c3 < 0 ? entry.substring(c2 + 1) : entry.substring(c2 + 1, c3)).toInt();
  if (s < 0) s = 0;
  if (d < 0) return false;
  start = (uint32_t)s;
  dur = (uint32_t)d;
  return true;
}

// Invokes fn(id, start, dur) for every curve entry contained in a line.
template <typename F>
static void forEachCurveInLine(const String &line, F fn) {
  if (line.startsWith("sC,")) {
    String id; uint32_t s, d;
    if (parseCurveEntry(line.substring(3), id, s, d)) fn(id, s, d);
  } else if (line.startsWith("sSY,sC,")) {
    String batch = line.substring(7);
    int p = 0;
    while (p < (int)batch.length()) {
      int semi = batch.indexOf(';', p);
      if (semi < 0) semi = batch.length();
      String entry = batch.substring(p, semi);
      p = semi + 1;
      String id; uint32_t s, d;
      if (entry.length() && parseCurveEntry(entry, id, s, d)) fn(id, s, d);
    }
  }
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
    forEachCurveInLine(line, [&](const String &, uint32_t s, uint32_t d) {
      if (s + d > maxEnd) maxEnd = s + d;
    });
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

// ── Low-level command sending (flow control) ──────────────────────────────────

static void sendLine(const String &cmd, const String &expect) {
  wsLog(">> " + cmd);
  picoSerial.print(cmd);
  picoSerial.print('\n');                    // the driver requires exactly '\n'
  expectedPrefix = expect;
  waitingReply = true;
  replyDeadline = millis() + REPLY_TIMEOUT_MS;
}

// ── Streaming playback ────────────────────────────────────────────────────────
//
// Why streaming: the driver buffers at most DRIVER_CURVE_SLOTS (8) curves per
// effector in a circular buffer — the 9th add overwrites the oldest one.
// Dumping a long animation up-front would therefore destroy curves that have
// not played yet. Instead we send each line shortly (STREAM_LOOKAHEAD_MS)
// before its start time, and additionally hold a line back until the curve
// it would overwrite in the driver's buffer has already finished.

// True when every effector in the line can accept one more curve without
// overwriting a still-pending one in the driver's circular buffer.
static bool canSendItem(const StreamItem &item, uint32_t animTime) {
  bool ok = true;
  forEachCurveInLine(item.line, [&](const String &id, uint32_t, uint32_t) {
    if (!ok) return;
    const auto it = effectorLedger.find(id);
    if (it == effectorLedger.end()) return;              // nothing sent yet
    const std::vector<uint32_t> &ends = it->second;
    if (ends.size() < DRIVER_CURVE_SLOTS) return;        // buffer not full
    // Sending one more overwrites the (size - SLOTS + 1)-th oldest entry:
    // it must have finished already.
    uint32_t overwrittenEnd = ends[ends.size() - DRIVER_CURVE_SLOTS];
    if (overwrittenEnd > animTime) ok = false;
  });
  return ok;
}

static void recordItemSent(const StreamItem &item) {
  forEachCurveInLine(item.line, [&](const String &id, uint32_t s, uint32_t d) {
    effectorLedger[id].push_back(s + d);
  });
}

static void buildStream(const AnimEntry &anim) {
  streamItems.clear();
  streamIdx = 0;
  effectorLedger.clear();

  int lineStart = 0;
  while (lineStart < (int)anim.commands.length()) {
    int lineEnd = anim.commands.indexOf('\n', lineStart);
    if (lineEnd < 0) lineEnd = anim.commands.length();
    String line = anim.commands.substring(lineStart, lineEnd);
    line.trim();
    lineStart = lineEnd + 1;
    if (!line.length()) continue;

    StreamItem item;
    item.line = line;
    item.startMs = UINT32_MAX;
    forEachCurveInLine(line, [&](const String &, uint32_t s, uint32_t) {
      if (s < item.startMs) item.startMs = s;
    });
    if (item.startMs == UINT32_MAX) item.startMs = 0;    // non-curve line
    streamItems.push_back(item);
  }

  // The export is chronological in practice; sort defensively anyway.
  std::stable_sort(streamItems.begin(), streamItems.end(),
                   [](const StreamItem &a, const StreamItem &b) {
                     return a.startMs < b.startMs;
                   });
}

static void startPlayback(size_t animIndex) {
  if (playerState != IDLE || animIndex >= animations.size()) return;

  cmdQueue.clear();
  qIndex = 0;

  if (!setupDone) {
    cmdQueue.push_back({"hRQ,0", "btngoHSK"});
    int lineStart = 0;
    while (lineStart < (int)setupCommands.length()) {
      int lineEnd = setupCommands.indexOf('\n', lineStart);
      if (lineEnd < 0) lineEnd = setupCommands.length();
      String line = setupCommands.substring(lineStart, lineEnd);
      line.trim();
      lineStart = lineEnd + 1;
      if (line.length()) cmdQueue.push_back({line, "OK"});
    }
  }
  cmdQueue.push_back({"tSYN,0", "OK"});

  buildStream(animations[animIndex]);
  animDurationMs = animations[animIndex].durationMs;

  playerState = SENDING;
  wsState("playing");
  wsLog("▶ " + animations[animIndex].name +
        " (" + String(animDurationMs) + " ms, " +
        String(streamItems.size()) + " lines)");
  sendLine(cmdQueue[0].cmd, cmdQueue[0].expect);
}

static void stopPlayback() {
  playerState = IDLE;
  waitingReply = false;
  cmdQueue.clear();
  streamItems.clear();
  effectorLedger.clear();
  wsLog(">> xC");
  picoSerial.print("xC\n");   // clears curves WITHOUT resetting the Pico
  wsState("idle");
  wsLog("Stopped (xC)");
}

static void abortPlayback(const String &reason) {
  wsLog("ERROR: " + reason);
  playerState = IDLE;
  waitingReply = false;
  cmdQueue.clear();
  streamItems.clear();
  effectorLedger.clear();
  wsState("idle");
}

// Called whenever the expected reply arrived.
static void onReplyReceived() {
  if (playerState == SENDING) {
    qIndex++;
    if (qIndex < cmdQueue.size()) {
      sendLine(cmdQueue[qIndex].cmd, cmdQueue[qIndex].expect);
    } else {
      // tSYN,0 acknowledged: the Pico's animation clock is now zero.
      setupDone = true;
      playerState = STREAMING;
      animStartMs = millis();
    }
  }
  // In STREAMING the scheduler in pollPlayer() decides what to send next.
}

static void handlePicoLine(const String &line) {
  wsLog("<< " + line);
  if (!waitingReply) return;
  if (line.startsWith(expectedPrefix)) {
    waitingReply = false;
    onReplyReceived();
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
    abortPlayback("timeout waiting for \"" + expectedPrefix + "\"");
    return;
  }

  if (playerState != STREAMING || waitingReply) return;

  uint32_t animTime = millis() - animStartMs;

  // Feed the next line when its send window opened AND the driver's buffer
  // for every effector it touches can take it.
  if (streamIdx < streamItems.size()) {
    const StreamItem &item = streamItems[streamIdx];
    bool windowOpen = (animTime + STREAM_LOOKAHEAD_MS >= item.startMs);
    if (windowOpen && canSendItem(item, animTime)) {
      recordItemSent(item);
      sendLine(item.line, "OK");
      streamIdx++;
    }
    return;
  }

  // Everything sent: wait for the animation to actually finish.
  if (animTime > animDurationMs) {
    playerState = IDLE;
    streamItems.clear();
    effectorLedger.clear();
    wsState("idle");
    wsLog("Animation finished");
  }
}

// ── Animation management (rename / delete) ────────────────────────────────────

static void renameAnimation(size_t idx, const String &newName) {
  if (idx >= animations.size() || !newName.length()) return;
  animations[idx].name = newName;
  if (saveProject()) {
    ws.textAll(buildAnimListJson());
    wsLog("Renamed to: " + newName);
  }
}

static void deleteAnimation(size_t idx) {
  if (idx >= animations.size()) return;
  if (playerState != IDLE) {
    wsLog("ERROR: stop playback before deleting");
    return;
  }
  String gone = animations[idx].name;
  animations.erase(animations.begin() + idx);
  if (saveProject()) {
    ws.textAll(buildAnimListJson());
    wsLog("Deleted: " + gone);
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
      wsLog("Project updated: " + controllerName + " (" +
            String(animations.size()) + " animations)");
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
