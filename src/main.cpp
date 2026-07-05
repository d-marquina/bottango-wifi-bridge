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
 *   - Multi-file animation library: every AnimationCommands.json exported
 *     by Bottango is stored as its OWN file under /anims/ in LittleFS.
 *     Each file keeps its own effector setup, so different animations may
 *     use different servo configurations. Re-uploading a file with the
 *     same name replaces it.
 *   - Automatic effector re-registration: when the selected animation
 *     belongs to a file whose setup differs from the one currently
 *     registered on the Pico, the bridge re-runs handshake + setup first.
 *   - Dynamic animation list (name + duration + source file).
 *   - Rename / delete animations (persisted into their source file; a
 *     file whose last animation is deleted is removed).
 *   - STREAMING playback (no curve-count limit): curves are sent just
 *     ahead of their start time, respecting the driver's 8-slot circular
 *     curve buffer per effector. Flow control honoured throughout.
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

static const char *ANIMS_DIR = "/anims";
static const char *LEGACY_PROJECT_FILE = "/project.json";   // pre-library layout
static const uint32_t REPLY_TIMEOUT_MS = 3000;

// Streaming: send each curve this many ms before its start time.
static const uint32_t STREAM_LOOKAHEAD_MS = 1000;

// The driver's per-effector circular curve buffer size (MAX_NUM_CURVES in
// abstract_effector.py). Sending curve N+1 overwrites the oldest of the
// last N — only safe once that one has already finished playing.
static const size_t DRIVER_CURVE_SLOTS = 8;

HardwareSerial picoSerial(2);

// ── Animation library (one ProjectFile per uploaded JSON) ────────────────────

struct ProjectFile {
  String path;         // e.g. /anims/00AnimationCommands.json
  String controller;   // "Controller Name" field
  String setup;        // effector registration lines, '\n'-separated
};

struct AnimEntry {
  int fileIdx;          // index into `files`
  String name;
  String commands;      // playback lines (sSY / sC) separated by '\n'
  String loopCommands;  // kept so rewriting the file is lossless
  uint32_t durationMs;
};

static std::vector<ProjectFile> files;
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
// order. Used to respect the driver's 8-slot buffer (see canSendItem).
static std::map<String, std::vector<uint32_t>> effectorLedger;

static bool waitingReply = false;
static String expectedPrefix;
static uint32_t replyDeadline = 0;

// Setup currently registered on the Pico ("" = none). When the next
// animation's file declares a different setup, the bridge re-registers.
static String registeredSetup;
static String pendingSetup;    // becomes registeredSetup once SENDING finishes

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

static String fileBasename(const String &path) {
  int slash = path.lastIndexOf('/');
  return (slash >= 0) ? path.substring(slash + 1) : path;
}

static String buildAnimListJson() {
  String controller = files.empty() ? "" : files[0].controller;
  String out = "{\"type\":\"anims\",\"controller\":\"" +
               jsonEscape(controller) + "\",\"list\":[";
  for (size_t i = 0; i < animations.size(); i++) {
    if (i) out += ',';
    out += "{\"name\":\"" + jsonEscape(animations[i].name) + "\",\"ms\":" +
           String(animations[i].durationMs) + ",\"file\":\"" +
           jsonEscape(fileBasename(files[animations[i].fileIdx].path)) + "\"}";
  }
  out += "]}";
  return out;
}

// ── Curve-line parsing ────────────────────────────────────────────────────────
// sC entry fields (after the "sC," prefix): id,start,dur,startY,cp1x,cp1y,...
// sSY batches several entries separated by ';' after the "sSY,sC," prefix.

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

// ── Library load / save (LittleFS, one file per uploaded JSON) ───────────────

static bool loadOneFile(const String &path) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;

  JsonDocument doc;   // ArduinoJson 7: elastic capacity
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    wsLog("ERROR: invalid JSON in " + fileBasename(path) + ": " + err.c_str());
    return false;
  }

  JsonArray controllers = doc.as<JsonArray>();
  if (controllers.isNull() || controllers.size() == 0) return false;
  JsonObject first = controllers[0];

  ProjectFile pf;
  pf.path       = path;
  pf.controller = (const char *)(first["Controller Name"] | "");
  pf.setup      = (const char *)(first["Setup"]["Controller Setup Commands"] | "");
  files.push_back(pf);
  int fileIdx = files.size() - 1;

  for (JsonObject a : first["Animations"].as<JsonArray>()) {
    AnimEntry e;
    e.fileIdx      = fileIdx;
    e.name         = (const char *)(a["Animation Name"] | "Unnamed");
    e.commands     = (const char *)(a["Animation Commands"] | "");
    e.loopCommands = (const char *)(a["Animation Loop Commands"] | "");
    e.durationMs   = computeDurationMs(e.commands);
    animations.push_back(e);
  }
  return true;
}

static void loadLibrary() {
  files.clear();
  animations.clear();

  if (!LittleFS.exists(ANIMS_DIR)) LittleFS.mkdir(ANIMS_DIR);

  // One-time migration from the previous single-file layout.
  if (LittleFS.exists(LEGACY_PROJECT_FILE)) {
    LittleFS.rename(LEGACY_PROJECT_FILE, String(ANIMS_DIR) + "/library.json");
    Serial.println("Migrated legacy project.json into /anims/library.json");
  }

  File dir = LittleFS.open(ANIMS_DIR);
  if (!dir) return;
  File f = dir.openNextFile();
  std::vector<String> paths;
  while (f) {
    String p = String("/") + f.path();
    p.replace("//", "/");
    if (!f.isDirectory() && p.endsWith(".json")) paths.push_back(p);
    f = dir.openNextFile();
  }
  std::sort(paths.begin(), paths.end());   // stable, predictable list order
  for (const String &p : paths) loadOneFile(p);

  Serial.printf("Library: %u files, %u animations\n",
                files.size(), animations.size());
}

// Rewrites one library file from the in-memory state (after rename/delete),
// preserving the Bottango export schema. Removes the file if it has no
// animations left. Returns true on success.
static bool saveFile(int fileIdx) {
  std::vector<const AnimEntry *> mine;
  for (const AnimEntry &e : animations)
    if (e.fileIdx == fileIdx) mine.push_back(&e);

  const ProjectFile &pf = files[fileIdx];

  if (mine.empty()) {
    LittleFS.remove(pf.path);
    return true;
  }

  JsonDocument doc;
  JsonArray controllers = doc.to<JsonArray>();
  JsonObject c = controllers.add<JsonObject>();
  c["Controller Name"] = pf.controller;
  c["Setup"]["Controller Setup Commands"] = pf.setup;
  JsonArray arr = c["Animations"].to<JsonArray>();
  for (const AnimEntry *e : mine) {
    JsonObject a = arr.add<JsonObject>();
    a["Animation Name"] = e->name;
    a["Animation Commands"] = e->commands;
    a["Animation Loop Commands"] = e->loopCommands;
  }

  File f = LittleFS.open(pf.path, "w");
  if (!f) {
    wsLog("ERROR: could not write " + fileBasename(pf.path));
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
// See README: curves are sent ~STREAM_LOOKAHEAD_MS before their start time,
// and a line is held back while it would overwrite a still-pending curve in
// the driver's 8-slot circular buffer (per effector).

static bool canSendItem(const StreamItem &item, uint32_t animTime) {
  bool ok = true;
  forEachCurveInLine(item.line, [&](const String &id, uint32_t, uint32_t) {
    if (!ok) return;
    const auto it = effectorLedger.find(id);
    if (it == effectorLedger.end()) return;              // nothing sent yet
    const std::vector<uint32_t> &ends = it->second;
    if (ends.size() < DRIVER_CURVE_SLOTS) return;        // buffer not full
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

  std::stable_sort(streamItems.begin(), streamItems.end(),
                   [](const StreamItem &a, const StreamItem &b) {
                     return a.startMs < b.startMs;
                   });
}

static void startPlayback(size_t animIndex) {
  if (playerState != IDLE || animIndex >= animations.size()) return;

  const AnimEntry &anim = animations[animIndex];
  const String &setup = files[anim.fileIdx].setup;

  cmdQueue.clear();
  qIndex = 0;

  // Re-register effectors whenever this animation's file declares a setup
  // different from the one currently active on the Pico. hRQ clears the
  // Pico's effector pool, so the new setup starts from a clean slate.
  if (registeredSetup != setup) {
    cmdQueue.push_back({"hRQ,0", "btngoHSK"});
    int lineStart = 0;
    while (lineStart < (int)setup.length()) {
      int lineEnd = setup.indexOf('\n', lineStart);
      if (lineEnd < 0) lineEnd = setup.length();
      String line = setup.substring(lineStart, lineEnd);
      line.trim();
      lineStart = lineEnd + 1;
      if (line.length()) cmdQueue.push_back({line, "OK"});
    }
    pendingSetup = setup;
  } else {
    pendingSetup = registeredSetup;
  }
  cmdQueue.push_back({"tSYN,0", "OK"});

  buildStream(anim);
  animDurationMs = anim.durationMs;

  playerState = SENDING;
  wsState("playing");
  wsLog("▶ " + anim.name + " (" + String(animDurationMs) + " ms, " +
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
  registeredSetup = "";   // setup state on the Pico is now uncertain
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
      registeredSetup = pendingSetup;
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
  if (saveFile(animations[idx].fileIdx)) {
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
  int fileIdx = animations[idx].fileIdx;
  animations.erase(animations.begin() + idx);
  if (saveFile(fileIdx)) {
    loadLibrary();               // re-index (the file may have been removed)
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
// Each uploaded JSON becomes its own file under /anims/, keyed by its
// original filename (sanitized). Re-uploading the same filename replaces it.

static String sanitizeFilename(const String &raw) {
  String base = fileBasename(raw);
  String out;
  for (size_t i = 0; i < base.length(); i++) {
    char c = base[i];
    if (isalnum((int)c) || c == '.' || c == '_' || c == '-') out += c;
    else out += '_';
  }
  if (!out.length()) out = "upload.json";
  if (!out.endsWith(".json")) out += ".json";
  return String(ANIMS_DIR) + "/" + out;
}

static void handleUploadChunk(AsyncWebServerRequest *req, String filename,
                              size_t index, uint8_t *data, size_t len, bool final) {
  static File uploadFile;
  if (index == 0) {
    uploadFile = LittleFS.open(sanitizeFilename(filename), "w");
  }
  if (uploadFile) uploadFile.write(data, len);
  if (final && uploadFile) {
    uploadFile.close();
    loadLibrary();
    registeredSetup = "";        // force re-registration on next play
    ws.textAll(buildAnimListJson());
    wsLog("Library updated: " + String(files.size()) + " files, " +
          String(animations.size()) + " animations");
  }
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  picoSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS: mount failed");
  }

  loadLibrary();   // load every persisted /anims/*.json

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
