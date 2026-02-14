#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// =====================================================
//  RELAY HW (ESP32-C3 mini)
// =====================================================
static const uint8_t RELAY_PINS[4] = {2, 4, 5, 6};
static const bool RELAY_ACTIVE_LOW = true;

static inline void relayWrite(uint8_t ch1to4, bool on) {
  uint8_t pin = RELAY_PINS[ch1to4 - 1];
  bool level = on;
  if (RELAY_ACTIVE_LOW) level = !on;
  digitalWrite(pin, level);
}

// ===================== NVS =====================
Preferences prefs;
static const char* NVS_NS        = "evepower";
static const char* KEY_RELAYMASK = "relayMask";

// rules per relay: count + rules bytes
static const char* KEY_RC[4] = {"rc1","rc2","rc3","rc4"};
static const char* KEY_RB[4] = {"rb1","rb2","rb3","rb4"}; // bytes rules[10]

// optional: store learned master MAC
static const char* KEY_MMAC = "masterMac";

// ===================== HELLO (from Master) =====================
typedef struct __attribute__((packed)) {
  uint8_t type;   // 2 = hello
  uint8_t ch;     // wifi channel of master
  uint32_t ms;
} HelloPacket;
static const uint8_t HELLO_TYPE = 2;

// ===================== POWER PACKETS =====================
static const uint8_t PWR_CMD_TYPE       = 10;
static const uint8_t PWR_STATE_TYPE     = 12;
static const uint8_t PWR_TIME_TYPE      = 13; // minute-only
static const uint8_t PWR_RELAYRULE_TYPE = 14; // rules per relay

typedef struct __attribute__((packed)) {
  uint8_t type;     // 10
  uint8_t maskSet;  // bit0..3 => set relay 1..4
  uint8_t maskVal;  // bit0..3 => ON state
  uint8_t applyNow; // ignored (compat)
  uint32_t ms;
} PowerCmdPacket;

typedef struct __attribute__((packed)) {
  uint16_t minuteOfDay; // 0..1439
  uint8_t  on;          // 1=ON 0=OFF
  uint8_t  daysMask;    // bit0..6 lun..dom
} RelayRuleBin;

typedef struct __attribute__((packed)) {
  uint8_t type;     // 14
  uint8_t ch;       // 1..4
  uint8_t count;    // 0..10
  RelayRuleBin rules[10];
  uint32_t ms;
} PowerRelayRulesPacket;

typedef struct __attribute__((packed)) {
  uint8_t type;       // 12
  uint8_t relayMask;  // bit0..3
  uint8_t timeValid;  // 0/1
  uint8_t resetReason;// esp_reset_reason()
  uint32_t ms;
} PowerStatePacket;

typedef struct __attribute__((packed)) {
  uint8_t type;         // 13
  uint16_t minuteOfDay; // 0..1439
  uint8_t weekdayMon0;  // 0=lun .. 6=dom
  uint8_t valid;        // 0/1
  uint32_t ms;
} PowerTimePacket;

// ===================== ESPNOW =====================
// master MAC learned automatically
static uint8_t MASTER_MAC[6] = {0,0,0,0,0,0};

RTC_DATA_ATTR int8_t lockedChannel = -1;
volatile bool gotHello = false;
volatile uint8_t helloCh = 1;

// ===================== STATE =====================
static uint8_t relayMask = 0;

// time tracking (minute precision)
static bool timeValid = false;
static uint16_t curMinOfDay = 0;  // 0..1439
static uint8_t  curWeekday  = 0;  // 0..6 lun..dom
static uint32_t lastTimeSyncMs = 0;

// rules storage in RAM
static RelayRuleBin rules[4][10];
static uint8_t ruleCount[4] = {0,0,0,0};

static inline bool relayMaskGet(uint8_t ch) { return (relayMask >> (ch - 1)) & 0x01; }
static inline void relayMaskSet(uint8_t ch, bool on) {
  if (on) relayMask |= (1 << (ch - 1));
  else    relayMask &= ~(1 << (ch - 1));
}

static void saveRelayMask() { prefs.putUChar(KEY_RELAYMASK, relayMask); }
static uint8_t resetReasonCompact() { return (uint8_t)esp_reset_reason(); }

static inline bool dayEnabled(uint8_t daysMask, uint8_t wdMon0) {
  return ((daysMask >> (wdMon0 % 7)) & 0x01) != 0;
}

// ===================== NVS RULES IO =====================
static void saveRules(uint8_t ch1to4) {
  int i = ch1to4 - 1;
  prefs.putUChar(KEY_RC[i], ruleCount[i]);
  prefs.putBytes(KEY_RB[i], rules[i], sizeof(rules[i]));
}

static void loadRulesAll() {
  for (int i=0; i<4; i++) {
    uint8_t c = prefs.getUChar(KEY_RC[i], 0);
    if (c > 10) c = 10;
    ruleCount[i] = c;

    size_t n = prefs.getBytes(KEY_RB[i], rules[i], sizeof(rules[i]));
    if (n != sizeof(rules[i])) {
      memset(rules[i], 0, sizeof(rules[i]));
      ruleCount[i] = 0;
    }
  }
}

static bool masterMacValid() {
  for (int i=0;i<6;i++) if (MASTER_MAC[i] != 0) return true;
  return false;
}

static void storeMasterMac(const uint8_t* mac) {
  memcpy(MASTER_MAC, mac, 6);
  prefs.putBytes(KEY_MMAC, MASTER_MAC, 6);
}

static void loadMasterMac() {
  uint8_t tmp[6] = {0,0,0,0,0,0};
  size_t n = prefs.getBytes(KEY_MMAC, tmp, 6);
  if (n == 6) memcpy(MASTER_MAC, tmp, 6);
}

// ensure peer exists for unicast back to master
static void ensureMasterPeer(uint8_t ch) {
  if (!masterMacValid()) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MASTER_MAC, 6);
  peer.channel = ch;
  peer.encrypt = false;

  esp_now_del_peer(MASTER_MAC);
  esp_now_add_peer(&peer);
}

// ===================== POWER STATE TX =====================
static void sendStateToMaster() {
  if (!masterMacValid()) return;

  PowerStatePacket st;
  st.type = PWR_STATE_TYPE;
  st.relayMask = relayMask;
  st.timeValid = timeValid ? 1 : 0;
  st.resetReason = resetReasonCompact();
  st.ms = millis();

  esp_now_send(MASTER_MAC, (uint8_t*)&st, sizeof(st));
}

// ===================== APPLY RULES (exact minute + normalize) =====================

// apply rules that match EXACTLY current minute (last wins due to iteration order)
static bool applyRulesExactNow() {
  if (!timeValid) return false;

  bool changed = false;

  for (uint8_t ch=1; ch<=4; ch++) {
    int idx = ch - 1;

    for (uint8_t k=0; k<ruleCount[idx]; k++) {
      const RelayRuleBin& r = rules[idx][k];
      if (r.minuteOfDay > 1439) continue;
      if (!dayEnabled(r.daysMask, curWeekday)) continue;

      if (r.minuteOfDay == curMinOfDay) {
        bool desired = (r.on == 1);
        if (relayMaskGet(ch) != desired) {
          relayWrite(ch, desired);
          relayMaskSet(ch, desired);
          changed = true;
        }
      }
    }
  }

  if (changed) saveRelayMask();
  return changed;
}

// compute desired state NOW based on the last applicable rule (today or yesterday crossing midnight)
static bool computeDesiredNow(uint8_t ch1to4, bool& outDesired) {
  if (!timeValid) return false;

  int idx = ch1to4 - 1;

  // bestScore is "minute position" where today: 0..1439, yesterday: -1440..-1
  int bestScore = -100000;
  bool found = false;
  bool bestDesired = false;

  uint8_t today = curWeekday;
  uint8_t yday  = (curWeekday + 6) % 7;

  for (uint8_t k=0; k<ruleCount[idx]; k++) {
    const RelayRuleBin& r = rules[idx][k];
    if (r.minuteOfDay > 1439) continue;

    // TODAY candidates: r.minuteOfDay <= curMinOfDay
    if (dayEnabled(r.daysMask, today) && r.minuteOfDay <= curMinOfDay) {
      int score = (int)r.minuteOfDay;
      // if same minute, later rule wins: we treat equality as overwrite by allowing >=
      if (!found || score > bestScore || score == bestScore) {
        bestScore = score;
        bestDesired = (r.on == 1);
        found = true;
      }
    }

    // YESTERDAY crossing midnight: r.minuteOfDay > curMinOfDay (it happened "before midnight")
    if (dayEnabled(r.daysMask, yday) && r.minuteOfDay > curMinOfDay) {
      int score = (int)r.minuteOfDay - 1440;
      if (!found || score > bestScore || score == bestScore) {
        bestScore = score;
        bestDesired = (r.on == 1);
        found = true;
      }
    }
  }

  if (!found) return false;
  outDesired = bestDesired;
  return true;
}

// normalize: set each relay to the state it should have NOW (even if we missed the exact minute event)
static bool normalizeAllRelaysNow() {
  if (!timeValid) return false;

  bool changed = false;
  for (uint8_t ch=1; ch<=4; ch++) {
    bool desired;
    if (!computeDesiredNow(ch, desired)) continue;

    if (relayMaskGet(ch) != desired) {
      relayWrite(ch, desired);
      relayMaskSet(ch, desired);
      changed = true;
    }
  }

  if (changed) saveRelayMask();
  return changed;
}

// ===================== ESPNOW CALLBACK =====================
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* srcMac = info->src_addr;

  // learn master mac from the first packet received
  if (!masterMacValid()) {
    storeMasterMac(srcMac);
    ensureMasterPeer((lockedChannel >= 1 && lockedChannel <= 13) ? (uint8_t)lockedChannel : 1);
  }

  // HELLO
  if (len == (int)sizeof(HelloPacket)) {
    HelloPacket h;
    memcpy(&h, data, sizeof(h));
    if (h.type == HELLO_TYPE) {
      gotHello = true;
      helloCh = h.ch;
    }
    return;
  }

  // CMD (manual ON/OFF)
  if (len == (int)sizeof(PowerCmdPacket)) {
    PowerCmdPacket c;
    memcpy(&c, data, sizeof(c));
    if (c.type != PWR_CMD_TYPE) return;

    bool changed = false;
    for (uint8_t ch = 1; ch <= 4; ch++) {
      uint8_t bit = 1 << (ch - 1);
      if (c.maskSet & bit) {
        bool on = (c.maskVal & bit) != 0;
        if (relayMaskGet(ch) != on) {
          relayWrite(ch, on);
          relayMaskSet(ch, on);
          changed = true;
        }
      }
    }

    if (changed) saveRelayMask();
    sendStateToMaster();
    return;
  }

  // RULES per relay
  if (len == (int)sizeof(PowerRelayRulesPacket)) {
    PowerRelayRulesPacket rp;
    memcpy(&rp, data, sizeof(rp));
    if (rp.type != PWR_RELAYRULE_TYPE) return;
    if (rp.ch < 1 || rp.ch > 4) return;

    int idx = rp.ch - 1;
    uint8_t c = rp.count;
    if (c > 10) c = 10;

    memcpy(rules[idx], rp.rules, sizeof(rules[idx]));
    ruleCount[idx] = c;

    saveRules(rp.ch);

    // IMPORTANT: after schedule update, normalize NOW
    bool changed = normalizeAllRelaysNow();
    (void)changed;

    sendStateToMaster();
    return;
  }

  // TIME SYNC
  if (len == (int)sizeof(PowerTimePacket)) {
    PowerTimePacket tp;
    memcpy(&tp, data, sizeof(tp));
    if (tp.type != PWR_TIME_TYPE) return;

    timeValid = (tp.valid == 1);
    if (timeValid) {
      curMinOfDay = tp.minuteOfDay % 1440;
      curWeekday  = tp.weekdayMon0 % 7;
      lastTimeSyncMs = millis();

      // FIX: apply rules immediately on time sync
      bool changed1 = applyRulesExactNow();
      bool changed2 = normalizeAllRelaysNow();
      (void)changed1; (void)changed2;
    }

    sendStateToMaster();
    return;
  }
}

// ===================== ESPNOW INIT =====================
static bool initEspNowOnChannel(uint8_t ch) {
  WiFi.mode(WIFI_STA);
  delay(20);

  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onEspNowRecv);

  // if we already know master mac, ensure peer for unicast
  ensureMasterPeer(ch);

  return true;
}

static bool findChannelFromHello(uint32_t maxMs = 7000) {
  gotHello = false;
  uint32_t start = millis();

  while (millis() - start < maxMs) {
    for (uint8_t ch = 1; ch <= 13; ch++) {
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);

      uint32_t t0 = millis();
      while (millis() - t0 < 260) {
        if (gotHello) { lockedChannel = (int8_t)helloCh; return true; }
        delay(5);
      }
    }
  }
  return false;
}

// ===================== SETUP =====================
static void relaysInitAndRestore() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    relayWrite(i + 1, false);
  }
  delay(150);

  relayMask = prefs.getUChar(KEY_RELAYMASK, 0);
  for (uint8_t ch = 1; ch <= 4; ch++) relayWrite(ch, relayMaskGet(ch));
}

void setup() {
  Serial.begin(115200);
  delay(200);

  prefs.begin(NVS_NS, false);
  loadMasterMac();
  loadRulesAll();
  relaysInitAndRestore();

  // bootstrap ESPNOW
  if (!initEspNowOnChannel(1)) Serial.println("ESP-NOW init FAIL (ch=1)");

  // channel lock via HELLO
  if (lockedChannel < 1 || lockedChannel > 13) {
    Serial.println("AUTO CH: scanning (waiting HELLO)...");
    if (!findChannelFromHello(7000)) {
      Serial.println("AUTO CH: not found -> fallback ch=1");
      lockedChannel = 1;
    } else {
      Serial.print("AUTO CH: locked="); Serial.println(lockedChannel);
    }
  } else {
    Serial.print("AUTO CH: using locked="); Serial.println(lockedChannel);
  }

  esp_now_deinit();
  delay(30);
  if (!initEspNowOnChannel((uint8_t)lockedChannel)) Serial.println("ESP-NOW init FAIL (locked)");

  // we don't have time yet: when time arrives we normalize
  sendStateToMaster();
}

void loop() {
  // advance time locally minute-by-minute
  if (timeValid) {
    uint32_t now = millis();
    uint32_t elapsed = now - lastTimeSyncMs;

    if (elapsed >= 60000UL) {
      uint32_t addMin = elapsed / 60000UL;
      lastTimeSyncMs += addMin * 60000UL;

      uint32_t total = (uint32_t)curMinOfDay + addMin;
      curWeekday  = (curWeekday + (total / 1440UL)) % 7;
      curMinOfDay = total % 1440UL;

      bool changed1 = applyRulesExactNow();
      bool changed2 = normalizeAllRelaysNow();
      if (changed1 || changed2) sendStateToMaster();
    }
  }

  delay(20);
}
