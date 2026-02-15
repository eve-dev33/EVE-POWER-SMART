/*
  EVE-POWER (ESP32-C3) - RELAY SLAVE (DEBUG)
  - Log estesi su UART0 (Serial) -> COM11 stabile
  - Supporto MAC master fisso (opzionale)
  - Handshake canale via HELLO (type=2)
  - Schedulazioni: esecuzione SOLO all'istante esatto (minuteOfDay match).
    (Niente "normalize" che può cambiare subito lo stato quando arriva una regola.)
  - EXTRA DEBUG: stampa sempre type/len dei pacchetti ricevuti

  MODIFICA RICHIESTA:
  - Prima di accettare QUALSIASI messaggio ESP-NOW, POWER deve ricevere HELLO e agganciarsi al canale del master
  - Dopo HELLO -> invia ACK "ok ho ricevuto il canale" (HELLO_ACK)
  - Poi accetta RULES e le salva. Se ok -> ACK ok=1, se KO -> ACK ok=0 + ERROR packet
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ===================== DEBUG =====================
#define DBG_ENABLED 1       //ABILITA LOG
#define DBG_BAUD 115200     //BAUND RATE COM
#define DBG_PORT Serial     //FORZA SERIALE

//attivare i messaggi di debug quando testi   DBG_ENABLED 1
//disattivarli quando il sistema gira normalmente DBG_ENABLED 0 //quando è disabilitato il compilatore rimuove il codice di stampa sul seriale
#if DBG_ENABLED
  #define DBG(...)   do { DBG_PORT.printf(__VA_ARGS__); } while(0)
  #define DBGLN(...) do { DBG_PORT.printf(__VA_ARGS__); DBG_PORT.print("\r\n"); } while(0)
#else
  #define DBG(...)
  #define DBGLN(...)
#endif

// ===================== RELAY =====================
static const uint8_t RELAY_COUNT = 4;               //numero di rele
static const uint8_t RELAY_PINS[RELAY_COUNT] = {2, 4, 5, 6};  //pin rele
static const bool RELAY_ACTIVE_LOW = true;
/* RELAY_ACTIVE_LOW indica lo stato di funzionamennto del rele
se il pin = ON → LOW → il modulo relè si ATTIVA
se il pin = OFF → HIGT(3.5/5V) → il modulo relè si SPEGNE
*/

//funziuone che scrive ON o OFF sul rele
//chqto4 è il numero del rele,
//on è lo stato True = acceso, false = spento
static inline void relayWrite(uint8_t ch1to4, bool on) {
  if (ch1to4 < 1 || ch1to4 > RELAY_COUNT) return; //controllo che il rele sia compreso tra 0 e RELAY_COUNT=4
  uint8_t pin = RELAY_PINS[ch1to4 - 1]; //perche l'arrey parte da 0  quindi se voglio accendere il primo rele avro (numero rele - 1) quindi 1 - 1 = 0 prima posizione dell'array
  bool level = on;
  if (RELAY_ACTIVE_LOW) level = !on; //inverte da ON a OFF e viceversa
  digitalWrite(pin, level);
}
static inline const char* onOff(bool on) { return on ? "ON" : "OFF"; }
//Utility per stampare "ON" o "OFF" nei log.

// ===================== NVS =====================
Preferences prefs;
static const char* NVS_NS        = "evepower";
static const char* KEY_RELAYMASK = "relayMask"; //quali relè sono ON/OFF (bitmask)
static const char* KEY_RC[RELAY_COUNT] = {"rc1","rc2","rc3","rc4"};//quanti schedule ci sono (0..10)
static const char* KEY_RB[RELAY_COUNT] = {"rb1","rb2","rb3","rb4"}; //il blocco binario con le regole (array da 10)
static const char* KEY_MMAC = "masterMac"; //MAC master (solo se NON fisso)


// ===================== Tipi di pacchetto POWER =====================
//Mappa che ESP-NOW usa il primo byte del pacchetto per dire che tipo di messaggio è.
static const uint8_t HELLO_TYPE         = 2;
static const uint8_t PWR_HELLO_ACK_TYPE = 3;  // NUOVO: ACK "canale ricevuto e agganciato"
static const uint8_t PWR_CMD_TYPE       = 10; //CMD : comandi manuali ai relè (bitmask)
static const uint8_t PWR_STATE_TYPE     = 12; //STATE: lo slave invia stato al master
static const uint8_t PWR_TIME_TYPE      = 13; //TIME: il master invia ora (minuto del giorno + weekday)
static const uint8_t PWR_RELAYRULE_TYPE = 14; //RULES: il master invia schedulazioni per un relè
static const uint8_t PWR_SCHED_ACK_TYPE = 15; //ACK: lo slave conferma “schedulazioni salvate”
static const uint8_t PWR_EXECUTED_TYPE  = 16; //EXECUTED: lo slave avvisa “ho eseguito una regole
static const uint8_t PWR_ERROR_TYPE     = 17; // NUOVO: errore (es. NVS save fallita)

// ===================== PACKETS =====================

//struttura del messggio HELLO aggancia” il canale WiFi del master
#pragma pack(push, 1)
typedef struct {
  uint8_t type;   // 2 = hello
  uint8_t ch;     // canale wifi dove orpera il master
  uint32_t ms;    //millis del master (solo debug)
} HelloPacket;
#pragma pack(pop)

// NUOVO: ACK a HELLO ("ok ho ricevuto il canale")
#pragma pack(push, 1)
typedef struct {
  uint8_t type;   // 3 = hello_ack
  uint8_t ch;     // canale agganciato
  uint8_t ok;     // 1 ok
  uint32_t ms;    // debug
} HelloAckPacket;
#pragma pack(pop)

//CMD: comandi manuali rele
#pragma pack(push, 1)
typedef struct {
  uint8_t type;
  uint8_t maskSet;          //: quali relè devo toccare (bit 0..3) maskSet=0b0011 vuol dire “aggiorna relè 1 e 2”  maskVal=0b0001 vuol dire “relè1 ON, relè2 OFF”
  uint8_t maskVal;          //: a che valore metterli (bit 0..3)
  uint8_t applyNow;         //non usato al momento
  uint32_t ms;              // debug
} PowerCmdPacket;

//Struttura schedulazione
typedef struct {
  uint16_t minuteOfDay; // 0..1439  ora * 60 + minuti --- Esempio se impostiamo 07:00 === 7*60 +0 = 420 oppure  23:59 === 23*60 + 59 = 1439
  uint8_t  on;          // 1=ON 0=OFF
  uint8_t  daysMask;    // bit0..6 lun..dom (lun=0, mar=1,......,dom=6)
} RelayRuleBin;

//RULES: regole per un singolo relè (ch)
typedef struct {
  uint8_t type;
  uint8_t ch;     // 1..4 //numero del rele
  uint8_t count;  // 0..10    //numero di schedulazioni (regole RULES)
  RelayRuleBin rules[10]; //è sempre presente ma usiamo solo le prime count
  uint32_t ms;
} PowerRelayRulesPacket;

//STATE: lo slave invia stato al master
#pragma pack(push, 1)
typedef struct {
  uint8_t type;
  uint8_t relayMask;
  uint8_t timeValid;
  uint8_t resetReason;
  uint32_t ms;
} PowerStatePacket; //manda mask relè, se ora è valida, reset reason

//TIME: il master invia ora (minuto del giorno + weekday)
#pragma pack(push, 1)
typedef struct {
  uint8_t type;
  uint16_t minuteOfDay;
  uint8_t weekdayMon0;
  uint8_t valid;
  uint32_t ms;
} PowerTimePacket; // master → slave: minuto del giorno + weekday

//ACK: lo slave conferma “schedulazioni salvate”
typedef struct {
  uint8_t type;
  uint8_t ch;
  uint8_t ok;
  uint8_t count;
  uint32_t ms;
} PowerScheduleAckPacket; //slave → master: “regole salvate ok”

//EXECUTED: lo slave avvisa “ho eseguito una regola”
typedef struct {
  uint8_t type;
  uint8_t ch;
  uint8_t state;
  uint16_t minuteOfDay;
  uint8_t weekdayMon0;
  uint32_t ms;
} PowerExecutedPacket;    //slave → master: “ho eseguito regola X”

// NUOVO: messaggio di errore (compact)
typedef struct {
  uint8_t type;     // 17
  uint8_t code;     // 1=NVS_SAVE_FAIL, 2=INVALID_CH, ecc...
  uint8_t ch;       // rele coinvolto (1..4) o 0
  uint8_t extra;    // info extra (opzionale)
  uint32_t ms;
} PowerErrorPacket;
#pragma pack(pop)

// ===================== MASTER MAC =====================
// Se vuoi fissare il MAC del master (consigliato per debug), metti 1 e compila con il MAC corretto.
#define USE_FIXED_MASTER_MAC 1 //se 1 accetta solo dal mac master inserito se 0 impara il MAC e lo impara con masterMacValid
#if USE_FIXED_MASTER_MAC
static uint8_t MASTER_MAC[6] = { 0x0C,0x4E,0xA0,0x30,0x37,0x20 }; // MAC MASTER
#else
static uint8_t MASTER_MAC[6] = {0,0,0,0,0,0};
#endif

//impara il mac del master, se lo trova torna vero
static inline bool masterMacValid() {
  for (int i = 0; i < 6; i++) if (MASTER_MAC[i] != 0) return true;
  return false;
}
//salva il mac del master
//1 Copia il MAC ricevuto nella variabile MASTER_MAC
//2. e lo salva nella memoria flash (NVS)
static void storeMasterMac(const uint8_t* mac) {
#if !USE_FIXED_MASTER_MAC                         //se non stai usando un MAC fisso per il master
  memcpy(MASTER_MAC, mac, 6);
  prefs.putBytes(KEY_MMAC, MASTER_MAC, 6);
#endif
}

//nel caso di reboot va a recuperare il mac del master dalla memoria
//legge dalla flash il MAC del master salvato in precedenza
//e lo mette nella variabile MASTER_MAC.
static void loadMasterMac() {
#if !USE_FIXED_MASTER_MAC
  uint8_t tmp[6] = {0,0,0,0,0,0};
  size_t n = prefs.getBytes(KEY_MMAC, tmp, 6);
  if (n == 6) memcpy(MASTER_MAC, tmp, 6);
#endif
}
//prende un MAC in formato byte e lo trasforma in testo tipo "AA:BB:CC:DD:EE:FF"
static void macToStr(const uint8_t* mac, char* out, size_t n) {
  snprintf(out, n, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ===================== STATE =====================
RTC_DATA_ATTR int8_t lockedChannel = -1; //lockedChannel è il canale del master, che rimane sbloccato con -1 perche ancora lo deve imparare
//il tipo RTC_DATA_ATTR dice all'esp32 questa variabile deve sopravvivere anche dopo il deep sleep

volatile bool gotHello = false; //“ho ricevuto un pacchetto HELLO?” iniziamente e a false
volatile uint8_t helloCh = 1; //“che canale mi ha detto il master nel HELLO?” inizialmente lo imposto a 1 per dargli un valore // verra scritto dentro onEspNowRecv

static uint8_t relayMask = 0; 
/*maschera di bit che rappresenta lo stato dei 4 relè (ON/OFF) in un solo numero.

bit0 = relè1
bit1 = relè2
bit2 = relè3
bit3 = relè4

Quindi:
relayMask = 0b0000 → tutti OFF
relayMask = 0b0001 → relè1 ON
relayMask = 0b0101 → relè1 e relè3 ON
relayMask = 0b1111 → tutti ON
*/

//Tempo ricevuto dal master
static bool timeValid = false; //“l’orario è valido?”
static uint16_t curMinOfDay = 0; //minuto del giorno (0..1439)
static uint8_t  curWeekday  = 0;//giorno della settimana (0..6)
static uint32_t lastTimeSyncMs = 0; //quando è arrivata l’ultima sincronizzazione, in millis()

static RelayRuleBin rules[RELAY_COUNT][10]; //rules è una matrice: prima dimensione = relè (0..3) --- seconda dimensione = fino a 10 regole per relè
/*Quindi:
rules[0][k] = regola k del relè 1
rules[3][k] = regola k del relè 4
*/


static uint8_t ruleCount[RELAY_COUNT] = {0,0,0,0};
/*dice quante regole sono davvero valide per quel relè:
se ruleCount[0]=3 → per relè1 usi solo rules[0][0..2]
*/

// finché non ricevo HELLO, ignoro tutto
static volatile bool channelReady = false;

/*funzione che per leggere e scrive il bit del rele */
static inline bool relayMaskGet(uint8_t ch) { return (relayMask >> (ch - 1)) & 0x01; }
/*prende ch da 1 a 4
Esempio: relayMask = 0b0101
relayMaskGet(1) → 1 (ON)
relayMaskGet(2) → 0 (OFF)
relayMaskGet(3) → 1 (ON)
relayMaskGet(4) → 0 (OFF)*/

static inline void relayMaskSet(uint8_t ch, bool on) {
  if (on) relayMask |= (1 << (ch - 1));
  else    relayMask &= ~(1 << (ch - 1));
}
/*Esempio: relayMask = 0b0101
relayMaskGet(1) → 1 (ON)
relayMaskGet(2) → 0 (OFF)
relayMaskGet(3) → 1 (ON)
relayMaskGet(4) → 0 (OFF)
*/

/*“scrivi su disco quali relè sono ON/OFF”*/
static void saveRelayMask() { prefs.putUChar(KEY_RELAYMASK, relayMask); }

//“ dico al master perché mi sono riavviato?”:
/*così il MASTER può sapere:
se lo slave si è riavviato
se è crashato
se ha perso corrente
*/
static uint8_t resetReasonCompact() { return (uint8_t)esp_reset_reason(); }

/*“oggi questa regola è attiva oggi?”*/
static inline bool dayEnabled(uint8_t daysMask, uint8_t wdMon0) {
  return ((daysMask >> (wdMon0 % 7)) & 0x01) != 0;
}

// ===================== NVS rules =====================
static bool saveRules(uint8_t ch1to4) {
  int i = ch1to4 - 1;

  // Salvataggio count + blocco regole
  size_t w1 = prefs.putUChar(KEY_RC[i], ruleCount[i]);
  size_t w2 = prefs.putBytes(KEY_RB[i], rules[i], sizeof(rules[i]));

  bool ok = (w1 == 1) && (w2 == sizeof(rules[i]));
  DBGLN("[NVS] saveRules ch=%u count=%u w1=%u w2=%u ok=%u",
        ch1to4, ruleCount[i], (unsigned)w1, (unsigned)w2, (unsigned)ok);

  return ok;
}

static void loadRulesAll() {
  for (int i = 0; i < RELAY_COUNT; i++) {
    uint8_t c = prefs.getUChar(KEY_RC[i], 0);
    if (c > 10) c = 10;
    ruleCount[i] = c;

    size_t n = prefs.getBytes(KEY_RB[i], rules[i], sizeof(rules[i]));
    if (n != sizeof(rules[i])) {
      memset(rules[i], 0, sizeof(rules[i]));
      ruleCount[i] = 0;
    }
    DBGLN("[NVS] loadRules ch=%d count=%u bytes=%u", i+1, ruleCount[i], (unsigned)n);
  }
}

// ===================== ESPNOW peers =====================
static void ensureMasterPeer(uint8_t /*ch*/) {
  if (!masterMacValid()) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MASTER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  esp_now_del_peer(MASTER_MAC);
  esp_err_t e = esp_now_add_peer(&peer);

  char macs[24]; macToStr(MASTER_MAC, macs, sizeof(macs));
  DBGLN("[ESPNOW] ensureMasterPeer mac=%s add_peer=%d", macs, (int)e);
}

static void sendHelloAckToMaster(uint8_t ch, bool ok = true) {
  if (!masterMacValid()) { DBGLN("[ESPNOW] sendHelloAck skipped (no master mac)"); return; }

  HelloAckPacket a;
  a.type = PWR_HELLO_ACK_TYPE;
  a.ch   = ch;
  a.ok   = ok ? 1 : 0;
  a.ms   = millis();

  esp_err_t e = esp_now_send(MASTER_MAC, (uint8_t*)&a, sizeof(a));
  DBGLN("[ESPNOW] TX HELLO_ACK ch=%u ok=%u -> %d", (unsigned)a.ch, (unsigned)a.ok, (int)e);
}

static void sendErrorToMaster(uint8_t code, uint8_t ch = 0, uint8_t extra = 0) {
  if (!masterMacValid()) { DBGLN("[ESPNOW] sendError skipped (no master mac)"); return; }

  PowerErrorPacket er;
  er.type  = PWR_ERROR_TYPE;
  er.code  = code;
  er.ch    = ch;
  er.extra = extra;
  er.ms    = millis();

  esp_err_t e = esp_now_send(MASTER_MAC, (uint8_t*)&er, sizeof(er));
  DBGLN("[ESPNOW] TX ERROR code=%u ch=%u extra=%u -> %d",
        (unsigned)code, (unsigned)ch, (unsigned)extra, (int)e);
}

static void sendStateToMaster() {
  if (!masterMacValid()) { DBGLN("[ESPNOW] sendState skipped (no master mac)"); return; }

  PowerStatePacket st;
  st.type = PWR_STATE_TYPE;
  st.relayMask = relayMask;
  st.timeValid = timeValid ? 1 : 0;
  st.resetReason = resetReasonCompact();
  st.ms = millis();

  esp_err_t e = esp_now_send(MASTER_MAC, (uint8_t*)&st, sizeof(st));
  DBGLN("[ESPNOW] TX STATE mask=%u timeValid=%u -> %d",
        (unsigned)st.relayMask, (unsigned)st.timeValid, (int)e);
}

static void sendScheduleAckToMaster(uint8_t ch1to4, uint8_t count, bool ok = true) {
  if (!masterMacValid()) { DBGLN("[ESPNOW] sendAck skipped (no master mac)"); return; }
  if (ch1to4 < 1 || ch1to4 > RELAY_COUNT) return;

  PowerScheduleAckPacket ack;
  ack.type = PWR_SCHED_ACK_TYPE;
  ack.ch = ch1to4;
  ack.ok = ok ? 1 : 0;
  ack.count = (count > 10) ? 10 : count;
  ack.ms = millis();

  esp_err_t e = esp_now_send(MASTER_MAC, (uint8_t*)&ack, sizeof(ack));
  DBGLN("[ESPNOW] TX ACK ch=%u ok=%u count=%u -> %d",
        (unsigned)ack.ch, (unsigned)ack.ok, (unsigned)ack.count, (int)e);
}

static void sendExecutedToMaster(uint8_t ch1to4, bool on) {
  if (!masterMacValid()) return;
  if (ch1to4 < 1 || ch1to4 > RELAY_COUNT) return;

  PowerExecutedPacket ex;
  ex.type = PWR_EXECUTED_TYPE;
  ex.ch = ch1to4;
  ex.state = on ? 1 : 0;
  ex.minuteOfDay = curMinOfDay;
  ex.weekdayMon0 = curWeekday;
  ex.ms = millis();

  esp_err_t e = esp_now_send(MASTER_MAC, (uint8_t*)&ex, sizeof(ex));
  DBGLN("[ESPNOW] TX EXEC ch=%u %s min=%u wd=%u -> %d",
        (unsigned)ex.ch, onOff(on), (unsigned)ex.minuteOfDay, (unsigned)ex.weekdayMon0, (int)e);
}

// ===================== SCHEDULE ENGINE =====================
// Esegue SOLO se minuteOfDay == curMinOfDay (quindi NON cambia stato "subito" quando arriva una regola)
static bool applyRulesExactNow(bool notifyExecuted) {
  if (!timeValid) return false;

  bool changed = false;

  for (uint8_t ch = 1; ch <= RELAY_COUNT; ch++) {
    int idx = ch - 1;

    for (uint8_t k = 0; k < ruleCount[idx]; k++) {
      const RelayRuleBin& r = rules[idx][k];
      if (r.minuteOfDay > 1439) continue;
      if (!dayEnabled(r.daysMask, curWeekday)) continue;

      if (r.minuteOfDay == curMinOfDay) {
        bool desired = (r.on == 1);
        if (relayMaskGet(ch) != desired) {
          DBGLN("[SCHED] FIRE ch=%u at=%u wd=%u -> %s",
                (unsigned)ch, (unsigned)curMinOfDay, (unsigned)curWeekday, onOff(desired));
          relayWrite(ch, desired);
          relayMaskSet(ch, desired);
          if (notifyExecuted) sendExecutedToMaster(ch, desired);
          changed = true;
        } else {
          DBGLN("[SCHED] FIRE ch=%u at=%u wd=%u -> already %s",
                (unsigned)ch, (unsigned)curMinOfDay, (unsigned)curWeekday, onOff(desired));
        }
      }
    }
  }

  if (changed) saveRelayMask();
  return changed;
}

// ===================== ESPNOW CALLBACK =====================
static uint8_t curChannel = 1;

void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !data || len <= 0) return;

  const uint8_t* srcMac = info->src_addr;

  char macs[24]; macToStr(srcMac, macs, sizeof(macs));
  uint8_t ptype = (uint8_t)data[0];
  DBGLN("[ESPNOW] RX len=%d type=%u from %s", len, (unsigned)ptype, macs);

#if !USE_FIXED_MASTER_MAC
  if (!masterMacValid()) {
    storeMasterMac(srcMac);
    DBGLN("[ESPNOW] Learned MASTER_MAC=%s", macs);
  }
#endif

  // ===================== BLOCCO: prima di HELLO ignoro tutto =====================
  // esp32 power prima di ricevere qualsiasi messaggio EP32-NOW deve prima agganciarsi sul canalee del master.
  if (!channelReady && ptype != HELLO_TYPE) {
    DBGLN("[POWER] Ignoro il memssaggio di tipo=%u (io spetto come prima cosa un messaggio di HELLO)", (unsigned)ptype);
    return;
  }

  // HELLO
  if (len == (int)sizeof(HelloPacket)) {
    HelloPacket h;
    memcpy(&h, data, sizeof(h));
    if (h.type != HELLO_TYPE) return;

    gotHello = true;
    helloCh = h.ch;

    DBGLN("[HELLO] ch=%u ms=%lu", (unsigned)h.ch, (unsigned long)h.ms);

    // Aggancia canale WiFi locale
    if (h.ch >= 1 && h.ch <= 13 && h.ch != curChannel) {
      curChannel = h.ch;
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(curChannel, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);
      DBGLN("[WIFI] set_channel=%u", (unsigned)curChannel);
    }

    // Ora il canale è pronto: da qui in poi accetto gli altri pacchetti
    channelReady = true;

    ensureMasterPeer(curChannel);

    // ACK "ok ho ricevuto il canale"
    sendHelloAckToMaster(curChannel, true);

    // Stato (utile al master dopo handshake)
    sendStateToMaster();
    return;
  }

  // CMD
  if (len == (int)sizeof(PowerCmdPacket)) {
    PowerCmdPacket c;
    memcpy(&c, data, sizeof(c));
    if (c.type != PWR_CMD_TYPE) return;

    DBGLN("[CMD] maskSet=%u maskVal=%u", (unsigned)c.maskSet, (unsigned)c.maskVal);

    bool changed = false;
    for (uint8_t ch = 1; ch <= RELAY_COUNT; ch++) {
      uint8_t bit = 1 << (ch - 1);
      if (c.maskSet & bit) {
        bool on = (c.maskVal & bit) != 0;
        if (relayMaskGet(ch) != on) {
          relayWrite(ch, on);
          relayMaskSet(ch, on);
          DBGLN("[RELAY] ch=%u -> %s (manual)", (unsigned)ch, onOff(on));
          changed = true;
        }
      }
    }

    if (changed) saveRelayMask();
    sendStateToMaster();
    return;
  }

  // RULES
  if (len == (int)sizeof(PowerRelayRulesPacket)) {
    PowerRelayRulesPacket rp;
    memcpy(&rp, data, sizeof(rp));
    if (rp.type != PWR_RELAYRULE_TYPE) return;
    if (rp.ch < 1 || rp.ch > RELAY_COUNT) {
      DBGLN("[RULES] invalid ch=%u", (unsigned)rp.ch);
      sendScheduleAckToMaster(rp.ch, 0, false);
      sendErrorToMaster(2 /*INVALID_CH*/, rp.ch, 0);
      return;
    }

    uint8_t c = rp.count;
    if (c > 10) c = 10;

    int idx = rp.ch - 1;
    memcpy(rules[idx], rp.rules, sizeof(rules[idx]));
    ruleCount[idx] = c;

    DBGLN("[RULES] ch=%u count=%u (saved, no immediate normalize)", (unsigned)rp.ch, (unsigned)ruleCount[idx]);
    for (uint8_t k = 0; k < ruleCount[idx]; k++) {
      const RelayRuleBin& r = rules[idx][k];
      DBGLN("  - #%u at=%u on=%u daysMask=0x%02X", (unsigned)k, (unsigned)r.minuteOfDay, (unsigned)r.on, (unsigned)r.daysMask);
    }

    // lo memorizza in memoria
    bool ok = saveRules(rp.ch);

    // Se e andato tutto bene manda un ACK di convalida, altrimenti manda un messaggiio di errore dicendo che non è riuscito a salvare.
    if (ok) {
      sendScheduleAckToMaster(rp.ch, ruleCount[idx], true);
    } else {
      sendScheduleAckToMaster(rp.ch, ruleCount[idx], false);
      sendErrorToMaster(1 /*NVS_SAVE_FAIL*/, rp.ch, ruleCount[idx]);
    }

    sendStateToMaster();
    return;
  }

  // TIME
  if (len == (int)sizeof(PowerTimePacket)) {
    PowerTimePacket tp;
    memcpy(&tp, data, sizeof(tp));
    if (tp.type != PWR_TIME_TYPE) return;

    timeValid = (tp.valid == 1);
    if (timeValid) {
      curMinOfDay = tp.minuteOfDay % 1440;
      curWeekday  = tp.weekdayMon0 % 7;
      lastTimeSyncMs = millis();

      DBGLN("[TIME] valid=1 min=%u wd=%u", (unsigned)curMinOfDay, (unsigned)curWeekday);

      bool changed = applyRulesExactNow(false);
      if (changed) DBGLN("[TIME] applied rules at this minute");
    } else {
      DBGLN("[TIME] valid=0 (waiting NTP on master)");
    }

    sendStateToMaster();
    return;
  }

  DBGLN("[ESPNOW] RX unknown (type=%u len=%d)", (unsigned)ptype, len);
}

// ===================== ESPNOW INIT =====================
static bool initEspNowOnChannel(uint8_t ch) {
  WiFi.mode(WIFI_STA);
  delay(20);

  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  curChannel = ch;

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onEspNowRecv);

  ensureMasterPeer(ch);
  return true;
}

static bool findChannelFromHello(uint32_t maxMs = 7000) {
  gotHello = false;
  uint32_t start = millis();

  DBGLN("[SCAN] searching HELLO up to %lu ms", (unsigned long)maxMs);

  while (millis() - start < maxMs) {
    for (uint8_t ch = 1; ch <= 13; ch++) {
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);

      uint32_t t0 = millis();
      while (millis() - t0 < 260) {
        if (gotHello) {
          lockedChannel = (int8_t)helloCh;
          DBGLN("[SCAN] HELLO found on ch=%u", (unsigned)helloCh);
          return true;
        }
        delay(5);
      }
    }
  }
  DBGLN("[SCAN] HELLO not found");
  return false;
}

static void relaysInitAndRestore() {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    relayWrite(i + 1, false);
  }
  delay(150);

  relayMask = prefs.getUChar(KEY_RELAYMASK, 0);
  DBGLN("[RELAY] restore mask=%u", (unsigned)relayMask);
  for (uint8_t ch = 1; ch <= RELAY_COUNT; ch++) relayWrite(ch, relayMaskGet(ch));
}

// ===================== SETUP / LOOP =====================
void setup() {
  DBG_PORT.begin(DBG_BAUD);
  delay(800);
  DBG_PORT.println("\n\n=== POWER BOOT ===");

  DBGLN("=== EVE-POWER SLAVE START ===");
  DBGLN("RELAY pins: %u,%u,%u,%u (activeLow=%u)",
        RELAY_PINS[0], RELAY_PINS[1], RELAY_PINS[2], RELAY_PINS[3],
        (unsigned)RELAY_ACTIVE_LOW);

  prefs.begin(NVS_NS, false);
  loadMasterMac();
  loadRulesAll();
  relaysInitAndRestore();

  char macs[24]; macToStr(MASTER_MAC, macs, sizeof(macs));
  DBGLN("MASTER_MAC=%s (fixed=%u)", macs, (unsigned)USE_FIXED_MASTER_MAC);

  // Finché non arriva HELLO, non consideriamo il canale "pronto"
  channelReady = false;

  if (!initEspNowOnChannel(1)) DBGLN("ESP-NOW init FAIL (ch=1)");
  else DBGLN("ESP-NOW init OK (ch=1)");

  if (lockedChannel < 1 || lockedChannel > 13) {
    DBGLN("AUTO CH: scanning (waiting HELLO)...");
    if (!findChannelFromHello(7000)) {
      DBGLN("AUTO CH: not found -> fallback ch=1");
      lockedChannel = 1;
    } else {
      DBGLN("AUTO CH: locked=%d", (int)lockedChannel);
    }
  } else {
    DBGLN("AUTO CH: using locked=%d", (int)lockedChannel);
  }

  esp_now_deinit();
  delay(30);

  if (!initEspNowOnChannel((uint8_t)lockedChannel)) DBGLN("ESP-NOW init FAIL (locked)");
  else DBGLN("ESP-NOW init OK (locked ch=%d)", (int)lockedChannel);

  // NON mando state qui: deve avvenire dopo HELLO (così rispettiamo "prima agganciati al canale")
  DBGLN("[BOOT] waiting HELLO to become channelReady...");
}

void loop() {
  // Avanza il tempo "a spanne" se non arrivano sync (per non perdere le regole nel tempo)
  if (timeValid) {
    uint32_t now = millis();
    uint32_t elapsed = now - lastTimeSyncMs;

    if (elapsed >= 60000UL) {
      uint32_t addMin = elapsed / 60000UL;
      lastTimeSyncMs += addMin * 60000UL;

      uint32_t total = (uint32_t)curMinOfDay + addMin;
      curWeekday  = (curWeekday + (total / 1440UL)) % 7;
      curMinOfDay = total % 1440UL;

      DBGLN("[TICK] +%lu min -> min=%u wd=%u",
            (unsigned long)addMin, (unsigned)curMinOfDay, (unsigned)curWeekday);

      bool changed = applyRulesExactNow(true);
      if (changed) sendStateToMaster();
    }
  }

  delay(20);
}
