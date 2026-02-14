# Protocollo POWER ↔ MASTER per schedulazioni relè

Questo documento definisce il flusso operativo richiesto tra:
- **POWER (slave ESP32-C3 relè)**
- **MASTER (bridge MQTT + ESP-NOW)**

I comandi manuali restano invariati.

## MQTT (APP ↔ MASTER)

### Manuale (immutato)
- `progetto/EVE/POWER/relay/%d/set` → `ON | OFF | TOGGLE`
- `progetto/EVE/POWER/relay/%d/state` → `ON | OFF`

### Schedulazioni (nuovo)
- APP → MASTER:
  - topic: `progetto/EVE/POWER/relay/%d/schedule/set`
  - payload: JSON array (max 10 regole per relay)
  - item:
    - `at`: `HH:MM`
    - `state`: `ON` / `OFF`
    - `days`: `1111111` (lun→dom)

- MASTER → APP (conferma retained):
  - topic: `progetto/EVE/POWER/relay/%d/schedule`
  - payload: `OK SCHEDULAZIONE`

- MASTER → APP (sync all'apertura, retained):
  - topic: `progetto/EVE/POWER/relay/%d/schedule/current`
  - payload: JSON regole attive

- SLAVE ACK salvataggio (via MASTER):
  - topic: `progetto/EVE/POWER/relay/%d/schedule/slave/ack`
  - payload: `OK`

- Esecuzione schedulazione:
  - topic: `progetto/EVE/POWER/relay/%d/executed`
  - payload: `ON | OFF`

- ACK app opzionale:
  - topic: `progetto/EVE/POWER/relay/%d/executed/ack`
  - payload: `OK`

## ESP-NOW (MASTER ↔ POWER)

Nel firmware POWER sono definiti questi pacchetti:
- `type=10` comando manuale (`PowerCmdPacket`)
- `type=12` stato relè (`PowerStatePacket`)
- `type=13` sync tempo (`PowerTimePacket`)
- `type=14` regole relay (`PowerRelayRulesPacket`)
- `type=15` **nuovo** ACK salvataggio schedule dal POWER (`PowerScheduleAckPacket`)
- `type=16` **nuovo** notifica esecuzione evento schedule dal POWER (`PowerExecutedPacket`)

### Dettagli nuovi pacchetti POWER → MASTER
- `PowerScheduleAckPacket` (`type=15`)
  - `ch` canale relay 1..4
  - `ok` (1 salvataggio avvenuto)
  - `count` regole memorizzate (0..10)
  - `ms` timestamp locale

- `PowerExecutedPacket` (`type=16`)
  - `ch` canale relay 1..4
  - `state` (1 ON / 0 OFF)
  - `minuteOfDay` minuto esecuzione
  - `weekdayMon0` giorno esecuzione (lun=0..dom=6)
  - `ms` timestamp locale

## Flusso completo impostazione schedulazione
1. APP pubblica JSON su `.../schedule/set`.
2. MASTER valida e converte in `PowerRelayRulesPacket` verso POWER.
3. POWER salva in NVS, normalizza stato e invia `type=15` al MASTER.
4. MASTER pubblica verso APP:
   - `.../schedule/slave/ack = OK`
   - `.../schedule = OK SCHEDULAZIONE` (retained)
   - `.../schedule/current = <json>` (retained)

## Flusso completo esecuzione schedulazione
1. Arriva minuto/orario valido nel POWER.
2. POWER attua relay e invia `type=16` al MASTER.
3. MASTER aggiorna e inoltra all'APP:
   - `.../executed = ON|OFF`
   - `.../state = ON|OFF`
4. APP può rispondere con `.../executed/ack = OK`.
