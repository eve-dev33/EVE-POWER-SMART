# Analisi del contesto: "3 progetti"

## Premessa importante
Nel repository attuale è presente **un solo progetto software effettivo** (firmware PlatformIO per ESP32-C3).
Non risultano, nello stato corrente del codice, gli altri due progetti separati (es. master/gateway o backend/app).

Detto questo, dal protocollo e dalle strutture dati in `src/main.cpp` è possibile ricostruire chiaramente il contesto in cui questo firmware vive: è il nodo "power/attuazione" di un sistema più ampio a 3 componenti.

---

## Progetto 1 (presente nel repo): firmware attuatore relè ESP32-C3

### Obiettivo funzionale
Controllare 4 relè, mantenere stato/regole in persistenza NVS, ricevere comandi e sincronizzazione tempo via ESP-NOW, e notificare lo stato al master.

### Stack e dipendenze
- PlatformIO + framework Arduino su board ESP32-C3 (`esp32-c3-devkitm-1`).
- Librerie dichiarate: ArduinoJson e PubSubClient (attualmente il file principale usa soprattutto API ESP32/Arduino/ESP-NOW).

### Architettura interna
1. **Layer hardware relè**
   - 4 pin relè (`2,4,5,6`) con logica `active low`.
   - Funzione dedicata per scrittura astratta `relayWrite(ch, on)`.

2. **Persistenza NVS (Preferences)**
   - Namespace `evepower`.
   - Stato persistente:
     - maschera relè (`relayMask`),
     - regole per ciascun canale (fino a 10 regole per relè),
     - MAC master appreso.

3. **Protocollo ESP-NOW**
   - Packet type principali:
     - `HELLO` (scoperta canale master),
     - `CMD` (on/off manuale con mask bitwise),
     - `RELAY RULES` (programmazione schedulazione per canale),
     - `TIME` (sincronizzazione minuto+giorno),
     - `STATE` (telemetria/stato inviato al master).
   - Apprendimento automatico del MAC del master al primo pacchetto ricevuto.

4. **Motore scheduler**
   - Due strategie complementari:
     - `applyRulesExactNow()`: applica eventi esatti sul minuto corrente.
     - `normalizeAllRelaysNow()`: riallinea ogni relè allo stato "corretto ora" anche se sono stati persi eventi precedenti.
   - Gestione attraversamento mezzanotte usando confronto "oggi vs ieri".

5. **Bootstrap radio/canale**
   - Avvio ESP-NOW su canale 1.
   - Scansione 1..13 in attesa `HELLO` per lock del canale.
   - Re-init ESP-NOW sul canale lockato.

6. **Loop runtime**
   - Se il tempo è valido, avanza localmente minuto per minuto via `millis()`.
   - Ad ogni scatto minuto: applica regole + normalizzazione + notifica stato se cambia qualcosa.

### Cosa fa bene
- Buona resilienza a reboot e perdita momentanea sync (grazie NVS + normalizzazione).
- Protocollo binario compatto, adatto a ESP-NOW.
- Separazione netta tra comando manuale, scheduling e sync tempo.

### Rischi/attenzioni
- Il progetto è fortemente dipendente da un master esterno che invia tempo/regole/comandi.
- Le credenziali in `include/secrets.h` indicano un contesto MQTT/HiveMQ non ancora utilizzato in questo firmware: probabile parte di altri progetti non inclusi qui.

---

## Progetto 2 (non presente nel repo, ma chiaramente richiesto dal protocollo): master/gateway ESP-NOW

### Responsabilità attese
- Emettere `HELLO` periodici per far lockare il canale ai nodi.
- Inviare `PowerCmdPacket`, `PowerRelayRulesPacket`, `PowerTimePacket`.
- Ricevere `PowerStatePacket` dai nodi.
- Gestire mapping nodi (MAC), provisioning e affidabilità trasmissione.

### Ingressi/uscite chiave
- **Ingresso dal cloud/app**: desiderata utente (switch, scheduler, timezone).
- **Uscita verso nodi**: pacchetti ESP-NOW tipizzati.
- **Ritorno al cloud/app**: stato aggiornato e diagnostica nodi.

---

## Progetto 3 (non presente nel repo, ma suggerito da `secrets.h`): backend/app cloud

### Responsabilità attese
- UI/UX di controllo relè (toggle e timer settimanale).
- Persistenza regole utente e distribuzione verso gateway.
- Gestione identità dispositivi/utenti.
- Telemetria e storico stati.

### Evidenze dal repo
- In `include/secrets.h` sono definiti `WIFI_SSID`, `MQTT_HOST`, `MQTT_USER`, `MQTT_PASS` (HiveMQ Cloud), indicando integrazione cloud prevista nell'ecosistema complessivo.

---

## Mappa del contesto end-to-end (ricostruita)
1. L'utente configura da app/cloud stato e regole.
2. Il backend inoltra al gateway/master.
3. Il gateway traduce in pacchetti ESP-NOW verso il nodo relè.
4. Il nodo applica subito o schedula, salva su NVS, e risponde con stato.
5. Il gateway riporta stato al cloud/app.

---

## Gap trovati per completare la "visione a 3 progetti"
- Mancano nel repository i sorgenti di:
  - master/gateway;
  - backend/app.
- Mancano documenti di contratto protocollo condiviso (versioning, compatibilità, retry policy, timezone/DST).

---

## Conclusione operativa
- **Stato attuale**: questo repo rappresenta con buona maturità il **progetto attuatore**.
- **Contesto complessivo**: il codice è progettato per operare dentro un sistema a 3 componenti (attuatori, gateway/master, cloud/app), ma gli altri 2 non sono inclusi qui.
- **Prossimo passo consigliato**: raccogliere i repository mancanti per una vera analisi comparata "tutti e 3 i progetti" (architettura, contratti e deployment end-to-end).
