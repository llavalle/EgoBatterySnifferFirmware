// SPDX-License-Identifier: GPL-3.0-or-later
#include <Arduino.h>

// EGO Battery D-pin passive decoder
// Wiring: Mega 5V --[470 ohm]--+-- D pin
//                              +-- Pin 2
//                              +-- A0   (analog tap for direction detection)
//         Mega GND ------------ - pin
//
// Pin 2 is INPUT (line idles HIGH at ~4.0V via the 470 ohm pull-up to 5V,
// matching the level real EGO tools drive). Pin 2 may pull LOW via OUTPUT/LOW
// during TX. Never drive OUTPUT HIGH - the resistor handles HIGH.
//
// A0 is read at the start of each frame to determine direction:
//   tool drives LOW to ~0 V    -> ADC < threshold -> TOOL->
//   battery drives LOW to ~0.7 V -> ADC > threshold -> BATT->
//
// Encoding (validated against live captures + CRC):
//   short LOW (~200us) = bit '1'
//   long  LOW (~600us) = bit '0'
//   sync = LOW ~1250us at start of each frame group (when present)
//   bytes are LSB-first; commands come byte-reversed on the wire
//   CRC = Dallas 1-Wire CRC8 (poly 0x31, refin/refout, init 0)
// Frames with 8N+1 bits carry a leading framing bit that must be dropped
// before byte extraction (e.g. 73-bit single message -> 72 bits = 9 bytes).
//
// Output is NDJSON: one self-describing JSON object per line, terminated by
// '\n'. Top-level types: hello | frame | tx | evt. Field schema is owned by
// guiplan.md (Phase 0).

#define DATA_PIN 2
#define DIR_PIN  A0             // analog tap for direction detection (same node as DATA_PIN)
#define EDGE_BUF_SIZE 128       // power of 2
#define FRAME_BUF_BYTES 32      // up to 256 bits per frame

// Direction discrimination threshold on the ADC (0..1023, 5V Vref).
// Tool drives LOW to ~0V (active drive). Battery's LOW depends on the tool's
// pull-up: ~0.7V with a strong pull-up, but as low as ~0.3V with a weaker
// one. 40 ~ 0.2V splits the two reliably across the tools tested so far.
// (TODO: adaptive threshold based on idle HIGH so we don't have to retune.)
const int16_t DIR_THRESHOLD_ADC = 40;

struct Edge {
  uint32_t time_us;
  uint8_t  state;
};
volatile Edge    edgeBuf[EDGE_BUF_SIZE];
volatile uint8_t writeIdx = 0;
volatile uint8_t droppedEdges = 0;
uint8_t readIdx = 0;

// Decoder thresholds (microseconds)
const uint16_t LOW_GLITCH_MAX  = 100;
const uint16_t LOW_SHORT_MAX   = 400;   // < this  -> bit '0'
const uint16_t LOW_LONG_MAX    = 900;   // < this  -> bit '1'
const uint16_t LOW_SYNC_MAX    = 2000;  // < this  -> sync
const uint32_t FRAME_GAP_US    = 1500;  // HIGH > this  -> end of frame
                                        // (inter-bit HIGH is 200-600us, so 1500us
                                        //  splits paired tool/battery messages safely)

uint8_t  frameBits[FRAME_BUF_BYTES];
uint16_t bitCount = 0;
uint32_t lastEdgeTime = 0;

uint32_t framesSeen = 0;

// Direction sample taken at the start of each frame: -1 = not measured.
volatile int16_t  frameDirAdc  = -1;
volatile uint32_t prevFallTime = 0;

void onEdge() {
  uint8_t next = (writeIdx + 1) & (EDGE_BUF_SIZE - 1);
  if (next == readIdx) { droppedEdges++; return; }
  uint32_t t = micros();
  uint8_t  s = digitalRead(DATA_PIN);
  edgeBuf[writeIdx].time_us = t;
  edgeBuf[writeIdx].state   = s;
  writeIdx = next;

  // On the first falling edge of a new frame, sample the LOW level on A0.
  // ADC prescaler is set to 16 in setup() -> ~13us per conversion, well
  // inside even a 200us short LOW.
  if (s == LOW) {
    if (t - prevFallTime > FRAME_GAP_US) {
      ADCSRA |= _BV(ADSC);
      while (ADCSRA & _BV(ADSC)) { /* ~13us */ }
      frameDirAdc = ADC;
    }
    prevFallTime = t;
  }
}

// Dallas 1-Wire CRC8: poly 0x31, refin/refout = true, init 0
uint8_t crc8_dallas(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ b) & 1;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      b >>= 1;
    }
  }
  return crc;
}

void appendBit(uint8_t bit) {
  if (bitCount >= (uint16_t)FRAME_BUF_BYTES * 8) return;
  uint16_t byteIdx   = bitCount >> 3;
  uint8_t  bitInByte = bitCount & 7;
  if (bitInByte == 0) frameBits[byteIdx] = 0;
  frameBits[byteIdx] |= (bit << bitInByte);   // LSB-first
  bitCount++;
}

void printHex2(uint8_t v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

// Drop bit 0 of the frame buffer; shift everything down by 1.
// Requires that frameBits has at least one storage byte beyond bitCount,
// and consumes the leading framing bit so the remaining stream is byte-aligned.
void shiftBufferRight1() {
  uint16_t totalBytes = (bitCount + 7) >> 3;
  for (uint16_t i = 0; i + 1 < totalBytes; i++) {
    frameBits[i] = (frameBits[i] >> 1) | ((frameBits[i + 1] & 1) << 7);
  }
  if (totalBytes > 0) {
    frameBits[totalBytes - 1] >>= 1;
  }
  bitCount--;
}

// Wire-order bytes for known commands (= command name byte-reversed)
// Diagnostic / discharge protocol:
const char START_WIRE[6]  = { 0x5F, 0x54, 0x52, 0x41, 0x54, 0x53 };  // "_TRATS" = START_
const char ADJUST_WIRE[6] = { 0x54, 0x53, 0x55, 0x4A, 0x44, 0x41 };  // "TSUJDA" = ADJUST
const char RD_VOL_WIRE[6] = { 0x4C, 0x4F, 0x56, 0x5F, 0x44, 0x52 };  // "LOV_DR" = RD_VOL
const char RD_TMP_WIRE[6] = { 0x50, 0x4D, 0x54, 0x5F, 0x44, 0x52 };  // "PMT_DR" = RD_TMP
const char RD_SPC_WIRE[6] = { 0x43, 0x50, 0x53, 0x5F, 0x44, 0x52 };  // "CPS_DR" = RD_SPC
const char RD_CAP_WIRE[6] = { 0x50, 0x41, 0x43, 0x5F, 0x44, 0x52 };  // "PAC_DR" = RD_CAP
const char RD_FSH_WIRE[6] = { 0x48, 0x53, 0x46, 0x5F, 0x44, 0x52 };  // "HSF_DR" = RD_FSH
// Charging protocol (charger <-> battery):
const char EVACHG_WIRE[6] = { 0x47, 0x48, 0x43, 0x41, 0x56, 0x45 };  // "GHCAVE" = EVACHG (enter charge mode)
const char GETCG1_WIRE[6] = { 0x31, 0x47, 0x43, 0x54, 0x45, 0x47 };  // "1GCTEG" = GETCG1
const char OUTCG1_WIRE[6] = { 0x31, 0x47, 0x43, 0x54, 0x55, 0x4F };  // "1GCTUO" = OUTCG1
const char GET_VM_WIRE[6] = { 0x4D, 0x56, 0x5F, 0x54, 0x45, 0x47 };  // "MV_TEG" = GET_VM (battery asks output voltage)
const char OUT_VM_WIRE[6] = { 0x4D, 0x56, 0x5F, 0x54, 0x55, 0x4F };  // "MV_TUO" = OUT_VM (charger reports voltage, /100 V)
const char GET_CM_WIRE[6] = { 0x4D, 0x43, 0x5F, 0x54, 0x45, 0x47 };  // "MC_TEG" = GET_CM (battery asks output current)
const char OUT_CM_WIRE[6] = { 0x4D, 0x43, 0x5F, 0x54, 0x55, 0x4F };  // "MC_TUO" = OUT_CM (charger reports current, /100 A)
const char FAN_ON_WIRE[6] = { 0x4E, 0x4F, 0x5F, 0x4E, 0x41, 0x46 };  // "NO_NAF" = FAN_ON
const char EN_OUT_WIRE[6] = { 0x54, 0x55, 0x4F, 0x5F, 0x4E, 0x45 };  // "TUO_NE" = EN_OUT (1=enable charger output)
const char CHGING_WIRE[6] = { 0x47, 0x4E, 0x49, 0x47, 0x48, 0x43 };  // "GNIGHC" = CHGING (charging heartbeat)
const char ADDCUR_WIRE[6] = { 0x52, 0x55, 0x43, 0x44, 0x44, 0x41 };  // "RUCDDA" = ADDCUR (battery requests delta current)
const char OUTCUR_WIRE[6] = { 0x52, 0x55, 0x43, 0x54, 0x55, 0x4F };  // "RUCTUO" = OUTCUR (charger reports actual current)
const char CHGPCT_WIRE[6] = { 0x54, 0x43, 0x50, 0x47, 0x48, 0x43 };  // "TCPGHC" = CHGPCT (state-of-charge %)

bool bytesEqual(const uint8_t *a, const char *b, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) if (a[i] != (uint8_t)b[i]) return false;
  return true;
}

bool autoAdjustEnabled = false;
bool pendingAutoAdjust = false;

// Non-blocking RD_VOL sweep across all 14 cells
struct {
  bool     active;
  uint8_t  nextCell;
  uint32_t lastTxMs;
} sweep = { false, 0, 0 };
uint16_t SWEEP_GAP_MS = 100;

// ---- NDJSON output helpers ----
// Field schema is owned by guiplan.md (Phase 0).
//
// Each emitter starts the object inline: Serial.print(F("{\"t\":\"<type>\""))
// and then chains fields via the helpers below. The first field is the type
// itself (no leading comma), so every helper emits a leading ',' and the
// caller closes with Serial.println('}').

void jsonField(const __FlashStringHelper *key) {
  Serial.print(F(",\""));
  Serial.print(key);
  Serial.print(F("\":"));
}

void jsonIntField(const __FlashStringHelper *key, int32_t val) {
  jsonField(key);
  Serial.print(val);
}

void jsonFstrField(const __FlashStringHelper *key, const __FlashStringHelper *val) {
  jsonField(key);
  Serial.print('"');
  Serial.print(val);
  Serial.print('"');
}

void jsonBytesField(const __FlashStringHelper *key, const uint8_t *bytes, uint16_t n) {
  jsonField(key);
  Serial.print('"');
  for (uint16_t i = 0; i < n; i++) {
    if (i > 0) Serial.print(' ');
    printHex2(bytes[i]);
  }
  Serial.print('"');
}

// Decode the 6-byte byte-reversed command name to its on-tool form (e.g.
// "LOV_DR" -> "RD_VOL") and emit it as a JSON string. Sanitizes any
// non-printable / quote-conflicting bytes to '.'.
void jsonCmdField(const uint8_t *cmdReversed) {
  jsonField(F("cmd"));
  Serial.print('"');
  for (int8_t i = 5; i >= 0; i--) {
    uint8_t c = cmdReversed[i];
    if (c >= 32 && c < 127 && c != '"' && c != '\\') Serial.write(c);
    else                                              Serial.write('.');
  }
  Serial.print('"');
}

// Open an evt object: {"t":"evt","ms":<millis>,"evt":"<name>"
// Caller adds optional fields, then closes with Serial.println('}').
void evtBegin(const __FlashStringHelper *evtName) {
  Serial.print(F("{\"t\":\"evt\""));
  jsonIntField(F("ms"), millis());
  jsonFstrField(F("evt"), evtName);
}

void processFrame() {
  if (bitCount == 0) return;
  framesSeen++;

  // Snapshot the direction sample taken at frame start (set in onEdge()).
  int16_t dirAdc = frameDirAdc;
  frameDirAdc = -1;
  bool fromTool = (dirAdc >= 0 && dirAdc <  DIR_THRESHOLD_ADC);
  bool fromBatt = (dirAdc >= 0 && dirAdc >= DIR_THRESHOLD_ADC);

  // 8N+1 bit frames (73, 145, ...) carry a leading framing bit -> drop it
  bool shifted = (bitCount & 7) == 1;
  if (shifted) shiftBufferRight1();

  uint16_t numBytes     = bitCount >> 3;
  uint8_t  leftoverBits = bitCount & 7;
  uint16_t totalBits    = bitCount + (shifted ? 1 : 0);

  Serial.print(F("{\"t\":\"frame\""));
  jsonIntField(F("ms"), millis());
  jsonIntField(F("idx"), framesSeen);
  if (fromTool)      jsonFstrField(F("dir"), F("TOOL"));
  else if (fromBatt) jsonFstrField(F("dir"), F("BATT"));
  else               jsonFstrField(F("dir"), F("?"));
  if (dirAdc >= 0) {
    // adc * 5000 / 1024 = millivolts on a 5V/10-bit ADC
    jsonIntField(F("dir_mv"), ((int32_t)dirAdc * 5000L) / 1024L);
  }
  jsonIntField(F("bits"), totalBits);
  jsonBytesField(F("bytes"), frameBits, numBytes);
  if (leftoverBits) {
    jsonIntField(F("leftover_bits"), leftoverBits);
  }

  bool crcOk = false;
  if (numBytes >= 2 && leftoverBits == 0) {
    uint8_t expected = crc8_dallas(frameBits, numBytes - 1);
    if (expected == frameBits[numBytes - 1]) {
      jsonFstrField(F("crc"), F("ok"));
      crcOk = true;
    } else {
      jsonFstrField(F("crc"), F("bad"));
      jsonField(F("crc_want"));
      Serial.print('"');
      printHex2(expected);
      Serial.print('"');
    }
  }

  // Frame-type interpretation
  if (numBytes == 5) {
    jsonFstrField(F("cmd"), F("ID"));
  } else if (numBytes == 9) {
    // For read commands: tool->batt is the query (byte[1]=index), batt->tool is
    // the response (data = LE uint16). For session frames (START_, ADJUST) the
    // payload is interpreted as raw data regardless. Fall back to the byte[0]==0
    // heuristic if we don't have a direction reading.
    bool isQuery = fromTool ? true
                 : fromBatt ? false
                 : (frameBits[0] == 0x00);

    jsonCmdField(&frameBits[2]);
    uint16_t v = ((uint16_t)frameBits[1] << 8) | frameBits[0];
    jsonIntField(F("data"), v);

    // Per-command interpretation -- fields per guiplan.md
    if (bytesEqual(&frameBits[2], ADJUST_WIRE, 6)) {
      // ADJUST is the gen marker: tool always sends 0x0000; the battery's
      // reply distinguishes generations -- Gen1 echoes 0x0000, Gen2 replies
      // 0x00A0. (Documented in firmware README; verified against a Gen1
      // Nexus 3000 capture. Re-check the 0x00A0 value when a Gen2 ADJUST
      // sniff lands.)
      if (fromBatt) {
        if      (v == 0x0000) jsonIntField(F("gen"), 1);
        else if (v == 0x00A0) jsonIntField(F("gen"), 2);
      }
    } else if (bytesEqual(&frameBits[2], RD_VOL_WIRE, 6)) {
      if (isQuery) jsonIntField(F("cell"), frameBits[1]);
      else         jsonIntField(F("v_cv"), v);
    } else if (bytesEqual(&frameBits[2], RD_TMP_WIRE, 6)) {
      if (isQuery) jsonIntField(F("sensor"), frameBits[1]);
      else         jsonIntField(F("temp_f"), v);
    } else if (bytesEqual(&frameBits[2], RD_SPC_WIRE, 6)) {
      if (!isQuery) {
        jsonIntField(F("s_count"), frameBits[1]);
        jsonIntField(F("p_count"), frameBits[0] + 1);
      }
    } else if (bytesEqual(&frameBits[2], RD_CAP_WIRE, 6)) {
      if (!isQuery) jsonIntField(F("ah_per_cell_x100"), v);
    } else if (bytesEqual(&frameBits[2], RD_FSH_WIRE, 6)) {
      // RD_FSH is a flash-byte read: addr = high byte of the 16-bit data
      // word, value = low byte. Tool writes addr with value=0; battery
      // echoes addr in the high byte and returns the byte at that addr.
      // Gen detection lives on ADJUST, not here -- the address being
      // queried tells us what the tool wants, not what the pack is.
      uint8_t addr = frameBits[1];
      uint8_t val  = frameBits[0];
      jsonIntField(F("fsh_addr"), addr);
      if (isQuery) {
        jsonIntField(F("q"), v);
      } else {
        jsonIntField(F("fsh_value"), val);
        jsonField(F("status"));
        Serial.print(F("\"0x"));
        printHex2(addr);
        printHex2(val);
        Serial.print('"');
      }

    // ---- Charging-protocol commands ----
    } else if (bytesEqual(&frameBits[2], START_WIRE, 6)) {
      if (fromTool) jsonIntField(F("chg_id"), v);
    } else if (bytesEqual(&frameBits[2], OUT_VM_WIRE, 6)) {
      jsonIntField(F("v_out_cv"), v);
    } else if (bytesEqual(&frameBits[2], OUT_CM_WIRE, 6)) {
      jsonIntField(F("i_out_ca"), v);
    } else if (bytesEqual(&frameBits[2], EN_OUT_WIRE, 6)) {
      jsonField(F("enabled"));
      Serial.print(v ? F("true") : F("false"));
    } else if (bytesEqual(&frameBits[2], CHGPCT_WIRE, 6)) {
      if (fromBatt) jsonIntField(F("soc_pct"), v);
    } else if (bytesEqual(&frameBits[2], ADDCUR_WIRE, 6)) {
      if (fromBatt) jsonIntField(F("add_cur"), v);
    } else if (bytesEqual(&frameBits[2], OUTCUR_WIRE, 6)) {
      if (fromTool) jsonIntField(F("cur"), v);
    } else if (bytesEqual(&frameBits[2], FAN_ON_WIRE, 6)) {
      if (fromBatt)      jsonIntField(F("fan_req"), v);
      else if (fromTool) jsonIntField(F("fan_set"), v);
    }

    if (crcOk && autoAdjustEnabled && bytesEqual(&frameBits[2], START_WIRE, 6)) {
      pendingAutoAdjust = true;
    }
  }
  // 12-byte (96-bit) and 22-byte+1leftover (178-bit) handshake blobs:
  // emitted as plain frame; no further interpretation.

  Serial.println('}');
}

void resetFrame() { bitCount = 0; }

// ---- Transmit ----
// Pulse widths chosen to match what the battery emits.
const uint16_t TX_SHORT_LOW_US = 200;   // bit '1'
const uint16_t TX_LONG_LOW_US  = 600;   // bit '0'
const uint16_t TX_SHORT_HI_US  = 200;   // HIGH after long  LOW (bit '0')
const uint16_t TX_LONG_HI_US   = 600;   // HIGH after short LOW (bit '1')

void txLow(uint16_t us) {
  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(DATA_PIN, LOW);
  delayMicroseconds(us);
  pinMode(DATA_PIN, INPUT);
}

void txBit(uint8_t bit) {
  if (bit) {
    txLow(TX_SHORT_LOW_US);
    delayMicroseconds(TX_LONG_HI_US);
  } else {
    txLow(TX_LONG_LOW_US);
    delayMicroseconds(TX_SHORT_HI_US);
  }
}

void txFrame(const uint8_t *bytes, uint8_t numBytes) {
  detachInterrupt(digitalPinToInterrupt(DATA_PIN));

  // Discard any pending edges - their timestamps predate our new lastEdgeTime
  // and any partial frame is going to be cut off by our TX anyway.
  readIdx = writeIdx;
  bitCount = 0;

  txBit(0);  // leading framing bit (drop on RX side)

  for (uint8_t i = 0; i < numBytes; i++) {
    uint8_t b = bytes[i];
    for (uint8_t j = 0; j < 8; j++) {
      txBit(b & 1);
      b >>= 1;
    }
  }

  lastEdgeTime = micros();
  attachInterrupt(digitalPinToInterrupt(DATA_PIN), onEdge, CHANGE);
}

// Build and send a 9-byte command frame.
// cmdReversed must be 6 chars (the on-wire byte order: e.g. "LOV_DR" for RD_VOL)
void sendCommand(const char *cmdReversed, uint16_t data) {
  uint8_t frame[9];
  frame[0] = data & 0xff;          // wire[0..1] = data uint16 LE
  frame[1] = (data >> 8) & 0xff;
  for (uint8_t i = 0; i < 6; i++) {
    frame[2 + i] = (uint8_t)cmdReversed[i];
  }
  frame[8] = crc8_dallas(frame, 8);

  Serial.print(F("{\"t\":\"tx\""));
  jsonIntField(F("ms"), millis());
  jsonIntField(F("bits"), 73);
  jsonBytesField(F("bytes"), frame, 9);
  jsonCmdField(&frame[2]);
  jsonIntField(F("data"), data);
  Serial.println('}');

  txFrame(frame, 9);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(DATA_PIN, INPUT);
  pinMode(DIR_PIN, INPUT);

  // Configure ADC for fast conversion on A0:
  //   AVcc reference, channel 0 (A0), prescaler 16 (~13us per conversion).
  ADMUX  = (1 << REFS0);                                  // AVcc, channel 0
  ADCSRA = (1 << ADEN) | (1 << ADPS2);                    // enable, prescaler 16
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) { /* discard first sample */ }
  (void)ADC;

  Serial.println(F("{\"t\":\"hello\",\"fw\":\"egosniffer/1.0\",\"build\":\"" __DATE__ " " __TIME__ "\"}"));

  evtBegin(F("line_init"));
  jsonFstrField(F("line"), digitalRead(DATA_PIN) ? F("HIGH") : F("LOW"));
  Serial.println('}');

  lastEdgeTime = micros();
  attachInterrupt(digitalPinToInterrupt(DATA_PIN), onEdge, CHANGE);
}

void loop() {
  // Drain edge buffer
  while (readIdx != writeIdx) {
    uint32_t t = edgeBuf[readIdx].time_us;
    uint8_t  s = edgeBuf[readIdx].state;
    readIdx = (readIdx + 1) & (EDGE_BUF_SIZE - 1);

    uint32_t duration = t - lastEdgeTime;

    if (s == HIGH) {
      // Just rose: previous LOW pulse lasted `duration`
      if (duration < LOW_GLITCH_MAX) {
        // ringing/noise - ignore
      } else if (duration < LOW_SHORT_MAX) {
        appendBit(1);   // short LOW = bit 1
      } else if (duration < LOW_LONG_MAX) {
        appendBit(0);   // long LOW = bit 0
      } else if (duration < LOW_SYNC_MAX) {
        // sync pulse - if we had bits, flush; then start new frame
        if (bitCount > 0) processFrame();
        resetFrame();
      } else {
        // unusually long LOW (e.g. "alone" beacon) - flush and report
        if (bitCount > 0) processFrame();
        resetFrame();
        evtBegin(F("long_low"));
        jsonIntField(F("dur_us"), duration);
        Serial.println('}');
      }
    } else {
      // Just fell: previous HIGH lasted `duration`
      if (duration > FRAME_GAP_US && bitCount > 0) {
        processFrame();
        resetFrame();
      }
    }

    lastEdgeTime = t;
  }

  // Timeout flush: line idle HIGH for too long after collecting bits
  if (bitCount > 0 && (micros() - lastEdgeTime) > FRAME_GAP_US) {
    processFrame();
    resetFrame();
  }

  if (droppedEdges) {
    uint8_t n = droppedEdges;
    droppedEdges = 0;
    evtBegin(F("dropped_edges"));
    jsonIntField(F("n"), n);
    Serial.println('}');
  }

  // Auto-respond to START_ (decoded in processFrame -> pendingAutoAdjust)
  if (pendingAutoAdjust) {
    pendingAutoAdjust = false;
    evtBegin(F("auto_adjust"));
    Serial.println('}');
    sendCommand("TSUJDA", 0x0000);
  }

  // Service the cell sweep
  if (sweep.active && (millis() - sweep.lastTxMs) >= SWEEP_GAP_MS) {
    if (sweep.nextCell >= 14) {
      sweep.active = false;
      evtBegin(F("sweep_done"));
      Serial.println('}');
    } else {
      sendCommand("LOV_DR", (uint16_t)sweep.nextCell << 8);  // cell index goes in byte 1
      sweep.lastTxMs = millis();
      sweep.nextCell++;
    }
  }

  // Serial command trigger
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'a' || c == 'A') {
      sendCommand("TSUJDA", 0x0000);   // ADJUST, data 0
    } else if (c == 't' || c == 'T') {
      sendCommand("PMT_DR", 0x0000);   // RD_TMP, sensor 0
    } else if (c == 's' || c == 'S') {
      sendCommand("CPS_DR", 0x0000);   // RD_SPC
    } else if (c == 'c' || c == 'C') {
      sendCommand("PAC_DR", 0x0000);   // RD_CAP
    } else if (c == 'f' || c == 'F') {
      sendCommand("HSF_DR", 0x3800);   // RD_FSH (Q=0x3800)
    } else if (c == 'r' || c == 'R') {
      sweep.active = true;
      sweep.nextCell = 0;
      sweep.lastTxMs = millis() - SWEEP_GAP_MS;   // fire first one immediately
      evtBegin(F("sweep_start"));
      Serial.println('}');
    } else if (c == 'x' || c == 'X') {
      autoAdjustEnabled = !autoAdjustEnabled;
      evtBegin(F("auto_adjust_toggle"));
      jsonField(F("on"));
      Serial.print(autoAdjustEnabled ? F("true") : F("false"));
      Serial.println('}');
    }
  }
}
