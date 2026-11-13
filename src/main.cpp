/*
 * CC1101 raw 433 MHz capture and repeat.
 *
 * ESP32-C3 Super Mini -> CC1101 (both are 3.3V, connect directly):
 *   GPIO 6  -> SCK
 *   GPIO 7  -> MISO (SO)
 *   GPIO 21 -> MOSI (SI)
 *   GPIO 20 -> CSN
 *   GPIO 10 -> GDO0
 *   3.3V    -> VCC
 *   GND     -> GND
 * Leave GPIO 2 and 8 free. They are read at reset.
 * GPIO 9 is BOOT: press to resend the last captured code.
 */

#include <SPI.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include "driver/gpio.h"
#define RF_ISR IRAM_ATTR
#else
#define RF_ISR
#endif
#include "replay_frame.h"

// ─── Pin Definitions ───────────────────────────────────────────
#if defined(ARDUINO_ARCH_ESP32)
#define CC1101_SCK   6
#define CC1101_MISO  7
#define CC1101_MOSI  21
#define CC1101_CS    20
#define CC1101_GDO0  10
#define SEND_BTN     9   // BOOT, active low
#else
#define CC1101_SCK   13
#define CC1101_MISO  12
#define CC1101_MOSI  11
#define CC1101_CS    10
#define CC1101_GDO0  2
#define SEND_BTN     3   // to GND, active low
#endif

// ─── CC1101 Register Addresses ─────────────────────────────────
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_PKTCTRL0 0x08
#define CC1101_FSCTRL1  0x0B
#define CC1101_FREQ2    0x0D
#define CC1101_FREQ1    0x0E
#define CC1101_FREQ0    0x0F
#define CC1101_MDMCFG4  0x10
#define CC1101_MDMCFG3  0x11
#define CC1101_MDMCFG2  0x12
#define CC1101_MDMCFG1  0x13
#define CC1101_DEVIATN  0x15
#define CC1101_MCSM1    0x17
#define CC1101_MCSM0    0x18
#define CC1101_FOCCFG   0x19
#define CC1101_AGCCTRL2 0x1B
#define CC1101_AGCCTRL1 0x1C
#define CC1101_AGCCTRL0 0x1D
#define CC1101_FREND0   0x22
#define CC1101_FREND1   0x21
#define CC1101_PATABLE  0x3E
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E

// ─── CC1101 Strobe Commands ────────────────────────────────────
#define CC1101_SRES    0x30
#define CC1101_SRX     0x34
#define CC1101_STX     0x35
#define CC1101_SIDLE   0x36
#define CC1101_SFRX    0x3A

// ─── Capture Settings ──────────────────────────────────────────
// One 1:2 frame is ~132 pulses. Hold two of them so a repeat can be framed.
#define MAX_PULSES       360
#define MIN_PULSE_US     80
#define MAX_PULSE_US     65000

// ─── Pulse Buffer ──────────────────────────────────────────────
volatile uint16_t pulseTimings[MAX_PULSES];  // uint16 to save RAM
volatile uint8_t pulseHigh[(MAX_PULSES + 7) / 8];  // 1 = that pulse was HIGH
volatile uint16_t pulseCount = 0;
volatile unsigned long lastEdgeTime = 0;
volatile bool capturing = false;
volatile bool captureReady = false;

static int RF_ISR gdo0Level() {
#if defined(ARDUINO_ARCH_ESP32)
  return gpio_get_level((gpio_num_t)CC1101_GDO0);
#else
  return digitalRead(CC1101_GDO0);
#endif
}

// ─── ISR: Record pulse timings on each edge ────────────────────
void RF_ISR handleGDO0Interrupt() {
  unsigned long now = micros();

  if (!capturing) return;

  if (lastEdgeTime > 0) {
    unsigned long duration = now - lastEdgeTime;

    if (duration >= MIN_PULSE_US && duration <= MAX_PULSE_US) {
      if (pulseCount < MAX_PULSES) {
        uint16_t idx = pulseCount;
        // Clamp to uint16 max. The pin has already changed, so the
        // pulse that just ended was high when the pin now reads low.
        pulseTimings[idx] = (duration > 65535) ? 65535 : (uint16_t)duration;
        uint8_t mask = (uint8_t)(1 << (idx & 7));
        if (gdo0Level() == LOW) pulseHigh[idx >> 3] |= mask;
        else pulseHigh[idx >> 3] &= (uint8_t)~mask;
        pulseCount = idx + 1;
      }
    }
  }

  lastEdgeTime = now;
}

// ─── SPI Communication ────────────────────────────────────────
void cc1101_writeReg(uint8_t addr, uint8_t value) {
  digitalWrite(CC1101_CS, LOW);
  while (digitalRead(CC1101_MISO));  // Wait MISO low (chip ready)
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(CC1101_CS, HIGH);
}

uint8_t cc1101_readReg(uint8_t addr) {
  digitalWrite(CC1101_CS, LOW);
  while (digitalRead(CC1101_MISO));
  SPI.transfer(addr | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CC1101_CS, HIGH);
  return val;
}

void cc1101_strobe(uint8_t strobe) {
  digitalWrite(CC1101_CS, LOW);
  while (digitalRead(CC1101_MISO));
  SPI.transfer(strobe);
  digitalWrite(CC1101_CS, HIGH);
}

void cc1101_writeBurst(uint8_t addr, const uint8_t *buf, uint8_t len) {
  digitalWrite(CC1101_CS, LOW);
  while (digitalRead(CC1101_MISO));
  SPI.transfer(addr | 0x40);
  for (uint8_t i = 0; i < len; i++) SPI.transfer(buf[i]);
  digitalWrite(CC1101_CS, HIGH);
}

// ─── CC1101 Reset ──────────────────────────────────────────────
void cc1101_reset() {
  digitalWrite(CC1101_CS, HIGH);
  delayMicroseconds(30);
  digitalWrite(CC1101_CS, LOW);
  delayMicroseconds(30);
  digitalWrite(CC1101_CS, HIGH);
  delayMicroseconds(45);

  cc1101_strobe(CC1101_SRES);
  delay(10);
}

// ─── Configure CC1101 for 433.92 MHz ASK/OOK RX ───────────────
void cc1101_initRaw433() {
  cc1101_reset();

  // GDO2 & GDO0: async serial data output
  cc1101_writeReg(CC1101_IOCFG2,   0x0D);
  cc1101_writeReg(CC1101_IOCFG0,   0x0D);

  // Packet handling OFF — raw/async mode
  cc1101_writeReg(CC1101_PKTCTRL0, 0x32);

  // Frequency: 433.92 MHz
  cc1101_writeReg(CC1101_FREQ2,    0x10);
  cc1101_writeReg(CC1101_FREQ1,    0xB0);
  cc1101_writeReg(CC1101_FREQ0,    0x71);

  // IF frequency
  cc1101_writeReg(CC1101_FSCTRL1,  0x06);

  // Modem: ASK/OOK, wide RX BW ~203 kHz, no sync
  cc1101_writeReg(CC1101_MDMCFG4,  0x87);
  cc1101_writeReg(CC1101_MDMCFG3,  0x32);
  cc1101_writeReg(CC1101_MDMCFG2,  0x30);
  cc1101_writeReg(CC1101_MDMCFG1,  0x22);

  cc1101_writeReg(CC1101_DEVIATN,  0x00);
  cc1101_writeReg(CC1101_MCSM1,    0x00);  // no CCA; stay in TX until SIDLE
  cc1101_writeReg(CC1101_MCSM0,    0x18);
  cc1101_writeReg(CC1101_FOCCFG,   0x16);

  // AGC tuned for OOK
  cc1101_writeReg(CC1101_AGCCTRL2, 0x04);
  cc1101_writeReg(CC1101_AGCCTRL1, 0x00);
  cc1101_writeReg(CC1101_AGCCTRL0, 0x92);

  cc1101_writeReg(CC1101_FREND1,   0x56);
  cc1101_writeReg(CC1101_FREND0,   0x11);  // OOK uses PATABLE[0] and [1]
  uint8_t pa[2] = {0x00, 0xC0};            // off, about +10 dBm
  cc1101_writeBurst(CC1101_PATABLE, pa, 2);

  // Freq synth cal
  cc1101_writeReg(CC1101_FSCAL3,   0xE9);
  cc1101_writeReg(CC1101_FSCAL2,   0x2A);
  cc1101_writeReg(CC1101_FSCAL1,   0x00);
  cc1101_writeReg(CC1101_FSCAL0,   0x1F);

  // Test registers for ASK
  cc1101_writeReg(CC1101_TEST2,    0x81);
  cc1101_writeReg(CC1101_TEST1,    0x35);
  cc1101_writeReg(CC1101_TEST0,    0x09);

  // Enter RX
  cc1101_strobe(CC1101_SIDLE);
  cc1101_strobe(CC1101_SFRX);
  cc1101_strobe(CC1101_SRX);
}

// Sync low between repeats. EV1527 data pulses stay under 2200 us.
// Raised only when the short/long data pulses themselves are longer.
static uint16_t syncUs = 2200;

struct Code {
  uint8_t method;
  uint8_t bits;
  uint8_t data[9];  // MSB first, packed
};

// Last accepted frame. BOOT (or pin 3 on Uno) resends it.
static Code queued;
static uint16_t queuedTe = 380;
static uint16_t lastShortUs = 380;
static bool queuedValid = false;
static bool repeatSend = false;
static bool queueUpdated = false;
static unsigned long quietUntil = 0;

// Method 2 framing. Repeats of one hold match; a new press may not.
static uint8_t learnedBits = 0;
static uint8_t pendingBits[80];
static uint8_t pendingN = 0;
static bool pendingRight = false;
static bool opened = false;  // a sync gap already consumed in this press
static uint8_t workBits[160];
static ReplayBuilder replayB;

static bool levelHigh(uint16_t i) {
  return (pulseHigh[i >> 3] & (uint8_t)(1 << (i & 7))) != 0;
}

static uint16_t captureReadUs(uint16_t i) {
  return pulseTimings[i];
}

static bool captureReadHigh(uint16_t i) {
  return levelHigh(i);
}

static void setLevel(uint16_t i, bool high) {
  uint8_t mask = (uint8_t)(1 << (i & 7));
  if (high) pulseHigh[i >> 3] |= mask;
  else pulseHigh[i >> 3] &= (uint8_t)~mask;
}

static bool isSync(uint16_t us) {
  return us >= syncUs;
}

static void consumeFront(uint16_t count) {
  noInterrupts();
  if (count > pulseCount) {
    count = pulseCount;
  }
  uint16_t left = pulseCount - count;
  for (uint16_t i = 0; i < left; i++) {
    pulseTimings[i] = pulseTimings[i + count];
    setLevel(i, levelHigh(i + count));
  }
  pulseCount = left;
  interrupts();
}

static void packBits(Code *out, uint8_t method, const uint8_t *bits, uint8_t n) {
  out->method = method;
  out->bits = n;
  for (uint8_t i = 0; i < sizeof(out->data); i++) out->data[i] = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (bits[i]) out->data[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
  }
}

static uint8_t bitOf(const Code *code, uint8_t i) {
  return (code->data[i >> 3] >> (7 - (i & 7))) & 1;
}

// Method 1 — EV1527 / PT2262. Short ~400 us, long ~3x, exactly 24 bits.
// Bit 1 = long high + short low. Bit 0 = short high + long low.
static bool decodeMethod1(uint16_t start, uint16_t count, Code *out) {
  if ((count % 2) == 1) {
    count--;
  }
  if (count != 48) {
    return false;
  }

  uint32_t sum = 0;
  uint16_t n = 0;
  for (uint16_t i = 0; i < count; i++) {
    uint16_t us = pulseTimings[start + i];
    if (us >= 150 && us < 900) {
      sum += us;
      n++;
    }
  }
  uint16_t threshold = (n >= 8) ? (uint16_t)((sum / n) * 2) : 700;

  uint8_t raw[24];
  for (uint8_t bit = 0; bit < 24; bit++) {
    uint16_t hi = pulseTimings[start + bit * 2];
    uint16_t lo = pulseTimings[start + bit * 2 + 1];
    if (isSync(hi) || isSync(lo)) {
      return false;
    }
    bool hiLong = hi >= threshold;
    bool loLong = lo >= threshold;
    if (hiLong == loLong) {
      return false;
    }
    raw[bit] = hiLong ? 1 : 0;
  }

  packBits(out, 1, raw, 24);
  return true;
}

// 1 = long high + short low. 0 = short high + long low. About 2:1.
static bool pwmBit(uint16_t hi, uint16_t lo, uint8_t *bit) {
  if (hi < 120 || lo < 120 || hi >= syncUs || lo >= syncUs) return false;
  if (hi > (uint16_t)(lo + (lo >> 1))) {
    *bit = 1;
    return true;
  }
  if (lo > (uint16_t)(hi + (hi >> 1))) {
    *bit = 0;
    return true;
  }
  return false;
}

// Skip an equal-width preamble, then read PWM pairs.
// useLevel forces each pair to start on a HIGH pulse.
static uint8_t collectPwm(uint16_t start, uint16_t count, uint16_t offset,
                          bool useLevel, uint8_t *out, uint8_t maxOut) {
  uint16_t i = offset;
  if (useLevel) {
    while (i < count && i < offset + 3 && !levelHigh(start + i)) i++;
  }

  uint8_t skipped = 0;
  while (i + 1 < count && skipped < 30) {
    if (useLevel && !levelHigh(start + i)) break;
    uint8_t bit;
    if (pwmBit(pulseTimings[start + i], pulseTimings[start + i + 1], &bit)) break;
    i += 2;
    skipped++;
  }

  uint8_t n = 0;
  uint8_t slips = 0;
  while (i + 1 < count && n < maxOut) {
    if (useLevel && !levelHigh(start + i)) {
      if (slips >= 2) break;
      slips++;
      i++;
      continue;
    }
    uint8_t bit;
    if (!pwmBit(pulseTimings[start + i], pulseTimings[start + i + 1], &bit)) {
      if (slips >= 2) break;
      slips++;
      i++;
      continue;
    }
    out[n++] = bit;
    i += 2;
  }
  return n;
}

static uint8_t collectBest(uint16_t start, uint16_t count, uint8_t *out, uint8_t maxOut) {
  // Recollect the phase that produced the most bits. Pulses already stored do not change.
  uint8_t nLevel = collectPwm(start, count, 0, true, out, maxOut);
  uint8_t n0 = collectPwm(start, count, 0, false, out, maxOut);
  uint8_t n1 = collectPwm(start, count, 1, false, out, maxOut);
  if (n1 >= n0 && n1 >= nLevel) return n1;
  if (n0 >= nLevel) return collectPwm(start, count, 0, false, out, maxOut);
  return collectPwm(start, count, 0, true, out, maxOut);
}

// Longest block that appears twice in a row. That block is one repeat.
static int8_t findRepeat(const uint8_t *bits, uint8_t n, uint8_t *startOut) {
  uint8_t maxL = n / 2;
  if (maxL > 80) maxL = 80;
  for (uint8_t L = maxL; L >= 32; L--) {
    uint8_t last = n - (uint8_t)(L * 2);
    for (uint8_t s = 0; s <= last; s++) {
      bool same = true;
      for (uint8_t k = 0; k < L; k++) {
        if (bits[s + k] != bits[s + L + k]) {
          same = false;
          break;
        }
      }
      if (same) {
        *startOut = s;
        return (int8_t)L;
      }
    }
    if (L == 32) break;
  }
  return -1;
}

static void copyBits(uint8_t *dst, const uint8_t *src, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) dst[i] = src[i];
}

static bool sameBits(const uint8_t *a, const uint8_t *b, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

static void printCode(uint8_t seq, const Code *code) {
  Serial.print((unsigned)seq);
  Serial.print(F("  M"));
  Serial.print((unsigned)code->method);
  Serial.print(F("  0x"));

  uint8_t extra = (uint8_t)((4 - (code->bits & 3)) & 3);
  uint8_t acc = 0;
  uint8_t got = extra;
  for (uint8_t i = 0; i < code->bits; i++) {
    acc = (uint8_t)((acc << 1) | bitOf(code, i));
    got++;
    if (got == 4) {
      Serial.print((unsigned)acc, HEX);
      acc = 0;
      got = 0;
    }
  }

  if (code->bits <= 32) {
    uint32_t value = 0;
    for (uint8_t i = 0; i < code->bits; i++) {
      value = (value << 1) | bitOf(code, i);
    }
    Serial.print(' ');
    Serial.print(value);
  }
  Serial.print(F("  "));
  Serial.print((unsigned)code->bits);
  Serial.println(F(" bits"));
}

static bool rxMuted() {
  return (long)(quietUntil - millis()) > 0;
}

static bool sameCode(const Code *a, const Code *b) {
  if (a->method != b->method || a->bits != b->bits) return false;
  uint8_t n = (uint8_t)((a->bits + 7) >> 3);
  for (uint8_t i = 0; i < n; i++) {
    if (a->data[i] != b->data[i]) return false;
  }
  return true;
}

static void rememberCode(const Code *code) {
  if (rxMuted()) return;
  // Our own transmit comes back on the air. Do not let that restart the wait.
  if (repeatSend && queuedValid && sameCode(code, &queued)) return;
  queued = *code;
  queuedTe = lastShortUs;
  queuedValid = true;
  repeatSend = false;
  queueUpdated = true;
  replayArm(&replayB);
}

static uint8_t emitCode(uint8_t seq, uint8_t method, const uint8_t *bits, uint8_t n) {
  Code code;
  packBits(&code, method, bits, n);
  printCode(++seq, &code);
  rememberCode(&code);
  return seq;
}

static void savePending(const uint8_t *bits, uint8_t n, bool rightAlign) {
  pendingN = n;
  pendingRight = rightAlign;
  copyBits(pendingBits, bits, n);
}

// rightAlign: junk is in front (capture started late). Otherwise the
// frame starts after the sync and extra pulses are at the end.
static uint8_t emitFrame(uint8_t seq, const uint8_t *bits, uint8_t n, bool rightAlign) {
  if (n < 32 || n > 80) return seq;
  if (learnedBits) {
    if (n < learnedBits || n > learnedBits + 8) return seq;
    if (rightAlign) bits += n - learnedBits;
    n = learnedBits;
  }
  return emitCode(seq, 2, bits, n);
}

// Print once a second copy confirms the length. Until then, hold the first.
static uint8_t holdUntilMatch(uint8_t seq, const uint8_t *bits, uint8_t n, bool rightAlign) {
  if (n < 32 || n > 80) return seq;

  if (learnedBits && n >= learnedBits && n <= learnedBits + 8) {
    if (pendingN >= learnedBits) {
      seq = emitFrame(seq, pendingBits, pendingN, pendingRight);
    }
    pendingN = 0;
    return emitFrame(seq, bits, n, rightAlign);
  }

  if (pendingN && n == pendingN && sameBits(pendingBits, bits, n)) {
    learnedBits = n;
    seq = emitCode(seq, 2, pendingBits, pendingN);
    pendingN = 0;
    return emitCode(seq, 2, bits, n);
  }

  if (!pendingN || n > pendingN) {
    savePending(bits, n, rightAlign);
  }
  return seq;
}

static uint8_t emitRepeats(uint8_t seq, const uint8_t *bits, uint8_t n) {
  uint8_t start = 0;
  int8_t L = findRepeat(bits, n, &start);
  if (L < 32) return seq;
  learnedBits = (uint8_t)L;
  pendingN = 0;
  uint8_t pos = start;
  while ((uint16_t)pos + (uint8_t)L <= n) {
    if (!sameBits(bits + start, bits + pos, (uint8_t)L)) break;
    seq = emitCode(seq, 2, bits + pos, (uint8_t)L);
    pos += (uint8_t)L;
  }
  return seq;
}

static uint8_t acceptMethod2(uint8_t seq, const uint8_t *bits, uint8_t n, bool bounded) {
  uint8_t before = seq;
  seq = emitRepeats(seq, bits, n);
  if (seq != before) return seq;
  if (n > 80) return seq;
  return holdUntilMatch(seq, bits, n, !bounded);
}

// Shorter half of each real bit. That width is the element time used to resend.
static uint16_t frameTe(uint16_t count) {
  uint32_t sum = 0;
  uint8_t n = 0;
  for (uint16_t i = 0; i + 1 < count && n < 48; i += 2) {
    uint16_t a = pulseTimings[i];
    uint16_t b = pulseTimings[i + 1];
    if (a < 150 || b < 150 || a >= syncUs || b >= syncUs) continue;
    bool one = a > (uint16_t)(b + (b >> 1));
    bool zero = b > (uint16_t)(a + (a >> 1));
    if (one == zero) continue;
    sum += one ? b : a;
    n++;
  }
  if (n < 4) return 380;
  uint16_t te = (uint16_t)(sum / n);
  if (te < 200) te = 200;
  if (te > 700) te = 700;
  return te;
}

static uint8_t decodeRange(uint8_t seq, uint16_t count, bool bounded) {
  if (count < 24) return seq;
  lastShortUs = frameTe(count);
  Code code;
  if (decodeMethod1(0, count, &code)) {
    printCode(++seq, &code);
    rememberCode(&code);
    return seq;
  }
  uint8_t n = collectBest(0, count, workBits, (uint8_t)sizeof(workBits));
  if (n < 32) return seq;
  return acceptMethod2(seq, workBits, n, bounded);
}

static void measureSync() {
  uint16_t sample[40];
  uint8_t n = 0;
  noInterrupts();
  uint16_t count = pulseCount;
  for (uint16_t i = 0; i < count && n < 40; i++) {
    uint16_t us = pulseTimings[i];
    if (us >= 150 && us < 2200) sample[n++] = us;
  }
  interrupts();

  if (n < 8) {
    syncUs = 2200;
    return;
  }
  for (uint8_t i = 1; i < n; i++) {
    uint16_t x = sample[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && sample[j] > x) {
      sample[j + 1] = sample[j];
      j--;
    }
    sample[j + 1] = x;
  }
  uint32_t thr = (uint32_t)sample[n / 2] * 3;
  if (thr < 2200) thr = 2200;
  if (thr > 3000) thr = 3000;
  syncUs = (uint16_t)thr;
}

static uint8_t flushPending(uint8_t seq) {
  // A single short chunk is usually a sliced frame. Matching repeats
  // already printed before they were stored here.
  if (pendingN >= 48) {
    seq = emitFrame(seq, pendingBits, pendingN, pendingRight);
  }
  pendingN = 0;
  return seq;
}

// Print every finished repeat already closed by a sync gap.
static uint8_t flushFrames(uint8_t seq) {
  measureSync();

  while (true) {
    uint16_t count;
    noInterrupts();
    count = pulseCount;
    interrupts();

    int16_t gap = -1;
    for (uint16_t i = 0; i < count; i++) {
      if (isSync(pulseTimings[i])) {
        gap = (int16_t)i;
        break;
      }
    }

    if (gap < 0) {
      // No gap yet. Once two frames could fit, decode them and free the buffer
      // so a long hold does not stop the capture. Tail decodes what arrives after.
      if (count >= 320) {
        seq = decodeRange(seq, count, false);
        consumeFront(count);
      } else if (count >= MAX_PULSES - 8) {
        consumeFront(count / 2);
      }
      return seq;
    }

    // Header (~4 ms) and the 14 ms inter-frame guard both look like sync
    // to the decoder. Stash preamble on the short one; assemble a raw
    // waveform on the long one so TX can replay pulses instead of bits.
    uint16_t gapUs = pulseTimings[gap];
    replayNoteGap(&replayB, (uint16_t)gap, gapUs, captureReadUs, captureReadHigh);
    if (gap >= 24) {
      seq = decodeRange(seq, (uint16_t)gap, opened);
    }
    consumeFront((uint16_t)gap + 1);
    opened = true;
  }
}

// Last repeat has no trailing edge, so its closing gap was never stored.
static uint8_t flushTail(uint8_t seq) {
  uint16_t count;
  noInterrupts();
  count = pulseCount;
  interrupts();

  if (count >= 80) {
    replayNoteGap(&replayB, count, REPLAY_DEFAULT_GUARD_US, captureReadUs, captureReadHigh);
  }
  if (count >= 24) {
    seq = decodeRange(seq, count, opened);
  }
  return flushPending(seq);
}

static void beginSignal() {
  pendingN = 0;
  opened = false;
  syncUs = 2200;
  replayClearPreamble(&replayB);
}

static void waitUs(uint16_t us) {
  while (us > 8000) {
    delayMicroseconds(8000);
    us -= 8000;
  }
  if (us) delayMicroseconds(us);
}

static void sendLevel(bool high, uint16_t us) {
  digitalWrite(CC1101_GDO0, high ? HIGH : LOW);
  waitUs(us);
}

static void sendFrame(const Code *code, uint16_t te) {
  uint16_t longUs = (code->method == 1) ? (uint16_t)(te * 3) : (uint16_t)(te * 2);
  if (code->method != 1) {
    for (uint8_t i = 0; i < 10; i++) {
      sendLevel(true, te);
      sendLevel(false, te);
    }
    sendLevel(false, (uint16_t)(te * 10));
  }
  for (uint8_t i = 0; i < code->bits; i++) {
    bool one = bitOf(code, i);
    sendLevel(true, one ? longUs : te);
    sendLevel(false, one ? te : longUs);
  }
  if (code->method == 1) sendLevel(false, (uint16_t)(te * 31));
}

static void waitMarc(uint8_t want) {
  for (uint8_t i = 0; i < 80; i++) {
    if ((cc1101_readReg(0xF5) & 0x1F) == want) return;
    delay(1);
  }
}

static void sendReplay(const ReplayWave *wave) {
  for (uint8_t n = 0; n < 8; n++) {
    for (uint16_t i = 0; i < wave->count; i++) {
      sendLevel(replayBit(wave->high, i), wave->us[i]);
    }
    sendLevel(false, wave->guardUs);
  }
}

// One button-press worth of the stored frame.
static void transmitQueued() {
  Code code = queued;
  uint16_t te = queuedTe;
  ReplayWave wave = replayB.armed;

  Serial.println(F("[<] Send"));
  printCode(1, &code);
  if (wave.count >= 80) {
    Serial.print(F("Replay "));
    Serial.print(wave.count);
    Serial.print(F(" p  guard "));
    Serial.println(wave.guardUs);
  } else {
    Serial.println(F("Rebuild (no waveform)"));
  }

  capturing = false;
  detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));
  repeatSend = true;

  cc1101_strobe(CC1101_SIDLE);
  // Async TX samples GDO0 at 8x the data rate. RX uses ~3.8 kBaud; bump
  // this so 400 us pulses are not quantized to the modem clock.
  cc1101_writeReg(CC1101_MDMCFG4, 0x8C);
  cc1101_writeReg(CC1101_MDMCFG3, 0xF8);
  cc1101_strobe(CC1101_STX);
  waitMarc(0x13);

  pinMode(CC1101_GDO0, OUTPUT);
  digitalWrite(CC1101_GDO0, LOW);
  if (wave.count >= 80) sendReplay(&wave);
  else for (uint8_t repeat = 0; repeat < 8; repeat++) sendFrame(&code, te);
  digitalWrite(CC1101_GDO0, LOW);

  pinMode(CC1101_GDO0, INPUT);
  cc1101_strobe(CC1101_SIDLE);
  cc1101_writeReg(CC1101_MDMCFG4, 0x87);
  cc1101_writeReg(CC1101_MDMCFG3, 0x32);
  cc1101_strobe(CC1101_SFRX);
  cc1101_strobe(CC1101_SRX);
  waitMarc(0x0D);

  quietUntil = millis() + 500;
  noInterrupts();
  pulseCount = 0;
  lastEdgeTime = 0;
  interrupts();
  attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), handleGDO0Interrupt, CHANGE);
  capturing = true;
  Serial.println(F("Ready\n"));
}

// ─── Reset capture state ───────────────────────────────────────
void resetCapture() {
  pulseCount = 0;
  lastEdgeTime = 0;
  captureReady = false;

  cc1101_strobe(CC1101_SIDLE);
  cc1101_strobe(CC1101_SFRX);
  cc1101_strobe(CC1101_SRX);

  attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), handleGDO0Interrupt, CHANGE);
  capturing = true;
}

// ─── Setup ─────────────────────────────────────────────────────
static void printBanner() {
  Serial.println(F("\n=== CC1101 433MHz Button Code ==="));
  Serial.println(F("M1 EV1527/PT2262   M2 1:2 PWM"));
  Serial.println(F("Hold the remote. Each repeat prints on its own line."));
  Serial.println(F("Press BOOT to resend the last captured code.\n"));
}

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_ARCH_ESP32)
  // Text sent before the monitor attaches is discarded. Wi-Fi also stretches pulses.
  Serial.setTxTimeoutMs(0);
  WiFi.mode(WIFI_OFF);
  unsigned long usbWait = millis() + 2000;
  while (!Serial && (long)(usbWait - millis()) > 0) delay(10);
#endif

  printBanner();
  replayReset(&replayB);

  // Init SPI
  pinMode(CC1101_CS, OUTPUT);
  digitalWrite(CC1101_CS, HIGH);
#if defined(ARDUINO_ARCH_ESP32)
  SPI.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
  SPI.setFrequency(4000000);
#else
  SPI.begin();
#endif

  pinMode(CC1101_GDO0, INPUT);
  pinMode(SEND_BTN, INPUT_PULLUP);

  // Init CC1101
  Serial.print(F("Init CC1101... "));
  cc1101_initRaw433();

  // Verify chip
  uint8_t partnum = cc1101_readReg(0xF0);
  uint8_t version = cc1101_readReg(0xF1);

  if (version == 0x00 || version == 0xFF) {
    Serial.println(F("FAILED!"));
    Serial.print(F("PARTNUM=0x")); Serial.print(partnum, HEX);
    Serial.print(F(" VERSION=0x")); Serial.println(version, HEX);
    Serial.println(F("Radio did not answer. Check 3.3V or module VCC, GND, CSN and MISO."));
    while (1) delay(1000);
  }

  Serial.println(F("OK"));
  Serial.print(F("PARTNUM=0x")); Serial.print(partnum, HEX);
  Serial.print(F(" VERSION=0x")); Serial.println(version, HEX);
  Serial.println(F("\nListening... Press your remote!\n"));

  // Start capturing
  attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), handleGDO0Interrupt, CHANGE);
  capturing = true;
}

// BOOT is active-low with pull-up. One edge per press.
static bool sendButtonDown() {
  static bool lastHigh = true;
  static unsigned long lastChange = 0;
  bool high = digitalRead(SEND_BTN) != LOW;
  unsigned long now = millis();
  if (high == lastHigh) return false;
  if (now - lastChange < 40) return false;
  lastHigh = high;
  lastChange = now;
  return !high;
}

// ─── Main Loop ─────────────────────────────────────────────────
void loop() {
  static unsigned long lastActivity = 0;
  static uint16_t lastPulseCount = 0;
  static bool signalDetected = false;
  static uint8_t seq = 0;

#if defined(ARDUINO_ARCH_ESP32)
  // The banner is lost if the monitor opens after boot. Print it on connect.
  static bool hostSeen = false;
  bool host = (bool)Serial;
  if (host && !hostSeen) {
    hostSeen = true;
    printBanner();
    Serial.println(F("Listening... Press your remote!\n"));
  } else if (!host) {
    hostSeen = false;
  }
#endif

  // Drop the echo of our own transmit so it does not replace the stored code.
  if (rxMuted()) {
    if (pulseCount) {
      noInterrupts();
      pulseCount = 0;
      lastEdgeTime = 0;
      interrupts();
    }
    lastPulseCount = 0;
    return;
  }

  if (!signalDetected && sendButtonDown()) {
    if (queuedValid) {
      transmitQueued();
    } else {
      Serial.println(F("No code stored"));
    }
    return;
  }

  uint16_t count;
  noInterrupts();
  count = pulseCount;
  interrupts();

  if (count > 0 && count != lastPulseCount) {
    lastActivity = millis();
    lastPulseCount = count;
    if (!signalDetected) {
      Serial.println(F("[>] Signal"));
      signalDetected = true;
      seq = 0;
      beginSignal();
    }
    seq = flushFrames(seq);
    noInterrupts();
    lastPulseCount = pulseCount;
    interrupts();
  }

  // Button released: no new edges for 60 ms.
  if (signalDetected && (millis() - lastActivity > 60)) {
    capturing = false;
    detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));

    seq = flushFrames(seq);
    seq = flushTail(seq);
    if (seq == 0) {
      Serial.println(F("No code"));
    } else if (queueUpdated) {
      Serial.println(F("Press BOOT to send"));
      queueUpdated = false;
    }

    lastPulseCount = 0;
    signalDetected = false;
    resetCapture();
    Serial.println(F("Ready\n"));
  }
}
