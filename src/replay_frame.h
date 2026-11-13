#pragma once

#include <stdint.h>
#include <string.h>

// One captured on-air frame: preamble + ~4 ms header + PWM data.
#if defined(ARDUINO_ARCH_AVR)
#define MAX_REPLAY 160
#define MAX_PREAMBLE 24
#else
#define MAX_REPLAY 240
#define MAX_PREAMBLE 40
#endif

// Header is ~10 Te (~4 ms). The real gap between repeats is ~14 ms.
#define REPLAY_GUARD_MIN_US 8000
#define REPLAY_DEFAULT_GUARD_US 14160

struct ReplayWave {
  uint16_t us[MAX_REPLAY];
  uint8_t high[(MAX_REPLAY + 7) / 8];
  uint16_t count;
  uint16_t guardUs;
};

typedef uint16_t (*ReplayReadUs)(uint16_t i);
typedef bool (*ReplayReadHigh)(uint16_t i);

struct ReplayBuilder {
  uint16_t preUs[MAX_PREAMBLE];
  uint8_t preHigh[(MAX_PREAMBLE + 7) / 8];
  uint16_t preCount;
  uint16_t headerUs;
  ReplayWave staged;
  ReplayWave armed;
};

static bool replayBit(const uint8_t *bits, uint16_t i) {
  return (bits[i >> 3] & (uint8_t)(1 << (i & 7))) != 0;
}

static void replaySetBit(uint8_t *bits, uint16_t i, bool high) {
  uint8_t mask = (uint8_t)(1 << (i & 7));
  if (high) bits[i >> 3] |= mask;
  else bits[i >> 3] &= (uint8_t)~mask;
}

static void replayReset(ReplayBuilder *b) {
  memset(b, 0, sizeof(*b));
  b->staged.guardUs = REPLAY_DEFAULT_GUARD_US;
  b->armed.guardUs = REPLAY_DEFAULT_GUARD_US;
}

static void replayClearPreamble(ReplayBuilder *b) {
  b->preCount = 0;
  b->headerUs = 0;
}

static void replayArm(ReplayBuilder *b) {
  if (b->staged.count >= 24) {
    b->armed = b->staged;
  }
}

// n = pulses before this gap. Header-sized gaps stash preamble;
// long gaps assemble preamble + header + data into staged.
static void replayNoteGap(ReplayBuilder *b, uint16_t n, uint16_t gapUs,
                          ReplayReadUs readUs, ReplayReadHigh readHigh) {
  if (gapUs < REPLAY_GUARD_MIN_US) {
    if (n > 0 && n <= MAX_PREAMBLE) {
      memset(b->preHigh, 0, sizeof(b->preHigh));
      for (uint16_t i = 0; i < n; i++) {
        b->preUs[i] = readUs(i);
        replaySetBit(b->preHigh, i, readHigh(i));
      }
      b->preCount = n;
      b->headerUs = gapUs;
    }
    return;
  }

  uint16_t header = b->headerUs;
  uint16_t need = (uint16_t)(b->preCount + (header ? 1 : 0) + n);
  if (n < 80 || need > MAX_REPLAY) return;
  if (b->staged.count >= 80 && n < 80) return;

  uint16_t o = 0;
  memset(b->staged.high, 0, sizeof(b->staged.high));
  for (uint16_t i = 0; i < b->preCount; i++) {
    b->staged.us[o] = b->preUs[i];
    replaySetBit(b->staged.high, o, replayBit(b->preHigh, i));
    o++;
  }
  if (header) {
    b->staged.us[o] = header;
    replaySetBit(b->staged.high, o, false);
    o++;
  }
  for (uint16_t i = 0; i < n; i++) {
    b->staged.us[o] = readUs(i);
    replaySetBit(b->staged.high, o, readHigh(i));
    o++;
  }
  b->staged.count = o;
  b->staged.guardUs = gapUs;
}
