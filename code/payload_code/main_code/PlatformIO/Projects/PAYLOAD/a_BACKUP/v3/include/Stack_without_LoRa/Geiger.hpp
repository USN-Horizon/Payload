#pragma once

// =============================================================================
//  Geiger.hpp  -  Stack_without_LoRa
//
//  Two Geiger inputs.  Every pulse gets a microsecond timestamp inside the
//  ISR and is queued in a per-channel ring buffer (single producer = the
//  ISR, single consumer = loop()), so the log can carry one row per pulse.
//  Totals are also kept for the 1 Hz RATE summary.
// =============================================================================

#include "Config.hpp"

struct GeigerEvent {
  uint64_t t_us;
  uint8_t  channel;   // 1 or 2
};

class Geiger {
public:
  // Configure the two inputs (INPUT_PULLUP, RISING edge = end of the LOW
  // pulse, same edge Stack_with_LoRa counts) and attach the ISRs.
  void begin();

  // Pop the oldest pending pulse from either channel, oldest first.
  // Returns false when nothing is pending.
  bool popEvent(GeigerEvent& out);

  // Take the counts accumulated since the previous call (and reset them).
  void takeInterval(uint32_t& ch1, uint32_t& ch2);

  uint32_t total1() const;
  uint32_t total2() const;
  uint32_t dropped() const { return dropped_; }   // Ring overflows (pulses not time-stamped).

  // ISR entry points (public so main.cpp can hand them to attachInterrupt).
  static void IRAM_ATTR isrCh1();
  static void IRAM_ATTR isrCh2();

private:
  struct Ring {
    volatile uint64_t t[GEIGER_EVENT_RING];
    volatile uint16_t head = 0;   // Written by ISR.
    volatile uint16_t tail = 0;   // Written by loop().
    volatile uint32_t total = 0;
    volatile uint32_t interval = 0;
    volatile uint32_t dropped = 0;
  };
  static Ring ring1_;
  static Ring ring2_;
  static void IRAM_ATTR push_(Ring& r);
  bool pop_(Ring& r, uint64_t& t);
  uint32_t dropped_ = 0;
};
