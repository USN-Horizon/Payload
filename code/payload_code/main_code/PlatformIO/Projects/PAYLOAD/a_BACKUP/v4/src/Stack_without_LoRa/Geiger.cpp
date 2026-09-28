#include "Stack_without_LoRa/Geiger.hpp"

Geiger::Ring Geiger::ring1_;
Geiger::Ring Geiger::ring2_;

void IRAM_ATTR Geiger::push_(Ring& r) {
  r.total++;
  r.interval++;
  uint16_t h = r.head;
  uint16_t next = static_cast<uint16_t>((h + 1) % GEIGER_EVENT_RING);
  if (next == r.tail) {          // Ring full: keep the count, drop the timestamp.
    r.dropped++;
    return;
  }
  r.t[h] = static_cast<uint64_t>(esp_timer_get_time());
  r.head = next;                 // Publish after the data is in place.
}

void IRAM_ATTR Geiger::isrCh1() { push_(ring1_); }
void IRAM_ATTR Geiger::isrCh2() { push_(ring2_); }

void Geiger::begin() {
#if USE_GEIGER
  pinMode(PIN_GEIGER_1, INPUT_PULLUP);
  pinMode(PIN_GEIGER_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_GEIGER_1), isrCh1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_GEIGER_2), isrCh2, RISING);
#endif
}

bool Geiger::pop_(Ring& r, uint64_t& t) {
  uint16_t tl = r.tail;
  if (tl == r.head) return false;
  t = r.t[tl];
  r.tail = static_cast<uint16_t>((tl + 1) % GEIGER_EVENT_RING);
  return true;
}

bool Geiger::popEvent(GeigerEvent& out) {
  // Peek both rings and pop whichever holds the older pulse so the log is
  // in time order across channels.
  bool has1 = ring1_.tail != ring1_.head;
  bool has2 = ring2_.tail != ring2_.head;
  if (!has1 && !has2) return false;
  uint64_t t1 = has1 ? ring1_.t[ring1_.tail] : UINT64_MAX;
  uint64_t t2 = has2 ? ring2_.t[ring2_.tail] : UINT64_MAX;
  if (t1 <= t2) { pop_(ring1_, out.t_us); out.channel = 1; }
  else          { pop_(ring2_, out.t_us); out.channel = 2; }
  return true;
}

void Geiger::takeInterval(uint32_t& ch1, uint32_t& ch2) {
  noInterrupts();
  ch1 = ring1_.interval; ring1_.interval = 0;
  ch2 = ring2_.interval; ring2_.interval = 0;
  dropped_ = ring1_.dropped + ring2_.dropped;
  interrupts();
}

uint32_t Geiger::total1() const { return ring1_.total; }
uint32_t Geiger::total2() const { return ring2_.total; }
