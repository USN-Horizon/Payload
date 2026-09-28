#pragma once

// =============================================================================
//  RowFmt.hpp  -  Stack_without_LoRa
//
//  Tiny fixed-buffer CSV row builder.  printf("%f") on the ESP32 costs tens
//  of microseconds per value; at 400 IMU rows/s with 6 values each that adds
//  up, so every number is formatted with integer arithmetic instead.
//  Values with decimals are passed pre-scaled (e.g. mg*100) and printed as
//  fixed-point.
// =============================================================================

#include <stdint.h>
#include <stddef.h>

class Row {
public:
  static constexpr size_t CAP = 224;

  const char* data() const { return b_; }
  size_t      size() const { return n_; }
  bool        overflowed() const { return ovf_; }

  void reset() { n_ = 0; ovf_ = false; }

  // First column: the record type tag.
  void tag(const char* t) { reset(); str(t); }

  void put(char c) {
    if (n_ < CAP - 1) b_[n_++] = c; else ovf_ = true;
  }
  void str(const char* s) { while (*s) put(*s++); }
  void sep() { put(','); }
  void endl() { put('\n'); }

  void u64(uint64_t v) {
    char t[21]; int i = 0;
    do { t[i++] = static_cast<char>('0' + (v % 10)); v /= 10; } while (v);
    while (i) put(t[--i]);
  }
  void i64(int64_t v) {
    if (v < 0) { put('-'); u64(static_cast<uint64_t>(-(v + 1)) + 1u); }
    else       { u64(static_cast<uint64_t>(v)); }
  }
  void u32(uint32_t v) { u64(v); }
  void i32(int32_t v)  { i64(v); }

  // Prints scaled / 10^decimals with exactly `decimals` digits after the
  // point, e.g. fixed(-123456, 2) -> "-1234.56".
  void fixed(int64_t scaled, uint8_t decimals) {
    if (scaled < 0) { put('-'); scaled = -scaled; }
    uint64_t p = 1;
    for (uint8_t i = 0; i < decimals; ++i) p *= 10;
    uint64_t v = static_cast<uint64_t>(scaled);
    u64(v / p);
    if (decimals == 0) return;
    put('.');
    uint64_t frac = v % p;
    // Leading zeros in the fraction.
    for (uint64_t q = p / 10; q > 0; q /= 10) {
      put(static_cast<char>('0' + (frac / q) % 10));
    }
  }

  // Convenience: column separators before values.
  void cu64(uint64_t v)                    { sep(); u64(v); }
  void ci32(int32_t v)                     { sep(); i32(v); }
  void cu32(uint32_t v)                    { sep(); u32(v); }
  void cfixed(int64_t s, uint8_t d)        { sep(); fixed(s, d); }
  void cstr(const char* s)                 { sep(); str(s); }
  void cempty()                            { sep(); }

private:
  char   b_[CAP];
  size_t n_   = 0;
  bool   ovf_ = false;
};
