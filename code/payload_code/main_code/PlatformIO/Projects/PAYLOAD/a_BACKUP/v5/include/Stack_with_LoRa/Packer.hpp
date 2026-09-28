#pragma once

// =============================================================================
//  Packer.hpp  -  Stack_with_LoRa
//
//  Packs several MAVLink frames into one fixed-length LoRa packet.
//
//  Why fixed length: the ground-station adapter (Ratatoskr-GS-adapter-firmware
//  main.cpp) does `radio->receive(buf, 128); Serial.write(buf, 128);` - it
//  always forwards 128 bytes, whatever the packet length was, and the Qt
//  ground station feeds every byte into mavlink_frame_char().  With packets
//  of varying length the bytes behind a short packet are leftovers of the
//  previous longer one, and any complete frame sitting there is decoded
//  AGAIN as fresh data.  Sending every packet with the same length (zero
//  padded) makes the leftover region constant garbage that never parses.
//
//  Why several frames per packet: every LoRa packet costs a preamble +
//  header (~50 ms at SF11) and every MAVLink frame costs 12 bytes of
//  header/CRC, so one message per packet spent ~75 % of the air time on
//  overhead and empty fields.  Packing cuts the air time per cycle by more
//  than half while carrying more data.
//
//  Plain C++ (no Arduino) so tools/gs_decode_sim.cpp can compile it on the PC.
// =============================================================================

#include <stdint.h>
#include <stddef.h>
#include <string.h>

template <size_t MAX_PACKET>
class MavlinkPacker {
public:
  typedef void (*EmitFn)(void* ctx, const uint8_t* packet, size_t len);

  MavlinkPacker() {}

  // packetLen: every emitted packet has exactly this length (<= MAX_PACKET).
  void configure(size_t packetLen, EmitFn emit, void* ctx) {
    packetLen_ = packetLen > MAX_PACKET ? MAX_PACKET : packetLen;
    emit_ = emit;
    ctx_  = ctx;
    len_  = 0;
    frames_ = 0;
  }

  // Input  : frame, n - one MAVLink wire frame.
  // What   : appends it; if it does not fit next to the pending frames the
  //          pending packet goes out first.  A frame longer than the packet
  //          length is refused.
  // Output : true if the frame was accepted.
  bool add(const uint8_t* frame, size_t n) {
    if (n == 0 || n > packetLen_) return false;
    if (len_ + n > packetLen_) flush();
    memcpy(buf_ + len_, frame, n);
    len_ += n;
    frames_++;
    return true;
  }

  // Input  : (none)
  // What   : zero-pads the pending frames to packetLen and emits the packet.
  //          Nothing happens when no frame is pending.
  // Output : (none)
  void flush() {
    if (len_ == 0) return;
    memset(buf_ + len_, 0, packetLen_ - len_);
    if (emit_) emit_(ctx_, buf_, packetLen_);
    packets_++;
    len_ = 0;
    frames_ = 0;
  }

  size_t   pending()     const { return len_; }
  size_t   packetLen()   const { return packetLen_; }
  uint32_t packetsSent() const { return packets_; }

private:
  uint8_t  buf_[MAX_PACKET];
  size_t   packetLen_ = MAX_PACKET;
  size_t   len_       = 0;
  uint8_t  frames_    = 0;
  uint32_t packets_   = 0;
  EmitFn   emit_      = nullptr;
  void*    ctx_       = nullptr;
};
