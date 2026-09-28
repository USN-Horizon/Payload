// gs_decode_sim.cpp - PC simulation of the ground-station receive chain.
//
// Replays what happens between the payload's LoRa packets and the Qt ground
// station (Nidhoggr) WITHOUT touching either:
//
//   payload frames -> MavlinkPacker (fixed-length packets)
//     -> adapter: `uint8_t received[128]; radio->receive(received,128);
//                  Serial.write(received,128);`  (forwards 128 bytes always)
//     -> Nidhoggr: mavlink_frame_char() on every byte, switch(msgid)
//
// It prints every decoded message the way packetparser.cpp would see it and
// flags STALE frames (a frame decoded again out of the adapter's leftover
// bytes).  Run:
//
//   g++ -std=c++17 -O1 -Wno-address-of-packed-member \
//       -I include/mavlink/library -I include/Stack_with_LoRa \
//       tools/gs_decode_sim.cpp -o /tmp/gs_decode_sim && /tmp/gs_decode_sim
//
// Exit code 0 = no stale frames in the fixed-length scheme.
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <set>

#include "HorizonDialect/mavlink.h"
#include "Packer.hpp"

static const uint8_t SYSID = 42, COMPID = 191;

struct Frame { std::vector<uint8_t> bytes; const char* name; };

static Frame mk(const char* name, mavlink_message_t& msg) {
  Frame f; f.name = name;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
  f.bytes.assign(buf, buf + n);
  return f;
}

// One flight cycle worth of frames, in the order App::runTxCycle_ queues them.
static std::vector<Frame> cycleFrames(uint32_t t, bool withHeartbeat, int variant) {
  std::vector<Frame> v;
  mavlink_message_t msg;
  mavlink_msg_scaled_imu_pack(SYSID, COMPID, &msg, t, -20 + variant, -436, -889, 0, 0, 0, 0, 0, 0, 0);
  v.push_back(mk("SCALED_IMU(acc)", msg));
  mavlink_msg_scaled_imu2_pack(SYSID, COMPID, &msg, t, 0, 0, 0, 8, 1 + variant, -4, 0, 0, 0, 0);
  v.push_back(mk("SCALED_IMU2(gyro)", msg));
  mavlink_msg_scaled_imu3_pack(SYSID, COMPID, &msg, t, 0, 0, 0, 0, 0, 0, 556, 154 + variant, 70, 0);
  v.push_back(mk("SCALED_IMU3(mag)", msg));
  mavlink_msg_scaled_pressure_pack(SYSID, COMPID, &msg, t, 997.67f + variant, 0.0f, 2672, 0);
  v.push_back(mk("SCALED_PRESSURE", msg));
  mavlink_msg_cosmic_radiation_pack(SYSID, COMPID, &msg, (uint64_t)t * 1000ULL, 3 + variant);
  v.push_back(mk("COSMIC_RADIATION", msg));
  mavlink_msg_vfr_hud_pack(SYSID, COMPID, &msg, 0.0f, 0.5f * variant, 0, 0, 0.0f, 0.0f);
  v.push_back(mk("VFR_HUD(vspeed)", msg));
  if (withHeartbeat) {
    mavlink_msg_heartbeat_pack(SYSID, COMPID, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0 | (3u << 8), MAV_STATE_ACTIVE);
    v.push_back(mk("HEARTBEAT", msg));
  }
  return v;
}

// ---- the receive chain --------------------------------------------------------
struct Chain {
  uint8_t received[128];          // adapter buffer: NOT cleared between packets
  mavlink_message_t msg;
  mavlink_status_t status;
  std::set<std::pair<uint32_t, uint8_t>> seen;   // (msgid, seq) already decoded
  int decoded = 0, stale = 0, badcrc = 0;
  Chain() { memset(received, 0xA5, sizeof(received)); memset(&msg, 0, sizeof(msg)); memset(&status, 0, sizeof(status)); }

  // pkt/len: the packet as it came off the air (len = real LoRa packet length).
  // A frame is STALE when it starts at or beyond `len`, i.e. it was decoded
  // out of the adapter's leftover bytes, not out of this packet.
  void onPacket(const uint8_t* pkt, size_t len, const char* tag) {
    size_t n = len > 128 ? 128 : len;
    memcpy(received, pkt, n);                       // radio->receive(received, 128)
    printf("  air: %-8s %3zu bytes -> adapter forwards 128\n", tag, len);
    for (int i = 0; i < 128; ++i) {                  // Serial.write(received, 128) -> parse()
      uint8_t res = mavlink_frame_char(MAVLINK_COMM_0, received[i], &msg, &status);
      if (res == MAVLINK_FRAMING_OK) {
        decoded++;
        int frameLen = MAVLINK_NUM_NON_PAYLOAD_BYTES + msg.len +
                       ((msg.incompat_flags & MAVLINK_IFLAG_SIGNED) ? MAVLINK_SIGNATURE_BLOCK_LEN : 0);
        int start = i + 1 - frameLen;
        bool dup = start >= static_cast<int>(n);
        if (dup) stale++;
        describe(dup, i);
      } else if (res == MAVLINK_FRAMING_BAD_CRC) {
        badcrc++;
        printf("    !! bad CRC at byte %d\n", i);
      }
    }
  }

  void describe(bool dup, int at) {
    printf("    %s@%3d seq=%3u ", dup ? "STALE " : "decode", at, msg.seq);
    switch (msg.msgid) {
      case MAVLINK_MSG_ID_SCALED_IMU: { mavlink_scaled_imu_t d; mavlink_msg_scaled_imu_decode(&msg, &d);
        printf("SCALED_IMU  acc=%d,%d,%d\n", d.xacc, d.yacc, d.zacc); break; }
      case MAVLINK_MSG_ID_SCALED_IMU2: { mavlink_scaled_imu_t d; mavlink_msg_scaled_imu_decode(&msg, &d);
        printf("SCALED_IMU2 gyro=%d,%d,%d\n", d.xgyro, d.ygyro, d.zgyro); break; }
      case MAVLINK_MSG_ID_SCALED_IMU3: { mavlink_scaled_imu3_t d; mavlink_msg_scaled_imu3_decode(&msg, &d);
        printf("SCALED_IMU3 mag=%d,%d,%d\n", d.xmag, d.ymag, d.zmag); break; }
      case MAVLINK_MSG_ID_SCALED_PRESSURE: { mavlink_scaled_pressure_t d; mavlink_msg_scaled_pressure_decode(&msg, &d);
        printf("SCALED_PRESSURE press=%.2f temp=%d\n", d.press_abs, d.temperature); break; }
      case MAVLINK_MSG_ID_COSMIC_RADIATION: { mavlink_cosmic_radiation_t d; mavlink_msg_cosmic_radiation_decode(&msg, &d);
        printf("COSMIC_RADIATION radiation=%u\n", d.radiation); break; }
      case MAVLINK_MSG_ID_VFR_HUD: { mavlink_vfr_hud_t d; mavlink_msg_vfr_hud_decode(&msg, &d);
        printf("VFR_HUD velocity=%.2f\n", d.groundspeed); break; }
      case MAVLINK_MSG_ID_HEARTBEAT: { mavlink_heartbeat_t d; mavlink_msg_heartbeat_decode(&msg, &d);
        printf("HEARTBEAT status=%u custom_mode=0x%lx\n", d.system_status, (unsigned long)d.custom_mode); break; }
      default: printf("msgid %u (ignored by GS)\n", (unsigned)msg.msgid);
    }
  }
};

static Chain* g_chain = nullptr;
static const char* g_tag = "";
static size_t g_totalAir = 0;
static void emit(void*, const uint8_t* pkt, size_t len) { g_totalAir += len; g_chain->onPacket(pkt, len, g_tag); }

// LoRa time on air, SF11 / BW 812.5 kHz / CR 4/8 / explicit header / 12-symbol preamble.
static double toaMs(size_t bytes) {
  const double tsym = 2048.0 / 812.5;   // ms
  double nPayload = 8 + 8 * ((8.0 * bytes - 4 * 11 + 28 + 16) / (4.0 * 11) > 0 ?
                            (int)((8.0 * bytes - 4 * 11 + 28 + 16 + 43) / (4.0 * 11)) : 0);
  return (12.25 + nPayload) * tsym;
}

int main(int argc, char** argv) {
  // --stream : same records on stdin, decoded as they arrive (used by
  // tools/gs_capture.py --decode for a live view of what the GS would show).
  if (argc == 2 && strcmp(argv[1], "--stream") == 0) {
    setvbuf(stdout, nullptr, _IOLBF, 0);
    Chain chain; g_chain = &chain;
    uint8_t rec[129];
    int n = 0;
    while (fread(rec, 1, 129, stdin) == 129) {
      char tag[16]; snprintf(tag, sizeof(tag), "pkt%d", n++);
      memcpy(chain.received, rec + 1, 128);
      chain.onPacket(chain.received, rec[0], tag);
      fflush(stdout);
    }
    printf("stream end: %d packets, decoded=%d stale=%d badcrc=%d\n", n, chain.decoded, chain.stale, chain.badcrc);
    return 0;
  }

  // --capture FILE : FILE holds records of [1 byte packet length][128-byte
  // adapter buffer] recorded from the real radio by tools/gs_capture.py (or
  // built by tools/hex_to_capture.py); decode them like Nidhoggr.
  if (argc == 3 && strcmp(argv[1], "--capture") == 0) {
    FILE* f = fopen(argv[2], "rb");
    if (!f) { perror(argv[2]); return 2; }
    Chain chain; g_chain = &chain;
    uint8_t rec[129];
    int n = 0;
    while (fread(rec, 1, 129, f) == 129) {
      char tag[16]; snprintf(tag, sizeof(tag), "pkt%d", n++);
      // The blob already is the adapter buffer (packet + leftovers): feed it
      // as-is, but tell the chain the real packet length for stale detection.
      memcpy(chain.received, rec + 1, 128);
      chain.onPacket(chain.received, rec[0], tag);
    }
    fclose(f);
    printf("capture: %d packets, decoded=%d stale=%d badcrc=%d\n", n, chain.decoded, chain.stale, chain.badcrc);
    return chain.stale == 0 ? 0 : 1;
  }

  printf("=== fixed-length packets (new scheme), %zu bytes each ===\n", (size_t)64);
  Chain chain; g_chain = &chain;
  MavlinkPacker<128> packer; packer.configure(64, emit, nullptr);
  size_t bytesOfData = 0;
  for (int cycle = 0; cycle < 4; ++cycle) {
    char tag[16]; snprintf(tag, sizeof(tag), "cyc%d", cycle); g_tag = tag;
    auto frames = cycleFrames(1000 + cycle * 2000, cycle % 2 == 0, cycle);
    for (auto& f : frames) { bytesOfData += f.bytes.size(); packer.add(f.bytes.data(), f.bytes.size()); }
    packer.flush();
  }
  printf("decoded=%d stale=%d badcrc=%d  packets=%u  air bytes=%zu for %zu frame bytes\n",
         chain.decoded, chain.stale, chain.badcrc, packer.packetsSent(), g_totalAir, bytesOfData);
  printf("time on air per full cycle (3 x 64 B): %.0f ms;  old scheme (10 frames of 21-36 B): ~%.0f ms\n",
         3 * toaMs(64), toaMs(21) * 3 + toaMs(22) + toaMs(28) + toaMs(33) + toaMs(20) + toaMs(30) * 3);

  printf("\n=== variable-length multi-frame packets (what NOT to do) ===\n");
  Chain chain2; g_chain = &chain2;
  MavlinkPacker<128> vp; vp.configure(128, emit, nullptr);
  // a long packet followed by a short one, no padding: emulate by emitting raw.
  {
    auto frames = cycleFrames(5000, true, 0);
    std::vector<uint8_t> big;
    for (auto& f : frames) big.insert(big.end(), f.bytes.begin(), f.bytes.end());
    g_tag = "long"; chain2.onPacket(big.data(), big.size() > 128 ? 128 : big.size(), g_tag);
    mavlink_message_t msg; mavlink_msg_heartbeat_pack(SYSID, COMPID, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    Frame hb = mk("HB", msg);
    g_tag = "short"; chain2.onPacket(hb.bytes.data(), hb.bytes.size(), g_tag);
  }
  printf("decoded=%d STALE=%d  <- stale frames re-decoded from the adapter's leftover bytes\n",
         chain2.decoded, chain2.stale);

  return chain.stale == 0 && chain.badcrc == 0 ? 0 : 1;
}
