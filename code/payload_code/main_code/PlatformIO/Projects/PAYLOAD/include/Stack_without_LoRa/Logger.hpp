#pragma once

// =============================================================================
//  Logger.hpp  -  Stack_without_LoRa
//
//  Row sink.  Every sensor row is pushed as finished text into a RAM ring
//  and written to the SD card in large chunks by service():
//
//    push() -> ring (48 kB) -> SD file (open for the whole run)
//                                 write  every 4 kB or 250 ms
//                                 fsync  every 1 s
//                                 remount + new part file after any error
//
//  LittleFS in the ESP32's own flash is a FALLBACK: it only receives rows
//  while the SD is unusable (at reduced rates), plus every EVT row, so a
//  card that dies mid-flight still leaves a trace of the flight.
//
//  One file per run, all record types mixed, first column = record type.
//  tools/split_log.py splits it into one CSV per sensor.
// =============================================================================

#include "Config.hpp"
#include "RowFmt.hpp"

enum class RowType : uint8_t { IMU, BARO, MAG, GEIG, RATE, TEMP, STAT, EVT };

class Logger {
public:
  // Mount SD + LittleFS, open this run's files and write the header lines.
  void begin(uint32_t run);

  // Rotate to a new run id (new files on both media).
  void newRun(uint32_t run);

  // Queue one finished row (text incl. trailing '\n').
  void push(RowType type, const Row& row);

  // EVT row: "EVT,t_us,text" to every medium, never decimated.
  void event(const char* text);

  // Move data from the ring to the card / flash.  Call every loop.
  void service(uint32_t nowMs);

  // Force pending data to the card (fsync).  Used by `sync` / `tail`.
  void flushNow();

  // ---- Serial helpers ------------------------------------------------------
  void listSd();
  void tailSd(size_t lines);
  void headSd();
  void dumpFlash();
  void wipeFlash();

  // ---- Stats for STAT rows / `status` -------------------------------------
  bool     sdOk()        const { return sdOk_; }
  bool     flashOk()     const { return flashOk_; }
  uint32_t sdBytes()     const { return sdBytes_; }
  uint32_t sdFails()     const { return sdFails_; }
  uint32_t ringDrops()   const { return ringDrops_; }
  uint32_t ringHighWater() const { return ringHigh_; }
  uint32_t flashBytes()  const { return flashBytes_; }
  uint16_t sdPart()      const { return sdPart_; }
  const char* sdPath()   const { return sdPath_; }

private:
  // Ring buffer of row text destined for the SD card.
  static uint8_t ring_[LOG_RING_B];
  size_t   head_ = 0, tail_ = 0, count_ = 0;
  uint32_t ringDrops_ = 0, ringHigh_ = 0;
  bool     ringPush_(const char* p, size_t n);
  size_t   ringPeek_(const uint8_t*& p) const;      // Longest contiguous run at tail.
  void     ringPop_(size_t n);

  // SD.
  bool     sdOk_ = false;
  File     sdFile_;
  char     sdPath_[48] = {0};
  uint16_t sdPart_ = 0;
  uint32_t sdBytes_ = 0, sdFails_ = 0;
  uint32_t lastSdWriteMs_ = 0, lastSdFlushMs_ = 0, lastSdMountMs_ = 0;
  bool     sdDirty_ = false;
  bool     sdMount_();
  bool     sdOpenPart_();
  void     sdFail_(const char* why);
  void     sdWrite_(uint32_t nowMs);

  // LittleFS fallback.
  bool     flashOk_ = false;
  char     flashPath_[48] = {0};
  uint32_t flashBytes_ = 0;
  static uint8_t flashRing_[FLASH_RING_B];
  size_t   flashCount_ = 0;
  uint32_t lastFlashWriteMs_ = 0;
  uint16_t flashDiv_[8] = {0};
  bool     flashAccept_(RowType t);
  void     flashPush_(const char* p, size_t n);
  void     flashWrite_(uint32_t nowMs);
  void     flashOpenRun_();
  void     flashDeleteRecursive_(const char* path);
  void     flashPrintRecursive_(const char* path);

  // Header lines written at the top of every file.
  void     writeHeader_(File& f);

  uint32_t run_ = 0;
};
