#pragma once

// =============================================================================
//  Thermal.hpp  -  Stack_with_LoRa
//
//  Thermal protection for the LoRa board.  The SX1280 (DS_SX1280 §3.2) and
//  the SKY65383-11 front-end are both rated for an operating temperature of
//  -40 .. +85 degC (junction max 125 degC).  The TMP1075 sits on the board
//  next to the module, so the dies run hotter than the reading; the level
//  table in Config.hpp keeps the READING well below 85 degC by cutting the
//  transmit duty cycle (and finally the PA bias) while the payload keeps
//  sending.  Logging to SD / flash is never throttled.
//
//  Levels are entered when the temperature reaches a level's `on_cC` and
//  left again only LORA_THERMAL_HYST_cC below it, so the radio does not
//  flap between levels.
// =============================================================================

#include "Config.hpp"

class LoraThermal {
public:
  // Input  : t_cC  - TMP1075 reading in centi-degC.
  // What   : re-evaluates the level with hysteresis.
  // Output : true if the level changed.
  bool update(int16_t t_cC) {
    lastTemp_ = t_cC;
    uint8_t lvl = level_;
    // Go up as far as the temperature says (can jump several levels).
    while (lvl + 1 < LORA_THERMAL_LEVEL_COUNT && t_cC >= LORA_THERMAL_LEVELS[lvl + 1].on_cC) lvl++;
    // Come down one level at a time, only when clearly below this level's entry.
    while (lvl > 0 && t_cC < LORA_THERMAL_LEVELS[lvl].on_cC - LORA_THERMAL_HYST_cC) lvl--;
    if (forced_ >= 0) lvl = static_cast<uint8_t>(forced_);
    bool changed = (lvl != level_);
    level_ = lvl;
    return changed;
  }

  // Bench aid: pin the level (-1 = automatic).
  void force(int8_t lvl) {
    forced_ = (lvl < 0 || lvl >= static_cast<int8_t>(LORA_THERMAL_LEVEL_COUNT)) ? -1 : lvl;
    if (forced_ >= 0) level_ = static_cast<uint8_t>(forced_);
  }
  bool forced() const { return forced_ >= 0; }

  uint8_t  level()         const { return level_; }
  int16_t  lastTemp()      const { return lastTemp_; }
  uint8_t  txIntervalTenths() const { return LORA_THERMAL_LEVELS[level_].txIntervalTenths; }
  bool     essentialOnly()    const { return LORA_THERMAL_LEVELS[level_].essentialOnly; }
  uint16_t dacMv()            const { return LORA_THERMAL_LEVELS[level_].dacMv; }
  bool     txAllowed()        const { return txIntervalTenths() != 0; }
  // Input  : baseMs - the interval to stretch (TX cycle or pad beacon).
  // Output : that interval scaled by the current level (0 when TX is off).
  uint32_t txIntervalMs(uint32_t baseMs) const {
    return baseMs * txIntervalTenths() / 10u;
  }

private:
  uint8_t level_    = 0;
  int8_t  forced_   = -1;
  int16_t lastTemp_ = 0;
};
