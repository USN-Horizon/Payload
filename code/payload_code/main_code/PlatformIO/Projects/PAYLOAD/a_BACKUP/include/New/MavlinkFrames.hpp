#pragma once

// =============================================================================
//  MavlinkFrames.hpp
//  Builds one self-contained MAVLink wire frame per sensor reading. Each
//  builder fills `out` (must be at least LORA_FRAME_MAX bytes) and returns
//  the number of bytes written. The output is exactly the byte stream that
//  goes on the LoRa air interface.
//
//  Mapping of sensor → MAVLink message (one packet per LoRa transmission):
//    Heartbeat     -> HEARTBEAT          (msg id 0,     common dialect)
//    Accel xyz     -> SCALED_IMU         (msg id 26,    common), acc fields.
//    Gyro xyz      -> SCALED_IMU2        (msg id 116,   common), gyro fields.
//    Mag xyz       -> SCALED_IMU3        (msg id 129,   common), mag fields.
//    Baro pressure -> SCALED_PRESSURE    (msg id 29,    common), press_abs.
//    Baro temp     -> NAMED_VALUE_INT    (msg id 252,   common), "BARO_T".
//    Geiger total  -> COSMIC_RADIATION   (msg id 16002, HorizonDialect):
//                                          counts1 + counts2, clamped uint16.
//    LoRa temp     -> NAMED_VALUE_INT    name "LORA_T", cdegC.
//    LoRa volt     -> NAMED_VALUE_INT    name "LORA_V", millivolts.
// =============================================================================

#include "Config.hpp"
#include "Sensors.hpp"

#if USE_LORA
namespace MavlinkFrames {

  // Input  : out - destination buffer of at least LORA_FRAME_MAX bytes.
  //          state - MAV_STATE for the heartbeat: ACTIVE in flight (default),
  //          STANDBY while waiting on the pad.
  // What   : packs a HEARTBEAT message that identifies this node.
  // Output : number of bytes written to `out`.
  size_t buildHeartbeat(uint8_t* out, uint8_t state = MAV_STATE_ACTIVE);

  // Input  : out, r - the AccelReading to encode.
  // What   : packs SCALED_IMU with xacc/yacc/zacc filled, others zero.
  // Output : number of bytes written.
  size_t buildAccel(uint8_t* out, const AccelReading& r);

  // Input  : out, r - the GyroReading to encode.
  // What   : packs SCALED_IMU2 with xgyro/ygyro/zgyro filled, others zero.
  // Output : number of bytes written.
  size_t buildGyro(uint8_t* out, const GyroReading& r);

  // Input  : out, r - the MagReading to encode.
  // What   : packs SCALED_IMU3 with xmag/ymag/zmag filled, others zero.
  // Output : number of bytes written.
  size_t buildMag(uint8_t* out, const MagReading& r);

  // Input  : out, r - the BaroReading to encode.
  // What   : packs SCALED_PRESSURE carrying press_abs (hPa).
  // Output : number of bytes written.
  size_t buildBaroPressure(uint8_t* out, const BaroReading& r);

  // Input  : out, r - the BaroReading to encode.
  // What   : packs NAMED_VALUE_INT "BARO_T" with the cdegC temperature.
  // Output : number of bytes written.
  size_t buildBaroTemp(uint8_t* out, const BaroReading& r);

  // Input  : out, r - the GeigerReading to encode.
  // What   : packs the HorizonDialect COSMIC_RADIATION message carrying the
  //          combined Geiger count (counts1 + counts2) for the interval,
  //          clamped to uint16_t. The reading's t_ms is converted to usec.
  // Output : number of bytes written.
  size_t buildCosmicRadiation(uint8_t* out, const GeigerReading& r);

  // Input  : out, r - the LoraTempReading.
  // What   : packs NAMED_VALUE_INT "LORA_T" with the cdegC temperature.
  // Output : number of bytes written.
  size_t buildLoraTemp(uint8_t* out, const LoraTempReading& r);

  // Input  : out, r - the LoraVoltReading.
  // What   : packs NAMED_VALUE_INT "LORA_V" with the millivolts.
  // Output : number of bytes written.
  size_t buildLoraVolt(uint8_t* out, const LoraVoltReading& r);
}
#endif
