#pragma once

// =============================================================================
//  MavlinkFrames.hpp  -  Stack_with_LoRa
//
//  One builder per MAVLink message the ground station decodes.  Each builder
//  fills `out` (at least LORA_FRAME_MAX bytes) with one MAVLink 2 wire frame
//  and returns its length; App packs several frames into one LoRa packet
//  (see Packer.hpp).
//
//  What the ground station (Avionics-main, Nidhoggr packetparser.cpp) reads,
//  and therefore what we send:
//    Accel xyz     -> SCALED_IMU        (msg 26)  xacc/yacc/zacc  [mg]
//    Gyro xyz      -> SCALED_IMU2       (msg 116) xgyro/ygyro/zgyro [mrad/s]
//    Mag xyz       -> SCALED_IMU3       (msg 129) xmag/ymag/zmag  [mgauss]
//    Baro          -> SCALED_PRESSURE   (msg 29)  press_abs [hPa] + temperature [cdegC]
//    Geiger        -> COSMIC_RADIATION  (msg 16002, HorizonDialect) radiation = CRD1+CRD2
//    Vert. speed   -> VFR_HUD           (msg 74)  groundspeed (the GS "velocity" plot)
//    Heartbeat     -> HEARTBEAT         (msg 0)   custom_mode = thermal level | phase << 8
//  The other fields of these messages are zero and MAVLink 2 trims trailing
//  zero bytes, which keeps the frames short (22..34 bytes).
// =============================================================================

#include "Config.hpp"
#include "Sensors.hpp"

#if USE_LORA
namespace MavlinkFrames {

  // Input  : out - destination buffer; state - MAV_STATE (ACTIVE in flight,
  //          STANDBY on the pad, CRITICAL/EMERGENCY when throttled, POWEROFF
  //          at the stop); customMode - thermal level | flight phase << 8.
  // What   : packs a HEARTBEAT message identifying this node.
  // Output : number of bytes written.
  size_t buildHeartbeat(uint8_t* out, uint8_t state = MAV_STATE_ACTIVE,
                        uint32_t customMode = 0);

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
  // What   : packs SCALED_PRESSURE with press_abs (hPa) and temperature
  //          (cdegC); the GS derives its altitude from press_abs.
  // Output : number of bytes written.
  size_t buildBaroPressure(uint8_t* out, const BaroReading& r);

  // Input  : out, r - the GeigerReading to encode.
  // What   : packs COSMIC_RADIATION with counts1 + counts2 (clamped to
  //          uint16) and the reading time in microseconds.
  // Output : number of bytes written.
  size_t buildCosmicRadiation(uint8_t* out, const GeigerReading& r);

  // Input  : out, verticalSpeed - m/s from the barometer (+ up).
  // What   : packs VFR_HUD with only groundspeed set (20 bytes on the wire).
  //          The GS plots VFR_HUD.groundspeed as "velocity" and has no other
  //          speed channel; the payload has no GPS, so it carries the
  //          barometric vertical speed.
  // Output : number of bytes written.
  size_t buildVfrHud(uint8_t* out, float verticalSpeed);
}
#endif
