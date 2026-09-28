#include "Stack_with_LoRa/MavlinkFrames.hpp"

#if USE_LORA

// =============================================================================
//  MavlinkFrames.cpp  -  Stack_with_LoRa
//  Each builder packs one MAVLink message and serializes it (MAVLink 2 wire
//  format, trailing zero bytes trimmed) into the caller's buffer.
// =============================================================================

namespace {

size_t serialize(const mavlink_message_t& msg, uint8_t* out) {
  return mavlink_msg_to_send_buffer(out, &msg);
}

}  // namespace

namespace MavlinkFrames {

size_t buildHeartbeat(uint8_t* out, uint8_t state, uint32_t customMode) {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(MAV_SYSID, MAV_COMPID, &msg,
                             MAV_TYPE_GENERIC,
                             MAV_AUTOPILOT_INVALID,
                             0, customMode, state);
  return serialize(msg, out);
}

size_t buildAccel(uint8_t* out, const AccelReading& r) {
  mavlink_message_t msg;
  // The GS reads only xacc/yacc/zacc from SCALED_IMU; the rest stays zero
  // and is trimmed off the wire (22-byte frame).
  mavlink_msg_scaled_imu_pack(MAV_SYSID, MAV_COMPID, &msg,
                              r.t_ms,
                              r.x_mg, r.y_mg, r.z_mg,
                              0, 0, 0,
                              0, 0, 0,
                              0);
  return serialize(msg, out);
}

size_t buildGyro(uint8_t* out, const GyroReading& r) {
  mavlink_message_t msg;
  // The GS reads only the gyro fields from SCALED_IMU2 (28-byte frame).
  mavlink_msg_scaled_imu2_pack(MAV_SYSID, MAV_COMPID, &msg,
                               r.t_ms,
                               0, 0, 0,
                               r.x_mrad_s, r.y_mrad_s, r.z_mrad_s,
                               0, 0, 0,
                               0);
  return serialize(msg, out);
}

size_t buildMag(uint8_t* out, const MagReading& r) {
  mavlink_message_t msg;
  // The GS reads only the mag fields from SCALED_IMU3 (34-byte frame).
  mavlink_msg_scaled_imu3_pack(MAV_SYSID, MAV_COMPID, &msg,
                               r.t_ms,
                               0, 0, 0,
                               0, 0, 0,
                               r.x_mgauss, r.y_mgauss, r.z_mgauss,
                               0);
  return serialize(msg, out);
}

size_t buildBaroPressure(uint8_t* out, const BaroReading& r) {
  mavlink_message_t msg;
  // press_abs (hPa) + temperature (cdegC).  The GS emits pressure,
  // temperature and a derived altitude from this one message, so the
  // temperature must be here (sending 0 made its temperature plot flip).
  mavlink_msg_scaled_pressure_pack(MAV_SYSID, MAV_COMPID, &msg,
                                   r.t_ms,
                                   r.pressure_hPa, 0.0f,
                                   r.temperature_cC, 0);
  return serialize(msg, out);
}

size_t buildCosmicRadiation(uint8_t* out, const GeigerReading& r) {
  mavlink_message_t msg;
  uint32_t total = r.counts1 + r.counts2;
  if (total > 0xFFFFu) total = 0xFFFFu;
  mavlink_msg_cosmic_radiation_pack(MAV_SYSID, MAV_COMPID, &msg,
                                    static_cast<uint64_t>(r.t_ms) * 1000ULL,
                                    static_cast<uint16_t>(total));
  return serialize(msg, out);
}

size_t buildVfrHud(uint8_t* out, float verticalSpeed) {
  mavlink_message_t msg;
  // Only groundspeed is set: MAVLink 2 zero-trimming then drops alt/climb/
  // heading/throttle from the wire (payload 8 bytes -> 20-byte frame).
  mavlink_msg_vfr_hud_pack(MAV_SYSID, MAV_COMPID, &msg,
                           0.0f, verticalSpeed, 0, 0, 0.0f, 0.0f);
  return serialize(msg, out);
}

}  // namespace MavlinkFrames

#endif  // USE_LORA
