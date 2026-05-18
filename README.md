# Payload

```text
payload/
├── code
│   └── payload_code
│       └── main_code
│           └── PlatformIO
│               └── Projects
│                   └── PAYLOAD
│                       ├── include
│                       │   ├── link
│                       │   │   └── mavlink
│                       │   │       └── v2.0
│                       │   │           ├── checksum.h
│                       │   │           ├── common
│                       │   │           │   ├── common.h
│                       │   │           │   ├── mavlink.h
│                       │   │           │   ├── mavlink_msg_actuator_control_target.h
│                       │   │           │   ├── mavlink_msg_actuator_output_status.h
│                       │   │           │   ├── mavlink_msg_adsb_vehicle.h
│                       │   │           │   ├── mavlink_msg_airspeed.h
│                       │   │           │   ├── mavlink_msg_ais_vessel.h
│                       │   │           │   ├── mavlink_msg_altitude.h
│                       │   │           │   ├── mavlink_msg_attitude.h
│                       │   │           │   ├── mavlink_msg_attitude_quaternion_cov.h
│                       │   │           │   ├── mavlink_msg_attitude_quaternion.h
│                       │   │           │   ├── mavlink_msg_attitude_target.h
│                       │   │           │   ├── mavlink_msg_att_pos_mocap.h
│                       │   │           │   ├── mavlink_msg_auth_key.h
│                       │   │           │   ├── mavlink_msg_autopilot_state_for_gimbal_device.h
│                       │   │           │   ├── mavlink_msg_available_modes.h
│                       │   │           │   ├── mavlink_msg_available_modes_monitor.h
│                       │   │           │   ├── mavlink_msg_battery_info.h
│                       │   │           │   ├── mavlink_msg_battery_status.h
│                       │   │           │   ├── mavlink_msg_button_change.h
│                       │   │           │   ├── mavlink_msg_camera_capture_status.h
│                       │   │           │   ├── mavlink_msg_camera_fov_status.h
│                       │   │           │   ├── mavlink_msg_camera_image_captured.h
│                       │   │           │   ├── mavlink_msg_camera_information.h
│                       │   │           │   ├── mavlink_msg_camera_settings.h
│                       │   │           │   ├── mavlink_msg_camera_thermal_range.h
│                       │   │           │   ├── mavlink_msg_camera_tracking_geo_status.h
│                       │   │           │   ├── mavlink_msg_camera_tracking_image_status.h
│                       │   │           │   ├── mavlink_msg_camera_trigger.h
│                       │   │           │   ├── mavlink_msg_canfd_frame.h
│                       │   │           │   ├── mavlink_msg_can_filter_modify.h
│                       │   │           │   ├── mavlink_msg_can_frame.h
│                       │   │           │   ├── mavlink_msg_cellular_config.h
│                       │   │           │   ├── mavlink_msg_cellular_status.h
│                       │   │           │   ├── mavlink_msg_change_operator_control_ack.h
│                       │   │           │   ├── mavlink_msg_change_operator_control.h
│                       │   │           │   ├── mavlink_msg_collision.h
│                       │   │           │   ├── mavlink_msg_command_ack.h
│                       │   │           │   ├── mavlink_msg_command_cancel.h
│                       │   │           │   ├── mavlink_msg_command_int.h
│                       │   │           │   ├── mavlink_msg_command_long.h
│                       │   │           │   ├── mavlink_msg_component_information_basic.h
│                       │   │           │   ├── mavlink_msg_component_information.h
│                       │   │           │   ├── mavlink_msg_component_metadata.h
│                       │   │           │   ├── mavlink_msg_control_system_state.h
│                       │   │           │   ├── mavlink_msg_current_event_sequence.h
│                       │   │           │   ├── mavlink_msg_current_mode.h
│                       │   │           │   ├── mavlink_msg_data_stream.h
│                       │   │           │   ├── mavlink_msg_data_transmission_handshake.h
│                       │   │           │   ├── mavlink_msg_debug_float_array.h
│                       │   │           │   ├── mavlink_msg_debug.h
│                       │   │           │   ├── mavlink_msg_debug_vect.h
│                       │   │           │   ├── mavlink_msg_distance_sensor.h
│                       │   │           │   ├── mavlink_msg_efi_status.h
│                       │   │           │   ├── mavlink_msg_encapsulated_data.h
│                       │   │           │   ├── mavlink_msg_esc_info.h
│                       │   │           │   ├── mavlink_msg_esc_status.h
│                       │   │           │   ├── mavlink_msg_estimator_status.h
│                       │   │           │   ├── mavlink_msg_event.h
│                       │   │           │   ├── mavlink_msg_extended_sys_state.h
│                       │   │           │   ├── mavlink_msg_fence_status.h
│                       │   │           │   ├── mavlink_msg_figure_eight_execution_status.h
│                       │   │           │   ├── mavlink_msg_file_transfer_protocol.h
│                       │   │           │   ├── mavlink_msg_flight_information.h
│                       │   │           │   ├── mavlink_msg_follow_target.h
│                       │   │           │   ├── mavlink_msg_fuel_status.h
│                       │   │           │   ├── mavlink_msg_generator_status.h
│                       │   │           │   ├── mavlink_msg_gimbal_device_attitude_status.h
│                       │   │           │   ├── mavlink_msg_gimbal_device_information.h
│                       │   │           │   ├── mavlink_msg_gimbal_device_set_attitude.h
│                       │   │           │   ├── mavlink_msg_gimbal_manager_information.h
│                       │   │           │   ├── mavlink_msg_gimbal_manager_set_attitude.h
│                       │   │           │   ├── mavlink_msg_gimbal_manager_set_manual_control.h
│                       │   │           │   ├── mavlink_msg_gimbal_manager_set_pitchyaw.h
│                       │   │           │   ├── mavlink_msg_gimbal_manager_status.h
│                       │   │           │   ├── mavlink_msg_global_position_int_cov.h
│                       │   │           │   ├── mavlink_msg_global_position_sensor.h
│                       │   │           │   ├── mavlink_msg_global_vision_position_estimate.h
│                       │   │           │   ├── mavlink_msg_gps2_raw.h
│                       │   │           │   ├── mavlink_msg_gps2_rtk.h
│                       │   │           │   ├── mavlink_msg_gps_global_origin.h
│                       │   │           │   ├── mavlink_msg_gps_inject_data.h
│                       │   │           │   ├── mavlink_msg_gps_input.h
│                       │   │           │   ├── mavlink_msg_gps_raw_int.h
│                       │   │           │   ├── mavlink_msg_gps_rtcm_data.h
│                       │   │           │   ├── mavlink_msg_gps_rtk.h
│                       │   │           │   ├── mavlink_msg_gps_status.h
│                       │   │           │   ├── mavlink_msg_high_latency2.h
│                       │   │           │   ├── mavlink_msg_high_latency.h
│                       │   │           │   ├── mavlink_msg_highres_imu.h
│                       │   │           │   ├── mavlink_msg_hil_actuator_controls.h
│                       │   │           │   ├── mavlink_msg_hil_controls.h
│                       │   │           │   ├── mavlink_msg_hil_gps.h
│                       │   │           │   ├── mavlink_msg_hil_optical_flow.h
│                       │   │           │   ├── mavlink_msg_hil_rc_inputs_raw.h
│                       │   │           │   ├── mavlink_msg_hil_sensor.h
│                       │   │           │   ├── mavlink_msg_hil_state.h
│                       │   │           │   ├── mavlink_msg_hil_state_quaternion.h
│                       │   │           │   ├── mavlink_msg_home_position.h
│                       │   │           │   ├── mavlink_msg_hygrometer_sensor.h
│                       │   │           │   ├── mavlink_msg_illuminator_status.h
│                       │   │           │   ├── mavlink_msg_isbd_link_status.h
│                       │   │           │   ├── mavlink_msg_landing_target.h
│                       │   │           │   ├── mavlink_msg_link_node_status.h
│                       │   │           │   ├── mavlink_msg_local_position_ned_cov.h
│                       │   │           │   ├── mavlink_msg_local_position_ned.h
│                       │   │           │   ├── mavlink_msg_local_position_ned_system_global_offset.h
│                       │   │           │   ├── mavlink_msg_log_data.h
│                       │   │           │   ├── mavlink_msg_log_entry.h
│                       │   │           │   ├── mavlink_msg_log_erase.h
│                       │   │           │   ├── mavlink_msg_logging_ack.h
│                       │   │           │   ├── mavlink_msg_logging_data_acked.h
│                       │   │           │   ├── mavlink_msg_logging_data.h
│                       │   │           │   ├── mavlink_msg_log_request_data.h
│                       │   │           │   ├── mavlink_msg_log_request_end.h
│                       │   │           │   ├── mavlink_msg_log_request_list.h
│                       │   │           │   ├── mavlink_msg_mag_cal_report.h
│                       │   │           │   ├── mavlink_msg_manual_control.h
│                       │   │           │   ├── mavlink_msg_manual_setpoint.h
│                       │   │           │   ├── mavlink_msg_memory_vect.h
│                       │   │           │   ├── mavlink_msg_message_interval.h
│                       │   │           │   ├── mavlink_msg_mission_ack.h
│                       │   │           │   ├── mavlink_msg_mission_clear_all.h
│                       │   │           │   ├── mavlink_msg_mission_count.h
│                       │   │           │   ├── mavlink_msg_mission_current.h
│                       │   │           │   ├── mavlink_msg_mission_item.h
│                       │   │           │   ├── mavlink_msg_mission_item_int.h
│                       │   │           │   ├── mavlink_msg_mission_item_reached.h
│                       │   │           │   ├── mavlink_msg_mission_request.h
│                       │   │           │   ├── mavlink_msg_mission_request_int.h
│                       │   │           │   ├── mavlink_msg_mission_request_list.h
│                       │   │           │   ├── mavlink_msg_mission_request_partial_list.h
│                       │   │           │   ├── mavlink_msg_mission_set_current.h
│                       │   │           │   ├── mavlink_msg_mission_write_partial_list.h
│                       │   │           │   ├── mavlink_msg_mount_orientation.h
│                       │   │           │   ├── mavlink_msg_named_value_float.h
│                       │   │           │   ├── mavlink_msg_named_value_int.h
│                       │   │           │   ├── mavlink_msg_nav_controller_output.h
│                       │   │           │   ├── mavlink_msg_obstacle_distance.h
│                       │   │           │   ├── mavlink_msg_odometry.h
│                       │   │           │   ├── mavlink_msg_onboard_computer_status.h
│                       │   │           │   ├── mavlink_msg_open_drone_id_arm_status.h
│                       │   │           │   ├── mavlink_msg_open_drone_id_authentication.h
│                       │   │           │   ├── mavlink_msg_open_drone_id_basic_id.h
│                       │   │           │   ├── mavlink_msg_open_drone_id_location.h
│                       │   │           │   ├── mavlink_msg_open_drone_id_message_pack.h
│                       │   │           │   ├── mavlink_msg_open_drone_id_operator_id.h
│                       │   │           │   ├── mavlink_msg_open_drone_id_self_id.h
│                       │   │           │   ├── mavlink_msg_open_drone_id_system.h
│                       │   │           │   ├── mavlink_msg_open_drone_id_system_update.h
│                       │   │           │   ├── mavlink_msg_optical_flow.h
│                       │   │           │   ├── mavlink_msg_optical_flow_rad.h
│                       │   │           │   ├── mavlink_msg_orbit_execution_status.h
│                       │   │           │   ├── mavlink_msg_param_error.h
│                       │   │           │   ├── mavlink_msg_param_ext_ack.h
│                       │   │           │   ├── mavlink_msg_param_ext_request_list.h
│                       │   │           │   ├── mavlink_msg_param_ext_request_read.h
│                       │   │           │   ├── mavlink_msg_param_ext_set.h
│                       │   │           │   ├── mavlink_msg_param_ext_value.h
│                       │   │           │   ├── mavlink_msg_param_map_rc.h
│                       │   │           │   ├── mavlink_msg_param_request_list.h
│                       │   │           │   ├── mavlink_msg_param_request_read.h
│                       │   │           │   ├── mavlink_msg_param_set.h
│                       │   │           │   ├── mavlink_msg_param_value.h
│                       │   │           │   ├── mavlink_msg_ping.h
│                       │   │           │   ├── mavlink_msg_play_tune.h
│                       │   │           │   ├── mavlink_msg_play_tune_v2.h
│                       │   │           │   ├── mavlink_msg_position_target_global_int.h
│                       │   │           │   ├── mavlink_msg_position_target_local_ned.h
│                       │   │           │   ├── mavlink_msg_power_status.h
│                       │   │           │   ├── mavlink_msg_protocol_version.h
│                       │   │           │   ├── mavlink_msg_radio_status.h
│                       │   │           │   ├── mavlink_msg_raw_imu.h
│                       │   │           │   ├── mavlink_msg_raw_pressure.h
│                       │   │           │   ├── mavlink_msg_raw_rpm.h
│                       │   │           │   ├── mavlink_msg_rc_channels.h
│                       │   │           │   ├── mavlink_msg_rc_channels_override.h
│                       │   │           │   ├── mavlink_msg_rc_channels_raw.h
│                       │   │           │   ├── mavlink_msg_rc_channels_scaled.h
│                       │   │           │   ├── mavlink_msg_relay_status.h
│                       │   │           │   ├── mavlink_msg_request_data_stream.h
│                       │   │           │   ├── mavlink_msg_request_event.h
│                       │   │           │   ├── mavlink_msg_resource_request.h
│                       │   │           │   ├── mavlink_msg_response_event_error.h
│                       │   │           │   ├── mavlink_msg_safety_allowed_area.h
│                       │   │           │   ├── mavlink_msg_safety_set_allowed_area.h
│                       │   │           │   ├── mavlink_msg_scaled_imu2.h
│                       │   │           │   ├── mavlink_msg_scaled_imu3.h
│                       │   │           │   ├── mavlink_msg_scaled_imu.h
│                       │   │           │   ├── mavlink_msg_scaled_pressure2.h
│                       │   │           │   ├── mavlink_msg_scaled_pressure3.h
│                       │   │           │   ├── mavlink_msg_scaled_pressure.h
│                       │   │           │   ├── mavlink_msg_serial_control.h
│                       │   │           │   ├── mavlink_msg_servo_output_raw.h
│                       │   │           │   ├── mavlink_msg_set_actuator_control_target.h
│                       │   │           │   ├── mavlink_msg_set_attitude_target.h
│                       │   │           │   ├── mavlink_msg_set_gps_global_origin.h
│                       │   │           │   ├── mavlink_msg_set_home_position.h
│                       │   │           │   ├── mavlink_msg_set_mode.h
│                       │   │           │   ├── mavlink_msg_set_position_target_global_int.h
│                       │   │           │   ├── mavlink_msg_set_position_target_local_ned.h
│                       │   │           │   ├── mavlink_msg_setup_signing.h
│                       │   │           │   ├── mavlink_msg_sim_state.h
│                       │   │           │   ├── mavlink_msg_smart_battery_info.h
│                       │   │           │   ├── mavlink_msg_statustext.h
│                       │   │           │   ├── mavlink_msg_storage_information.h
│                       │   │           │   ├── mavlink_msg_supported_tunes.h
│                       │   │           │   ├── mavlink_msg_sys_status.h
│                       │   │           │   ├── mavlink_msg_system_time.h
│                       │   │           │   ├── mavlink_msg_terrain_check.h
│                       │   │           │   ├── mavlink_msg_terrain_data.h
│                       │   │           │   ├── mavlink_msg_terrain_report.h
│                       │   │           │   ├── mavlink_msg_terrain_request.h
│                       │   │           │   ├── mavlink_msg_time_estimate_to_target.h
│                       │   │           │   ├── mavlink_msg_timesync.h
│                       │   │           │   ├── mavlink_msg_trajectory_representation_bezier.h
│                       │   │           │   ├── mavlink_msg_trajectory_representation_waypoints.h
│                       │   │           │   ├── mavlink_msg_tunnel.h
│                       │   │           │   ├── mavlink_msg_uavcan_node_info.h
│                       │   │           │   ├── mavlink_msg_uavcan_node_status.h
│                       │   │           │   ├── mavlink_msg_utm_global_position.h
│                       │   │           │   ├── mavlink_msg_v2_extension.h
│                       │   │           │   ├── mavlink_msg_vfr_hud.h
│                       │   │           │   ├── mavlink_msg_vibration.h
│                       │   │           │   ├── mavlink_msg_vicon_position_estimate.h
│                       │   │           │   ├── mavlink_msg_video_stream_information.h
│                       │   │           │   ├── mavlink_msg_video_stream_status.h
│                       │   │           │   ├── mavlink_msg_vision_position_estimate.h
│                       │   │           │   ├── mavlink_msg_vision_speed_estimate.h
│                       │   │           │   ├── mavlink_msg_wheel_distance.h
│                       │   │           │   ├── mavlink_msg_wifi_config_ap.h
│                       │   │           │   ├── mavlink_msg_winch_status.h
│                       │   │           │   ├── mavlink_msg_wind_cov.h
│                       │   │           │   ├── testsuite.h
│                       │   │           │   └── version.h
│                       │   │           ├── mavlink_conversions.h
│                       │   │           ├── mavlink_get_info.h
│                       │   │           ├── mavlink_helpers.h
│                       │   │           ├── mavlink_sha256.h
│                       │   │           ├── mavlink_types.h
│                       │   │           ├── minimal
│                       │   │           │   ├── mavlink.h
│                       │   │           │   ├── mavlink_msg_heartbeat.h
│                       │   │           │   ├── minimal.h
│                       │   │           │   ├── testsuite.h
│                       │   │           │   └── version.h
│                       │   │           ├── protocol.h
│                       │   │           └── standard
│                       │   │               ├── mavlink.h
│                       │   │               ├── mavlink_msg_autopilot_version.h
│                       │   │               ├── mavlink_msg_global_position_int.h
│                       │   │               ├── standard.h
│                       │   │               ├── testsuite.h
│                       │   │               └── version.h
│                       │   ├── mavlink
│                       │   │   ├── CMakeLists.txt
│                       │   │   ├── component_metadata
│                       │   │   │   ├── actuators.example.json
│                       │   │   │   ├── actuators.schema.json
│                       │   │   │   ├── actuators.translation.json
│                       │   │   │   ├── general.schema.json
│                       │   │   │   ├── parameter.schema.json
│                       │   │   │   ├── parameter.translation.json
│                       │   │   │   ├── peripherals.schema.json
│                       │   │   │   └── translation.schema.json
│                       │   │   ├── CONTRIBUTING.md
│                       │   │   ├── COPYING
│                       │   │   ├── doc
│                       │   │   │   ├── Doxyfile
│                       │   │   │   ├── mavlink.css
│                       │   │   │   ├── mavlink.php
│                       │   │   │   ├── mavlink_to_html_table.xsl
│                       │   │   │   ├── mavlink_xml_to_markdown.py
│                       │   │   │   ├── README.md
│                       │   │   │   └── requirements.txt
│                       │   │   ├── examples
│                       │   │   │   └── c
│                       │   │   │       ├── CMakeLists.txt
│                       │   │   │       ├── README.md
│                       │   │   │       └── udp_example.c
│                       │   │   ├── library
│                       │   │   │   ├── checksum.h
│                       │   │   │   ├── common
│                       │   │   │   │   ├── common.h
│                       │   │   │   │   ├── common.hpp
│                       │   │   │   │   ├── gtestsuite.hpp
│                       │   │   │   │   ├── mavlink.h
│                       │   │   │   │   ├── mavlink_msg_actuator_control_target.h
│                       │   │   │   │   ├── mavlink_msg_actuator_control_target.hpp
│                       │   │   │   │   ├── mavlink_msg_actuator_output_status.h
│                       │   │   │   │   ├── mavlink_msg_actuator_output_status.hpp
│                       │   │   │   │   ├── mavlink_msg_adsb_vehicle.h
│                       │   │   │   │   ├── mavlink_msg_adsb_vehicle.hpp
│                       │   │   │   │   ├── mavlink_msg_airspeed.h
│                       │   │   │   │   ├── mavlink_msg_airspeed.hpp
│                       │   │   │   │   ├── mavlink_msg_ais_vessel.h
│                       │   │   │   │   ├── mavlink_msg_ais_vessel.hpp
│                       │   │   │   │   ├── mavlink_msg_altitude.h
│                       │   │   │   │   ├── mavlink_msg_altitude.hpp
│                       │   │   │   │   ├── mavlink_msg_attitude.h
│                       │   │   │   │   ├── mavlink_msg_attitude.hpp
│                       │   │   │   │   ├── mavlink_msg_attitude_quaternion_cov.h
│                       │   │   │   │   ├── mavlink_msg_attitude_quaternion_cov.hpp
│                       │   │   │   │   ├── mavlink_msg_attitude_quaternion.h
│                       │   │   │   │   ├── mavlink_msg_attitude_quaternion.hpp
│                       │   │   │   │   ├── mavlink_msg_attitude_target.h
│                       │   │   │   │   ├── mavlink_msg_attitude_target.hpp
│                       │   │   │   │   ├── mavlink_msg_att_pos_mocap.h
│                       │   │   │   │   ├── mavlink_msg_att_pos_mocap.hpp
│                       │   │   │   │   ├── mavlink_msg_auth_key.h
│                       │   │   │   │   ├── mavlink_msg_auth_key.hpp
│                       │   │   │   │   ├── mavlink_msg_autopilot_state_for_gimbal_device.h
│                       │   │   │   │   ├── mavlink_msg_autopilot_state_for_gimbal_device.hpp
│                       │   │   │   │   ├── mavlink_msg_available_modes.h
│                       │   │   │   │   ├── mavlink_msg_available_modes.hpp
│                       │   │   │   │   ├── mavlink_msg_available_modes_monitor.h
│                       │   │   │   │   ├── mavlink_msg_available_modes_monitor.hpp
│                       │   │   │   │   ├── mavlink_msg_battery_info.h
│                       │   │   │   │   ├── mavlink_msg_battery_info.hpp
│                       │   │   │   │   ├── mavlink_msg_battery_status.h
│                       │   │   │   │   ├── mavlink_msg_battery_status.hpp
│                       │   │   │   │   ├── mavlink_msg_button_change.h
│                       │   │   │   │   ├── mavlink_msg_button_change.hpp
│                       │   │   │   │   ├── mavlink_msg_camera_capture_status.h
│                       │   │   │   │   ├── mavlink_msg_camera_capture_status.hpp
│                       │   │   │   │   ├── mavlink_msg_camera_fov_status.h
│                       │   │   │   │   ├── mavlink_msg_camera_fov_status.hpp
│                       │   │   │   │   ├── mavlink_msg_camera_image_captured.h
│                       │   │   │   │   ├── mavlink_msg_camera_image_captured.hpp
│                       │   │   │   │   ├── mavlink_msg_camera_information.h
│                       │   │   │   │   ├── mavlink_msg_camera_information.hpp
│                       │   │   │   │   ├── mavlink_msg_camera_settings.h
│                       │   │   │   │   ├── mavlink_msg_camera_settings.hpp
│                       │   │   │   │   ├── mavlink_msg_camera_thermal_range.h
│                       │   │   │   │   ├── mavlink_msg_camera_thermal_range.hpp
│                       │   │   │   │   ├── mavlink_msg_camera_tracking_geo_status.h
│                       │   │   │   │   ├── mavlink_msg_camera_tracking_geo_status.hpp
│                       │   │   │   │   ├── mavlink_msg_camera_tracking_image_status.h
│                       │   │   │   │   ├── mavlink_msg_camera_tracking_image_status.hpp
│                       │   │   │   │   ├── mavlink_msg_camera_trigger.h
│                       │   │   │   │   ├── mavlink_msg_camera_trigger.hpp
│                       │   │   │   │   ├── mavlink_msg_canfd_frame.h
│                       │   │   │   │   ├── mavlink_msg_canfd_frame.hpp
│                       │   │   │   │   ├── mavlink_msg_can_filter_modify.h
│                       │   │   │   │   ├── mavlink_msg_can_filter_modify.hpp
│                       │   │   │   │   ├── mavlink_msg_can_frame.h
│                       │   │   │   │   ├── mavlink_msg_can_frame.hpp
│                       │   │   │   │   ├── mavlink_msg_cellular_config.h
│                       │   │   │   │   ├── mavlink_msg_cellular_config.hpp
│                       │   │   │   │   ├── mavlink_msg_cellular_status.h
│                       │   │   │   │   ├── mavlink_msg_cellular_status.hpp
│                       │   │   │   │   ├── mavlink_msg_change_operator_control_ack.h
│                       │   │   │   │   ├── mavlink_msg_change_operator_control_ack.hpp
│                       │   │   │   │   ├── mavlink_msg_change_operator_control.h
│                       │   │   │   │   ├── mavlink_msg_change_operator_control.hpp
│                       │   │   │   │   ├── mavlink_msg_collision.h
│                       │   │   │   │   ├── mavlink_msg_collision.hpp
│                       │   │   │   │   ├── mavlink_msg_command_ack.h
│                       │   │   │   │   ├── mavlink_msg_command_ack.hpp
│                       │   │   │   │   ├── mavlink_msg_command_cancel.h
│                       │   │   │   │   ├── mavlink_msg_command_cancel.hpp
│                       │   │   │   │   ├── mavlink_msg_command_int.h
│                       │   │   │   │   ├── mavlink_msg_command_int.hpp
│                       │   │   │   │   ├── mavlink_msg_command_long.h
│                       │   │   │   │   ├── mavlink_msg_command_long.hpp
│                       │   │   │   │   ├── mavlink_msg_component_information_basic.h
│                       │   │   │   │   ├── mavlink_msg_component_information_basic.hpp
│                       │   │   │   │   ├── mavlink_msg_component_information.h
│                       │   │   │   │   ├── mavlink_msg_component_information.hpp
│                       │   │   │   │   ├── mavlink_msg_component_metadata.h
│                       │   │   │   │   ├── mavlink_msg_component_metadata.hpp
│                       │   │   │   │   ├── mavlink_msg_control_system_state.h
│                       │   │   │   │   ├── mavlink_msg_control_system_state.hpp
│                       │   │   │   │   ├── mavlink_msg_current_event_sequence.h
│                       │   │   │   │   ├── mavlink_msg_current_event_sequence.hpp
│                       │   │   │   │   ├── mavlink_msg_current_mode.h
│                       │   │   │   │   ├── mavlink_msg_current_mode.hpp
│                       │   │   │   │   ├── mavlink_msg_data_stream.h
│                       │   │   │   │   ├── mavlink_msg_data_stream.hpp
│                       │   │   │   │   ├── mavlink_msg_data_transmission_handshake.h
│                       │   │   │   │   ├── mavlink_msg_data_transmission_handshake.hpp
│                       │   │   │   │   ├── mavlink_msg_debug_float_array.h
│                       │   │   │   │   ├── mavlink_msg_debug_float_array.hpp
│                       │   │   │   │   ├── mavlink_msg_debug.h
│                       │   │   │   │   ├── mavlink_msg_debug.hpp
│                       │   │   │   │   ├── mavlink_msg_debug_vect.h
│                       │   │   │   │   ├── mavlink_msg_debug_vect.hpp
│                       │   │   │   │   ├── mavlink_msg_distance_sensor.h
│                       │   │   │   │   ├── mavlink_msg_distance_sensor.hpp
│                       │   │   │   │   ├── mavlink_msg_efi_status.h
│                       │   │   │   │   ├── mavlink_msg_efi_status.hpp
│                       │   │   │   │   ├── mavlink_msg_encapsulated_data.h
│                       │   │   │   │   ├── mavlink_msg_encapsulated_data.hpp
│                       │   │   │   │   ├── mavlink_msg_esc_info.h
│                       │   │   │   │   ├── mavlink_msg_esc_info.hpp
│                       │   │   │   │   ├── mavlink_msg_esc_status.h
│                       │   │   │   │   ├── mavlink_msg_esc_status.hpp
│                       │   │   │   │   ├── mavlink_msg_estimator_status.h
│                       │   │   │   │   ├── mavlink_msg_estimator_status.hpp
│                       │   │   │   │   ├── mavlink_msg_event.h
│                       │   │   │   │   ├── mavlink_msg_event.hpp
│                       │   │   │   │   ├── mavlink_msg_extended_sys_state.h
│                       │   │   │   │   ├── mavlink_msg_extended_sys_state.hpp
│                       │   │   │   │   ├── mavlink_msg_fence_status.h
│                       │   │   │   │   ├── mavlink_msg_fence_status.hpp
│                       │   │   │   │   ├── mavlink_msg_figure_eight_execution_status.h
│                       │   │   │   │   ├── mavlink_msg_figure_eight_execution_status.hpp
│                       │   │   │   │   ├── mavlink_msg_file_transfer_protocol.h
│                       │   │   │   │   ├── mavlink_msg_file_transfer_protocol.hpp
│                       │   │   │   │   ├── mavlink_msg_flight_information.h
│                       │   │   │   │   ├── mavlink_msg_flight_information.hpp
│                       │   │   │   │   ├── mavlink_msg_follow_target.h
│                       │   │   │   │   ├── mavlink_msg_follow_target.hpp
│                       │   │   │   │   ├── mavlink_msg_fuel_status.h
│                       │   │   │   │   ├── mavlink_msg_fuel_status.hpp
│                       │   │   │   │   ├── mavlink_msg_generator_status.h
│                       │   │   │   │   ├── mavlink_msg_generator_status.hpp
│                       │   │   │   │   ├── mavlink_msg_gimbal_device_attitude_status.h
│                       │   │   │   │   ├── mavlink_msg_gimbal_device_attitude_status.hpp
│                       │   │   │   │   ├── mavlink_msg_gimbal_device_information.h
│                       │   │   │   │   ├── mavlink_msg_gimbal_device_information.hpp
│                       │   │   │   │   ├── mavlink_msg_gimbal_device_set_attitude.h
│                       │   │   │   │   ├── mavlink_msg_gimbal_device_set_attitude.hpp
│                       │   │   │   │   ├── mavlink_msg_gimbal_manager_information.h
│                       │   │   │   │   ├── mavlink_msg_gimbal_manager_information.hpp
│                       │   │   │   │   ├── mavlink_msg_gimbal_manager_set_attitude.h
│                       │   │   │   │   ├── mavlink_msg_gimbal_manager_set_attitude.hpp
│                       │   │   │   │   ├── mavlink_msg_gimbal_manager_set_manual_control.h
│                       │   │   │   │   ├── mavlink_msg_gimbal_manager_set_manual_control.hpp
│                       │   │   │   │   ├── mavlink_msg_gimbal_manager_set_pitchyaw.h
│                       │   │   │   │   ├── mavlink_msg_gimbal_manager_set_pitchyaw.hpp
│                       │   │   │   │   ├── mavlink_msg_gimbal_manager_status.h
│                       │   │   │   │   ├── mavlink_msg_gimbal_manager_status.hpp
│                       │   │   │   │   ├── mavlink_msg_global_position_int_cov.h
│                       │   │   │   │   ├── mavlink_msg_global_position_int_cov.hpp
│                       │   │   │   │   ├── mavlink_msg_global_position_sensor.h
│                       │   │   │   │   ├── mavlink_msg_global_position_sensor.hpp
│                       │   │   │   │   ├── mavlink_msg_global_vision_position_estimate.h
│                       │   │   │   │   ├── mavlink_msg_global_vision_position_estimate.hpp
│                       │   │   │   │   ├── mavlink_msg_gps2_raw.h
│                       │   │   │   │   ├── mavlink_msg_gps2_raw.hpp
│                       │   │   │   │   ├── mavlink_msg_gps2_rtk.h
│                       │   │   │   │   ├── mavlink_msg_gps2_rtk.hpp
│                       │   │   │   │   ├── mavlink_msg_gps_global_origin.h
│                       │   │   │   │   ├── mavlink_msg_gps_global_origin.hpp
│                       │   │   │   │   ├── mavlink_msg_gps_inject_data.h
│                       │   │   │   │   ├── mavlink_msg_gps_inject_data.hpp
│                       │   │   │   │   ├── mavlink_msg_gps_input.h
│                       │   │   │   │   ├── mavlink_msg_gps_input.hpp
│                       │   │   │   │   ├── mavlink_msg_gps_raw_int.h
│                       │   │   │   │   ├── mavlink_msg_gps_raw_int.hpp
│                       │   │   │   │   ├── mavlink_msg_gps_rtcm_data.h
│                       │   │   │   │   ├── mavlink_msg_gps_rtcm_data.hpp
│                       │   │   │   │   ├── mavlink_msg_gps_rtk.h
│                       │   │   │   │   ├── mavlink_msg_gps_rtk.hpp
│                       │   │   │   │   ├── mavlink_msg_gps_status.h
│                       │   │   │   │   ├── mavlink_msg_gps_status.hpp
│                       │   │   │   │   ├── mavlink_msg_high_latency2.h
│                       │   │   │   │   ├── mavlink_msg_high_latency2.hpp
│                       │   │   │   │   ├── mavlink_msg_high_latency.h
│                       │   │   │   │   ├── mavlink_msg_high_latency.hpp
│                       │   │   │   │   ├── mavlink_msg_highres_imu.h
│                       │   │   │   │   ├── mavlink_msg_highres_imu.hpp
│                       │   │   │   │   ├── mavlink_msg_hil_actuator_controls.h
│                       │   │   │   │   ├── mavlink_msg_hil_actuator_controls.hpp
│                       │   │   │   │   ├── mavlink_msg_hil_controls.h
│                       │   │   │   │   ├── mavlink_msg_hil_controls.hpp
│                       │   │   │   │   ├── mavlink_msg_hil_gps.h
│                       │   │   │   │   ├── mavlink_msg_hil_gps.hpp
│                       │   │   │   │   ├── mavlink_msg_hil_optical_flow.h
│                       │   │   │   │   ├── mavlink_msg_hil_optical_flow.hpp
│                       │   │   │   │   ├── mavlink_msg_hil_rc_inputs_raw.h
│                       │   │   │   │   ├── mavlink_msg_hil_rc_inputs_raw.hpp
│                       │   │   │   │   ├── mavlink_msg_hil_sensor.h
│                       │   │   │   │   ├── mavlink_msg_hil_sensor.hpp
│                       │   │   │   │   ├── mavlink_msg_hil_state.h
│                       │   │   │   │   ├── mavlink_msg_hil_state.hpp
│                       │   │   │   │   ├── mavlink_msg_hil_state_quaternion.h
│                       │   │   │   │   ├── mavlink_msg_hil_state_quaternion.hpp
│                       │   │   │   │   ├── mavlink_msg_home_position.h
│                       │   │   │   │   ├── mavlink_msg_home_position.hpp
│                       │   │   │   │   ├── mavlink_msg_hygrometer_sensor.h
│                       │   │   │   │   ├── mavlink_msg_hygrometer_sensor.hpp
│                       │   │   │   │   ├── mavlink_msg_illuminator_status.h
│                       │   │   │   │   ├── mavlink_msg_illuminator_status.hpp
│                       │   │   │   │   ├── mavlink_msg_isbd_link_status.h
│                       │   │   │   │   ├── mavlink_msg_isbd_link_status.hpp
│                       │   │   │   │   ├── mavlink_msg_landing_target.h
│                       │   │   │   │   ├── mavlink_msg_landing_target.hpp
│                       │   │   │   │   ├── mavlink_msg_link_node_status.h
│                       │   │   │   │   ├── mavlink_msg_link_node_status.hpp
│                       │   │   │   │   ├── mavlink_msg_local_position_ned_cov.h
│                       │   │   │   │   ├── mavlink_msg_local_position_ned_cov.hpp
│                       │   │   │   │   ├── mavlink_msg_local_position_ned.h
│                       │   │   │   │   ├── mavlink_msg_local_position_ned.hpp
│                       │   │   │   │   ├── mavlink_msg_local_position_ned_system_global_offset.h
│                       │   │   │   │   ├── mavlink_msg_local_position_ned_system_global_offset.hpp
│                       │   │   │   │   ├── mavlink_msg_log_data.h
│                       │   │   │   │   ├── mavlink_msg_log_data.hpp
│                       │   │   │   │   ├── mavlink_msg_log_entry.h
│                       │   │   │   │   ├── mavlink_msg_log_entry.hpp
│                       │   │   │   │   ├── mavlink_msg_log_erase.h
│                       │   │   │   │   ├── mavlink_msg_log_erase.hpp
│                       │   │   │   │   ├── mavlink_msg_logging_ack.h
│                       │   │   │   │   ├── mavlink_msg_logging_ack.hpp
│                       │   │   │   │   ├── mavlink_msg_logging_data_acked.h
│                       │   │   │   │   ├── mavlink_msg_logging_data_acked.hpp
│                       │   │   │   │   ├── mavlink_msg_logging_data.h
│                       │   │   │   │   ├── mavlink_msg_logging_data.hpp
│                       │   │   │   │   ├── mavlink_msg_log_request_data.h
│                       │   │   │   │   ├── mavlink_msg_log_request_data.hpp
│                       │   │   │   │   ├── mavlink_msg_log_request_end.h
│                       │   │   │   │   ├── mavlink_msg_log_request_end.hpp
│                       │   │   │   │   ├── mavlink_msg_log_request_list.h
│                       │   │   │   │   ├── mavlink_msg_log_request_list.hpp
│                       │   │   │   │   ├── mavlink_msg_mag_cal_report.h
│                       │   │   │   │   ├── mavlink_msg_mag_cal_report.hpp
│                       │   │   │   │   ├── mavlink_msg_manual_control.h
│                       │   │   │   │   ├── mavlink_msg_manual_control.hpp
│                       │   │   │   │   ├── mavlink_msg_manual_setpoint.h
│                       │   │   │   │   ├── mavlink_msg_manual_setpoint.hpp
│                       │   │   │   │   ├── mavlink_msg_memory_vect.h
│                       │   │   │   │   ├── mavlink_msg_memory_vect.hpp
│                       │   │   │   │   ├── mavlink_msg_message_interval.h
│                       │   │   │   │   ├── mavlink_msg_message_interval.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_ack.h
│                       │   │   │   │   ├── mavlink_msg_mission_ack.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_clear_all.h
│                       │   │   │   │   ├── mavlink_msg_mission_clear_all.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_count.h
│                       │   │   │   │   ├── mavlink_msg_mission_count.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_current.h
│                       │   │   │   │   ├── mavlink_msg_mission_current.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_item.h
│                       │   │   │   │   ├── mavlink_msg_mission_item.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_item_int.h
│                       │   │   │   │   ├── mavlink_msg_mission_item_int.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_item_reached.h
│                       │   │   │   │   ├── mavlink_msg_mission_item_reached.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_request.h
│                       │   │   │   │   ├── mavlink_msg_mission_request.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_request_int.h
│                       │   │   │   │   ├── mavlink_msg_mission_request_int.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_request_list.h
│                       │   │   │   │   ├── mavlink_msg_mission_request_list.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_request_partial_list.h
│                       │   │   │   │   ├── mavlink_msg_mission_request_partial_list.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_set_current.h
│                       │   │   │   │   ├── mavlink_msg_mission_set_current.hpp
│                       │   │   │   │   ├── mavlink_msg_mission_write_partial_list.h
│                       │   │   │   │   ├── mavlink_msg_mission_write_partial_list.hpp
│                       │   │   │   │   ├── mavlink_msg_mount_orientation.h
│                       │   │   │   │   ├── mavlink_msg_mount_orientation.hpp
│                       │   │   │   │   ├── mavlink_msg_named_value_float.h
│                       │   │   │   │   ├── mavlink_msg_named_value_float.hpp
│                       │   │   │   │   ├── mavlink_msg_named_value_int.h
│                       │   │   │   │   ├── mavlink_msg_named_value_int.hpp
│                       │   │   │   │   ├── mavlink_msg_nav_controller_output.h
│                       │   │   │   │   ├── mavlink_msg_nav_controller_output.hpp
│                       │   │   │   │   ├── mavlink_msg_obstacle_distance.h
│                       │   │   │   │   ├── mavlink_msg_obstacle_distance.hpp
│                       │   │   │   │   ├── mavlink_msg_odometry.h
│                       │   │   │   │   ├── mavlink_msg_odometry.hpp
│                       │   │   │   │   ├── mavlink_msg_onboard_computer_status.h
│                       │   │   │   │   ├── mavlink_msg_onboard_computer_status.hpp
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_arm_status.h
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_arm_status.hpp
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_authentication.h
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_authentication.hpp
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_basic_id.h
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_basic_id.hpp
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_location.h
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_location.hpp
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_message_pack.h
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_message_pack.hpp
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_operator_id.h
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_operator_id.hpp
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_self_id.h
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_self_id.hpp
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_system.h
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_system.hpp
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_system_update.h
│                       │   │   │   │   ├── mavlink_msg_open_drone_id_system_update.hpp
│                       │   │   │   │   ├── mavlink_msg_optical_flow.h
│                       │   │   │   │   ├── mavlink_msg_optical_flow.hpp
│                       │   │   │   │   ├── mavlink_msg_optical_flow_rad.h
│                       │   │   │   │   ├── mavlink_msg_optical_flow_rad.hpp
│                       │   │   │   │   ├── mavlink_msg_orbit_execution_status.h
│                       │   │   │   │   ├── mavlink_msg_orbit_execution_status.hpp
│                       │   │   │   │   ├── mavlink_msg_param_error.h
│                       │   │   │   │   ├── mavlink_msg_param_error.hpp
│                       │   │   │   │   ├── mavlink_msg_param_ext_ack.h
│                       │   │   │   │   ├── mavlink_msg_param_ext_ack.hpp
│                       │   │   │   │   ├── mavlink_msg_param_ext_request_list.h
│                       │   │   │   │   ├── mavlink_msg_param_ext_request_list.hpp
│                       │   │   │   │   ├── mavlink_msg_param_ext_request_read.h
│                       │   │   │   │   ├── mavlink_msg_param_ext_request_read.hpp
│                       │   │   │   │   ├── mavlink_msg_param_ext_set.h
│                       │   │   │   │   ├── mavlink_msg_param_ext_set.hpp
│                       │   │   │   │   ├── mavlink_msg_param_ext_value.h
│                       │   │   │   │   ├── mavlink_msg_param_ext_value.hpp
│                       │   │   │   │   ├── mavlink_msg_param_map_rc.h
│                       │   │   │   │   ├── mavlink_msg_param_map_rc.hpp
│                       │   │   │   │   ├── mavlink_msg_param_request_list.h
│                       │   │   │   │   ├── mavlink_msg_param_request_list.hpp
│                       │   │   │   │   ├── mavlink_msg_param_request_read.h
│                       │   │   │   │   ├── mavlink_msg_param_request_read.hpp
│                       │   │   │   │   ├── mavlink_msg_param_set.h
│                       │   │   │   │   ├── mavlink_msg_param_set.hpp
│                       │   │   │   │   ├── mavlink_msg_param_value.h
│                       │   │   │   │   ├── mavlink_msg_param_value.hpp
│                       │   │   │   │   ├── mavlink_msg_ping.h
│                       │   │   │   │   ├── mavlink_msg_ping.hpp
│                       │   │   │   │   ├── mavlink_msg_play_tune.h
│                       │   │   │   │   ├── mavlink_msg_play_tune.hpp
│                       │   │   │   │   ├── mavlink_msg_play_tune_v2.h
│                       │   │   │   │   ├── mavlink_msg_play_tune_v2.hpp
│                       │   │   │   │   ├── mavlink_msg_position_target_global_int.h
│                       │   │   │   │   ├── mavlink_msg_position_target_global_int.hpp
│                       │   │   │   │   ├── mavlink_msg_position_target_local_ned.h
│                       │   │   │   │   ├── mavlink_msg_position_target_local_ned.hpp
│                       │   │   │   │   ├── mavlink_msg_power_status.h
│                       │   │   │   │   ├── mavlink_msg_power_status.hpp
│                       │   │   │   │   ├── mavlink_msg_protocol_version.h
│                       │   │   │   │   ├── mavlink_msg_protocol_version.hpp
│                       │   │   │   │   ├── mavlink_msg_radio_status.h
│                       │   │   │   │   ├── mavlink_msg_radio_status.hpp
│                       │   │   │   │   ├── mavlink_msg_raw_imu.h
│                       │   │   │   │   ├── mavlink_msg_raw_imu.hpp
│                       │   │   │   │   ├── mavlink_msg_raw_pressure.h
│                       │   │   │   │   ├── mavlink_msg_raw_pressure.hpp
│                       │   │   │   │   ├── mavlink_msg_raw_rpm.h
│                       │   │   │   │   ├── mavlink_msg_raw_rpm.hpp
│                       │   │   │   │   ├── mavlink_msg_rc_channels.h
│                       │   │   │   │   ├── mavlink_msg_rc_channels.hpp
│                       │   │   │   │   ├── mavlink_msg_rc_channels_override.h
│                       │   │   │   │   ├── mavlink_msg_rc_channels_override.hpp
│                       │   │   │   │   ├── mavlink_msg_rc_channels_raw.h
│                       │   │   │   │   ├── mavlink_msg_rc_channels_raw.hpp
│                       │   │   │   │   ├── mavlink_msg_rc_channels_scaled.h
│                       │   │   │   │   ├── mavlink_msg_rc_channels_scaled.hpp
│                       │   │   │   │   ├── mavlink_msg_relay_status.h
│                       │   │   │   │   ├── mavlink_msg_relay_status.hpp
│                       │   │   │   │   ├── mavlink_msg_request_data_stream.h
│                       │   │   │   │   ├── mavlink_msg_request_data_stream.hpp
│                       │   │   │   │   ├── mavlink_msg_request_event.h
│                       │   │   │   │   ├── mavlink_msg_request_event.hpp
│                       │   │   │   │   ├── mavlink_msg_resource_request.h
│                       │   │   │   │   ├── mavlink_msg_resource_request.hpp
│                       │   │   │   │   ├── mavlink_msg_response_event_error.h
│                       │   │   │   │   ├── mavlink_msg_response_event_error.hpp
│                       │   │   │   │   ├── mavlink_msg_safety_allowed_area.h
│                       │   │   │   │   ├── mavlink_msg_safety_allowed_area.hpp
│                       │   │   │   │   ├── mavlink_msg_safety_set_allowed_area.h
│                       │   │   │   │   ├── mavlink_msg_safety_set_allowed_area.hpp
│                       │   │   │   │   ├── mavlink_msg_scaled_imu2.h
│                       │   │   │   │   ├── mavlink_msg_scaled_imu2.hpp
│                       │   │   │   │   ├── mavlink_msg_scaled_imu3.h
│                       │   │   │   │   ├── mavlink_msg_scaled_imu3.hpp
│                       │   │   │   │   ├── mavlink_msg_scaled_imu.h
│                       │   │   │   │   ├── mavlink_msg_scaled_imu.hpp
│                       │   │   │   │   ├── mavlink_msg_scaled_pressure2.h
│                       │   │   │   │   ├── mavlink_msg_scaled_pressure2.hpp
│                       │   │   │   │   ├── mavlink_msg_scaled_pressure3.h
│                       │   │   │   │   ├── mavlink_msg_scaled_pressure3.hpp
│                       │   │   │   │   ├── mavlink_msg_scaled_pressure.h
│                       │   │   │   │   ├── mavlink_msg_scaled_pressure.hpp
│                       │   │   │   │   ├── mavlink_msg_serial_control.h
│                       │   │   │   │   ├── mavlink_msg_serial_control.hpp
│                       │   │   │   │   ├── mavlink_msg_servo_output_raw.h
│                       │   │   │   │   ├── mavlink_msg_servo_output_raw.hpp
│                       │   │   │   │   ├── mavlink_msg_set_actuator_control_target.h
│                       │   │   │   │   ├── mavlink_msg_set_actuator_control_target.hpp
│                       │   │   │   │   ├── mavlink_msg_set_attitude_target.h
│                       │   │   │   │   ├── mavlink_msg_set_attitude_target.hpp
│                       │   │   │   │   ├── mavlink_msg_set_gps_global_origin.h
│                       │   │   │   │   ├── mavlink_msg_set_gps_global_origin.hpp
│                       │   │   │   │   ├── mavlink_msg_set_home_position.h
│                       │   │   │   │   ├── mavlink_msg_set_home_position.hpp
│                       │   │   │   │   ├── mavlink_msg_set_mode.h
│                       │   │   │   │   ├── mavlink_msg_set_mode.hpp
│                       │   │   │   │   ├── mavlink_msg_set_position_target_global_int.h
│                       │   │   │   │   ├── mavlink_msg_set_position_target_global_int.hpp
│                       │   │   │   │   ├── mavlink_msg_set_position_target_local_ned.h
│                       │   │   │   │   ├── mavlink_msg_set_position_target_local_ned.hpp
│                       │   │   │   │   ├── mavlink_msg_setup_signing.h
│                       │   │   │   │   ├── mavlink_msg_setup_signing.hpp
│                       │   │   │   │   ├── mavlink_msg_sim_state.h
│                       │   │   │   │   ├── mavlink_msg_sim_state.hpp
│                       │   │   │   │   ├── mavlink_msg_smart_battery_info.h
│                       │   │   │   │   ├── mavlink_msg_smart_battery_info.hpp
│                       │   │   │   │   ├── mavlink_msg_statustext.h
│                       │   │   │   │   ├── mavlink_msg_statustext.hpp
│                       │   │   │   │   ├── mavlink_msg_storage_information.h
│                       │   │   │   │   ├── mavlink_msg_storage_information.hpp
│                       │   │   │   │   ├── mavlink_msg_supported_tunes.h
│                       │   │   │   │   ├── mavlink_msg_supported_tunes.hpp
│                       │   │   │   │   ├── mavlink_msg_sys_status.h
│                       │   │   │   │   ├── mavlink_msg_sys_status.hpp
│                       │   │   │   │   ├── mavlink_msg_system_time.h
│                       │   │   │   │   ├── mavlink_msg_system_time.hpp
│                       │   │   │   │   ├── mavlink_msg_terrain_check.h
│                       │   │   │   │   ├── mavlink_msg_terrain_check.hpp
│                       │   │   │   │   ├── mavlink_msg_terrain_data.h
│                       │   │   │   │   ├── mavlink_msg_terrain_data.hpp
│                       │   │   │   │   ├── mavlink_msg_terrain_report.h
│                       │   │   │   │   ├── mavlink_msg_terrain_report.hpp
│                       │   │   │   │   ├── mavlink_msg_terrain_request.h
│                       │   │   │   │   ├── mavlink_msg_terrain_request.hpp
│                       │   │   │   │   ├── mavlink_msg_time_estimate_to_target.h
│                       │   │   │   │   ├── mavlink_msg_time_estimate_to_target.hpp
│                       │   │   │   │   ├── mavlink_msg_timesync.h
│                       │   │   │   │   ├── mavlink_msg_timesync.hpp
│                       │   │   │   │   ├── mavlink_msg_trajectory_representation_bezier.h
│                       │   │   │   │   ├── mavlink_msg_trajectory_representation_bezier.hpp
│                       │   │   │   │   ├── mavlink_msg_trajectory_representation_waypoints.h
│                       │   │   │   │   ├── mavlink_msg_trajectory_representation_waypoints.hpp
│                       │   │   │   │   ├── mavlink_msg_tunnel.h
│                       │   │   │   │   ├── mavlink_msg_tunnel.hpp
│                       │   │   │   │   ├── mavlink_msg_uavcan_node_info.h
│                       │   │   │   │   ├── mavlink_msg_uavcan_node_info.hpp
│                       │   │   │   │   ├── mavlink_msg_uavcan_node_status.h
│                       │   │   │   │   ├── mavlink_msg_uavcan_node_status.hpp
│                       │   │   │   │   ├── mavlink_msg_utm_global_position.h
│                       │   │   │   │   ├── mavlink_msg_utm_global_position.hpp
│                       │   │   │   │   ├── mavlink_msg_v2_extension.h
│                       │   │   │   │   ├── mavlink_msg_v2_extension.hpp
│                       │   │   │   │   ├── mavlink_msg_vfr_hud.h
│                       │   │   │   │   ├── mavlink_msg_vfr_hud.hpp
│                       │   │   │   │   ├── mavlink_msg_vibration.h
│                       │   │   │   │   ├── mavlink_msg_vibration.hpp
│                       │   │   │   │   ├── mavlink_msg_vicon_position_estimate.h
│                       │   │   │   │   ├── mavlink_msg_vicon_position_estimate.hpp
│                       │   │   │   │   ├── mavlink_msg_video_stream_information.h
│                       │   │   │   │   ├── mavlink_msg_video_stream_information.hpp
│                       │   │   │   │   ├── mavlink_msg_video_stream_status.h
│                       │   │   │   │   ├── mavlink_msg_video_stream_status.hpp
│                       │   │   │   │   ├── mavlink_msg_vision_position_estimate.h
│                       │   │   │   │   ├── mavlink_msg_vision_position_estimate.hpp
│                       │   │   │   │   ├── mavlink_msg_vision_speed_estimate.h
│                       │   │   │   │   ├── mavlink_msg_vision_speed_estimate.hpp
│                       │   │   │   │   ├── mavlink_msg_wheel_distance.h
│                       │   │   │   │   ├── mavlink_msg_wheel_distance.hpp
│                       │   │   │   │   ├── mavlink_msg_wifi_config_ap.h
│                       │   │   │   │   ├── mavlink_msg_wifi_config_ap.hpp
│                       │   │   │   │   ├── mavlink_msg_winch_status.h
│                       │   │   │   │   ├── mavlink_msg_winch_status.hpp
│                       │   │   │   │   ├── mavlink_msg_wind_cov.h
│                       │   │   │   │   ├── mavlink_msg_wind_cov.hpp
│                       │   │   │   │   ├── testsuite.h
│                       │   │   │   │   └── version.h
│                       │   │   │   ├── HorizonDialect
│                       │   │   │   │   ├── gtestsuite.hpp
│                       │   │   │   │   ├── HorizonDialect.h
│                       │   │   │   │   ├── HorizonDialect.hpp
│                       │   │   │   │   ├── mavlink.h
│                       │   │   │   │   ├── mavlink_msg_cosmic_radiation.h
│                       │   │   │   │   ├── mavlink_msg_cosmic_radiation.hpp
│                       │   │   │   │   ├── mavlink_msg_flight_states.h
│                       │   │   │   │   ├── mavlink_msg_flight_states.hpp
│                       │   │   │   │   ├── mavlink_msg_payload_temperature.h
│                       │   │   │   │   ├── mavlink_msg_payload_temperature.hpp
│                       │   │   │   │   ├── testsuite.h
│                       │   │   │   │   └── version.h
│                       │   │   │   ├── mavlink_conversions.h
│                       │   │   │   ├── mavlink_get_info.h
│                       │   │   │   ├── mavlink_helpers.h
│                       │   │   │   ├── mavlink_sha256.h
│                       │   │   │   ├── mavlink_types.h
│                       │   │   │   ├── message.hpp
│                       │   │   │   ├── minimal
│                       │   │   │   │   ├── gtestsuite.hpp
│                       │   │   │   │   ├── mavlink.h
│                       │   │   │   │   ├── mavlink_msg_heartbeat.h
│                       │   │   │   │   ├── mavlink_msg_heartbeat.hpp
│                       │   │   │   │   ├── minimal.h
│                       │   │   │   │   ├── minimal.hpp
│                       │   │   │   │   ├── testsuite.h
│                       │   │   │   │   └── version.h
│                       │   │   │   ├── msgmap.hpp
│                       │   │   │   ├── protocol.h
│                       │   │   │   └── standard
│                       │   │   │       ├── gtestsuite.hpp
│                       │   │   │       ├── mavlink.h
│                       │   │   │       ├── mavlink_msg_autopilot_version.h
│                       │   │   │       ├── mavlink_msg_autopilot_version.hpp
│                       │   │   │       ├── mavlink_msg_global_position_int.h
│                       │   │   │       ├── mavlink_msg_global_position_int.hpp
│                       │   │   │       ├── standard.h
│                       │   │   │       ├── standard.hpp
│                       │   │   │       ├── testsuite.h
│                       │   │   │       └── version.h
│                       │   │   ├── mavgenerate.py
│                       │   │   ├── MAVLinkConfig.cmake.in
│                       │   │   ├── message_definitions
│                       │   │   │   └── v1.0
│                       │   │   │       ├── all.xml
│                       │   │   │       ├── ardupilotmega.xml
│                       │   │   │       ├── ASLUAV.xml
│                       │   │   │       ├── AVSSUAS.xml
│                       │   │   │       ├── common.xml
│                       │   │   │       ├── csAirLink.xml
│                       │   │   │       ├── cubepilot.xml
│                       │   │   │       ├── development.xml
│                       │   │   │       ├── HorizonDialect
│                       │   │   │       ├── icarous.xml
│                       │   │   │       ├── loweheiser.xml
│                       │   │   │       ├── marsh.xml
│                       │   │   │       ├── matrixpilot.xml
│                       │   │   │       ├── minimal.xml
│                       │   │   │       ├── paparazzi.xml
│                       │   │   │       ├── python_array_test.xml
│                       │   │   │       ├── standard.xml
│                       │   │   │       ├── stemstudios.xml
│                       │   │   │       ├── storm32.xml
│                       │   │   │       ├── test.xml
│                       │   │   │       ├── ualberta.xml
│                       │   │   │       └── uAvionix.xml
│                       │   │   ├── pymavlink
│                       │   │   ├── README.md
│                       │   │   ├── ruff.toml
│                       │   │   └── scripts
│                       │   │       ├── check_api_break.py
│                       │   │       ├── format_xml.sh
│                       │   │       ├── test.sh
│                       │   │       ├── update_c_library.sh
│                       │   │       ├── update_generated_repos.sh
│                       │   │       └── xml_consistency_check.py
│                       │   ├── New
│                       │   │   ├── App.hpp
│                       │   │   ├── Config.hpp
│                       │   │   ├── LoraTx.hpp
│                       │   │   ├── MavlinkFrames.hpp
│                       │   │   ├── Sensors.hpp
│                       │   │   └── Storage.hpp
│                       │   └── Old
│                       │       ├── Config.hpp
│                       │       ├── FlashLogger.hpp
│                       │       ├── GeigerCounter.hpp
│                       │       ├── LoRaLink.hpp
│                       │       ├── SdLogger.hpp
│                       │       ├── TelemetryApp.hpp
│                       │       └── TelemetryRingBuffer.hpp
│                       ├── lib
│                       │   └── README
│                       ├── platformio.ini
│                       ├── README.md
│                       ├── src
│                       │   ├── New
│                       │   │   ├── App.cpp
│                       │   │   ├── LoraTx.cpp
│                       │   │   ├── main.cpp
│                       │   │   ├── MavlinkFrames.cpp
│                       │   │   ├── Sensors.cpp
│                       │   │   └── Storage.cpp
│                       │   ├── Old
│                       │   │   ├── FlashLogger.cpp
│                       │   │   ├── GeigerCounter.cpp
│                       │   │   ├── LoRaLink.cpp
│                       │   │   ├── paylaod.cpp
│                       │   │   ├── SdLogger.cpp
│                       │   │   ├── TelemetryApp.cpp
│                       │   │   └── TelemetryRingBuffer.cpp
│                       │   └── tests
│                       │       ├── test_all_addr.cpp
│                       │       ├── test_baro.cpp
│                       │       ├── test_flash.cpp
│                       │       ├── test_geiger.cpp
│                       │       ├── test_i2c_scan.cpp
│                       │       ├── test_imu.cpp
│                       │       ├── test_lora.cpp
│                       │       ├── test_lora_sd_flash.cpp
│                       │       ├── test_lora_temp.cpp
│                       │       ├── test_lora_volt.cpp
│                       │       ├── test_mag.cpp
│                       │       └── test_sdcard.cpp
│                       ├── test
│                       │   └── README
│                       └── Text
│                           ├── Flow_Chart.md
│                           └── New_README.md
└── README.md

37 directories, 858 files


```

