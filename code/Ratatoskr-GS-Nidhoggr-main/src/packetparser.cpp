#include "packetparser.h"
#include <cmath>
#include <cstring>
#include <QDebug>

PacketParser::PacketParser(QObject *parent) : QObject(parent)
{
    memset(&m_msg, 0, sizeof(m_msg));
    memset(&m_status, 0, sizeof(m_status));
}

void PacketParser::parse(const QByteArray &data)
{
    qDebug() << "parse() bytes:" << data.size() << "hex:" << data.left(16).toHex(' ');
    for (int i = 0; i < data.size(); ++i) {
        const uint8_t byte = static_cast<uint8_t>(data.at(i));
        const uint8_t res = mavlink_frame_char(MAVLINK_COMM_0, byte, &m_msg, &m_status);
        switch (res) {
        case MAVLINK_FRAMING_OK:
            handleMessage(m_msg);
            break;
        case MAVLINK_FRAMING_BAD_CRC:
            qDebug() << "MAVLink bad CRC (total errors:" << m_status.parse_error << ")";
            break;
        case MAVLINK_FRAMING_BAD_SIGNATURE:
            qDebug() << "MAVLink bad signature";
            break;
        default:
            break; // MAVLINK_FRAMING_INCOMPLETE: normal mid-packet, no log
        }
    }
}

void PacketParser::handleMessage(const mavlink_message_t &msg)
{
    qDebug() << "got message";
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_SCALED_IMU: {
        mavlink_scaled_imu_t imu;
        mavlink_msg_scaled_imu_decode(&msg, &imu);
        emit accelerationReceived(imu.xacc / 1000.0, imu.yacc / 1000.0, imu.zacc / 1000.0, msg.compid);
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU2: {
        mavlink_scaled_imu_t imu;
        mavlink_msg_scaled_imu_decode(&msg, &imu);
        emit rotationReceived(imu.xgyro / 1000.0, imu.ygyro / 1000.0, imu.zgyro / 1000.0, msg.compid);
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU3: {
        mavlink_scaled_imu3_t imu;
        mavlink_msg_scaled_imu3_decode(&msg, &imu);
        emit magnetometerReceived(imu.xmag / 10.0, imu.ymag / 10.0, imu.zmag / 10.0, msg.compid);
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE: {
        mavlink_scaled_pressure_t pres;
        mavlink_msg_scaled_pressure_decode(&msg, &pres);
        emit pressureReceived(pres.press_abs, msg.compid);
        emit temperatureReceived(pres.temperature / 100.0, msg.compid);
        double alt = 44330.0 * (1.0 - pow(pres.press_abs / 1013.25, 0.1903));
        emit altitudeReceived(alt, msg.compid);
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
        mavlink_vfr_hud_t hud;
        mavlink_msg_vfr_hud_decode(&msg, &hud);
        emit velocityReceived(hud.groundspeed, msg.compid);
        break;
    }
    case MAVLINK_MSG_ID_COSMIC_RADIATION: {
        mavlink_cosmic_radiation_t rad;
        mavlink_msg_cosmic_radiation_decode(&msg, &rad);
        emit radiationReceived(rad.radiation);
        break;
    }
    case MAVLINK_MSG_ID_NAMED_VALUE_INT: {
        mavlink_named_value_int_t temp;
        mavlink_msg_named_value_int_decode(&msg, &temp);
        QLatin1StringView name(temp.name, strnlen(temp.name, sizeof(temp.name)));

        if (name == "BARO_T") {
            emit temperatureReceived(temp.value, msg.compid);
        } else if (name == "LORA_T") {
            // transceiver temperature
        } else if (name == "LORA_V") {
            // transceiver voltage
        }
        break;
    }
    case MAVLINK_MSG_ID_HEARTBEAT: {
        emit heartbeatReceived(msg.compid);
        break;
    }
    default:
        break;
    }
}
