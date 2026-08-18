#ifndef PACKETPARSER_H
#define PACKETPARSER_H
#pragma once
#include <QObject>
#include <QByteArray>
#include <cstdint>
#include "mavlink/HorizonDialect/mavlink.h"

class PacketParser : public QObject
{
    Q_OBJECT
public:
    explicit PacketParser(QObject *parent = nullptr);
    void parse(const QByteArray &data);

signals:
    // compid identifies which board sent the message (see horizoncomponents.h)
    // so callers can route the sample into the right avionics/payload model.
    void altitudeReceived(double value, uint8_t compid);
    void velocityReceived(double value, uint8_t compid);
    void accelerationReceived(double x, double y, double z, uint8_t compid);
    void rotationReceived(double x, double y, double z, uint8_t compid);
    void magnetometerReceived(double x, double y, double z, uint8_t compid);
    void pressureReceived(double value, uint8_t compid);
    void radiationReceived(double value);
    void temperatureReceived(double value, uint8_t compid);
    void heartbeatReceived(uint8_t compid);

private:
    mavlink_message_t m_msg;
    mavlink_status_t m_status;
    void handleMessage(const mavlink_message_t &msg);
};

#endif // PACKETPARSER_H
