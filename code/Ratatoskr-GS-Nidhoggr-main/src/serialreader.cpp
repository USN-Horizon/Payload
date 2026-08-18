#include "serialreader.h"
#include <QDebug>

SerialReader::SerialReader(QObject *parent)
    : QObject(parent)
    , serialPort(new QSerialPort(this))
{
    connect(serialPort, &QSerialPort::readyRead,
            this, &SerialReader::handleReadyRead);
    connect(serialPort, &QSerialPort::errorOccurred,
            this, &SerialReader::handleError);
}

SerialReader::~SerialReader()
{
    closePort();
}

bool SerialReader::openPort(const QString &portName, qint32 baudRate)
{
    serialPort->setPortName(portName);
    serialPort->setBaudRate(baudRate);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (serialPort->open(QIODevice::ReadWrite)) {
        serialPort->setDataTerminalReady(false);
        qDebug() << "Serial port opened:" << portName << "at" << baudRate << "baud";
        return true;
    } else {
        emit errorOccurred("Failed to open port: " + serialPort->errorString());
        return false;
    }
}

void SerialReader::closePort()
{
    if (serialPort->isOpen()) {
        serialPort->close();
        qDebug() << "Serial port closed";
    }
}

bool SerialReader::isOpen() const
{
    return serialPort->isOpen();
}

QStringList SerialReader::availablePorts()
{
    QStringList ports;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        ports << info.portName() + " (" + info.description() + ")";
    }
    return ports;
}

// Arduino Nano 33 IoT: VID 0x2341, PID 0x8057
namespace {
    constexpr quint16 ARDUINO_VID = 0x2341;
    constexpr quint16 NANO_33_IOT_PID = 0x8057;
}

QString SerialReader::findHorizonPort()
{
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        if (info.hasVendorIdentifier() && info.hasProductIdentifier()
            && info.vendorIdentifier() == ARDUINO_VID
            && info.productIdentifier() == NANO_33_IOT_PID) {
            qDebug() << "Found Horizon module on" << info.portName()
                     << "(" << info.description() << ")";
            return info.portName();
        }
    }
    return {};
}

void SerialReader::handleReadyRead()
{
    emit rawPacketReceived(serialPort->readAll());
}

void SerialReader::handleError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError && error != QSerialPort::TimeoutError) {
        emit errorOccurred(serialPort->errorString());
        qDebug() << "Serial error:" << serialPort->errorString();
        qDebug() << "Serial error code:" << error << serialPort->errorString();
    }
}
