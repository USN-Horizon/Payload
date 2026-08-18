#ifndef SERIALREADER_H
#define SERIALREADER_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QByteArray>

class SerialReader : public QObject
{
    Q_OBJECT

public:
    explicit SerialReader(QObject *parent = nullptr);
    ~SerialReader();

    bool openPort(const QString &portName, qint32 baudRate = 115200);
    void closePort();
    bool isOpen() const;

    static QStringList availablePorts();
    static QString findHorizonPort();

signals:
    void rawPacketReceived(const QByteArray &packet);
    void errorOccurred(const QString &error);

private slots:
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);

private:
    QSerialPort *serialPort;
};

#endif // SERIALREADER_H
