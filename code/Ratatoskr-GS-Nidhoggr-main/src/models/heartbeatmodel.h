#ifndef HEARTBEATMODEL_H
#define HEARTBEATMODEL_H

#include <QObject>

// Tracks when a board (avionics or payload) was last heard from. One
// instance per board; PacketParser::heartbeatReceived is routed to the
// right instance by component id, same pattern as the other per-board
// models in FlightModels.
class HeartbeatModel : public QObject
{
    Q_OBJECT
    Q_PROPERTY(qint64 lastBeatMs READ lastBeatMs NOTIFY lastBeatChanged)

public:
    explicit HeartbeatModel(QObject *parent = nullptr);

    // Epoch milliseconds (QDateTime::currentMSecsSinceEpoch()) of the most
    // recent heartbeat, or -1 if none received yet. Wall-clock time (not
    // the mission-relative elapsedTimer the other models use) so QML can
    // compare it directly against Date.now().
    qint64 lastBeatMs() const;

    Q_INVOKABLE void recordBeat();

signals:
    void lastBeatChanged();

private:
    qint64 m_lastBeatMs = -1;
};

#endif // HEARTBEATMODEL_H
