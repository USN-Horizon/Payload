#include "heartbeatmodel.h"
#include <QDateTime>

HeartbeatModel::HeartbeatModel(QObject *parent) : QObject(parent) {}

qint64 HeartbeatModel::lastBeatMs() const
{
    return m_lastBeatMs;
}

void HeartbeatModel::recordBeat()
{
    m_lastBeatMs = QDateTime::currentMSecsSinceEpoch();
    emit lastBeatChanged();
}
