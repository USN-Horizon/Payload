#include "missionmanager.h"
#include "utils/flightlogfactory.h"
#include <QDateTime>
#include <QRegularExpression>
#include <QStandardPaths>

MissionManager::MissionManager(FlightModels &avionics_models, FlightModels &payload_models, CsvLogger &logger, QObject *parent) : QObject(parent), m_avionics_models(avionics_models), m_payload_models(payload_models), m_logger(logger), m_state(CaptureState{}) {}

QString MissionManager::missionName() const {return m_missionName;}

QString MissionManager::modeText() const
{
    if (!isPlayback()) return "Capture Mode";
    return "Playback Mode";
}

void MissionManager::setMissionName(const QString &name) {
    if (name != m_missionName) {
        m_missionName = name;
        emit missionNameChanged();
    }
}

bool MissionManager::importMissionData(const QString &path)
{
    const ParsedFlightLog data = FlightLogFactory::parse(path);
    if (!data.valid) return false;

    // Entering playback stops live-capture logging; a fresh log is opened
    // if/when the user returns to capture mode via startCapture().
    m_logger.stop();

    FlightLogFactory::populateAltitude(m_avionics_models.altitude, data);
    FlightLogFactory::populateVelocity(m_avionics_models.velocity, data);
    if (m_avionics_models.state)
        FlightLogFactory::populateState(m_avionics_models.state, data);
    FlightLogFactory::populateLocation(m_avionics_models.location, data);

    m_state = PlaybackState {
        path
    };
    emit stateChanged();

    return true;
}

bool MissionManager::isPlayback() const {
    return std::holds_alternative<PlaybackState>(m_state);
}

QString MissionManager::logFilePath() const
{
    const QString base = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    const QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");

    QString name = m_missionName.isEmpty() ? QStringLiteral("mission") : m_missionName;
    // Keep the mission name filesystem-safe rather than rejecting odd input.
    static const QRegularExpression unsafeChars(QStringLiteral("[^A-Za-z0-9_-]+"));
    name.replace(unsafeChars, QStringLiteral("_"));

    return QStringLiteral("%1/missions/%2_%3.csv").arg(base, name, timestamp);
}

bool MissionManager::beginLogging()
{
    const QString path = logFilePath();
    if (m_logger.start(path)) return true;

    emit logFileFailed(path, m_logger.lastError());
    return false;
}

/**
 *	Though Capture mode is default state, we want to be able to start capture mode again after user has dismissed playback mode
 */
bool MissionManager::startCapture() {
    if (!beginLogging()) return false;

    m_state = CaptureState {};
    emit stateChanged();
    return true;
}
