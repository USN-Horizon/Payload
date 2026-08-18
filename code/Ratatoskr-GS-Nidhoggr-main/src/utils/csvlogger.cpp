#include "utils/csvlogger.h"
#include <QDir>
#include <QFileInfo>
#include <QDebug>

#ifdef Q_OS_WIN
#include <io.h>
#else
#include <unistd.h>
#endif

namespace {
constexpr int kSyncIntervalMs = 200;
}

CsvLogger::CsvLogger(QObject *parent) : QObject(parent)
{
    m_syncTimer.setInterval(kSyncIntervalMs);
    connect(&m_syncTimer, &QTimer::timeout, this, &CsvLogger::syncToDisk);
}

CsvLogger::~CsvLogger()
{
    stop();
}

bool CsvLogger::start(const QString &filePath)
{
    stop();

    const QFileInfo info(filePath);
    QDir().mkpath(info.absolutePath());

    m_file.setFileName(filePath);
    if (!m_file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "CsvLogger: could not open" << filePath << m_file.errorString();
        return false;
    }

    m_stream.setDevice(&m_file);
    m_stream << "elapsed_ms,compid,metric,v0,v1,v2\n";
    m_stream.flush();
    m_syncTimer.start();
    return true;
}

void CsvLogger::stop()
{
    if (!isActive()) return;
    m_syncTimer.stop();
    syncToDisk();
    m_stream.setDevice(nullptr);
    m_file.close();
}

bool CsvLogger::isActive() const
{
    return m_file.isOpen();
}

QString CsvLogger::lastError() const
{
    return m_file.errorString();
}

void CsvLogger::logScalar(qint64 elapsedMs, uint8_t compid, const QString &metric, double value)
{
    writeRow(elapsedMs, compid, metric, QString::number(value, 'f', 6), QString(), QString());
}

void CsvLogger::logVector(qint64 elapsedMs, uint8_t compid, const QString &metric, double x, double y, double z)
{
    writeRow(elapsedMs, compid, metric,
              QString::number(x, 'f', 6), QString::number(y, 'f', 6), QString::number(z, 'f', 6));
}

void CsvLogger::logEvent(qint64 elapsedMs, uint8_t compid, const QString &metric)
{
    writeRow(elapsedMs, compid, metric, QString(), QString(), QString());
}

void CsvLogger::writeRow(qint64 elapsedMs, uint8_t compid, const QString &metric,
                          const QString &v0, const QString &v1, const QString &v2)
{
    if (!isActive()) return;

    m_stream << elapsedMs << ',' << static_cast<int>(compid) << ',' << metric << ','
              << v0 << ',' << v1 << ',' << v2 << '\n';

    m_stream.flush();
}

void CsvLogger::syncToDisk()
{
    if (!isActive()) return;

    m_stream.flush();
#ifdef Q_OS_WIN
    _commit(m_file.handle());
#else
    fsync(m_file.handle());
#endif
}
