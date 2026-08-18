#ifndef CSVLOGGER_H
#define CSVLOGGER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <cstdint>

class CsvLogger : public QObject
{
    Q_OBJECT
public:
    explicit CsvLogger(QObject *parent = nullptr);
    ~CsvLogger() override;

    // Closes any currently open log, creates parent directories as needed,
    // and opens filePath for a fresh mission, writing the header row.
    bool start(const QString &filePath);
    void stop();
    bool isActive() const;
    QString lastError() const;

public slots:
    void logScalar(qint64 elapsedMs, uint8_t compid, const QString &metric, double value);
    void logVector(qint64 elapsedMs, uint8_t compid, const QString &metric, double x, double y, double z);
    void logEvent(qint64 elapsedMs, uint8_t compid, const QString &metric);

private:
    void writeRow(qint64 elapsedMs, uint8_t compid, const QString &metric,
                  const QString &v0, const QString &v1, const QString &v2);

    void syncToDisk();

    QFile m_file;
    QTextStream m_stream;
    QTimer m_syncTimer;
};

#endif // CSVLOGGER_H
