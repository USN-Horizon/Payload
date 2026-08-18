#include "humidityutils.h"
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QString>


QList<TimestampedHumidity> humidity_utils::parseCsv(const QString& a_fileName)
{
    QList<TimestampedHumidity> result;
    QFile file(a_fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Could not open file:" << file.errorString();
        return {};
    }

    QTextStream in(&file);

    // Skip header line
    if (!in.atEnd()) {
        QString header = in.readLine();
        // Optional: validate header
        if (header != "Timestamp,Humidity") {
            qDebug() << "Warning: Unexpected header format";
        }
    }

    // Read data lines
    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line.isEmpty()) {
            continue;  // Skip empty lines
        }

        // Split line by comma
        QStringList parts = line.split(',');
        if (parts.size() >= 2) {
            TimestampedHumidity data;
            bool okTimestamp, okHumidity;

            // Parse relative timestamp (milliseconds since boot)
            data.timestamp = parts[0].toLongLong(&okTimestamp);
            if (!okTimestamp) {
                qDebug() << "Warning: Failed to parse timestamp from:" << parts[0];
                continue;
            }

            // Parse humidity
            data.humidity = parts[1].toDouble(&okHumidity);
            if (!okHumidity) {
                qDebug() << "Warning: Failed to parse humidity from:" << parts[1];
                continue;
            }

            result.append(data);
        } else {
            qDebug() << "Warning: Line does not contain enough values:" << line;
        }
    }

    file.close();
    return result;
}

