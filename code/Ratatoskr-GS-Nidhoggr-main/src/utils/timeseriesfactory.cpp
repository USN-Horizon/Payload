#include "timeseriesfactory.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QFileInfo>

TimeSeriesModel* TimeSeriesFactory::fromCsv(
    const QString& csvFilePath,
    const QString& timeColumn,
    const QString& valueColumn,
    double timeMultiplier,
    bool hasHeader,
    QChar separator)
{
    QVector<qreal> xValues;
    QVector<qreal> yValues;

    QFile file(csvFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Could not open file:" << file.errorString();
        return new TimeSeriesModel();
    }

    QTextStream in(&file);
    int lineNumber = 0;
    int timeColIndex = -1;
    int valueColIndex = -1;

    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line.isEmpty()) {
            continue;
        }

        QStringList fields = line.split(separator);

        // Process header if present
        if (hasHeader && lineNumber == 0) {
            // Try to find column indices by name
            for (int i = 0; i < fields.size(); ++i) {
                if (fields[i] == timeColumn) {
                    timeColIndex = i;
                }
                if (fields[i] == valueColumn) {
                    valueColIndex = i;
                }
            }

            // If column names weren't found, try using them as numeric indices
            if (timeColIndex == -1) {
                bool ok;
                int idx = timeColumn.toInt(&ok);
                if (ok && idx >= 0 && idx < fields.size()) {
                    timeColIndex = idx;
                }
            }

            if (valueColIndex == -1) {
                bool ok;
                int idx = valueColumn.toInt(&ok);
                if (ok && idx >= 0 && idx < fields.size()) {
                    valueColIndex = idx;
                }
            }

            if (timeColIndex == -1 || valueColIndex == -1) {
                qDebug() << "Column not found. Available columns:" << fields;
                file.close();
                return new TimeSeriesModel();
            }

            lineNumber++;
            continue;
        }

        // Process data rows
        if (fields.size() > qMax(timeColIndex, valueColIndex)) {
            bool okTime, okValue;
            qreal timeValue = fields[timeColIndex].toDouble(&okTime);
            qreal dataValue = fields[valueColIndex].toDouble(&okValue);

            if (okTime && okValue) {
                // Apply time multiplier to convert to milliseconds
                xValues.append(timeValue * timeMultiplier);
                yValues.append(dataValue);
            } else if (!okTime) {
                qDebug() << "Problem reading time value: " << fields[timeColIndex];
            } else if (!okValue) {
                qDebug() << "Problem reading data value: " << fields[timeColIndex];
            }
        }

        lineNumber++;
    }

    file.close();

    // Create and populate the model
    TimeSeriesModel *model = new TimeSeriesModel();
    model->setData(xValues, yValues);

    // Set column labels
    QFileInfo fileInfo(csvFilePath);
    QString yLabel = valueColumn;
    model->setColumnLabels("Time (ms)", yLabel);

    return model;
}

TimeSeriesModel* TimeSeriesFactory::fromCsvByIndex(
    const QString& csvFilePath,
    int timeColumnIndex,
    int valueColumnIndex,
    double timeMultiplier,
    bool hasHeader,
    QChar separator)
{
    QVector<qreal> xValues;
    QVector<qreal> yValues;

    QFile file(csvFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Could not open file:" << file.errorString();
        return new TimeSeriesModel();
    }

    QTextStream in(&file);
    int lineNumber = 0;

    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line.isEmpty()) {
            continue;
        }

        QStringList fields = line.split(separator);

        // Skip header if present
        if (hasHeader && lineNumber == 0) {
            lineNumber++;
            continue;
        }

        // Process data rows
        if (fields.size() > qMax(timeColumnIndex, valueColumnIndex)) {
            bool okTime, okValue;
            qreal timeValue = fields[timeColumnIndex].toDouble(&okTime);
            qreal dataValue = fields[valueColumnIndex].toDouble(&okValue);

            if (okTime && okValue) {
                // Apply time multiplier to convert to milliseconds
                xValues.append(timeValue * timeMultiplier);
                yValues.append(dataValue);
            }
        }

        lineNumber++;
    }

    file.close();

    // Create and populate the model
    TimeSeriesModel *model = new TimeSeriesModel();
    model->setData(xValues, yValues);

    // Set column labels based on header if available
    if (hasHeader) {
        QFile headerFile(csvFilePath);
        if (headerFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QTextStream headerStream(&headerFile);
            QString headerLine = headerStream.readLine();
            QStringList headers = headerLine.split(separator);

            QString yLabel = (valueColumnIndex < headers.size()) ?
                                 headers[valueColumnIndex] : QString("Value");

            model->setColumnLabels("Time (ms)", yLabel);
            headerFile.close();
        }
    } else {
        model->setColumnLabels("Time (ms)", "Value");
    }

    return model;
}

TimeSeriesModel* TimeSeriesFactory::fromVectors(
    const QVector<qreal>& xValues,
    const QVector<qreal>& yValues)
{
    TimeSeriesModel *model = new TimeSeriesModel();
    model->setData(xValues, yValues);
    return model;
}

TimeSeriesModel* TimeSeriesFactory::fromFlightLog(
    const QString& flightLogPath,
    const QString& valueColumn)
{
    // Flight logs typically have a "seconds" column for time
    return fromCsv(flightLogPath, "seconds", valueColumn, 1000.0); // Convert seconds to ms
}

bool TimeSeriesFactory::processCsv(
    const QString& csvFilePath,
    const std::function<void(const QStringList&, int, QVector<qreal>&, QVector<qreal>&)>& processor,
    QVector<qreal>& xValues,
    QVector<qreal>& yValues,
    bool hasHeader,
    QChar separator)
{
    QFile file(csvFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Could not open file:" << file.errorString();
        return false;
    }

    QTextStream in(&file);
    int lineNumber = 0;

    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line.isEmpty()) {
            continue;
        }

        QStringList fields = line.split(separator);

        // Skip header if present
        if (hasHeader && lineNumber == 0) {
            lineNumber++;
            continue;
        }

        // Use the custom processor
        processor(fields, lineNumber - (hasHeader ? 1 : 0), xValues, yValues);

        lineNumber++;
    }

    file.close();
    return true;
}
