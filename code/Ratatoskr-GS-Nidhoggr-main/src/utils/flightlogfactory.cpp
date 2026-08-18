#include "utils/flightlogfactory.h"
#include "utils/timeseriesfactory.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>

TimeSeriesModel* FlightLogFactory::createAltitudeModel(const QString& flightLogPath)
{
    TimeSeriesModel* model = TimeSeriesFactory::fromCsv(
        flightLogPath,
        "seconds",
        "altitude[m]",
        1000.0  // Convert seconds to milliseconds
        );

    // Set appropriate column labels
    model->setColumnLabels("Time (ms)", "Altitude (m)");
    return model;
}

TimeSeriesModel* FlightLogFactory::createVelocityModel(const QString& flightLogPath)
{
    TimeSeriesModel* model = TimeSeriesFactory::fromCsv(
        flightLogPath,
        "seconds",
        "velocity[m/s]",
        1000.0  // Convert seconds to milliseconds
        );

    // Set appropriate column labels
    model->setColumnLabels("Time (ms)", "Velocity (m/s)");
    return model;
}

TimeSeriesModel* FlightLogFactory::createBatteryModel(const QString& flightLogPath)
{
    // Use custom processing to convert decivolts to volts
    QVector<qreal> xValues;
    QVector<qreal> yValues;

    TimeSeriesFactory::processCsv(
        flightLogPath,
        [](const QStringList& fields, int lineNumber, QVector<qreal>& xValues, QVector<qreal>& yValues) {
            if (fields.size() >= 8) {
                bool okTime, okBattery;
                qreal timeValue = fields[0].toDouble(&okTime);  // seconds column
                qreal batteryValue = fields[7].toDouble(&okBattery);  // battery[decivolts] column

                if (okTime && okBattery) {
                    // Convert seconds to milliseconds and decivolts to volts
                    xValues.append(timeValue * 1000.0);
                    yValues.append(batteryValue / 10.0); // Convert decivolts to volts
                }
            }
        },
        xValues,
        yValues
        );

    TimeSeriesModel* model = new TimeSeriesModel();
    model->setData(xValues, yValues);
    model->setColumnLabels("Time (ms)", "Battery (V)");
    return model;
}

FlightStateModel* FlightLogFactory::createStateModel()
{
    // Create a new model
    FlightStateModel* model = new FlightStateModel();
    return model;
}

LocationModel* FlightLogFactory::createLocationModel()
{
    // Create a new model
    LocationModel* model = new LocationModel();
    return model;
}

/**
 * @brief Create a flight state model from flight log data
 * @param flightLogPath Path to the flight log CSV
 * @return A new FlightStateModel populated with state data
 */
FlightStateModel* FlightLogFactory::createStateModelFromCsv(const QString& flightLogPath)
{
    // Create a new model
    FlightStateModel* model = new FlightStateModel();

    // Load data from the flight log
    QVector<qreal> timeValues;
    QVector<int> stateValues;

    QFile file(flightLogPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Could not open file:" << file.errorString();
        return model;
    }

    QTextStream in(&file);
    bool hasHeader = true;
    int lineNumber = 0;

    // Find the state column index (typically 1 based on your CSV sample)
    int stateColumnIndex = 1;
    int timeColumnIndex = 0;

    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line.isEmpty()) {
            continue;
        }

        // Skip header row
        if (hasHeader && lineNumber == 0) {
            lineNumber++;
            continue;
        }

        // Parse the line with European number format handling
        // First, split by commas but be careful with quoted values
        QStringList fields;
        bool inQuotes = false;
        QString currentField;

        for (int i = 0; i < line.size(); i++) {
            QChar c = line.at(i);

            if (c == '\"') {
                inQuotes = !inQuotes;
            } else if (c == ',' && !inQuotes) {
                fields.append(currentField);
                currentField.clear();
            } else {
                currentField.append(c);
            }
        }
        // Add the last field
        fields.append(currentField);

        // Ensure we have both time and state columns
        if (fields.size() > qMax(timeColumnIndex, stateColumnIndex)) {
            bool okTime, okState;

            // Parse time value
            QString timeStr = fields[timeColumnIndex];
            timeStr.remove('\"');
            timeStr.replace(',', '.');
            qreal timeValue = timeStr.toDouble(&okTime);

            // Parse state value
            QString stateStr = fields[stateColumnIndex];
            stateStr.remove('\"');
            int stateValue = stateStr.toInt(&okState);

            if (okTime && okState) {
                // Convert seconds to milliseconds
                timeValues.append(timeValue * 1000.0);
                stateValues.append(stateValue);
                qDebug() << timeValue * 1000.0 << ", " << stateValue;
            } else {
                if (!okTime) {
                    qDebug() << "Problem reading time value:" << fields[timeColumnIndex];
                }
                if (!okState) {
                    qDebug() << "Problem reading state value:" << fields[stateColumnIndex];
                }
            }
        }

        lineNumber++;
    }

    file.close();

    // Update the model with state data
    model->setStateData(timeValues, stateValues);

    qDebug() << "Loaded state model with" << model->rowCount() << "data points";

    return model;
}

TimeSeriesModel* FlightLogFactory::createModelFromCsv(const QString& flightLogPath, const QString& columnName)
{
    // Handle special cases
    if (columnName == "altitude[m]") {
        return createAltitudeModel(flightLogPath);
    } else if (columnName == "velocity[m/s]") {
        return createVelocityModel(flightLogPath);
    } else if (columnName == "battery[decivolts]") {
        return createBatteryModel(flightLogPath);
    }

    // Default case: just use the column as-is
    return TimeSeriesFactory::fromCsv(
        flightLogPath,
        "seconds",
        columnName,
        1000.0  // Convert seconds to milliseconds
        );
}

TimeSeriesModel* FlightLogFactory::createModel(const QString& columnName)
{
    auto *model = new TimeSeriesModel();
    model->setColumnLabels("Time (ms)", columnName);
    return model;
}

ParsedFlightLog FlightLogFactory::parse(const QString& flightLogPath)
{
    ParsedFlightLog result;

    QFile file(flightLogPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "FlightLogFactory::parse: could not open file:" << file.errorString();
        return result;
    }

    QTextStream in(&file);
    if (in.atEnd()) { file.close(); return result; }

    // Find column indices from header
    const QStringList headers = in.readLine().split(',');
    int colTime = -1, colState = 1, colAlt = -1, colVel = -1;
    int colBattery = -1, colLat = -1, colLon = -1;

    for (int i = 0; i < headers.size(); ++i) {
        const QString h = headers[i].trimmed();
        if      (h == "ts[deciseconds]")    colTime    = i;
        else if (h == "state")              colState   = i;
        else if (h == "altitude[m]")        colAlt     = i;
        else if (h == "velocity[m/s]")      colVel     = i;
        else if (h == "battery[decivolts]") colBattery = i;
        else if (h == "lat[deg/10000]")     colLat     = i;
        else if (h == "lon[deg/10000]")     colLon     = i;
    }

    if (colTime == -1) {
        qDebug() << "FlightLogFactory::parse: 'seconds' column not found. Header:" << headers;
        file.close();
        return result;
    }

    bool firstLine = true;
    qreal timeOffsetMs = 0;


    while (!in.atEnd()) {
        const QString line = in.readLine();
        if (line.isEmpty()) continue;

        const QStringList fields = line.split(',');
        if (fields.size() <= colTime) continue;

        bool okTime;
        // the source is in deciseconds, as set by cats, so we need to go desiseconds -> milliseconds == 10^2
        const qreal timeMs = fields[colTime].toDouble(&okTime) * 100.0 - timeOffsetMs;

        // depending on when the flight computer was started, the first row could have a very large ts number.
        // therefore we need to use the first row as the baseline
        if (firstLine) {
            timeOffsetMs = timeMs;
            firstLine = false;
        }

        if (!okTime) continue;

        auto readReal = [&](int col, double scale = 1.0) -> qreal {
            if (col == -1 || fields.size() <= col) return 0.0;
            bool ok;
            const qreal v = fields[col].toDouble(&ok);
            return ok ? v * scale : 0.0;
        };
        auto readInt = [&](int col) -> int {
            if (col == -1 || fields.size() <= col) return 0;
            bool ok;
            const int v = fields[col].toInt(&ok);
            return ok ? v : 0;
        };

        result.timeMs.append(timeMs);
        result.state.append(readInt(colState));
        if (colAlt     != -1) result.altitude.append(readReal(colAlt));
        if (colVel     != -1) result.velocity.append(readReal(colVel));
        if (colBattery != -1) result.battery.append(readReal(colBattery, 0.1));
        if (colLat     != -1) result.latitude.append(readReal(colLat, 1.0 / 10000.0));
        if (colLon     != -1) result.longitude.append(readReal(colLon, 1.0 / 10000.0));
    }

    file.close();
    result.valid = true;
    qDebug() << "FlightLogFactory::parse: loaded" << result.timeMs.size() << "rows";
    return result;
}

void FlightLogFactory::populateAltitude(TimeSeriesModel* model, const ParsedFlightLog& data)
{
    model->setData(data.timeMs, data.altitude);
    model->setColumnLabels("Time (ms)", "Altitude (m)");
}

void FlightLogFactory::populateVelocity(TimeSeriesModel* model, const ParsedFlightLog& data)
{
    model->setData(data.timeMs, data.velocity);
    model->setColumnLabels("Time (ms)", "Velocity (m/s)");
}

void FlightLogFactory::populateBattery(TimeSeriesModel* model, const ParsedFlightLog& data)
{
    model->setData(data.timeMs, data.battery);
    model->setColumnLabels("Time (ms)", "Battery (V)");
}

void FlightLogFactory::populateLocation(LocationModel* model, const ParsedFlightLog& data)
{
    model->setData(data.timeMs, data.latitude, data.longitude);
}

void FlightLogFactory::populateState(FlightStateModel* model, const ParsedFlightLog& data)
{
    model->setStateData(data.timeMs, data.state);
}
