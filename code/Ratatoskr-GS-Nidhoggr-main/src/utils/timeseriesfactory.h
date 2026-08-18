#ifndef TIMESERIESFACTORY_H
#define TIMESERIESFACTORY_H

#include "models/timeseriesmodel.h"
#include <QString>
#include <QVector>
#include <QStringList>
#include <functional>

/**
 * @brief The TimeSeriesFactory class creates time series models from various data sources
 */
class TimeSeriesFactory
{
public:
    /**
     * @brief Create a time series model from a CSV file with custom column mappings
     * @param csvFilePath Path to the CSV file
     * @param timeColumn Column name or index for the time values
     * @param valueColumn Column name or index for the data values
     * @param timeMultiplier Multiplier to convert time units to milliseconds
     * @param hasHeader Whether the CSV has a header row
     * @param separator CSV separator character
     * @return A new TimeSeriesModel populated with the CSV data
     */
    static TimeSeriesModel* fromCsv(
        const QString& csvFilePath,
        const QString& timeColumn,
        const QString& valueColumn,
        double timeMultiplier = 1.0,
        bool hasHeader = true,
        QChar separator = ',');

    /**
     * @brief Create a time series model from a CSV file with column indices
     * @param csvFilePath Path to the CSV file
     * @param timeColumnIndex Index of the time column
     * @param valueColumnIndex Index of the value column
     * @param timeMultiplier Multiplier to convert time units to milliseconds
     * @param hasHeader Whether the CSV has a header row
     * @param separator CSV separator character
     * @return A new TimeSeriesModel populated with the CSV data
     */
    static TimeSeriesModel* fromCsvByIndex(
        const QString& csvFilePath,
        int timeColumnIndex,
        int valueColumnIndex,
        double timeMultiplier = 1.0,
        bool hasHeader = true,
        QChar separator = ',');

    /**
     * @brief Create a time series model from X and Y vectors
     * @param xValues Vector of X values (time)
     * @param yValues Vector of Y values (data)
     * @return A new TimeSeriesModel populated with the vector data
     */
    static TimeSeriesModel* fromVectors(
        const QVector<qreal>& xValues,
        const QVector<qreal>& yValues);

    /**
     * @brief Create a time series model from flight log data
     * @param flightLogPath Path to the flight log CSV
     * @param valueColumn Column to use for values (e.g. "altitude[m]", "velocity[m/s]")
     * @return A new TimeSeriesModel populated with the flight log data
     */
    static TimeSeriesModel* fromFlightLog(
        const QString& flightLogPath,
        const QString& valueColumn);

    /**
     * @brief Process a CSV file with a custom processor function
     * @param csvFilePath Path to the CSV file
     * @param processor Function that processes each line and adds data to vectors
     * @param xValues Vector to store X values
     * @param yValues Vector to store Y values
     * @param hasHeader Whether the CSV has a header row
     * @param separator CSV separator character
     * @return True if successful
     */
    static bool processCsv(
        const QString& csvFilePath,
        const std::function<void(const QStringList&, int, QVector<qreal>&, QVector<qreal>&)>& processor,
        QVector<qreal>& xValues,
        QVector<qreal>& yValues,
        bool hasHeader = true,
        QChar separator = ',');
};

#endif // TIMESERIESFACTORY_H
