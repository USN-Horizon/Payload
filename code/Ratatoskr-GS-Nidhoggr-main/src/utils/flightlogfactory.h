#ifndef FLIGHTLOGFACTORY_H
#define FLIGHTLOGFACTORY_H

#include "models/timeseriesmodel.h"
#include "models/flightstatemodel.h"
#include "models/locationmodel.h"
#include <QString>
#include <QVector>

struct ParsedFlightLog {
    QVector<qreal> timeMs;
    QVector<int>   state;
    QVector<qreal> altitude;
    QVector<qreal> velocity;
    QVector<qreal> battery;
    QVector<qreal> latitude;
    QVector<qreal> longitude;
    bool valid = false;
};

/**
 * @brief The FlightLogFactory class creates time series models from flight log data
 */
class FlightLogFactory
{
public:
    /**
     * @brief Create a time series model for altitude data
     * @param flightLogPath Path to the flight log CSV
     * @return A new TimeSeriesModel populated with altitude data
     */
    static TimeSeriesModel* createAltitudeModel(const QString& flightLogPath);

    /**
     * @brief Create a time series model for velocity data
     * @param flightLogPath Path to the flight log CSV
     * @return A new TimeSeriesModel populated with velocity data
     */
    static TimeSeriesModel* createVelocityModel(const QString& flightLogPath);

    /**
     * @brief Create a time series model for battery data
     * @param flightLogPath Path to the flight log CSV
     * @return A new TimeSeriesModel populated with battery data
     */
    static TimeSeriesModel* createBatteryModel(const QString& flightLogPath);

    /**
     * @brief Create location model for location data
     * @param flightLogPath Path to the flight log CSV
     * @return A new TimeSeriesModel populated with battery data
     */
    static LocationModel* createLocationModel();


    /**
     * @brief Create a time series model for any column in the flight log
     * @param flightLogPath Path to the flight log CSV
     * @param columnName Name of the column to use for data
     * @return A new TimeSeriesModel populated with the specified data
     */
    static TimeSeriesModel* createModelFromCsv( const QString& flightLogPath, const QString& columnName);

    static TimeSeriesModel* createModel(const QString& columnName);
    /**
     * @brief Create a flight state model from flight log data
     * @param flightLogPath Path to the flight log CSV
     * @return A new FlightStateModel populated with state data
     */
    static FlightStateModel* createStateModelFromCsv(const QString& flightLogPath);

    static FlightStateModel* createStateModel();

    /**
     * @brief Parse a flight log CSV into column vectors in a single file pass
     * @param flightLogPath Path to the flight log CSV
     * @return ParsedFlightLog with all columns extracted and unit-converted
     */
    static ParsedFlightLog parse(const QString& flightLogPath);

    static void populateAltitude(TimeSeriesModel* model, const ParsedFlightLog& data);
    static void populateVelocity(TimeSeriesModel* model, const ParsedFlightLog& data);
    static void populateBattery(TimeSeriesModel* model, const ParsedFlightLog& data);
    static void populateLocation(LocationModel* model, const ParsedFlightLog& data);
    static void populateState(FlightStateModel* model, const ParsedFlightLog& data);
};

#endif // FLIGHTLOGFACTORY_H
