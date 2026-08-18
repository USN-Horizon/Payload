#ifndef FLIGHTSTATEMODEL_H
#define FLIGHTSTATEMODEL_H

#include "timeseriesmodel.h"
#include "utils/flightstateutils.h"
#include <QObject>
#include <QString>

/**
 * @brief The FlightStateModel class extends TimeSeriesModel to handle flight state data
 */
class FlightStateModel : public TimeSeriesModel
{
    Q_OBJECT

    Q_PROPERTY(FlightState currentState READ currentState NOTIFY currentStateChanged)
    Q_PROPERTY(QString currentStateName READ currentStateName NOTIFY currentStateChanged)
    Q_PROPERTY(QColor currentStateColor READ currentStateColor NOTIFY currentStateChanged)

public:
    explicit FlightStateModel(QObject *parent = nullptr);

    /**
     * @brief Get the current flight state
     * @return The current flight state
     */
    FlightState currentState() const;

    /**
     * @brief Get the name of the current flight state
     * @return The name of the current state
     */
    QString currentStateName() const;

    /**
     * @brief Get the color associated with the current flight state
     * @return The color of the current state
     */
    QColor currentStateColor() const;

    /**
     * @brief Set the current flight state
     * @param state The new flight state
     */
    void setCurrentState(FlightState state);

    /**
     * @brief Set the current flight state by integer value
     * @param stateInt The new flight state as an integer (0-7)
     */
    Q_INVOKABLE void setCurrentStateByInt(int stateInt);

    /**
     * @brief Convert an integer to a FlightState enum value
     * @param stateInt The state as an integer (0-7)
     * @return The corresponding FlightState enum value
     */
    static FlightState intToState(int stateInt);

    /**
     * @brief Add a state data point
     * @param time Time in milliseconds
     * @param state State value (0-7)
     */
    Q_INVOKABLE void appendStateData(qreal time, int state);

    /**
     * @brief Update the model with new state data
     * @param times Vector of time values
     * @param states Vector of state values
     */
    void setStateData(const QVector<qreal>& times, const QVector<int>& states);

signals:
    /**
     * @brief Signal emitted when the current state changes
     */
    void currentStateChanged();

    /**
     * @brief Signal emitted when a state transition occurs
     * @param fromState The previous state
     * @param toState The new state
     * @param event The event that triggered the transition
     */
    void stateTransition(FlightState fromState, FlightState toState, FlightEvent event);

private:
    FlightState m_currentState;
};

#endif // FLIGHTSTATEMODEL_H
