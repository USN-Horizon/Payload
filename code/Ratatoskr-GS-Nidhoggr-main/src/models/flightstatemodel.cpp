#include "flightstatemodel.h"

FlightStateModel::FlightStateModel(QObject *parent)
    : TimeSeriesModel(parent),
    m_currentState(Unknown)
{
    // Set column labels for flight state model
    setColumnLabels("Time (ms)", "State");
}

FlightState FlightStateModel::currentState() const
{
    return m_currentState;
}

QString FlightStateModel::currentStateName() const
{
    return FlightStateUtils::getStateName(m_currentState);
}

QColor FlightStateModel::currentStateColor() const
{
    return FlightStateUtils::getStateColor(m_currentState);
}

void FlightStateModel::setCurrentState(FlightState state)
{
    if (m_currentState != state) {
        FlightState oldState = m_currentState;
        m_currentState = state;

        // Emit state transition signal if it's a valid transition
        FlightEvent event = FlightStateUtils::getTransitionEvent(oldState, state);
        emit stateTransition(oldState, state, event);

        // Emit current state changed
        emit currentStateChanged();
    }
}

void FlightStateModel::setCurrentStateByInt(int stateInt)
{
    setCurrentState(intToState(stateInt));
}

FlightState FlightStateModel::intToState(int stateInt)
{
    if (stateInt >= 0 && stateInt <= 7) {
        return static_cast<FlightState>(stateInt);
    }
    return Unknown;
}

void FlightStateModel::appendStateData(qreal time, int state)
{
    // Convert state int to state enum
    FlightState flightState = intToState(state);

    // Add data point to the model
    appendData(time, static_cast<qreal>(state));

    // Update current state if this is the latest data point
    if (rowCount() == 1 || time > data(index(rowCount() - 2, 0)).toReal()) {
        setCurrentState(flightState);
    }
}

void FlightStateModel::setStateData(const QVector<qreal>& times, const QVector<int>& states)
{
    if (times.isEmpty() || states.isEmpty() || times.size() != states.size()) {
        return;
    }

    // Prepare data vectors
    QVector<qreal> xValues = times;
    QVector<qreal> yValues;

    // Convert state integers to qreal values for the model
    for (int state : states) {
        yValues.append(static_cast<qreal>(state));
    }

    // Set the data in the model
    setData(xValues, yValues);

    // Update current state to the last state in the data
    setCurrentState(intToState(states.last()));
}
