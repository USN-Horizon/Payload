#ifndef FLIGHTSTATEUTILS_H
#define FLIGHTSTATEUTILS_H

#include <QString>
#include <QMap>
#include <QColor>

/**
 * @brief The FlightState enum represents the different states of the flight
 */
enum FlightState {
    Unknown,
    Calibrating,
    Ready,
    Thrusting,
    Coasting,
    Drogue,
    Main,
    Touchdown,
};

/**
 * @brief The FlightEvent enum represents the events that trigger state transitions
 */
enum FlightEvent {
    EV_None,
    EV_Ready,
    EV_Liftoff,
    EV_Burnout,
    EV_Apogee,
    EV_Main_Deployment,
    EV_Touchdown,
};

/**
 * @brief The FlightStateUtils class provides utility functions for working with flight states
 */
class FlightStateUtils {
public:
    /**
     * @brief Get the name of a flight state
     * @param state The state enum value
     * @return The name of the state as a string
     */
    static QString getStateName(FlightState state) {
        static const QMap<FlightState, QString> stateNames = {
            {Unknown, "Unknown"},
            {Calibrating, "Calibrating"},
            {Ready, "Ready"},
            {Thrusting, "Thrusting"},
            {Coasting, "Coasting"},
            {Drogue, "Drogue"},
            {Main, "Main"},
            {Touchdown, "Touchdown"},
        };

        return stateNames.value(state, "Unknown");
    }

    /**
     * @brief Get a color associated with a flight state for visualization
     * @param state The state enum value
     * @return A color for the state
     */
    static QColor getStateColor(FlightState state) {
        static const QMap<FlightState, QColor> stateColors = {
            {Unknown, QColor("#7f8c8d")},       // Gray
            {Calibrating, QColor("#3498db")},  // Blue
            {Ready, QColor("#2ecc71")},        // Green
            {Thrusting, QColor("#e74c3c")},    // Red
            {Coasting, QColor("#f39c12")},     // Orange
            {Drogue, QColor("#9b59b6")},       // Purple
            {Main, QColor("#1abc9c")},         // Teal
            {Touchdown, QColor("#27ae60")},    // Dark Green
        };

        return stateColors.value(state, QColor("#7f8c8d"));
    }

    /**
     * @brief Get the event that triggers a transition between states
     * @param fromState The starting state
     * @param toState The ending state
     * @return The event that triggers the transition, or EV_None if not a valid transition
     */
    static FlightEvent getTransitionEvent(FlightState fromState, FlightState toState) {
        if (fromState == Calibrating && toState == Ready) return EV_Ready;
        if (fromState == Ready && toState == Thrusting) return EV_Liftoff;
        if (fromState == Thrusting && toState == Coasting) return EV_Burnout;
        if (fromState == Coasting && toState == Drogue) return EV_Apogee;
        if (fromState == Drogue && toState == Main) return EV_Main_Deployment;
        if (fromState == Main && toState == Touchdown) return EV_Touchdown;

        return EV_None;
    }

    /**
     * @brief Get the name of a flight event
     * @param event The event enum value
     * @return The name of the event as a string
     */
    static QString getEventName(FlightEvent event) {
        static const QMap<FlightEvent, QString> eventNames = {
            {EV_None, "None"},
            {EV_Ready, "EV_Ready"},
            {EV_Liftoff, "EV_Liftoff"},
            {EV_Burnout, "EV_Burnout"},
            {EV_Apogee, "EV_Apogee"},
            {EV_Main_Deployment, "EV_Main_Deployment"},
            {EV_Touchdown, "EV_Touchdown"},
        };

        return eventNames.value(event, "Unknown");
    }
};

#endif // FLIGHTSTATEUTILS_H
