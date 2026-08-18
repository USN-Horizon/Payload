// Timer.cpp
#include "timer.h"
#include <cmath>

CountupTimer::CountupTimer(QObject *parent)
    : QObject(parent), m_startValue(0.0), m_running(false)
{
    connect(&m_updateTimer, &QTimer::timeout, this, &CountupTimer::updateTime);
    // Update display more frequently for smoother appearance
    m_updateTimer.setInterval(80); // 80ms. Not round number so that the last digit also updates
}

double CountupTimer::seconds() const
{
    if (!m_running)
        return m_startValue + m_pausedElapsed;

    // Calculate exact time difference in seconds
    // time is shown as the difference between m_startTime (constant) and currentDateTime
    qint64 msecsSinceStart = m_startTime.msecsTo(QDateTime::currentDateTime());
    return m_startValue + (msecsSinceStart / 1000.0);
}

QString CountupTimer::timeString() const
{
    // Get current seconds value
    double currentSeconds = seconds();

    // Format the time as +MM:SS.ss or -MM:SS.ss
    double absSeconds = std::abs(currentSeconds);
    int minutes = static_cast<int>(absSeconds) / 60;
    double secs = std::fmod(absSeconds, 60.0);

    QString sign = (currentSeconds < 0) ? "-" : "+";

    // Format with two decimal places for subseconds
    return QString("%1%2:%3")
        .arg(sign)
        .arg(minutes, 2, 10, QChar('0'))
        .arg(secs, 5, 'f', 2, QChar('0'));
}

void CountupTimer::start()
{
    m_startTime = QDateTime::currentDateTime();

    if (!m_running) {
        m_running = true;
        m_updateTimer.start();
        emit runningChanged();
    }

    emit secondsChanged();
}

void CountupTimer::resume()
{
    m_startTime = QDateTime::currentDateTime().addMSecs(-1000 * m_pausedElapsed);

    if (!m_running) {
        m_running = true;
        m_updateTimer.start();
        emit runningChanged();
    }

    emit secondsChanged();
}

void CountupTimer::stop()
{
    if (m_running) {
        // Save elapsed time before flipping m_running, so seconds() still reads live time
        m_pausedElapsed = m_startTime.msecsTo(QDateTime::currentDateTime()) / 1000.0;

        m_running = false;
        m_updateTimer.stop();

        emit runningChanged();
        emit secondsChanged();
    }
}

void CountupTimer::reset()
{
    bool wasRunning = m_running;
    if (wasRunning) {
        stop();
    }

    m_startTime = QDateTime::currentDateTime();
    m_pausedElapsed = 0;

    //m_startValue = -10.0;
    emit secondsChanged();

    // Optionally restart if it was running
    if (wasRunning) {
        start();
    }
}

void CountupTimer::updateTime()
{
    // This just triggers UI updates, actual time is calculated on demand
    emit secondsChanged();
}
