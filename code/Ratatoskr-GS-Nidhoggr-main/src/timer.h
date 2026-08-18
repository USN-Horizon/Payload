// Timer.h
#ifndef TIMER_H
#define TIMER_H

#include <QObject>
#include <QTimer>
#include <QDateTime>

class CountupTimer : public QObject
{
    Q_OBJECT
    Q_PROPERTY(double seconds READ seconds NOTIFY secondsChanged)
    Q_PROPERTY(QString timeString READ timeString NOTIFY secondsChanged)
    Q_PROPERTY(bool running READ isRunning NOTIFY runningChanged)

public:
    explicit CountupTimer(QObject *parent = nullptr);

    double seconds() const;
    QString timeString() const;
    bool isRunning() const { return m_running; }

public slots:
    void start();
    void resume();
    void stop();
    void reset();

signals:
    void secondsChanged();
    void runningChanged();

private slots:
    void updateTime();

private:
    QTimer m_updateTimer;
    QDateTime m_startTime;
    // The time that shows when reset is hit
    const double m_startValue;
    // the time that has passed the moment stop was clicked
    double m_pausedElapsed;
    bool m_running;
};

#endif // TIMER_H
