#ifndef TIMEWINDOWPROXYMODEL_H
#define TIMEWINDOWPROXYMODEL_H

#include <QAbstractProxyModel>
#include <QTimer>
#include <QVariantMap>
#include "timer.h"

/**
 * @brief Exposes a sliding time window over an append-only, time-ordered source
 * model as a contiguous tail of rows.
 *
 * The source model keeps the full session history (it is the in-memory store
 * used for export); this proxy only ever presents the rows whose time falls in
 * [currentTime - windowSize, currentTime]. Because the source's time column
 * (column 0, Qt::DisplayRole) is monotonically increasing, the visible window is
 * always a contiguous suffix [lo, hi) of the source, found with a binary search.
 * Per-frame cost is therefore O(log n) + O(points entering/leaving) rather than
 * the O(n) full rescan a QSortFilterProxyModel would do on every invalidate.
 *
 * Time values are passed through unchanged (absolute). Consumers that want a
 * "time ago" axis scroll the axis range against currentTime instead of having
 * every point relabelled each tick.
 */
class TimeWindowProxyModel : public QAbstractProxyModel
{
    Q_OBJECT

    Q_PROPERTY(CountupTimer* timer READ timer WRITE setTimer NOTIFY timerChanged)
    Q_PROPERTY(double windowSize READ windowSize WRITE setWindowSize NOTIFY windowSizeChanged)
    Q_PROPERTY(bool invertTimeAxis READ invertTimeAxis WRITE setInvertTimeAxis NOTIFY invertTimeAxisChanged)
    Q_PROPERTY(double currentTime READ currentTime NOTIFY currentTimeChanged)

public:
    explicit TimeWindowProxyModel(QObject *parent = nullptr);

    // Timer that provides the current time
    CountupTimer* timer() const;
    void setTimer(CountupTimer* timer);

    // Size of the time window in seconds
    double windowSize() const;
    void setWindowSize(double size);

    // Retained for QML API compatibility. Time is now stored/exposed as absolute
    // values and the axis is scrolled instead of relabelling points, so this flag
    // no longer transforms data; newest samples sit at the high end of the axis.
    bool invertTimeAxis() const;
    void setInvertTimeAxis(bool invert);

    // Current time value in seconds (from the timer)
    double currentTime() const;

    // Get data for a proxy row, delegating to the source model's get()
    Q_INVOKABLE QVariantMap get(int row) const;

    // QAbstractProxyModel interface
    void setSourceModel(QAbstractItemModel *sourceModel) override;
    QModelIndex mapFromSource(const QModelIndex &sourceIndex) const override;
    QModelIndex mapToSource(const QModelIndex &proxyIndex) const override;

    // QAbstractItemModel interface
    QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex &child) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;

private slots:
    void tick();
    void onSourceRowsInserted();
    void onSourceReset();
    void onSourceDataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight,
                             const QVector<int> &roles);

signals:
    void timerChanged();
    void windowSizeChanged();
    void invertTimeAxisChanged();
    void currentTimeChanged();

private:
    // Time of a source row (column 0, Qt::DisplayRole), in ms
    double sourceTime(int sourceRow) const;
    // First source row with time >= value (or srcRows if none)
    int lowerBound(double valueMs, int srcRows) const;
    // First source row with time > value (or srcRows if none)
    int upperBound(double valueMs, int srcRows) const;
    // Recompute the [m_lo, m_lo + m_count) window, emitting incremental
    // insert/remove signals when it slides forward, or a reset otherwise.
    void recompute(bool forceReset);

    CountupTimer* m_timer;
    double m_windowSize;
    bool m_invertTimeAxis;
    QTimer m_updateTimer;
    double m_lastTime;

    int m_lo;     // first visible source row
    int m_count;  // number of visible rows
};

#endif // TIMEWINDOWPROXYMODEL_H
