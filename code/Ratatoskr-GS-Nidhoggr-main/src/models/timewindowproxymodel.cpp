#include "timewindowproxymodel.h"

TimeWindowProxyModel::TimeWindowProxyModel(QObject *parent)
    : QAbstractProxyModel(parent),
    m_timer(nullptr),
    m_windowSize(60.0), // Default 60 second window
    m_invertTimeAxis(true),
    m_lastTime(0.0),
    m_lo(0),
    m_count(0)
{
    // Drive the sliding window from a steady tick so old points expire even when
    // no new data arrives. 50ms == 20fps, smooth enough for scrolling.
    connect(&m_updateTimer, &QTimer::timeout, this, &TimeWindowProxyModel::tick);
    m_updateTimer.start(50);
}

CountupTimer* TimeWindowProxyModel::timer() const
{
    return m_timer;
}

void TimeWindowProxyModel::setTimer(CountupTimer* timer)
{
    if (m_timer == timer)
        return;

    if (m_timer)
        disconnect(m_timer, &CountupTimer::secondsChanged, this, &TimeWindowProxyModel::tick);

    m_timer = timer;

    if (m_timer)
        connect(m_timer, &CountupTimer::secondsChanged, this, &TimeWindowProxyModel::tick);

    emit timerChanged();
    recompute(true);
    emit currentTimeChanged();
}

double TimeWindowProxyModel::windowSize() const
{
    return m_windowSize;
}

void TimeWindowProxyModel::setWindowSize(double size)
{
    if (m_windowSize == size)
        return;

    m_windowSize = size;
    emit windowSizeChanged();
    // The window can grow at the front, which is not a forward slide, so reset.
    recompute(true);
}

bool TimeWindowProxyModel::invertTimeAxis() const
{
    return m_invertTimeAxis;
}

void TimeWindowProxyModel::setInvertTimeAxis(bool invert)
{
    if (m_invertTimeAxis == invert)
        return;

    m_invertTimeAxis = invert;
    emit invertTimeAxisChanged();
}

double TimeWindowProxyModel::currentTime() const
{
    return m_timer ? m_timer->seconds() : 0.0;
}

QVariantMap TimeWindowProxyModel::get(int row) const
{
    if (!sourceModel() || row < 0 || row >= m_count)
        return QVariantMap();

    QVariantMap result;
    QMetaObject::invokeMethod(
        const_cast<QAbstractItemModel*>(sourceModel()), "get",
        Q_RETURN_ARG(QVariantMap, result),
        Q_ARG(int, m_lo + row)
    );
    return result;
}

void TimeWindowProxyModel::setSourceModel(QAbstractItemModel *sourceModel)
{
    if (this->sourceModel()) {
        disconnect(this->sourceModel(), nullptr, this, nullptr);
    }

    QAbstractProxyModel::setSourceModel(sourceModel);

    if (sourceModel) {
        connect(sourceModel, &QAbstractItemModel::rowsInserted,
                this, &TimeWindowProxyModel::onSourceRowsInserted);
        connect(sourceModel, &QAbstractItemModel::rowsRemoved,
                this, &TimeWindowProxyModel::onSourceReset);
        connect(sourceModel, &QAbstractItemModel::modelReset,
                this, &TimeWindowProxyModel::onSourceReset);
        connect(sourceModel, &QAbstractItemModel::layoutChanged,
                this, &TimeWindowProxyModel::onSourceReset);
        connect(sourceModel, &QAbstractItemModel::dataChanged,
                this, &TimeWindowProxyModel::onSourceDataChanged);
    }

    recompute(true);
}

QModelIndex TimeWindowProxyModel::mapFromSource(const QModelIndex &sourceIndex) const
{
    if (!sourceIndex.isValid())
        return QModelIndex();

    const int row = sourceIndex.row() - m_lo;
    if (row < 0 || row >= m_count)
        return QModelIndex();

    return createIndex(row, sourceIndex.column());
}

QModelIndex TimeWindowProxyModel::mapToSource(const QModelIndex &proxyIndex) const
{
    if (!proxyIndex.isValid() || !sourceModel())
        return QModelIndex();

    return sourceModel()->index(m_lo + proxyIndex.row(), proxyIndex.column());
}

QModelIndex TimeWindowProxyModel::index(int row, int column, const QModelIndex &parent) const
{
    if (parent.isValid() || row < 0 || row >= m_count || column < 0 || column >= columnCount())
        return QModelIndex();

    return createIndex(row, column);
}

QModelIndex TimeWindowProxyModel::parent(const QModelIndex &) const
{
    return QModelIndex();
}

int TimeWindowProxyModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;
    return m_count;
}

int TimeWindowProxyModel::columnCount(const QModelIndex &parent) const
{
    if (parent.isValid() || !sourceModel())
        return 0;
    return sourceModel()->columnCount();
}

double TimeWindowProxyModel::sourceTime(int sourceRow) const
{
    return sourceModel()->data(sourceModel()->index(sourceRow, 0), Qt::DisplayRole).toDouble();
}

int TimeWindowProxyModel::lowerBound(double valueMs, int srcRows) const
{
    int lo = 0, hi = srcRows;
    while (lo < hi) {
        const int mid = lo + (hi - lo) / 2;
        if (sourceTime(mid) < valueMs)
            lo = mid + 1;
        else
            hi = mid;
    }
    return lo;
}

int TimeWindowProxyModel::upperBound(double valueMs, int srcRows) const
{
    int lo = 0, hi = srcRows;
    while (lo < hi) {
        const int mid = lo + (hi - lo) / 2;
        if (sourceTime(mid) <= valueMs)
            lo = mid + 1;
        else
            hi = mid;
    }
    return lo;
}

void TimeWindowProxyModel::recompute(bool forceReset)
{
    const int srcRows = sourceModel() ? sourceModel()->rowCount() : 0;

    int newLo, newCount;
    if (srcRows <= 0) {
        newLo = 0;
        newCount = 0;
    } else if (!m_timer) {
        // No clock yet: show the whole source so static/imported data still renders.
        newLo = 0;
        newCount = srcRows;
    } else {
        const double currentMs = m_timer->seconds() * 1000.0;
        const double lowMs = currentMs - m_windowSize * 1000.0;
        const int lo = lowerBound(lowMs, srcRows);
        const int hi = upperBound(currentMs, srcRows);
        newLo = lo;
        newCount = qMax(0, hi - lo);
    }

    const int oldLo = m_lo;
    const int oldCount = m_count;
    const int oldHi = oldLo + oldCount;   // one-past-end (source coords)
    const int newHi = newLo + newCount;

    // Fast path: the window slid forward and still overlaps the previous one, so
    // we can express the change as a front-removal plus a back-insertion.
    const bool simpleSlide = !forceReset && oldCount > 0 &&
                             newLo >= oldLo && newHi >= oldHi && newLo <= oldHi;

    if (!simpleSlide) {
        beginResetModel();
        m_lo = newLo;
        m_count = newCount;
        endResetModel();
        return;
    }

    const int removeFront = newLo - oldLo;
    if (removeFront > 0) {
        beginRemoveRows(QModelIndex(), 0, removeFront - 1);
        m_lo = newLo;
        m_count = oldCount - removeFront;
        endRemoveRows();
    }

    const int addBack = newHi - oldHi;
    if (addBack > 0) {
        beginInsertRows(QModelIndex(), m_count, m_count + addBack - 1);
        m_count += addBack;
        endInsertRows();
    }
}

void TimeWindowProxyModel::tick()
{
    recompute(false);

    const double cur = currentTime();
    if (cur != m_lastTime) {
        m_lastTime = cur;
        emit currentTimeChanged();
    }
}

void TimeWindowProxyModel::onSourceRowsInserted()
{
    // Appends slide the window forward; keep it incremental.
    recompute(false);
}

void TimeWindowProxyModel::onSourceReset()
{
    recompute(true);
}

void TimeWindowProxyModel::onSourceDataChanged(const QModelIndex &topLeft,
                                               const QModelIndex &bottomRight,
                                               const QVector<int> &roles)
{
    if (!topLeft.isValid() || !bottomRight.isValid())
        return;

    // Clip the changed source range to the visible window and forward it.
    const int first = qMax(topLeft.row(), m_lo);
    const int last = qMin(bottomRight.row(), m_lo + m_count - 1);
    if (first > last)
        return;

    emit dataChanged(index(first - m_lo, topLeft.column()),
                     index(last - m_lo, bottomRight.column()),
                     roles);
}
