#include "timeseriesmodel.h"

TimeSeriesModel::TimeSeriesModel(QObject *parent)
    : QAbstractListModel(parent),
    m_xLabel("Time (ms)"),
    m_yLabel("Value")
{}

int TimeSeriesModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid()) return 0;
    return m_xData.size();
}

int TimeSeriesModel::columnCount(const QModelIndex &parent) const
{
    if (parent.isValid()) return 0;
    return COLUMN_COUNT; // 2: column 0 = X (time), column 1 = Y (value)
}

QVariant TimeSeriesModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= m_xData.size())
        return QVariant();

    switch (role) {
    case Qt::DisplayRole:
        return index.column() == Y_COLUMN ? m_yData.at(index.row())
                                          : m_xData.at(index.row());
    case XRole:
        return m_xData.at(index.row());
    case YRole:
        return m_yData.at(index.row());
    default:
        return QVariant();
    }
}

QHash<int, QByteArray> TimeSeriesModel::roleNames() const
{
    QHash<int, QByteArray> roles = QAbstractListModel::roleNames();
    roles[XRole] = "x";
    roles[YRole] = "y";
    return roles;
}

void TimeSeriesModel::appendData(qreal x, qreal y)
{
    const int row = m_xData.size();
    beginInsertRows(QModelIndex(), row, row);
    m_xData.append(x);
    m_yData.append(y);
    endInsertRows();
}

void TimeSeriesModel::removeData(int row)
{
    if (row < 0 || row >= m_xData.size()) return;
    beginRemoveRows(QModelIndex(), row, row);
    m_xData.removeAt(row);
    m_yData.removeAt(row);
    endRemoveRows();
}

QVariantMap TimeSeriesModel::get(int row) const
{
    QVariantMap map;
    if (row >= 0 && row < m_xData.size()) {
        map["x"] = m_xData.at(row);
        map["y"] = m_yData.at(row);
    }
    return map;
}

void TimeSeriesModel::setColumnLabels(const QString &xLabel, const QString &yLabel)
{
    m_xLabel = xLabel;
    m_yLabel = yLabel;
}

void TimeSeriesModel::setData(const QVector<qreal> &xValues, const QVector<qreal> &yValues)
{
    const int newCount = qMin(xValues.size(), yValues.size());
    const int oldCount = m_xData.size();

    // Remove old rows first so the view never holds stale indices
    if (oldCount > 0) {
        beginRemoveRows(QModelIndex(), 0, oldCount - 1);
        m_xData.clear();
        m_yData.clear();
        endRemoveRows();
    }

    if (newCount > 0) {
        beginInsertRows(QModelIndex(), 0, newCount - 1);
        m_xData = xValues.mid(0, newCount);
        m_yData = yValues.mid(0, newCount);
        endInsertRows();
    }
}

void TimeSeriesModel::clearData()
{
    if (m_xData.isEmpty()) return;
    beginRemoveRows(QModelIndex(), 0, m_xData.size() - 1);
    m_xData.clear();
    m_yData.clear();
    endRemoveRows();
}
