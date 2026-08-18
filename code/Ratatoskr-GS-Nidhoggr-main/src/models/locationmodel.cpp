#include "locationmodel.h"

LocationModel::LocationModel(QObject *parent)
    : QAbstractListModel(parent),
    m_xLabel("Time (ms)"),
    m_yLatLabel("Latitude"),
    m_yLonLabel("Longitude")
{}

int LocationModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid()) return 0;
    return m_xData.size();
}

int LocationModel::columnCount(const QModelIndex &parent) const
{
    if (parent.isValid()) return 0;
    return COLUMN_COUNT; // 2: column 0 = X (time), column 1 = Y (value)
}

QVariant LocationModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= m_xData.size())
        return QVariant();

    switch (role) {
    case Qt::DisplayRole:
        if (index.column() == X_COLUMN) return m_xData.at(index.row());
        return index.column() == Y_LAT_COLUMN ? m_yLatData.at(index.row()) :
                                                m_yLonData.at(index.row());
    case XRole:
        return m_xData.at(index.row());
    case YLatRole:
        return m_yLatData.at(index.row());
    case YLonRole:
        return m_yLonData.at(index.row());
    default:
        return QVariant();
    }
}

QHash<int, QByteArray> LocationModel::roleNames() const
{
    QHash<int, QByteArray> roles = QAbstractListModel::roleNames();
    roles[XRole] = "x";
    roles[YLatRole] = "y_lat";
    roles[YLonRole] = "y_lon";
    return roles;
}

void LocationModel::appendData(qreal x, qreal y_lat, qreal y_lon)
{
    const int row = m_xData.size();
    beginInsertRows(QModelIndex(), row, row);
    m_xData.append(x);
    m_yLatData.append(y_lat);
    m_yLonData.append(y_lon);
    endInsertRows();
}

void LocationModel::removeData(int row)
{
    if (row < 0 || row >= m_xData.size()) return;
    beginRemoveRows(QModelIndex(), row, row);
    m_xData.removeAt(row);
    m_yLatData.removeAt(row);
    m_yLonData.removeAt(row);
    endRemoveRows();
}

QVariantMap LocationModel::get(int row) const
{
    QVariantMap map;
    if (row >= 0 && row < m_xData.size()) {
        map["x"] = m_xData.at(row);
        map["y_lat"] = m_yLatData.at(row);
        map["y_lon"] = m_yLonData.at(row);
    }
    return map;
}

void LocationModel::setData(const QVector<qreal> &xValues, const QVector<qreal> &yLatValues, const QVector<qreal> &yLonValues)
{
    const int newCount = qMin(xValues.size(), qMin(yLatValues.size(), yLonValues.size()));
    const int oldCount = m_xData.size();

    // Remove old rows first so the view never holds stale indices
    if (oldCount > 0) {
        beginRemoveRows(QModelIndex(), 0, oldCount - 1);
        m_xData.clear();
        m_yLatData.clear();
        m_yLonData.clear();
        endRemoveRows();
    }

    if (newCount > 0) {
        beginInsertRows(QModelIndex(), 0, newCount - 1);
        m_xData = xValues.mid(0, newCount);
        m_yLatData = yLatValues.mid(0, newCount);
        m_yLonData = yLonValues.mid(0, newCount);
        endInsertRows();
    }
}

void LocationModel::clearData()
{
    if (m_xData.isEmpty()) return;
    beginRemoveRows(QModelIndex(), 0, m_xData.size() - 1);
    m_xData.clear();
    m_yLatData.clear();
    m_yLonData.clear();
    endRemoveRows();
}
