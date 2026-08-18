// humiditymodel.cpp
#include "humiditymodel.h"
#include <QVariant>

HumidityModel::HumidityModel(QObject *parent)
    : QStandardItemModel(0, COLUMN_COUNT, parent)
{
    // Set horizontal header labels
    setHorizontalHeaderLabels({"X Value", "Y Value"});

    // Example data
    appendData(0, 1);
    appendData(1, 3);
    appendData(2, 2);
}

HumidityModel::HumidityModel(int rows, int columns, QObject *parent)
    : QStandardItemModel(rows, columns, parent)
{
    setHorizontalHeaderLabels({"X Value", "Y Value"});
}

QHash<int, QByteArray> HumidityModel::roleNames() const {
    QHash<int, QByteArray> roles = QStandardItemModel::roleNames();
    roles[XRole] = "x";
    roles[YRole] = "y";
    return roles;
}

void HumidityModel::setHumidityList(const humidity_utils::HumidityList &newHumidityList)
{
    // Clear existing data
    removeRows(0, rowCount());

    // Reserve rows for better performance
    setRowCount(newHumidityList.size());

    // Add data from the humidity list
    for (int i = 0; i < newHumidityList.size(); ++i) {
        const TimestampedHumidity& data = newHumidityList.at(i);
        qreal x = static_cast<qreal>(data.timestamp);
        qreal y = data.humidity;

        // Create and set items for X and Y columns
        QStandardItem *xItem = new QStandardItem();
        xItem->setData(x, Qt::DisplayRole);
        xItem->setData(x, XRole);

        QStandardItem *yItem = new QStandardItem();
        yItem->setData(y, Qt::DisplayRole);
        yItem->setData(y, YRole);

        setItem(i, X_COLUMN, xItem);
        setItem(i, Y_COLUMN, yItem);
    }
}

void HumidityModel::appendData(qreal x, qreal y)
{
    int row = rowCount();

    // Create items for new row
    QStandardItem *xItem = new QStandardItem();
    xItem->setData(x, Qt::DisplayRole);
    xItem->setData(x, XRole);

    QStandardItem *yItem = new QStandardItem();
    yItem->setData(y, Qt::DisplayRole);
    yItem->setData(y, YRole);

    // Create a row with the items
    QList<QStandardItem*> rowItems;
    rowItems << xItem << yItem;

    // Append the row
    appendRow(rowItems);
}

void HumidityModel::removeData(int row)
{
    if (row >= 0 && row < rowCount()) {
        removeRow(row);
    }
}

QVariantMap HumidityModel::get(int row) const
{
    QVariantMap map;

    if (row >= 0 && row < rowCount()) {
        QModelIndex xIndex = index(row, X_COLUMN);
        QModelIndex yIndex = index(row, Y_COLUMN);

        map["x"] = data(xIndex, Qt::DisplayRole).toReal();
        map["y"] = data(yIndex, Qt::DisplayRole).toReal();
    }

    return map;
}
