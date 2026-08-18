#ifndef LOCATIONMODEL_H
#define LOCATIONMODEL_H

#include <QAbstractListModel>
#include <QHash>
#include <QVector>

/**
 * @brief The TimeSeriesModel class represents generic time-series data
 * where X is time in milliseconds and Y is any double value.
 */
class LocationModel : public QAbstractListModel
{
    Q_OBJECT

public:
    enum Columns {
        X_COLUMN = 0,
        Y_LAT_COLUMN = 1,
        Y_LON_COLUMN = 2,
        COLUMN_COUNT = 3
    };

    enum Roles {
        XRole = Qt::UserRole + 1,
        YLatRole,
        YLonRole
    };

    explicit LocationModel(QObject *parent = nullptr);

    // QAbstractListModel interface
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

    /**
     * @brief Append a single data point
     */
    Q_INVOKABLE void appendData(qreal x, qreal y_lat, qreal y_lon);

    /**
     * @brief Remove data at specified row
     */
    Q_INVOKABLE void removeData(int row);

    /**
     * @brief Get data as a QVariantMap for a specific row
     * @return Map with "x" and "y" keys
     */
    Q_INVOKABLE QVariantMap get(int row) const;

    /**
     * @brief Bulk-replace all data using proper insert/remove signals
     */
    void setData(const QVector<qreal> &xValues, const QVector<qreal> &yLatValues, const QVector<qreal> &yLonValues);

    /**
     * @brief Clear all data
     */
    void clearData();

private:
    QVector<qreal> m_xData;
    QVector<qreal> m_yLatData;
    QVector<qreal> m_yLonData;
    QString m_xLabel;
    QString m_yLatLabel;
    QString m_yLonLabel;
};

#endif // LOCATIONMODEL_H
