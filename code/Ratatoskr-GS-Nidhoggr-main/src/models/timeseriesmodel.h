#ifndef TIMESERIESMODEL_H
#define TIMESERIESMODEL_H

#include <QAbstractListModel>
#include <QHash>
#include <QVector>

/**
 * @brief The TimeSeriesModel class represents generic time-series data
 * where X is time in milliseconds and Y is any double value.
 */
class TimeSeriesModel : public QAbstractListModel
{
    Q_OBJECT

public:
    enum Columns {
        X_COLUMN = 0,
        Y_COLUMN = 1,
        COLUMN_COUNT = 2
    };

    enum Roles {
        XRole = Qt::UserRole + 1,
        YRole
    };

    explicit TimeSeriesModel(QObject *parent = nullptr);

    // QAbstractListModel interface
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

    /**
     * @brief Append a single data point
     */
    Q_INVOKABLE void appendData(qreal x, qreal y);

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
     * @brief Set column labels (cosmetic metadata for consumers)
     */
    void setColumnLabels(const QString &xLabel, const QString &yLabel);

    /**
     * @brief Bulk-replace all data using proper insert/remove signals
     */
    void setData(const QVector<qreal> &xValues, const QVector<qreal> &yValues);

    /**
     * @brief Clear all data
     */
    void clearData();

private:
    QVector<qreal> m_xData;
    QVector<qreal> m_yData;
    QString m_xLabel;
    QString m_yLabel;
};

#endif // TIMESERIESMODEL_H
