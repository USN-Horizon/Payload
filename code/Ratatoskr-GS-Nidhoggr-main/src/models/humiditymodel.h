// humiditymodel.h
#pragma once
#include <QStandardItemModel>
#include "utils/humidityutils.h"

class HumidityModel : public QStandardItemModel {
    Q_OBJECT
public:
    enum Roles {
        XRole = Qt::UserRole + 1,
        YRole
    };

    explicit HumidityModel(QObject *parent = nullptr);
    explicit HumidityModel(int rows, int columns, QObject *parent = nullptr);

    QHash<int, QByteArray> roleNames() const override;

    void setHumidityList(const humidity_utils::HumidityList &newHumidityList);
    void appendData(qreal x, qreal y);

    Q_INVOKABLE void removeData(int row);
    Q_INVOKABLE QVariantMap get(int row) const;

    // Optional: If you want to explicitly maintain column count
    static const int X_COLUMN = 0;
    static const int Y_COLUMN = 1;
    static const int COLUMN_COUNT = 2;
};
