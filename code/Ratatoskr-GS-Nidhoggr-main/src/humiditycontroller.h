#pragma once
#include <QObject>
#include <QtQml/qqml.h>
#include "utils/humidityutils.h"

class HumidityController : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(QString name READ name WRITE setname NOTIFY nameChanged FINAL)
    Q_PROPERTY(humidity_utils::HumidityList humidityList READ humidityList WRITE setHumidityList NOTIFY humidityListChanged FINAL)

public:
    explicit HumidityController(QObject *parent = nullptr);

    QString name() const;
    void setname(const QString &newName);

    humidity_utils::HumidityList humidityList() const;
    void setHumidityList(const humidity_utils::HumidityList &newHumidityList);

signals:
    void nameChanged();
    void humidityListChanged();

private:
    QString m_name;
    humidity_utils::HumidityList m_humidityList;
};
