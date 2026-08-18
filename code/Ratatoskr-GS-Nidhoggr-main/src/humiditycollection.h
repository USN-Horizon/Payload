#pragma once
#include <QObject>
#include <QtQml/qqml.h>
#include "humiditycontroller.h"
#include "models/humiditymodel.h"

class HumidityCollection : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(HumidityController* humidityController MEMBER m_humidityController CONSTANT )
    Q_PROPERTY(HumidityModel* humidityModel MEMBER m_humidityModel CONSTANT )

public:
    explicit HumidityCollection(QObject *parent = nullptr);

    // Q_INVOKABLE makes this method callable from QML
    Q_INVOKABLE bool importMissionData(const QString& filePath);

private:
    void registerControllers();
    HumidityController* m_humidityController { nullptr };
    HumidityModel* m_humidityModel { nullptr };
};
