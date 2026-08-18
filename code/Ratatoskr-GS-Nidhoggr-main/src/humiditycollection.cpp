#include "humiditycollection.h"

#include <QDebug>


HumidityCollection::HumidityCollection(QObject *parent)
    : QObject{parent}
    , m_humidityController { new HumidityController( this ) }
    , m_humidityModel { new HumidityModel( this ) }
{
    registerControllers();

    // here used to be default import file path for mission, no need anymore
}

void HumidityCollection::registerControllers()
{
    qmlRegisterType<HumidityController>("FmControllers", 1, 0, "HumidityController");
    qmlRegisterType<HumidityModel>("FmControllers", 1, 0, "HumidityModel");
}

bool HumidityCollection::importMissionData(const QString& filePath)
{
    qDebug() << "Importing mission data from:" << filePath;

    // TODO: Implement CSV parsing and load into models, though probably delegated from master controller instead of directly here.. (?)

    // Return true on success, false on failure
    return true;
}

