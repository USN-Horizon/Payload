#include "humiditycontroller.h"

HumidityController::HumidityController(QObject *parent)
    : QObject{parent}
    , m_name { "Hello World!" }
{}

QString HumidityController::name() const
{
    return m_name;
}

void HumidityController::setname(const QString &newName)
{
    if (m_name == newName)
        return;
    m_name = newName;
    emit nameChanged();
}

humidity_utils::HumidityList HumidityController::humidityList() const
{
    return m_humidityList;
}

void HumidityController::setHumidityList(const humidity_utils::HumidityList &newHumidityList)
{
    //if (m_humidityList == newHumidityList)
    //    return;
    m_humidityList = newHumidityList;
    emit humidityListChanged();
}
