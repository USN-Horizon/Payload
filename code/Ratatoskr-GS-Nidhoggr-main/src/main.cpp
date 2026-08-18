#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <models/timewindowproxymodel.h>
#include <models/flightstatemodel.h>
#include <models/heartbeatmodel.h>
#include <utils/flightlogfactory.h>
#include <utils/csvlogger.h>
#include <QDateTime>
#include <cmath>
#include <QElapsedTimer>
#include <QMessageBox>
#include "humiditycollection.h"
#include "missionmanager.h"
#include "timer.h"
#include "serialreader.h"
#include "packetparser.h"
#include "horizoncomponents.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QQmlApplicationEngine engine;

    engine.addImportPath( "qrc:" );

    FlightStateModel* stateModel = FlightLogFactory::createStateModel();
    LocationModel* locationModel = FlightLogFactory::createLocationModel();

    // Avionics and payload each have their own IMU/baro, so each gets its own
    // set of time-series models. GPS and flight-phase state only exist on
    // avionics, so those two are shared (not duplicated); radiation only
    // exists on the payload.
    FlightModels avionicsModels = {
        FlightLogFactory::createModel("acceleration[m/s]"),
        FlightLogFactory::createModel("rotation[deg/s]"),
        nullptr, // magnetometer: avionics will not report magnetometer values
        FlightLogFactory::createModel("pressure[m/s]"),
        FlightLogFactory::createModel("temperature[C]"),
        FlightLogFactory::createModel("altitude[m]"),
        FlightLogFactory::createModel("velocity[m/s]"),
        nullptr, // radiation: no sensor on avionics
        locationModel,
        stateModel
    };

    FlightModels payloadModels = {
        FlightLogFactory::createModel("acceleration[m/s]"),
        FlightLogFactory::createModel("rotation[deg/s]"),
        FlightLogFactory::createModel("teslas[T]"),
        FlightLogFactory::createModel("pressure[m/s]"),
        FlightLogFactory::createModel("temperature[C]"),
        FlightLogFactory::createModel("altitude[m]"),
        nullptr, // speed: will not report velocity
        FlightLogFactory::createModel("cosmic_radiation[c/m]"),
        nullptr, // location: no GPS on the payload
        nullptr // state: flight-phase state machine lives on avionics
    };

    CsvLogger* csvLogger = new CsvLogger(&app);
    MissionManager* missionManager = new MissionManager(avionicsModels, payloadModels, *csvLogger);

    // The mission log is a backup black box, if it can't be created, the
    // ground station must not silently run without one. Exit app instead
    QObject::connect(missionManager, &MissionManager::logFileFailed,
                     [](const QString &path, const QString &error) {
                         qCritical() << "CsvLogger: could not create mission log" << path << error;
                         QMessageBox::critical(nullptr, "Mission log file error",
                             QString("Could not create the mission log file:\n%1\n\n%2\n\n"
                                     "Ratatoskr GS cannot run without keeping a backup telemetry log.").arg(path, error));
                     });

    if (!missionManager->startCapture()) {
        return 1;
    }

    HeartbeatModel* avionicsHeartbeat = new HeartbeatModel(&app);
    HeartbeatModel* payloadHeartbeat = new HeartbeatModel(&app);

    //Serial reader & Parse packer
    SerialReader* serialReader = new SerialReader(&app);
    PacketParser* parser = new PacketParser(&app);

    QElapsedTimer* elapsedTimer = new QElapsedTimer();
    elapsedTimer->start();

    bool* timerReset = new bool(false);

    QObject::connect(serialReader, &SerialReader::rawPacketReceived,
                     parser, &PacketParser::parse);

    // Picks which model set a sample belongs to based on the sending
    // component's MAVLink component ID (see horizoncomponents.h).
    auto modelsFor = [avionicsModels, payloadModels](uint8_t compid) {
        return compid == HorizonComponent::Payload ? payloadModels : avionicsModels;
    };

    QObject::connect(parser, &PacketParser::altitudeReceived,
                     [modelsFor, elapsedTimer, timerReset, csvLogger](double value, uint8_t compid) {
                         if (!*timerReset) {
                             elapsedTimer->restart();
                             *timerReset = true;
                         }
                         modelsFor(compid).altitude->appendData(elapsedTimer->elapsed(), value);
                         csvLogger->logScalar(elapsedTimer->elapsed(), compid, "altitude", value);
                     });
    QObject::connect(parser, &PacketParser::velocityReceived,
                     [modelsFor, elapsedTimer, csvLogger](double value, uint8_t compid) {
                         modelsFor(compid).velocity->appendData(elapsedTimer->elapsed(), value);
                         csvLogger->logScalar(elapsedTimer->elapsed(), compid, "velocity", value);
                     });
    QObject::connect(parser, &PacketParser::pressureReceived,
                     [modelsFor, elapsedTimer, csvLogger](double value, uint8_t compid) {
                         modelsFor(compid).pressure->appendData(elapsedTimer->elapsed(), value);
                         csvLogger->logScalar(elapsedTimer->elapsed(), compid, "pressure", value);
                     });
    QObject::connect(parser, &PacketParser::accelerationReceived,
                     [modelsFor, elapsedTimer, csvLogger](double x, double y, double z, uint8_t compid) {
                         qreal magnitude = std::sqrt(x*x + y*y + z*z);
                         modelsFor(compid).acceleration->appendData(elapsedTimer->elapsed(), magnitude);
                         csvLogger->logVector(elapsedTimer->elapsed(), compid, "acceleration", x, y, z);
                     });
    QObject::connect(parser, &PacketParser::rotationReceived,
                     [modelsFor, elapsedTimer, csvLogger](double x, double y, double z, uint8_t compid) {
                         qreal magnitude = std::sqrt(x*x + y*y + z*z);
                         modelsFor(compid).rotation->appendData(elapsedTimer->elapsed(), magnitude);
                         csvLogger->logVector(elapsedTimer->elapsed(), compid, "rotation", x, y, z);
                     });
    QObject::connect(parser, &PacketParser::magnetometerReceived,
                     [modelsFor, elapsedTimer, csvLogger](double x, double y, double z, uint8_t compid) {
                         qreal magnitude = std::sqrt(x*x + y*y + z*z);
                         modelsFor(compid).magnetometer->appendData(elapsedTimer->elapsed(), magnitude);
                         csvLogger->logVector(elapsedTimer->elapsed(), compid, "magnetometer", x, y, z);
                     });
    QObject::connect(parser, &PacketParser::temperatureReceived,
                     [modelsFor, elapsedTimer, csvLogger](double value, uint8_t compid) {
                         modelsFor(compid).temperature->appendData(elapsedTimer->elapsed(), value);
                         csvLogger->logScalar(elapsedTimer->elapsed(), compid, "temperature", value);
                     });
    QObject::connect(parser, &PacketParser::radiationReceived,
                     [payloadModels, elapsedTimer, csvLogger](double value) {
                         payloadModels.radiation->appendData(elapsedTimer->elapsed(), value);
                         csvLogger->logScalar(elapsedTimer->elapsed(), HorizonComponent::Payload, "radiation", value);
                     });
    QObject::connect(parser, &PacketParser::heartbeatReceived,
                     [avionicsHeartbeat, payloadHeartbeat, elapsedTimer, csvLogger](uint8_t compid) {
                         HeartbeatModel* model = compid == HorizonComponent::Payload ? payloadHeartbeat : avionicsHeartbeat;
                         model->recordBeat();
                         csvLogger->logEvent(elapsedTimer->elapsed(), compid, "heartbeat");
                     });

    QObject::connect(serialReader, &SerialReader::errorOccurred,
                     [](const QString &error) {
                         qDebug() << "Serial error:" << error;
                     });

    engine.rootContext()->setContextProperty("serialReader", serialReader);

    // An explicit port (e.g. an emulator/virtual PTY) overrides auto-discovery:
    //   HORIZON_SERIAL_PORT=/dev/pts/5 ./HorizonGroundstation
    QString port = qEnvironmentVariable("HORIZON_SERIAL_PORT");
    if (port.isEmpty()) {
        port = SerialReader::findHorizonPort();
    }
    if (port.isEmpty()) {
        qDebug() << "Horizon module not found, no Arduino Nano 33 IoT based chip detected";
    } else if (serialReader->openPort(port, 115200)) {
        qDebug() << "Listening on" << port;
    } else {
        qDebug() << "Could not open" << port;
    }

    qmlRegisterType<CountupTimer>("com.horizon.components", 1, 0, "CountupTimer");
    qmlRegisterType<TimeSeriesModel>("com.horizon.components", 1, 0, "AltitudeModel");
    qmlRegisterType<TimeWindowProxyModel>("com.horizon.components", 1, 0, "TimeWindowProxyModel");
    qmlRegisterType<HumidityCollection>("HumidityCollection", 1, 0, "HumidityCollection");
    qmlRegisterType<FlightStateModel>("com.horizon.components", 1, 0, "FlightStateModel");
    qmlRegisterType<HeartbeatModel>("com.horizon.components", 1, 0, "HeartbeatModel");


    engine.rootContext()->setContextProperty("missionManager", missionManager );

    engine.rootContext()->setContextProperty( "avionicsAccelerationM", avionicsModels.acceleration);
    engine.rootContext()->setContextProperty( "avionicsRotationM", avionicsModels.rotation);
    engine.rootContext()->setContextProperty( "avionicsPressureM", avionicsModels.pressure);
    engine.rootContext()->setContextProperty( "avionicsTemperatureM", avionicsModels.temperature);
    engine.rootContext()->setContextProperty( "avionicsAltitudeM", avionicsModels.altitude);
    engine.rootContext()->setContextProperty( "avionicsVelocityM", avionicsModels.velocity);

    engine.rootContext()->setContextProperty( "payloadAccelerationM", payloadModels.acceleration);
    engine.rootContext()->setContextProperty( "payloadRotationM", payloadModels.rotation);
    engine.rootContext()->setContextProperty( "payloadMagnetometerM", payloadModels.magnetometer);
    engine.rootContext()->setContextProperty( "payloadPressureM", payloadModels.pressure);
    engine.rootContext()->setContextProperty( "payloadTemperatureM", payloadModels.temperature);
    engine.rootContext()->setContextProperty( "payloadAltitudeM", payloadModels.altitude);
    engine.rootContext()->setContextProperty( "payloadVelocityM", payloadModels.velocity);
    engine.rootContext()->setContextProperty( "payloadRadiationM", payloadModels.radiation);

    engine.rootContext()->setContextProperty("locationM", locationModel);
    engine.rootContext()->setContextProperty( "stateM", stateModel );

    engine.rootContext()->setContextProperty( "avionicsHeartbeatM", avionicsHeartbeat);
    engine.rootContext()->setContextProperty( "payloadHeartbeatM", payloadHeartbeat);

    // Connect to state transitions to log them
    QObject::connect(stateModel, &FlightStateModel::stateTransition,
                     [](FlightState fromState, FlightState toState, FlightEvent event) {
                         qDebug() << "State transition:"
                                  << FlightStateUtils::getStateName(fromState)
                                  << "->"
                                  << FlightStateUtils::getStateName(toState)
                                  << "triggered by"
                                  << FlightStateUtils::getEventName(event);
                     });

    const QUrl url(u"qrc:/main.qml"_qs);
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
                         if (!obj && url == objUrl)
                             QCoreApplication::exit(-1);
                     }, Qt::QueuedConnection);
    engine.load(url);

    QObject* timerStarter = new QObject(&app);
    QObject::connect(parser, &PacketParser::altitudeReceived,
                     timerStarter, [&engine, timerStarter]() {
                         QObject* root = engine.rootObjects().isEmpty() ? nullptr : engine.rootObjects().first();
                         if (!root) return;
                         QObject* topBarObj = root->findChild<QObject*>("topBar");
                         if (!topBarObj) return;
                         QObject* timer = topBarObj->property("missionTimer").value<QObject*>();
                         if (!timer) return;
                         QMetaObject::invokeMethod(timer, "resume");

                         // Only disconnect this specific connection by deleting the context object
                         timerStarter->deleteLater();
                     });

    return app.exec();
}
