QT += core gui widgets quick qml graphs serialport
CONFIG += c++17
CONFIG += console

# CONFIG += qmltypes
# QML_IMPORT_NAME = FmControllers
# QML_IMPORT_MAJOR_VERSION = 1

INCLUDEPATH = src

SOURCES += \
        src/mavlink_msg_entry.cpp \
        src/main.cpp \
        src/humiditycollection.cpp \
        src/missionmanager.cpp \
        src/models/flightstatemodel.cpp \
        src/packetparser.cpp \
        src/serialreader.cpp \
        src/utils/flightstateutils.cpp \
        src/models/timeseriesmodel.cpp \
        src/models/timewindowproxymodel.cpp \
        src/humiditycontroller.cpp \
        src/timer.cpp \
        src/utils/flightlogfactory.cpp \
        src/utils/csvlogger.cpp \
        src/utils/humidityutils.cpp \
        src/models/humiditymodel.cpp \
        src/utils/timeseriesfactory.cpp \
        src/models/locationmodel.cpp \
        src/models/heartbeatmodel.cpp

HEADERS +=  \
    src/humiditycollection.h \
    src/missionmanager.h \
    src/models/flightstatemodel.h \
    src/packetparser.h \
    src/serialreader.h \
    src/utils/flightstateutils.h \
    src/models/timeseriesmodel.h \
    src/models/timewindowproxymodel.h \
    src/humiditycontroller.h \
    src/timer.h \
    src/utils/flightlogfactory.h \
    src/utils/csvlogger.h \
    src/utils/humidityutils.h \
    src/models/humiditymodel.h \
    src/utils/timeseriesfactory.h \
    src/models/locationmodel.h \
    src/models/heartbeatmodel.h

		
RESOURCES += qml/qml.qrc \
    resources/resources.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH = qml

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

# Important: Serialport library linked
win32: LIBS += -L$$[QT_INSTALL_LIBS] -lQt6SerialPort

