import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import com.horizon.components

RowLayout {
    id: root

    property alias missionTimer: missionTimer
    property string missionName: "Mission Name"

    CountupTimer {
        id: missionTimer
    }

    Rectangle {
        id: missionContainer
        Layout.margins: 10
        implicitWidth: missionRow.implicitWidth + 20
        height: 60
        color: "#1e1e1e"
        border.color: "#333333"
        radius: 5

        visible: missionManager.isPlayback

        RowLayout {
            id: missionRow
            anchors.centerIn: parent
            spacing: 12

            ColumnLayout {
                spacing: 2
                Text {
                    text: "PRELOADED MISSION"
                    color: "#aaaaaa"
                    font.pixelSize: 10
                }
                Text {
                    text: root.missionName
                    color: "white"
                    font.pixelSize: 14
                    font.bold: true
                }
            }

            RowLayout {
                spacing: 4

                Button {
                    text: "Start"
                    onClicked: missionTimer.resume()
                }
                Button {
                    text: "Stop"
                    onClicked: missionTimer.stop()
                }
                Button {
                    text: "Reset"
                    onClicked: {
                        missionTimer.stop();
                        missionTimer.reset();
                    }
                }
                Button {
                    icon.source: "qrc:/icons/close.svg"
                    icon.width: 16
                    icon.height: 16
                    onClicked: {
                        missionTimer.stop();
                        missionTimer.reset();
                        missionManager.startCapture();
                    }
                    background: Rectangle {
                        implicitWidth: 40
                        implicitHeight: 40
                        color: "#2e2e32"
                        radius: height / 2
                    }
                }
            }
        }
    }

    Item { Layout.fillWidth: true }

    Rectangle {
        id: timerContainer
        Layout.margins: 10
        implicitWidth: timerRow.implicitWidth + 20
        height: 60
        color: "#1e1e1e"
        border.color: "#333333"
        radius: 5

        RowLayout {
            id: timerRow
            anchors.centerIn: parent
            spacing: 12

            ColumnLayout {
                spacing: 2
                Text {
                    text: "MISSION TIME"
                    color: "#aaaaaa"
                    font.pixelSize: 10
                }
                Text {
                    text: missionTimer.timeString
                    color: "#00ff00"
                    font.pixelSize: 24
                    font.bold: true
                    font.family: "Monospace"
                }
            }
        }
    }
}
