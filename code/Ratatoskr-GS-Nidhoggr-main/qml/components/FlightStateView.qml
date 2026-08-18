import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import com.horizon.components

Item {
    id: root
    implicitHeight: 170

    // Properties
    property alias stateModel: stateWindow.sourceModel
    property alias timer: stateWindow.timer
    property alias windowSize: stateWindow.windowSize

    // Internal time window proxy model
    TimeWindowProxyModel {
        id: stateWindow
        invertTimeAxis: true
        windowSize: 300 // Default 60 seconds window
    }

    Timer {
        id: stateUpdateTimer
        interval: 100 // Update 10 times per second
        running: timer ? timer.running : false
        repeat: true

        onTriggered: {
            // Get most recent state from proxy model
            updateCurrentStateFromProxyModel()
        }

        function updateCurrentStateFromProxyModel() {
            // Only proceed if we have data
            if (!stateWindow || stateWindow.rowCount() === 0)
                return;

            // Find most recent data point
            let mostRecentRow = -1;
            let mostRecentTime = Number.MAX_VALUE;

            for (let i = 0; i < stateWindow.rowCount(); i++) {
                // Get time value (which is "time ago" when invertTimeAxis is true)
                let timeValue = stateWindow.data(stateWindow.index(i, 0));

                // Smallest "time ago" is most recent
                if (timeValue < mostRecentTime) {
                    mostRecentTime = timeValue;
                    mostRecentRow = i;
                }
            }

            // If we found a valid row, update the current state
            if (mostRecentRow >= 0) {
                let stateValue = stateWindow.data(stateWindow.index(mostRecentRow, 1));
                stateModel.setCurrentStateByInt(stateValue);
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 2

        // Title
        Text {
            id: stateTitle
            text: "Flight State"
            color: "#DBEB00"
            font.pixelSize: 16
            font.bold: true
            Layout.topMargin: 10
            Layout.leftMargin: 10
        }

        // State timeline
        Rectangle {
            id: stateTimeline
            Layout.fillWidth: true
            Layout.preferredHeight: 120
            Layout.leftMargin: 10
            Layout.rightMargin: 10
            color: "#1e1e1e"
            border.color: "#333333"
            radius: 5

            Row {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 2

                // Flight state bubbles
                Repeater {
                    model: 8 // Number of states (0-7)

                    delegate: Item {
                        width: parent.width / 8
                        height: parent.height

                        Rectangle {
                            id: stateBubble
                            width: parent.width - 20
                            height: parent.height - 20
                            radius: width / 2
                            color: getStateColor(index)
                            opacity: (stateModel && stateModel.currentState === index) ? 1.0 : 0.3
                            border.color: (stateModel && stateModel.currentState === index) ? "white" : "transparent"
                            border.width: 2
                            anchors.centerIn: parent

                            // Helper function to get the color for a state
                            function getStateColor(stateIndex) {
                                switch(stateIndex) {
                                    case 1: return "#3498db"; // Calibrating - Blue
                                    case 2: return "#2ecc71"; // Ready - Green
                                    case 3: return "#e74c3c"; // Thrusting - Red
                                    case 4: return "#f39c12"; // Coasting - Orange
                                    case 5: return "#9b59b6"; // Drogue - Purple
                                    case 6: return "#1abc9c"; // Main - Teal
                                    case 7: return "#27ae60"; // Touchdown - Dark Green
                                    default: return "#7f8c8d"; // Unknown - Gray
                                }
                            }
                        }

                        Text {
                            text: getStateName(index)
                            anchors.top: parent.bottom
                            anchors.topMargin: -54 - parent.width / 14
                            anchors.horizontalCenter: parent.horizontalCenter
                            color: (stateModel && stateModel.currentState === index) ? "#2B2B00" : "#aaaaaa"
                            opacity: 0.85
                            font.bold: (stateModel && stateModel.currentState === index)
                            font.pixelSize: parent.width / 7

                            // Helper function to get the name for a state
                            function getStateName(stateIndex) {
                                switch(stateIndex) {
                                    case 1: return "Calibrating";
                                    case 2: return "Ready";
                                    case 3: return "Thrusting";
                                    case 4: return "Coasting";
                                    case 5: return "Drogue";
                                    case 6: return "Main";
                                    case 7: return "Touchdown";
                                    default: return "Unknown";
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
