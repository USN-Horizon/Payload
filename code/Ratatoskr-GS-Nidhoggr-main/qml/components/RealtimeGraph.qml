import QtQuick
import QtGraphs
import com.horizon.components

Item {
    id: root

    // Properties to configure the graph
    property alias title: graphTitle.text
    property alias yAxisMax: axisY.max
    property alias yAxisMin: axisY.min
    property alias yAxisInterval: axisY.tickInterval
    property alias yAxisLabel: axisY.titleText
    property alias model: timeWindow.sourceModel
    property alias timer: timeWindow.timer
    property alias windowSize: timeWindow.windowSize
    property string unitText: ""
    property color lineColor: "#2CDE85"

    // Internal time window proxy model
    TimeWindowProxyModel {
        id: timeWindow
        invertTimeAxis: true
        windowSize: 60 // Default 60 seconds window

        // Monitor changes to ensure graph updates
        onCurrentTimeChanged: {
            if (graphView.visible) {
                graphView.update()
            }
        }
    }

    Item {
        id: titleRow
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
            bottom: parent.bottom
        }

        // Main graph
        GraphsView {
            id: graphView
            anchors {
                top: parent.top
                left: parent.left
                right: valueColumn.left
                bottom: parent.bottom
                margins: 5
            }

            // Low-level timer to ensure smooth animation
            Timer {
                interval: 33 // ~30 fps
                running: true
                repeat: true
                onTriggered: {
                    if (timeWindow.timer && timeWindow.timer.running) {
                        graphView.update()
                    }
                }
            }

            // Graph styling
            theme: GraphsTheme {
                readonly property color c1: "#DBEB00"
                readonly property color c2: "#373F26"
                readonly property color c3: Qt.lighter(c2, 1.5)

                colorScheme: GraphsTheme.ColorScheme.Dark
                seriesColors: [root.lineColor, "#DBEB00"]
                grid.mainColor: c3
                grid.subColor: c2
                axisX.mainColor: c3
                axisY.mainColor: c3
                axisX.subColor: c2
                axisY.subColor: c2
                axisX.labelTextColor: c1
                axisY.labelTextColor: c1
            }

            // X-axis (time)
            //
            // Data carries absolute timestamps (ms). Instead of relabelling every
            // point each frame, we scroll the axis window against the clock: the
            // right edge tracks "now" and the left edge trails by windowSize.
            axisX: ValueAxis {
                id: axisX
                readonly property real nowMs: timeWindow.currentTime * 1000
                min: nowMs - timeWindow.windowSize * 1000
                max: nowMs
                tickInterval: timeWindow.windowSize * 1000 / 6
                subTickCount: 1
                labelDecimals: 0
                titleText: "Time (milliseconds ago)"

                // Custom label format to show as "X sec ago" (now == 0s)
                labelFormat: function(value) {
                    return ((axisX.nowMs - value) / 1000).toFixed(0) + "s";
                }
            }

            // Y-axis (data value)
            axisY: ValueAxis {
                id: axisY
                min: 0
                max: 500
                subTickCount: 4
                labelDecimals: 0
                titleText: "Value"
            }

            // Data series
            LineSeries {
                id: lineSeries
                name: "Data"
                width: 2

                // Map model data to series
                XYModelMapper {
                    id: mapper
                    model: timeWindow
                    series: lineSeries
                    xSection: 0
                    ySection: 1
                }
            }
        }

        Column {
            id: valueColumn
            width: 120

            anchors {
                top: parent.top
                right: parent.right
                bottom: parent.bottom
                margins: 10
            }
            spacing: 10

            Text {
                id: graphTitle
                text: "Metric Name"
                color: "#DBEB00"
                font.pixelSize: 16
                font.bold: true
            }

            Text {
                text: {
                        var _ = timeWindow.currentTime; // this line is appearently needed to keep component reactive (like in vue..... :( )
                        if (timeWindow.rowCount() > 0) {
                            var lastRow = timeWindow.rowCount() - 1;
                            var value = timeWindow.data(timeWindow.index(lastRow, 1));
                            return value !== undefined ? value.toFixed(1) + unitText : "---";
                        }
                        return "---";
                    }

                color: "#00ff00"
                font.pixelSize: 20
                font.family: "Monospace"
            }
        }

    }


}
