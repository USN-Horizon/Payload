import QtQuick
import QtQuick.Layouts
import QtLocation
import QtPositioning
import QtQuick.Controls
import com.horizon.components

Item {
    id: root
    Layout.fillWidth: true
    Layout.fillHeight: true

    // Properties to configure the graph
    property alias model: timeWindow.sourceModel
    property alias timer: timeWindow.timer

    property real currentLat: 0
    property real currentLon: 0

    ListModel {
        id: trailModel
    }

    function formatCoord(deg, posLabel, negLabel) {
        var dir = deg >= 0 ? posLabel : negLabel
        var abs = Math.abs(deg)
        var d = Math.floor(abs)
        var minFrac = (abs - d) * 60
        var m = Math.floor(minFrac)
        var s = ((minFrac - m) * 60).toFixed(1)
        return d + "° " + m + "' " + s + "\" " + dir
    }

    // Internal time window proxy model
    TimeWindowProxyModel {
        id: timeWindow
        invertTimeAxis: true
        windowSize: 30

        property real lastSeenTime: 0

        // Monitor changes to ensure graph updates
        onCurrentTimeChanged: {
            // Detect reset: time jumped backwards → clear trail
            if (currentTime < lastSeenTime - 1) {
                trailModel.clear()
            }
            lastSeenTime = currentTime

            var n = timeWindow.rowCount()
            if (n > 0) {
                var point = timeWindow.get(n - 1)
                if (point.y_lat && point.y_lon) {
                    root.currentLat = point.y_lat
                    root.currentLon = point.y_lon
                    map.center = QtPositioning.coordinate(point.y_lat, point.y_lon)

                    // Append to trail if this is a new coordinate
                    var count = trailModel.count
                    var last = count > 0 ? trailModel.get(count - 1) : null
                    if (!last || last.lat !== point.y_lat || last.lon !== point.y_lon) {
                        trailModel.append({ lat: point.y_lat, lon: point.y_lon })
                    }
                }
            }
        }
    }

    Plugin {
        id: mapPlugin
        name: "osm"
        PluginParameter { name: "osm.mapping.providersrepository.disabled"; value: "true" }
        PluginParameter { name: "osm.mapping.custom.host"; value: "https://tile.openstreetmap.org/%z/%x/%y.png" }
        PluginParameter { name: "osm.mapping.custom.mapcopyright"; value: "© OpenStreetMap contributors" }
    }

    Map {
        id: map
        plugin: mapPlugin
        center: QtPositioning.coordinate(59.6645, 9.6440) // Krona
        zoomLevel: 16
        property geoCoordinate startCentroid

        Component.onCompleted: {
            for (var i = 0; i < supportedMapTypes.length; i++) {
                if (supportedMapTypes[i].style === MapType.CustomMap) {
                    activeMapType = supportedMapTypes[i]
                    break
                }
            }
        }

        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            //margins: 10
        }

        MapItemView {
            model: trailModel
            delegate: MapQuickItem {
                coordinate: QtPositioning.coordinate(model.lat, model.lon)
                anchorPoint.x: 3
                anchorPoint.y: 3
                sourceItem: Rectangle {
                    width: 6
                    height: 6
                    radius: 3
                    color: "#442CDE85"
                    border.color: "#662CDE85"
                    border.width: 1
                }
            }
        }

        MapItemView {
            model: timeWindow
            delegate: MapQuickItem {
                coordinate: QtPositioning.coordinate(model.y_lat, model.y_lon)
                anchorPoint.x: 4
                anchorPoint.y: 4
                sourceItem: Rectangle {
                    width: 8
                    height: 8
                    radius: 4
                    color: "#882CDE85"
                    border.color: "#2CDE85"
                    border.width: 1
                }
            }
        }

        Rectangle {
            anchors.centerIn: parent
            width: 10
            height: 10
            radius: 5
            color: "#dd31f904"  // Semi-transparent
            border.color: "#013300"
            border.width: 1
        }


        Rectangle {
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.margins: 8
        width: coordsColumn.implicitWidth + 16
        height: coordsColumn.implicitHeight + 16
        color: "#cc1e1e1e"  // Semi-transparent
        border.color: "#333333"
        border.width: 1
        radius: 4

        ColumnLayout {
        id: coordsColumn
        anchors.fill: parent
        anchors.margins: 8

        Text {
            id: coordsLabel
            text: qsTr("Last Recieved Coordinates")
            color: "#888888"
            font.pixelSize: 12
        }

        RowLayout {
            Text {
            id: coordsValue
            text: root.currentLat === 0 && root.currentLon === 0
                ? qsTr("No data")
                : root.formatCoord(root.currentLat, "N", "S") + "  " + root.formatCoord(root.currentLon, "E", "W")
            color: "white"
            font.pixelSize: 16
            }

            Button {
            text: qsTr("📋")
            onClicked: {
                clipHelper.text = root.currentLat.toFixed(6) + ", " + root.currentLon.toFixed(6)
                clipHelper.selectAll()
                clipHelper.copy()
            }
            }

            TextEdit {
                id: clipHelper
                visible: false
            }

        }
        }
        }

        PinchHandler {
            id: pinch
            target: null
            onActiveChanged: if (active) {
                map.startCentroid = map.toCoordinate(pinch.centroid.position, false)
            }
            onScaleChanged: (delta) => {
                map.zoomLevel += Math.log2(delta)
                map.alignCoordinateToPoint(map.startCentroid, pinch.centroid.position)
            }
            onRotationChanged: (delta) => {
                map.bearing -= delta
                map.alignCoordinateToPoint(map.startCentroid, pinch.centroid.position)
            }
            grabPermissions: PointerHandler.TakeOverForbidden
        }
        WheelHandler {
            id: wheel
            // workaround for QTBUG-87646 / QTBUG-112394 / QTBUG-112432:
            // Magic Mouse pretends to be a trackpad but doesn't work with PinchHandler
            // and we don't yet distinguish mice and trackpads on Wayland either
            acceptedDevices: Qt.platform.pluginName === "cocoa" || Qt.platform.pluginName === "wayland"
                ? PointerDevice.Mouse | PointerDevice.TouchPad
                : PointerDevice.Mouse
            rotationScale: 1/120
            property: "zoomLevel"
        }
        DragHandler {
            id: drag
            target: null
            onTranslationChanged: (delta) => map.pan(-delta.x, -delta.y)
        }
        Shortcut {
            enabled: map.zoomLevel < map.maximumZoomLevel
            sequence: StandardKey.ZoomIn
            onActivated: map.zoomLevel = Math.round(map.zoomLevel + 1)
        }
        Shortcut {
            enabled: map.zoomLevel > map.minimumZoomLevel
            sequence: StandardKey.ZoomOut
            onActivated: map.zoomLevel = Math.round(map.zoomLevel - 1)
        }
    }
}
