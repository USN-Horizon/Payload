import QtQuick
import QtQuick.Layouts

// Small indicator that should change color based on how long ago a board's
// last heartbeat arrived. Bind `model` to avionicsHeartbeatM or
// payloadHeartbeatM (context properties set up in main.cpp) or any other
// HeartbeatModel instance.
//
// HeartbeatModel exposes:
//   lastBeatMs: epoch ms (QDateTime::currentMSecsSinceEpoch()) of the most
//               recent heartbeat, or -1 if none has arrived yet.
//               NOTIFY'd, so it updates live as beats come in.
ColumnLayout {
    id: heartbeatRoot
    required property QtObject model

    Rectangle {
        id: root

        color: "#aaaaaa"

        width: 56
        height: 26
        radius: height / 4

        Layout.topMargin: 10

        Text {
            id: connectionStabilityText
            text: ""
            color: "#ffffff"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            width: parent.width
            height: parent.height
        }

        // Ticks continuously (not just when a new beat arrives) so the
        // indicator can keep aging between beats. 33ms matches the ~30fps
        // redraw rate RealtimeGraph.qml already uses elsewhere in the app for
        // this kind of continuously-updating visual.
        Timer {
        interval: 33
        running: true
        repeat: true
        onTriggered: {
            if (heartbeatRoot.model.lastBeatMs === -1) return;
            let elapsed = (Date.now() - heartbeatRoot.model.lastBeatMs) / 1000;

            // print time since last beat if more than 6s ago
            if (elapsed>10) lastBeatText.text = elapsed + "s ago";
                else lastBeatText.text = "";

            if (elapsed>20) connectionStabilityText.text = "Dropped";
                else if (elapsed>10) connectionStabilityText.text = "Degraded";
                else connectionStabilityText.text = "Good";

            let badFactor = Math.min(255, Math.floor((elapsed / 10) * 256)); // "bad factor" canon term: 0-255
            let hexR = badFactor.toString(16).padStart(2, '0');
            let hexG = (255-badFactor).toString(16).padStart(2, '0');
            root.color = "#" + hexR + hexG + "00";
        }
        }
    }
    Text {
        id: lastBeatText
        color: "#aaaaaa"
    }

}


