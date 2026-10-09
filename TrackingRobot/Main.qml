import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import TrackingRobot

Window {
    id: rootWindow
    width: 1280
    height: 760
    visible: true
    title: "Rokae Helios | Operator HUD"
    visibility: Window.Maximized

    // Globalna paleta kolorów dla łatwej zmiany
    readonly property color colorBg: "#0b0c10"
    readonly property color colorPanel: "#151720"
    readonly property color colorAccent: "#00e5ff" // Neonowy Cyjan
    readonly property color colorSuccess: "#00ff66" // Toksyczna zieleń
    readonly property color colorWarning: "#ffcc00"
    readonly property color colorDanger: "#ff2a2a"
    readonly property color colorText: "#ffffff"
    readonly property color colorTextMuted: "#8a8d98"

    color: colorBg

    RowLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 20

        // ==========================================
        // LEWA STRONA: Ekran Kamery (Viewport)
        // ==========================================
        Rectangle {
            id: videoContainer
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#050508"
            radius: 12
            border.color: "#2a2c3a"
            border.width: 2
            clip: true

            // Ramka akcentująca (cyberpunkowy detal)
            Rectangle {
                anchors.top: parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                width: 150
                height: 3
                color: colorAccent
            }

            VideoItem {
                id: liveDisplay
                objectName: "cameraDisplay"
                anchors.fill: parent
            }

            // Ekran ładowania/braku połączenia
            Rectangle {
                anchors.fill: parent
                color: "#050508"
                visible: !robotClient.isConnected

                ColumnLayout {
                    anchors.centerIn: parent
                    spacing: 15
                    
                    BusyIndicator {
                        Layout.alignment: Qt.AlignHCenter
                        running: !robotClient.isConnected
                        width: 60
                        height: 60
                    }

                    Text {
                        text: robotClient.statusText.toUpperCase()
                        color: colorAccent
                        font.pixelSize: 22
                        font.bold: true
                        font.letterSpacing: 2
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }

            // Wskaźnik LIVE (Pill)
            Rectangle {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.margins: 15
                width: 140
                height: 34
                color: "#cc0b0c10"
                radius: 17
                border.color: "#33ffffff"
                border.width: 1
                visible: robotClient.isConnected

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 8

                    // Pulsująca czerwona kropka
                    Rectangle {
                        width: 10
                        height: 10
                        radius: 5
                        color: colorDanger
                        
                        SequentialAnimation on opacity {
                            loops: Animation.Infinite
                            NumberAnimation { to: 0.2; duration: 800 }
                            NumberAnimation { to: 1.0; duration: 800 }
                        }
                    }

                    Text {
                        text: "LIVE STREAM"
                        color: "#ffffff"
                        font.pixelSize: 12
                        font.bold: true
                        font.letterSpacing: 1
                        Layout.alignment: Qt.AlignVCenter
                    }
                }
            }
        }

        // ==========================================
        // PRAWA STRONA: Panel Dowodzenia
        // ==========================================
        Rectangle {
            id: sidePanel
            Layout.preferredWidth: 360
            Layout.fillHeight: true
            color: colorPanel
            radius: 12
            border.color: "#232635"
            border.width: 1

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 20
                spacing: 20

                // Nagłówek Panelu
                ColumnLayout {
                    spacing: 5
                    Layout.fillWidth: true

                    Text {
                        text: "ROKAE COMMAND"
                        color: colorText
                        font.pixelSize: 24
                        font.bold: true
                        font.letterSpacing: 1
                    }
                    Text {
                        text: "System Telemetrii Operatora"
                        color: colorAccent
                        font.pixelSize: 12
                        font.bold: true
                    }
                }

                Rectangle { Layout.fillWidth: true; height: 1; color: "#232635" }

                // Wskaźnik statusu
                RowLayout {
                    Layout.fillWidth: true
                    Text { 
                        text: "LINK STATUS:" 
                        color: colorTextMuted
                        font.pixelSize: 13
                        font.bold: true 
                    }
                    Item { Layout.fillWidth: true }
                    Text {
                        text: robotClient.statusText.toUpperCase()
                        color: robotClient.isConnected ? colorSuccess : colorDanger
                        font.bold: true
                        font.pixelSize: 14
                        font.letterSpacing: 1
                    }
                }

                // ==========================================
                // KARTA TELEMETRII
                // ==========================================
                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: telemetryLayout.height + 30
                    color: "#0f1015"
                    radius: 8
                    border.color: "#1e202d"

                    ColumnLayout {
                        id: telemetryLayout
                        anchors.top: parent.top
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.margins: 15
                        spacing: 12

                        Text {
                            text: "KINEMATYKA OPERATORA"
                            color: colorTextMuted
                            font.pixelSize: 12
                            font.bold: true
                        }

                        // Komponent pomocniczy do wierszy danych
                        component TelemetryRow: RowLayout {
                            property string label
                            property string value
                            property color valColor

                            Layout.fillWidth: true
                            Text { text: label; color: colorText; font.pixelSize: 14 }
                            Item { Layout.fillWidth: true }
                            Text {
                                text: value
                                color: valColor
                                font.pixelSize: 18
                                font.bold: true
                                font.family: "Courier New" // Monospace żeby liczby nie skakały
                            }
                        }

                        TelemetryRow {
                            label: "Prawy łokieć"
                            value: videoReceiver.rightElbowAngle > 0 ? videoReceiver.rightElbowAngle.toFixed(1) + "°" : "---.-°"
                            valColor: colorAccent
                        }
                        TelemetryRow {
                            label: "Lewy łokieć"
                            value: videoReceiver.leftElbowAngle > 0 ? videoReceiver.leftElbowAngle.toFixed(1) + "°" : "---.-°"
                            valColor: colorAccent
                        }
                        TelemetryRow {
                            label: "Obrót głowy (Yaw)"
                            value: videoReceiver.headYaw.toFixed(1) + "°"
                            valColor: colorWarning
                        }
                        TelemetryRow {
                            label: "Pochylenie tułowia"
                            value: videoReceiver.torsoRoll.toFixed(1) + "°"
                            valColor: colorWarning
                        }
                    }
                }

                Item { Layout.fillHeight: true } // Wypycha przyciski na sam dół

                // ==========================================
                // PRZYCISKI AKCJI
                // ==========================================
                
                // Przycisk Połączenia
                Button {
                    id: btnConnect
                    Layout.fillWidth: true
                    height: 50

                    contentItem: Text {
                        text: robotClient.isConnected ? "ZAKOŃCZ SESJĘ" : "INICJALIZUJ POŁĄCZENIE"
                        font.pixelSize: 14
                        font.bold: true
                        font.letterSpacing: 1
                        color: robotClient.isConnected ? colorDanger : colorBg
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }

                    background: Rectangle {
                        radius: 6
                        color: robotClient.isConnected ? "transparent" : (btnConnect.pressed ? "#00b3cc" : colorAccent)
                        border.color: robotClient.isConnected ? colorDanger : colorAccent
                        border.width: 2
                        
                        // Efekt podświetlenia przy najechaniu (hover)
                        Rectangle {
                            anchors.fill: parent
                            radius: 6
                            color: "#ffffff"
                            opacity: btnConnect.hovered ? 0.1 : 0.0
                            Behavior on opacity { NumberAnimation { duration: 150 } }
                        }
                    }

                    onClicked: {
                        if (robotClient.isConnected) {
                            robotClient.disconnectFromRobot();
                        } else {
                            robotClient.connectToRobot("10.111.169.242", 9090);
                        }
                    }
                }

                // E-STOP (Przywrócony i zrobiony na "groźnie")
                Button {
                    id: btnEstop
                    Layout.fillWidth: true
                    height: 60

                    contentItem: Text {
                        text: "EMERGENCY STOP"
                        font.pixelSize: 18
                        font.bold: true
                        font.letterSpacing: 2
                        color: "#ffffff"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }

                    background: Rectangle {
                        radius: 6
                        color: btnEstop.pressed ? "#cc0000" : colorDanger
                        border.color: "#ff6666"
                        border.width: 1
                        
                        // Paski ostrzegawcze (opcjonalny detal)
                        Rectangle {
                            anchors.bottom: parent.bottom
                            anchors.horizontalCenter: parent.horizontalCenter
                            width: parent.width - 20
                            height: 3
                            color: "#ffffff"
                            opacity: 0.5
                        }
                    }

                    onClicked: {
                        robotClient.emergencyStop();
                    }
                }
            }
        }
    }
}