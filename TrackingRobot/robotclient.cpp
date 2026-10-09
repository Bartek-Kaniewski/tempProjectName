#include "robotclient.h"
#include <QDebug>
int timer = 50;
RobotClient::RobotClient(QObject *parent) : QObject(parent) {
    connect(&m_webSocket, &QWebSocket::connected, this, &RobotClient::onConnected);
    connect(&m_webSocket, &QWebSocket::disconnected, this, &RobotClient::onDisconnected);
    connect(&m_webSocket, &QWebSocket::errorOccurred, this, &RobotClient::onError);
    m_sendTimer.start();
}

RobotClient::~RobotClient() {
    disconnectFromRobot();
}

void RobotClient::connectToRobot(const QString &ip, int port) {
    QString url = QString("ws://%1:%2").arg(ip).arg(port);
    m_statusText = "Łączenie...";
    emit statusChanged(m_statusText);
    m_webSocket.open(QUrl(url));
}

void RobotClient::disconnectFromRobot() {
    m_webSocket.close();
}

void RobotClient::onConnected() {
    m_connected = true;
    m_statusText = "POŁĄCZONY";
    emit connectionChanged(true);
    emit statusChanged(m_statusText);
    qDebug() << "[ROS] Połączono z robotem przez WebSocket!";

    // W ROS 2 podajemy typ bez dodatkowego /msg/:
    advertiseTopic("/teleop/joint_commands", "std_msgs/Float64MultiArray");

    // Od razu wysyłamy zerową ramkę inicjalizacyjną, aby temat pojawił się w ROS 2
    // sendJointAngles(0.0, 0.0, 0.0, 0.0);
}

void RobotClient::onDisconnected() {
    m_connected = false;
    m_statusText = "ROZŁĄCZONY";
    emit connectionChanged(false);
    emit statusChanged(m_statusText);
    qDebug() << "[ROS] Rozłączono z robotem.";
}

void RobotClient::onError(QAbstractSocket::SocketError error) {
    m_statusText = "BŁĄD POŁĄCZENIA";
    emit statusChanged(m_statusText);
    qWarning() << "[ROS] Błąd WebSocket:" << m_webSocket.errorString();
}

void RobotClient::advertiseTopic(const QString &topic, const QString &type) {
    QJsonObject adv;
    adv["op"] = "advertise";
    adv["topic"] = topic;
    adv["type"] = type;
    m_webSocket.sendTextMessage(QString::fromUtf8(QJsonDocument(adv).toJson(QJsonDocument::Compact)));
}

void RobotClient::sendJointAngles(double shoulderYaw, double shoulderPitch, double elbowAngle, double headYaw, double torsoRoll) {
    if (!m_connected) return;
    if (m_sendTimer.elapsed() < timer) {
        return;
    }
    m_sendTimer.restart();
    QJsonObject msg;
    QJsonObject layout;
    layout["dim"] = QJsonArray();
    layout["data_offset"] = 0;
    msg["layout"] = layout;

    QJsonArray dataArray;
    dataArray.append(shoulderYaw);   // data[0] -> Oś 1 (Tors)
    dataArray.append(shoulderPitch); // data[1] -> Oś 2 (Uniesienie)
    dataArray.append(elbowAngle);    // data[2] -> Oś 4 (Łokieć)
    dataArray.append(headYaw);
    dataArray.append(torsoRoll);
    msg["data"] = dataArray;

    publishJson("/teleop/joint_commands", msg);
}


void RobotClient::emergencyStop() {
    qWarning() << "[ROS] !!! WYWOŁANO STOP AWARYJNY !!!";
    if (m_connected) {
        // Publikacja komendy zatrzymania
        QJsonObject stopMsg;
        QJsonArray zeros;
        zeros.append(0.0);
        zeros.append(0.0);
        zeros.append(0.0);
        zeros.append(0.0);
        stopMsg["data"] = zeros;
        publishJson("/teleop/joint_commands", stopMsg);
    }
}

void RobotClient::publishJson(const QString &topic, const QJsonObject &msg) {
    QJsonObject pub;
    pub["op"] = "publish";
    pub["topic"] = topic;
    pub["msg"] = msg;

    m_webSocket.sendTextMessage(QString::fromUtf8(QJsonDocument(pub).toJson(QJsonDocument::Compact)));
}