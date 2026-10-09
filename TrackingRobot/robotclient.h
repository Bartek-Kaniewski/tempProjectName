#pragma once

#include <QObject>
#include <QWebSocket>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QElapsedTimer>

class RobotClient : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool isConnected READ isConnected NOTIFY connectionChanged)
    Q_PROPERTY(QString statusText READ statusText NOTIFY statusChanged)

public:
    explicit RobotClient(QObject *parent = nullptr);
    ~RobotClient();

    bool isConnected() const { return m_connected; }
    QString statusText() const { return m_statusText; }

    Q_INVOKABLE void connectToRobot(const QString &ip = "10.111.169.242", int port = 9090);
    Q_INVOKABLE void disconnectFromRobot();
    Q_INVOKABLE void emergencyStop();

public slots:
    // Wysyła wygładzone kąty człowieka do robota
    void sendJointAngles(double shoulderYaw, double shoulderPitch, double elbowAngle, double headYaw, double torsoRoll);

signals:
    void connectionChanged(bool connected);
    void statusChanged(const QString &status);

private slots:
    void onConnected();
    void onDisconnected();
    void onError(QAbstractSocket::SocketError error);

private:
    QWebSocket m_webSocket;

    QElapsedTimer m_sendTimer;
    bool m_connected = false;
    QString m_statusText = "ROZŁĄCZONY";

    void advertiseTopic(const QString &topic, const QString &type);
    void publishJson(const QString &topic, const QJsonObject &msg);
};