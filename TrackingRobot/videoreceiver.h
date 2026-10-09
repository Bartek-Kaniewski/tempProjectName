#pragma once

#include <QObject>
#include <QImage>
#include <QThread>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "posedetector.h"
#include "motionfilter.h"

class VideoWorker : public QObject {
    Q_OBJECT
public:
    explicit VideoWorker(const QString &sourceUrl, QObject *parent = nullptr);
    void stop();

signals:
    void frameReady(const QImage &image);
    void statusChanged(const QString &status);
    void allAnglesUpdated(double rightShoulderYaw, double rightShoulderPitch, double rightElbow, double headYaw, double torsoRoll);
    void finished();

public slots:
    void process();

private:
    QString m_sourceUrl;
    std::atomic<bool> m_running{false};
    PoseDetector m_poseDetector;

    // Filtry wygładzające drgania
    MotionFilter m_rightElbowFilter{0.25f, 0.8f};
    MotionFilter m_leftElbowFilter{0.25f, 0.8f};
    MotionFilter m_headYawFilter{0.20f, 0.5f};
    MotionFilter m_torsoRollFilter{0.15f, 0.5f};
    MotionFilter m_rightShoulderFilter{0.15f, 0.5f};
};

class VideoReceiver : public QObject {
    Q_OBJECT
    Q_PROPERTY(double rightElbowAngle READ rightElbowAngle NOTIFY anglesChanged)
    Q_PROPERTY(double leftElbowAngle READ leftElbowAngle NOTIFY anglesChanged)
    Q_PROPERTY(double headYaw READ headYaw NOTIFY anglesChanged)
    Q_PROPERTY(double torsoRoll READ torsoRoll NOTIFY anglesChanged)
    Q_PROPERTY(double rightShoulderPitch READ rightShoulderPitch NOTIFY anglesChanged)
    Q_PROPERTY(double rightShoulderYaw READ rightShoulderYaw NOTIFY anglesChanged)

public:
    explicit VideoReceiver(QObject *parent = nullptr);
    ~VideoReceiver();

    double rightElbowAngle() const { return m_rightElbowAngle; }
    double leftElbowAngle() const { return m_leftElbowAngle; }
    double headYaw() const { return m_headYaw; }
    double torsoRoll() const { return m_torsoRoll; }
    double rightShoulderPitch() const { return m_rightShoulderPitch; }
    double rightShoulderYaw() const { return m_rightShoulderYaw; }

    void startStream(const QString &sourceUrl);
    void stopStream();

signals:
    void frameReady(const QImage &image);
    void statusChanged(const QString &status);
    void anglesChanged();

private slots:
    void onAllAnglesReceived(double rightShoulder, double rightElbow, double leftElbow, double headYaw, double torsoRoll);

private:
    QThread *m_thread = nullptr;
    VideoWorker *m_worker = nullptr;
    double m_rightShoulderPitch = 0.0;
    double m_rightShoulderYaw = 0.0;
    double m_rightElbowAngle = 0.0;
    double m_leftElbowAngle = 0.0;
    double m_headYaw = 0.0;
    double m_torsoRoll = 0.0;
};