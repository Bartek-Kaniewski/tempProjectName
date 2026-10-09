#include "videoreceiver.h"
#include <QDebug>

VideoWorker::VideoWorker(const QString &sourceUrl, QObject *parent)
    : QObject(parent), m_sourceUrl(sourceUrl) {}

void VideoWorker::stop() {
    m_running = false;
}

void VideoWorker::process() {
    m_running = true;
    emit statusChanged("Ładowanie modelu AI...");

    if (!m_poseDetector.loadModel("yolov8n-pose.onnx")) {
        qWarning() << "OSTRZEŻENIE: Nie udało się wczytać yolov8n-pose.onnx!";
    }

    emit statusChanged("Łączenie ze źródłem wideo...");

    cv::VideoCapture cap;
    bool isNumeric;
    int camIndex = m_sourceUrl.toInt(&isNumeric);
    if (isNumeric) {
        cap.open(camIndex);
    } else {
        cap.open(m_sourceUrl.toStdString());
    }

    if (!cap.isOpened()) {
        emit statusChanged("BŁĄD: Nie można otworzyć strumienia: " + m_sourceUrl);
        emit finished();
        return;
    }

    emit statusChanged("Połączono. Śledzenie aktywne.");

    cv::Mat frame;
    while (m_running) {
        if (!cap.read(frame) || frame.empty()) {
            QThread::msleep(10);
            continue;
        }

        // 1. Detekcja szkieletu człowieka
        auto poses = m_poseDetector.processFrame(frame);
        if (!poses.empty()) {
            const auto &p = poses[0];

            double filteredRightYaw      = m_headYawFilter.filter(p.rightShoulderYaw);
            double filteredRightShoulder = m_rightShoulderFilter.filter(p.rightShoulderPitch);
            double filteredRightElbow    = m_rightElbowFilter.filter(p.rightElbowAngle);
            double filteredHeadYaw       = m_headYawFilter.filter(p.headYaw);
            double filteredTorsoRoll     = m_torsoRollFilter.filter(p.torsoRoll);

            // Wysyłamy WSZYSTKIE 5 kątów do wątku głównego:
            emit allAnglesUpdated(filteredRightYaw, filteredRightShoulder, filteredRightElbow, filteredHeadYaw, filteredTorsoRoll);
        }
        // 2. Konwersja BGR -> RGB do wyświetlenia w QML
        cv::Mat rgbFrame;
        cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);

        QImage qimg(rgbFrame.data,
                    rgbFrame.cols,
                    rgbFrame.rows,
                    static_cast<int>(rgbFrame.step),
                    QImage::Format_RGB888);

        emit frameReady(qimg.copy());
    }

    cap.release();
    emit statusChanged("Zatrzymano.");
    emit finished();
}

VideoReceiver::VideoReceiver(QObject *parent) : QObject(parent) {}

VideoReceiver::~VideoReceiver() {
    stopStream();
}

void VideoReceiver::startStream(const QString &sourceUrl) {
    stopStream();

    m_thread = new QThread(this);
    m_worker = new VideoWorker(sourceUrl);
    m_worker->moveToThread(m_thread);

    connect(m_thread, &QThread::started, m_worker, &VideoWorker::process);
    connect(m_worker, &VideoWorker::frameReady, this, &VideoReceiver::frameReady);
    connect(m_worker, &VideoWorker::statusChanged, this, &VideoReceiver::statusChanged);
    connect(m_worker, &VideoWorker::allAnglesUpdated, this, &VideoReceiver::onAllAnglesReceived);

    connect(m_worker, &VideoWorker::finished, m_thread, &QThread::quit);
    connect(m_worker, &VideoWorker::finished, m_worker, &QObject::deleteLater);
    connect(m_thread, &QThread::finished, m_thread, &QObject::deleteLater);

    m_thread->start();
}

void VideoReceiver::stopStream() {
    if (m_worker) {
        m_worker->stop();
    }
    if (m_thread) {
        m_thread->quit();
        m_thread->wait();
        m_thread = nullptr;
        m_worker = nullptr;
    }
}

void VideoReceiver::onAllAnglesReceived(double rightShoulderYaw, double rightShoulderPitch, double rightElbow, double headYaw, double torsoRoll) {
    m_rightShoulderYaw = rightShoulderYaw;
    m_rightShoulderPitch = rightShoulderPitch;
    m_rightElbowAngle = rightElbow;
    m_headYaw = headYaw;
    m_torsoRoll = torsoRoll;

    emit anglesChanged();
}