#pragma once

#include <QQuickPaintedItem>
#include <QImage>
#include <QPainter>

class VideoItem : public QQuickPaintedItem {
    Q_OBJECT
    QML_ELEMENT
public:
    explicit VideoItem(QQuickItem *parent = nullptr);

    void paint(QPainter *painter) override;

public slots:
    void onNewFrame(const QImage &image);

private:
    QImage m_currentFrame;
};