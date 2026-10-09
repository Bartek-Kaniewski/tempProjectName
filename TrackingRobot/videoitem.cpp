#include "videoitem.h"

VideoItem::VideoItem(QQuickItem *parent) : QQuickPaintedItem(parent) {
    setAntialiasing(true);
}

void VideoItem::onNewFrame(const QImage &image) {
    m_currentFrame = image;
    update();
}

void VideoItem::paint(QPainter *painter) {
    if (m_currentFrame.isNull()) {
        return;
    }

    QRectF targetRect = boundingRect();
    QImage scaled = m_currentFrame.scaled(targetRect.size().toSize(),
                                          Qt::KeepAspectRatio,
                                          Qt::SmoothTransformation);

    float x = (targetRect.width() - scaled.width()) / 2.0f;
    float y = (targetRect.height() - scaled.height()) / 2.0f;
    painter->drawImage(QPointF(x, y), scaled);
}