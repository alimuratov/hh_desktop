

#include "mapvizwidget.h"
#include <QProcess>

MapvizWidget::MapvizWidget(QWidget* parent)
    : QWidget(parent)
{
    mapvizProcess = new QProcess(this);
}

MapvizWidget::~MapvizWidget() = default;
