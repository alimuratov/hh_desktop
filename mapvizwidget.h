#ifndef MAPVIZWIDGET_H
#define MAPVIZWIDGET_H

#include <QWidget>





class QProcess;
class QLabel;

class MapvizWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MapvizWidget(QWidget* parent = nullptr);
    ~MapvizWidget();

private:
    QLabel* statusLabel;
    QProcess* mapvizProcess;
};

#endif // MAPVIZWIDGET_H
