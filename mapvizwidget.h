#ifndef MAPVIZWIDGET_H
#define MAPVIZWIDGET_H

#include <QWidget>

namespace mapviz {
class MapvizFrame;
}

class MapvizWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MapvizWidget(const QString& configPath, QWidget* parent = nullptr);
    ~MapvizWidget();

private:
    mapviz::MapvizFrame* frame_;
};

#endif // MAPVIZWIDGET_H
