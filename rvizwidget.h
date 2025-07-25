#ifndef RVIZWIDGET_H
#define RVIZWIDGET_H

#include <QWidget>

namespace rviz {
class VisualizationFrame;
}

class RvizWidget : public QWidget
{
    Q_OBJECT
public:
    explicit RvizWidget(const QString& configPath, QWidget* parent = nullptr);
    ~RvizWidget();

private:
    rviz::VisualizationFrame* frame_;
};

#endif // RVIZWIDGET_H
