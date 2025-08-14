#include "mapvizwidget.h"

#include <mapviz/mapviz_frame.h>
#include <mapviz/yaml_config_reader.h>
#include <mapviz/config.h>
#include <QVBoxLayout>
#include <QFileInfo>

MapvizWidget::MapvizWidget(const QString& configPath, QWidget* parent)
    : QWidget(parent)
{
    frame_ = new mapviz::MapvizFrame();
    frame_->initialize();
    frame_->setMenuBar(nullptr);

    if (QFileInfo::exists(configPath)) {
        mapviz::YamlConfigReader reader;
        mapviz::Config config;
        reader.readFile(config, configPath);
        frame_->load(config);
    }

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(frame_);
}

MapvizWidget::~MapvizWidget() = default;
