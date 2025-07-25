#include "rvizwidget.h"

#include <rviz/visualization_frame.h>
#include <rviz/yaml_config_reader.h>
#include <rviz/config.h>
#include <QVBoxLayout>
#include <QFileInfo>

RvizWidget::RvizWidget(const QString& configPath, QWidget* parent)
    : QWidget(parent)
{
    frame_ = new rviz::VisualizationFrame();
    frame_->setSplashPath( "" );
    frame_->initialize();
    frame_->setMenuBar(nullptr);

    if (QFileInfo::exists(configPath)) {
        rviz::YamlConfigReader reader;
        rviz::Config config;
        reader.readFile(config, configPath.toStdString());
        frame_->load(config);
    }

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(frame_);
}

RvizWidget::~RvizWidget() = default;

