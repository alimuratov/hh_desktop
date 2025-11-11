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
    // Set the path to the "splash" image file. This image is shown during initialization 
    // and loading of the first config file. Default is a file in the RViz package. To prevent splash image from showing, set this to an empty string.
    frame_->setSplashPath( "" );
    frame_->initialize();
    frame_->setMenuBar(nullptr);

    if (QFileInfo::exists(configPath)) {
        rviz::YamlConfigReader reader;
        rviz::Config config;
        reader.readFile(config, configPath);
        frame_->load(config);
    }

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(frame_);
}

RvizWidget::~RvizWidget() = default;

