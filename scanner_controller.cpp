#include "scanner_controller.h"
#include <QDebug>

ScannerController::ScannerController(ros::NodeHandle& nh, QObject* parent)
    : QObject(parent)
    , recorder(std::make_unique<RosbagRecorder>(this))
    , diagnosticsMonitor(std::make_unique<DiagnosticsMonitor>(nh, this))
{
    // Connect RosbagRecorder signals
    connect(recorder.get(), &RosbagRecorder::processStarted,
            this, &ScannerController::recordingStarted);
    
    connect(recorder.get(), &RosbagRecorder::processFinished,
            this, &ScannerController::recordingStopped);
    
    connect(recorder.get(), &RosbagRecorder::processError,
            this, &ScannerController::recordingError);
    
    // Connect DiagnosticsMonitor signals
    connect(diagnosticsMonitor.get(), &DiagnosticsMonitor::statusChanged,
            this, &ScannerController::diagnosticsUpdated);
}

void ScannerController::startRecording(const QString& filename) {
    if (recorder->isRecording()) {
        emit recordingError("Recording already in progress");
        return;
    }
    
    QStringList topics = {
        "/river_msgs/camera/color/CameraInfo",
        "/river_msgs/camera/depth/CameraInfo",
        "/river_msgs/camera/color/image",
        "/river_msgs/camera/depth/image",
        "/velodyne_points",
        "/imu/data",
        "/tf",
        "/tf_static"
    };
    
    recorder->startRecording(filename, topics);
}

void ScannerController::stopRecording() {
    recorder->stopRecording();
}

bool ScannerController::isRecording() const {
    return recorder->isRecording();
}