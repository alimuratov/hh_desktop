#pragma once
#include <QObject>
#include <memory>
#include "rosbagrecorder.h"
#include "diagnostics_monitor.h"

class ScannerController : public QObject {
    Q_OBJECT
public:
    explicit ScannerController(ros::NodeHandle& nh, QObject* parent = nullptr);
    
    void startRecording(const QString& filename);
    void stopRecording();
    bool isRecording() const;
    
signals:
    void recordingStarted();
    void recordingStopped();
    void recordingError(const QString& error);
    void diagnosticsUpdated(const QString& taskName, int level, 
                           const QString& message, const QString& frequency);
    
private:
    std::unique_ptr<RosbagRecorder> recorder;
    std::unique_ptr<DiagnosticsMonitor> diagnosticsMonitor;
};