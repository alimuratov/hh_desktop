#pragma once
#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QString>
#include <memory>
#include <unordered_map>
#include <QStringList>
#include "rosbagrecorder.h"
#include "diagnostics_monitor.h"
#include "qstring_hash.h"

// Forward declaration
struct ProcessConfig;

class ScannerController : public QObject {
    Q_OBJECT
public:
    explicit ScannerController(ros::NodeHandle& nh, QObject* parent = nullptr);
    
    // -------------------------- Recording --------------------------
    void startRecording(const QString& filename, const QStringList& topics);
    void stopRecording();
    bool isRecording() const;

    // -------------------------- Driver control --------------------------
    void startDrivers();
    void stopDrivers();
    void startSlam();
    void stopSlam();
    void startDynamicReconfigure();
    void stopDynamicReconfigure();

    // -------------------------- Mapviz control --------------------------
    void startMapviz();
    void stopMapviz();

    // -------------------------- Query state/prerequisites --------------------------
    bool isRunning(const QString& key) const;
    bool canStart(const QString& key) const;
    bool hasExternalRunning(const QString& key) const;
    void cleanupExternal(const QString& key);
    void cleanupAllExternals();

    
signals:
    void recordingStarted();
    void recordingStopped();
    void recordingError(const QString& error);
    void diagnosticsUpdated(const QString& taskName, int level, 
                           const QString& message, const QString& frequency);

    void driverStarted(const QString& key);
    void driverStopped(const QString& key);
    void driverCrashed(const QString& key);
    void driverError(const QString& key, const QString& errorMessage);
    void driverOutput(const QString& key, const QString& output);
    
private:
    // recording and diagnostics
    std::unique_ptr<RosbagRecorder> recorder;
    std::unique_ptr<DiagnosticsMonitor> diagnosticsMonitor;

    // process management
    using DriverMap = std::unordered_map<QString, std::unique_ptr<QProcess>, QStringHash>;
    DriverMap drivers_;
    QTimer* rosTimer_;
    QTimer* driversDumpTimer_ = nullptr;

    // Generic process control
    void startProcess(const QString& key);
    void stopProcess(const QString& key);

    // helpers
    std::unique_ptr<QProcess> createDriverProcess(const ProcessConfig& config);
    void shutdownProcess(const QString& key);
    bool killProcessGroup(qint64 pid, int sig, int waitMs);
    QString findVictimKey(QProcess* proc);
    QList<qint64> findExternalPids(const QString& key) const;

    void handleProcessCrash(const QString& crashedProc);
    void handleProcessCompletion(const QString& completedProc);

private slots:
    void readDriverOutput();
    void processCrashed();
    void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void dumpDriversSnapshot();
};