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

// hash functor for QString to use with std::unordered_map
struct QStringHash {
    std::size_t operator()(const QString &s) const noexcept { return qHash(s); }
};

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

    // helpers
    std::unique_ptr<QProcess> createDriverProcess(const QString& scriptPath,
                                                  const QString& key);
    void shutdownProcess(const QString& key);
    bool killProcessGroup(qint64 pid, int sig, int waitMs);
    QString findVictimKey(QProcess* proc);

    void handleProcessCrash(const QString& crashedProc);
    void handleProcessCompletion(const QString& completedProc);

private slots:
    void readDriverOutput();
    void processCrashed();
    void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
};