#ifndef ROSBAGRECORDER_H
#define ROSBAGRECORDER_H

#include <QObject>
#include <QStringList>
#include <QProcess>
#include <memory>
#include <csignal>

class RosbagRecorder : public QObject
{
    Q_OBJECT
public:
    // declare a constructor as explicit to prevent unintended implicit type conversions.
    explicit RosbagRecorder(QObject *parent = nullptr);
    ~RosbagRecorder();
    bool isRecording() const { return isRecording_; }

public slots:
    void startRecording(const QString &bagName, const QSet<QString> &topics);
    void stopRecording();

signals:
    void recordingStarted();
    void recordingStopped();
    void processOutput(const QString &output);
    void recordingError(const QString &error);

private slots:
    void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onProcessError(QProcess::ProcessError error);
    void onReadyReadStandardOutput();

private:
    bool killProcessGroup(qint64 pid, int sig, int waitMs);
    std::unique_ptr<QProcess> rosbagProc_;
    bool isRecording_ = false;
};

#endif // ROSBAGRECORDER_H
