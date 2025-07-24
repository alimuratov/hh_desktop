#include "rosbag_recorder.h"
#include <QDateTime>

RosbagRecorder::RosbagRecorder(QObject *parent) : QObject(parent), m_rosbagProcess(nullptr) {}

void RosbagRecorder::startRecording(const QString& bagName, const QStringList &topics)
{
    rosbagProc = std::make_unique<QProcess>(this);
    rosbagProc->setProcessChannelMode(QProcess::MergedChannels);

    const QString program("/usr/bin/setsid");
    const QStringList args {"/bin/bash", "-c", QStringLiteral("exec %1").arg(scriptPath)};
    
    connect(rosbagProc.get(), &QProcess::finished, this, &RosbagRecorder::onProcessFinished);
    QStringList arguments;
    arguments << "record" << "-O" << << "--lz4" << "--tcpnodelay" << bagName;
    arguments.append(topics);

    connect(rosbagProc.get(), &QProcess::started, this, [this, p = proc.get()]{
        qDebug() << "rosbag record PID=" << p->processId();
    });

    if (!proc->waitForStarted()) {
        QMessageBox::critical(this, tr("Failed to start"),
                              tr("%1 driver could not be launched").arg(key)); 
        return;
    } else {
        emit recordingStarted();
    }
}

void RosbagRecorder::stopRecording()
{
    qint64 pid = rosbagProc->processId();

    if (!killProcessGroup(pid, SIGINT, 2000) && !killProcessGroup(pid, SIGTERM, 2000)) {
        killProcessGroup(pid, SIGKILL, 0);
    }


}

bool killProcessGroup(qint64 pid, int sig, int waitMs) {
    if (pid <= 0) return true;

    ::kill(-pid, sig); // only sends a signal SIGTERM (doesn't wait for the processes in that group to act on it)
    if (waitMs == 0) {
        return false; 
    }

    const qint64 t0 = QDateTime::currentMSecsSinceEpoch();
    while (QDateTime::currentMSecsSinceEpoch() - t0 < waitMs) {
        // ::kill(pid, 0) is a POSIX "probe": it delivers no signal but reutrns 0 if the process still exists
        if (::kill(pid, 0) == -1 && errno == ESRCH) return true;
        QThread::msleep(50); // suspend the current thread
    }
    return false; 
}

