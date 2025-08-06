#include "rosbagrecorder.h"

#include <QDateTime>
#include <QDebug>
#include <QMessageBox>
#include <QThread>
#include <QDir>

#include <errno.h>
#include <signal.h>
#include <unistd.h>

RosbagRecorder::RosbagRecorder(QObject *parent)
    : QObject(parent)
{
}

RosbagRecorder::~RosbagRecorder()
{
    // ensure the recording process is terminated if the object is destroyed
    if (rosbagProc_)
        stopRecording();
}


void RosbagRecorder::startRecording(const QString &bagName,
                                    const QSet<QString> &topics)
{
    if (rosbagProc_) {
        qWarning() << "rosbag recorder already running";
        return;
    }

    rosbagProc_ = std::make_unique<QProcess>(this);
    rosbagProc_->setProcessChannelMode(QProcess::MergedChannels);
    rosbagProc_->setWorkingDirectory(QDir::homePath() + "/rosbags");
    qDebug() << "Directory set to:" << rosbagProc_->workingDirectory();

    QStringList topicList = topics.values();

    const QString program("/usr/bin/setsid");
    QString cmd = QStringLiteral("exec rosbag record -O %1 --lz4 --tcpnodelay %2")
                       .arg(bagName, topicList.join(' '));
    const QStringList args{"/bin/bash", "-c", cmd};
 

    connect(rosbagProc_.get(), &QProcess::readyReadStandardOutput,
            this, &RosbagRecorder::onReadyReadStandardOutput);
    connect(rosbagProc_.get(),
            qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this, &RosbagRecorder::onProcessFinished);
    connect(rosbagProc_.get(), &QProcess::started, this, [p = rosbagProc_.get()] {
        qDebug() << "rosbag record PID=" << p->processId();
    });

    rosbagProc_->start(program, args);
    if (!rosbagProc_->waitForStarted()) {
        QMessageBox::critical(nullptr, tr("Failed to start"),
                              tr("rosbag record could not be launched"));
        rosbagProc_.reset();
    } else {
        emit recordingStarted();
        isRecording = true;
    }
}

void RosbagRecorder::stopRecording()
{
    if (!rosbagProc_)
        return;

    const qint64 pid = rosbagProc_->processId();

    if (!killProcessGroup(pid, SIGINT, 2000) &&
        !killProcessGroup(pid, SIGTERM, 2000)) {
        killProcessGroup(pid, SIGKILL, 0);
    }

    rosbagProc_->waitForFinished();
    rosbagProc_.reset();
    emit recordingStopped();
    isRecording = false;
}

void RosbagRecorder::onProcessFinished(int, QProcess::ExitStatus)
{
    rosbagProc_.reset();
    emit recordingStopped();
    isRecording = false;
}

void RosbagRecorder::onReadyReadStandardOutput()
{
    if (!rosbagProc_)
        return;

    const QString out = QString::fromUtf8(rosbagProc_->readAllStandardOutput());
    emit processOutput(out);
}

bool RosbagRecorder::killProcessGroup(qint64 pid, int sig, int waitMs)
{
    if (pid <= 0)
        return true;

    ::kill(-pid, sig);
    if (waitMs == 0) {
        return false;
    }

    const qint64 t0 = QDateTime::currentMSecsSinceEpoch();
    while (QDateTime::currentMSecsSinceEpoch() - t0 < waitMs) {
        if (::kill(pid, 0) == -1 && errno == ESRCH)
            return true;
        QThread::msleep(50);
    }
    return false;
}

