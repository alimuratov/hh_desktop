#include "mainwindow.h"
#include "ui_mainwindow.h"

// anonymous namespace grants internal linkage to everything declared inside it, meaning that the constants exist only within this translation unit
// implications: 
// 1. no other cpp file can collide with these names
// 2. constants are not exported
// 3. we avoid one-definition-rule headaches that arise if you move the constants to a header
// we can freely change their values here without forcing a full rebuild of every file that includes the header
// and there is zero risk of multiple definitions at link time 
namespace {
    constexpr char kCameraKey[] = "camera";
    constexpr char kLidarKey[] = "lidar";
    constexpr char kCameraScript[] = "/home/kodifly/setup_scripts/camera_setup.sh";
    constexpr char kLidarScript[] = "/home/kodifly/setup_scripts/lidar_setup.sh";
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->startDriversButton, &QPushButton::clicked, this, &MainWindow::startDrivers);
    connect(ui->stopDriversButton, &QPushButton::clicked, this, &MainWindow::stopDrivers);
}

// -------------------------- Buttons --------------------------

void MainWindow::startDrivers()
{
    startCamera();
    startLidar(); 
}

void MainWindow::stopDrivers() {
    stopCamera();
    stopLidar(); 
}

// -------------------------- Driver Utilities --------------------------

void MainWindow::startCamera() {
    // if the camera driver is already running
    if (drivers_.count(kCameraKey)) return;

    auto proc = createDriverProcess(kCameraScript, kCameraKey);
    if (!proc) return;
    drivers_.emplace(kCameraKey, std::move(proc)); 
}

void MainWindow::stopCamera() {
    shutdownDriver(kCameraKey); 
} 

void MainWindow::startLidar() {
    if (drivers_.count(kLidarKey)) return;

    auto proc = createDriverProcess(kLidarScript, kLidarKey);
    if (!proc) return;
    drivers_.emplace(kLidarKey, std::move(proc)); 
}

void MainWindow::stopLidar() {
    shutdownDriver(kLidarKey);
}

// -------------------------- Process Factory --------------------------

std::unique_ptr<QProcess> MainWindow::createDriverProcess(const QString& scriptPath, const QString& key) {
    auto proc = std::make_unique<QProcess>(this);
    // tells QProcess to merge the child's stdout and stderr streams into a single channel before the process starts
    // with the default SeparateChannels mode you would need two different ready-read signals or two reads to catch everything the script prints 
    proc->setProcessChannelMode(QProcess::MergedChannels); // stderr + stdout together

    // running the script through setsid makes it the leader of a new process group
    // every ROS node it spawns inherits the same PGID, which lets us alter kill the whole tree with killpg() 
    const QString program("/usr/bin/setsid");
    const QStringList args {"/bin/bash", "-c", QStringLiteral("exec %1").arg(scriptPath)};

    // readyReadStandardOutput() is a signal that merely tells us "new bytes have arrived"
    connect(proc.get(), &QProcess::started, this, [this, key, p = proc.get()]{
        qDebug() << key << " PID=" << p->processId();
    });
    connect(proc.get(), &QProcess::readyReadStandardOutput, this, &MainWindow::readDriverOutput);
    connect(proc.get(), &QProcess::errorOccurred, this, &MainWindow::driverCrashed);
    connect(proc.get(), qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this, &MainWindow::driverCrashed); 

    proc->start(program, args);
    // blocks the calling thead until the child process has actually been forked 
    // it then eturns true if the QProcess state became Running, otherwise false 
    if (!proc->waitForStarted()) {
        QMessageBox::critical(this, tr("Failed to start"),
                              tr("%1 driver could not be launched").arg(key)); 
        return nullptr;
    }
    return proc; 
}

// -------------------------- Generic Slots --------------------------

void MainWindow::readDriverOutput() {
    auto* senderProc = qobject_cast<QProcess*>(sender());
    if (!senderProc) return; 
    const QString text = QString::fromUtf8(senderProc->readAllStandardOutput());
    qDebug().noquote() << text; 
}

void MainWindow::driverCrashed() {
    // if you drop the asterisk here, the deduced type is still QProcess*, so the program works
    // but the declartion would look as if senderProc might be an object rather than a pointer 
    auto* senderProc = qobject_cast<QProcess*>(sender()); 
    if (!senderProc) return; 

    QString victimKey; 
    for (const auto& pair : drivers_) {
        if (pair.second.get() == senderProc) {
            victimKey = pair.first; 
            break; 
        }
    }
    if (victimKey.isEmpty()) return; 

    QMessageBox::warning(this, tr("Driver stopped"), tr("%1 has stopped running.").arg(victimKey));
    shutdownDriver(victimKey); 
}

// -------------------------- Shutdown Helpers --------------------------

void MainWindow::shutdownDriver(const QString& key) {
    auto it = drivers_.find(key);
    if (it == drivers_.end()) return; 

    const qint64 pid = it->second->processId();

    if (!killProcessGroup(pid, SIGINT, 2000) && !killProcessGroup(pid, SIGTERM, 2000)) {
        killProcessGroup(pid, SIGKILL, 0);
    }

    drivers_.erase(it); 
}

bool MainWindow::killProcessGroup(qint64 pid, int sig, int waitMs) {
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

// -------------------------- closeEvent --------------------------

void MainWindow::closeEvent(QCloseEvent* event) {
    for (auto it = drivers_.begin(); it != drivers_.end(); ) {
        const QString key = it->first;
        ++it;
        shutdownDriver(key);
    }
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() = default; 

