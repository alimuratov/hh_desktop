#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QDebug>
#include <QProcess>
#include <QTimer>
#include <QString>
#include <QMessageBox>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void startDrivers();
    void readDriverOutput();
    void pollDrivers();
    void driverCrashed(); 
private:
    Ui::MainWindow *ui;
    QProcess* driverProcess{nullptr};
    QTimer pollTimer; 
    QString lastStatus{"INIT"};
};
#endif // MAINWINDOW_H
