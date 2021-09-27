#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <QTimer>
#include <Base_control/ControlCmd.h>
#include <std_msgs/String.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void next();

private:
    Ui::MainWindow *ui;
    QTimer* timer;
    ros::NodeHandle n;
    ros::ServiceClient Client;
    ros::Publisher Pub;
    Base_control::ControlCmd cmd;
    std_msgs::String pub_str;
};
#endif // MAINWINDOW_H
