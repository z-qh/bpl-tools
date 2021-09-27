#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QSpinBox>
#include <QSlider>
#include <QLabel>

#include <QDesktopWidget>

#include <QtDebug>
#include <QEvent>
#include <QMouseEvent>

#include "ros/ros.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    ros::NodeHandle* nh;
};
#endif // MAINWINDOW_H
