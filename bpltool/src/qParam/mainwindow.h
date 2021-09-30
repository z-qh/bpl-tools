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

#include <QHBoxLayout>
#include <QVBoxLayout>

#include "QPushButton"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "iostream"
#include "string"

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
    int pubQueueSize = 1;                               //ros publisher q size

private://sliderbar
    int appWidth = 800;//windows size
    int appHeight = 600;

    void initSliderBar();                               //init func
    void deInitSliderBar();
    bool eventFilter(QObject *obj, QEvent *event);      //mouse catach func

    QSpinBox *rollSpinBox, *pitchSpinBox,*yawSpinBox;   //adjust box
    QSlider *rollSlider, *pitchSlider, *yawSlider;      //slider
    QLabel *rollT, *pitchT, *yawT;                      //show value
    QLabel *rollTitle, *pitchTitle, *yawTitle;
    double roll = 0, pitch = 0, yaw = 0;                //var
    double rollLast = 0, pitchLast = 0, yawLast = 0;
    ros::Publisher *rollPub, *pitchPub, *yawPub;        //ros publisher to debug
    std::string rollTopic = "/R";                       //ros pub topics
    std::string pitchTopic = "/P";
    std::string yawTopic = "/Y";

private://button control
    ros::Publisher *keyControlPub;
    QPushButton *activeStopButton;
    bool activeStopButtonState = false;
    QLabel *buttonRunState;
    void buttonControlInit();
    void buttonControlDeInit();
};
#endif // MAINWINDOW_H
