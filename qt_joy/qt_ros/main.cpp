#include "mainwindow.h"
#include <ros/ros.h>
#include <QApplication>
#include <Base_control/ControlCmd.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "qt_joy");
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
