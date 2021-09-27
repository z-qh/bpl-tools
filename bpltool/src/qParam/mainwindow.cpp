#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    nh = new ros::NodeHandle("~");

}

MainWindow::~MainWindow()
{
    delete nh;
    delete ui;
}
