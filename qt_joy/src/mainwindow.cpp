#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QTimer>
#include <Base_control/ControlCmd.h>
#include <std_msgs/String.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->label->setText("Yes!");
    timer = new QTimer(this);
    timer->setInterval(1000 / 1);
    connect(timer, SIGNAL(timeout()), this, SLOT(next()));
    timer->start();
    Pub = n.advertise<std_msgs::String>("test", 100);
    Client = n.serviceClient<Base_control::ControlCmd>("Controler");
    this->cmd.request.xx = 0;
    this->cmd.request.yy = 0;
    pub_str.data = "123";
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::next()
{
  ui->label->setText("Yes");
  Client.call(this->cmd);
}

void MainWindow::on_pushButton_clicked()
{
    ui->label->setText("");
    Pub.publish(pub_str);
}
