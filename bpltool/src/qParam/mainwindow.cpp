#include "mainwindow.h"
#include "./ui_mainwindow.h"


QVector<double> cloudShift{-4.0, -1.0, 0, 1.0, 4.0};
int lenSize = 20;
int widthSize = 60;
float mDis = 50;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    nh = new ros::NodeHandle("~");
    setFixedSize(appWidth, appHeight);
    int currentScreenWidth = QApplication::desktop()->width();
    int currentScreenHeight = QApplication::desktop()->height();
    qDebug() << currentScreenWidth  << currentScreenHeight;
    move((currentScreenWidth-appWidth)/2, (currentScreenHeight-appHeight)/2);
    //initSliderBar();
    //buttonControlInit();
    initButtonCloud();
}

MainWindow::~MainWindow()
{
    delete nh;
    //deInitSliderBar();
    //buttonControlDeInit();
    deinitButtonCloud();
    delete ui;
}
////////////////////////////////////////////
void MainWindow::deInitSliderBar(){
    delete rollSpinBox;         //free memory
    delete pitchSpinBox;
    delete yawSpinBox;
    delete rollSlider;
    delete pitchSlider;
    delete yawSlider;
    delete rollT;
    delete pitchT;
    delete yawT;
    delete rollPub;
    delete pitchPub;
    delete yawPub;
    delete rollTitle;
    delete pitchTitle;
    delete yawTitle;
}
void MainWindow::initSliderBar(){
    int min = -314;
    int max = 314;
    int singleStep = 1;
    double barStartPosi = 0.2;
    double barWidthfactor = 0.6;
    double barendPosi = 0.85;
    int upBoard = 20;
    int barHeight = 50;
    int index = 0;
    int adBoxWidth = 100, adBoxHeight = 50;
    int adBoxStartX = 60;
    int vertialDis = 50;
    int barTitleStartPosi = 20;


    //init slider bar
    rollSlider = new QSlider(this);
    rollSlider->setOrientation(Qt::Horizontal);
    rollSlider->setRange(min, max);
    rollSlider->setSingleStep(singleStep);
    rollSlider->move(appWidth * barStartPosi, upBoard+index*vertialDis);
    rollSlider->resize(appWidth * barWidthfactor, barHeight);
    rollSpinBox = new QSpinBox(this);
    rollSpinBox->setRange(min, max);
    rollSpinBox->setSingleStep(singleStep);
    rollSpinBox->move(adBoxStartX, upBoard+index*vertialDis);
    rollSpinBox->resize(adBoxWidth, adBoxHeight);
    connect(rollSpinBox, SIGNAL(valueChanged(int)), rollSlider, SLOT(setValue(int)));
    connect(rollSlider, SIGNAL(valueChanged(int)), rollSpinBox, SLOT(setValue(int)));
    rollT = new QLabel(this);
    rollT->resize(adBoxWidth, adBoxHeight);
    rollT->move(appWidth * barendPosi, upBoard+index*vertialDis);
    rollT->setText(QString::number(roll));
    rollSlider->installEventFilter(this);
    rollSpinBox->installEventFilter(this);
    rollTitle = new QLabel(this);
    rollTitle->resize(adBoxWidth/2, adBoxHeight);
    rollTitle->move(barTitleStartPosi, upBoard+index*vertialDis);
    rollTitle->setText(QString("R"));

    index++;


    //init slider bar
    pitchSlider = new QSlider(this);
    pitchSlider->setOrientation(Qt::Horizontal);
    pitchSlider->setRange(min, max);
    pitchSlider->setSingleStep(singleStep);
    pitchSlider->move(appWidth * barStartPosi, upBoard+index*vertialDis);
    pitchSlider->resize(appWidth * barWidthfactor, barHeight);
    pitchSpinBox = new QSpinBox(this);
    pitchSpinBox->setRange(min, max);
    pitchSpinBox->setSingleStep(singleStep);
    pitchSpinBox->move(adBoxStartX, upBoard+index*vertialDis);
    pitchSpinBox->resize(adBoxWidth, adBoxHeight);
    connect(pitchSpinBox, SIGNAL(valueChanged(int)), pitchSlider, SLOT(setValue(int)));
    connect(pitchSlider, SIGNAL(valueChanged(int)), pitchSpinBox, SLOT(setValue(int)));
    pitchT = new QLabel(this);
    pitchT->resize(adBoxWidth, adBoxHeight);
    pitchT->move(appWidth * barendPosi, upBoard+index*vertialDis);
    pitchT->setText(QString::number(pitch));
    pitchSlider->installEventFilter(this);
    pitchSpinBox->installEventFilter(this);
    pitchTitle = new QLabel(this);
    pitchTitle->resize(adBoxWidth/2, adBoxHeight);
    pitchTitle->move(barTitleStartPosi, upBoard+index*vertialDis);
    pitchTitle->setText(QString("P"));
    index++;

    //init slider bar
    yawSlider = new QSlider(this);
    yawSlider->setOrientation(Qt::Horizontal);
    yawSlider->setRange(min, max);
    yawSlider->setSingleStep(singleStep);
    yawSlider->move(appWidth * barStartPosi, upBoard+index*vertialDis);
    yawSlider->resize(appWidth * barWidthfactor, barHeight);
    yawSpinBox = new QSpinBox(this);
    yawSpinBox->setRange(min, max);
    yawSpinBox->setSingleStep(singleStep);
    yawSpinBox->move(adBoxStartX, upBoard+index*vertialDis);
    yawSpinBox->resize(adBoxWidth, adBoxHeight);
    connect(yawSpinBox, SIGNAL(valueChanged(int)), yawSlider, SLOT(setValue(int)));
    connect(yawSlider, SIGNAL(valueChanged(int)), yawSpinBox, SLOT(setValue(int)));
    yawT = new QLabel(this);
    yawT->resize(adBoxWidth, adBoxHeight);
    yawT->move(appWidth * barendPosi, upBoard+index*vertialDis);
    yawT->setText(QString::number(yaw));
    yawSlider->installEventFilter(this);
    yawSpinBox->installEventFilter(this);
    yawTitle = new QLabel(this);
    yawTitle->resize(adBoxWidth/2, adBoxHeight);
    yawTitle->move(barTitleStartPosi, upBoard+index*vertialDis);
    yawTitle->setText(QString("Y"));

    //init publisher
    rollPub = new ros::Publisher();
    pitchPub = new ros::Publisher();
    yawPub = new ros::Publisher();
    *rollPub = nh->advertise<std_msgs::Float64>(rollTopic, pubQueueSize);
    *pitchPub = nh->advertise<std_msgs::Float64>(pitchTopic, pubQueueSize);
    *yawPub = nh->advertise<std_msgs::Float64>(yawTopic, pubQueueSize);

    //debug info
    qDebug() << QString("windows size: width ") << appWidth << QString(" height ") << appHeight;
    qDebug() << QString("bar left") << rollSlider->geometry().topLeft().x();
    qDebug() << QString("spinBox left") << rollSpinBox->geometry().topLeft().x();
}
void MainWindow::buttonControlInit() {
    int buttonWidth = 100;
    int buttonHeigh = 60;
    double buttonXStartFactor = 0.1;
    int buttonYstart = 200;

    activeStopButton = new QPushButton(this);
    activeStopButton->resize(buttonWidth, buttonHeigh);
    activeStopButton->move(appWidth * buttonXStartFactor, buttonYstart);
    activeStopButton->setText(QString("start"));
    activeStopButton->installEventFilter(this);

    //ros publisher
    keyControlPub = new ros::Publisher();
    *keyControlPub = nh->advertise<std_msgs::String>("/keyboardControlCmd", pubQueueSize);
}
void MainWindow::buttonControlDeInit() {
    delete activeStopButton;
    delete keyControlPub;
}

////////////////////////////////////////////

void MainWindow::initButtonCloud(){
    openFileCloud = new QPushButton(this);
    openFileCloud->resize(100, 50);
    openFileCloud->move(100, 100);
    openFileCloud->setText(QString("open cloud"));

    updateSc = new QPushButton(this);
    updateSc->resize(100, 50);
    updateSc->move(100, 200);
    updateSc->setText(QString("update"));

    connect(openFileCloud, &QPushButton::clicked, this, &MainWindow::handleFileOpen);
    connect(updateSc, &QPushButton::clicked, this, &MainWindow::handleUpdate);
}
void MainWindow::deinitButtonCloud(){
    delete openFileCloud;
}

void MainWindow::handleFileOpen(){
    QString fileName = QFileDialog::getOpenFileName(
            this, tr("open pcd file"),
            "./", tr("pcd files(*.pcd );;All files (*.*)"));
    qDebug() << fileName;
}

void MainWindow::handleUpdate(){

}

////////////////////////////////////////////

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    //push button
    if(obj == activeStopButton && event->type() == QEvent::MouseButtonRelease){
        activeStopButtonState = !activeStopButtonState;
        if(activeStopButtonState){
            std_msgs::String tempMsg;
            tempMsg.data = std::string("true");
            keyControlPub->publish(tempMsg);
            activeStopButton->setText(QString("stop"));
        }else{
            std_msgs::String tempMsg;
            tempMsg.data = std::string("false");
            keyControlPub->publish(tempMsg);
            activeStopButton->setText(QString("start"));
        }
    }
    //spinbox and slider
    if(obj==rollSlider || obj == rollSpinBox)
    {
        roll = rollSlider->value() / 100.0;
        rollT->setText(QString::number(roll));
        if(roll != rollLast)
        {
            std_msgs::Float64 tempMsgs;
            tempMsgs.data = roll;
            rollPub->publish(tempMsgs);
            rollLast = roll;
        };
    }else if(obj == pitchSlider || obj == pitchSpinBox){
        pitch = pitchSlider->value() / 100.0;
        pitchT->setText(QString::number(pitch));
        if(pitch != pitchLast){
            std_msgs::Float64 tempMsgs;
            tempMsgs.data = pitch;
            pitchPub->publish(tempMsgs);
            pitchLast = pitch;
        }
    }else if(obj == yawSlider || obj == yawSpinBox){
        yaw = yawSlider->value() / 100.0;
        yawT->setText(QString::number(yaw));
        if(yaw != yawLast){
            std_msgs::Float64 tempMsgs;
            tempMsgs.data = yaw;
            yawPub->publish(tempMsgs);
            yawLast = yaw;
        }
    }
    return QObject::eventFilter(obj,event);
}



