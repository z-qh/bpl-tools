#include <iostream>
#include "ros/ros.h"
#include "serial/serial.h"

serial::Serial ser;

// 请求数据帧的结构体
typedef struct{
    uint16_t header;
    uint8_t cmdId;
    uint8_t size;
    uint8_t steerId;
    uint16_t angle;
    uint16_t time;
    uint16_t power;
    uint8_t checksum;
}PackageTypeDef;

void send_test()
{
    PackageTypeDef pkg;
    uint32_t bSum=0;

    pkg.header = 0x4c12;//16
    pkg.cmdId = 8;//8
    pkg.size = 7;//8
    pkg.steerId = 0;//8
    pkg.angle = 0x0384;//16
    pkg.time = 0x07;//16
    pkg.power = 0x00;//16
    pkg.checksum = 0;//8

    bSum = (bSum + pkg.header&0xff) % 256;
    bSum = (bSum + (pkg.header>>8)&0xff) % 256;

    bSum = (bSum + pkg.cmdId) % 256;

    bSum = (bSum + pkg.size) % 256;

    bSum = (bSum + pkg.steerId) % 256;

    bSum = (bSum + pkg.angle&0xff) % 256;
    bSum = (bSum + (pkg.angle>>8)&0xff) % 256;

    bSum = (bSum + pkg.time&0xff) % 256;
    bSum = (bSum + (pkg.time>>8)&0xff) % 256;

    bSum = (bSum + pkg.power&0xff) % 256;
    bSum = (bSum + (pkg.power>>8)&0xff) % 256;

    pkg.checksum = bSum;

    uint8_t data[12];
    data[0] = pkg.header&0xff;
    data[1] = (pkg.header>>8)&0xff;
    data[2] = pkg.cmdId;
    data[3] = pkg.size;
    data[4] = pkg.steerId;
    data[5] = pkg.angle&0xff;
    data[6] = (pkg.angle>>8)&0xff;
    data[7] = pkg.time&0xff;
    data[8] = (pkg.time>>8)&0xff;
    data[9] = pkg.power&0xff;
    data[10] = (pkg.power>>8)&0xff;
    data[11] = pkg.checksum;

    ser.write(data, 12);
//    for(int i = 0; i < 12; i++)
//    {
//        std::cout << data[i];
//    }
//    std::cout << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "steer_test");
    ros::NodeHandle nh;


    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        std::cout << "Yes" << std::endl;
    }
    catch (serial::IOException& e)
    {
        std::cout << "error 111" << std::endl;
        return 0;
    }
    if(ser.isOpen())
    {
        std::cout << "Yes" << std::endl;
    }
    else
    {
        std::cout << "error 111" << std::endl;
        return 0;
    }
    ros::Rate loop(1);

    while (ros::ok())
    {
        ros::spinOnce();
        send_test();
    }

    return 0;
}