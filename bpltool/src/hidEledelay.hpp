#include "libusb-1.0/libusb.h"
#include <iostream>

/*
 * 使用说明
 * 首先这个库依赖libusb这个库，Ubuntu要确认以下安装：
 * sudo apt-get install libudev-dev libusb-1.0-0-dev libfox-1.6-dev
 * 调用本HPP的源文件要链接到usb-1.0，示例如下
 * target_link_libraries(projectName usb-1.0)
 */

//供应商ID，同一设备固定
#define VendorID 0x16c0
//产品ID，同一设备固定
#define ProductID 0x05df


class hid{
private:
    //各类端口号以及。。。
    uint16_t VID;
    uint16_t PID;
    libusb_context* ctx;
    libusb_device** devs;
    libusb_device_handle* devHandle;
    struct libusb_device_descriptor desc;
    int r;
    size_t cnt;
    uint8_t instruct[0x30];
public:
    int state;
    bool isopen;
public:
    //默认构造
    hid(): VID(VendorID), PID(ProductID), ctx(nullptr), state(0), isopen(false)
    {
        instruct[0] = 0x00;
        instruct[1] = 0x00;
        instruct[2] = 0x00;
        instruct[3] = 0x00;
        instruct[4] = 0x00;
        instruct[5] = 0x00;
        instruct[6] = 0x00;
        instruct[7] = 0x00;
        instruct[8] = 0x00^0x00^0x00^0x00^0x00^0x00;
        init();
        if(state != -1)
            std::cout << "Init success!" << std::endl;
        disconnect();
    }
    //析构
    ~hid()
    {
        libusb_release_interface(devHandle, 0);
        libusb_attach_kernel_driver(devHandle, 0);
        libusb_close(devHandle);
        libusb_exit(ctx);
    }
    //初始化，不成功则整个类失效
    void init()
    {
        r = libusb_init(&ctx);
        if(r < 0)
        {
            std::cout << "Init error!" << std::endl;
            state = -1;
            return;
        }
        //libusb_set_debug(ctx, 3);
        devHandle = libusb_open_device_with_vid_pid(ctx, VID, PID);
        if(devHandle == nullptr)
        {
            std::cout << "Open device error!" << std::endl;
            return;
            state = -1;
        }
        if(libusb_kernel_driver_active(devHandle, 0) == 1)
        {
            std::cout << "Kernel driver active" << std::endl;
            if(libusb_detach_kernel_driver(devHandle, 0) == 0)
                std::cout << "Kernel driver detached!" << std::endl;
        }
        r = libusb_claim_interface(devHandle, 0);
        if(r < 0)
        {
            std::cout << "Cannot claim interface!" << std::endl;
            state = -1;
            return;
        }

    }
//紧紧对外暴露接口函数
public:
    void connect()
    {
        instruct[0] = 0xff;
        instruct[1] = 0x01;
        if(state != -1)
        {
            libusb_control_transfer(devHandle, 0x21, 0x09, 0x0300, 0x00, instruct, 0x20, 18);
            isopen = true;
        }
    }
    void disconnect()
    {
        instruct[0] = 0xfd;
        instruct[1] = 0x01;
        if(state != -1)
        {
            libusb_control_transfer(devHandle, 0x21, 0x09, 0x0300, 0x00, instruct, 0x20, 18);
            isopen = false;
        }
    }
};

