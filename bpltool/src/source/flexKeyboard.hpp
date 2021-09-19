#include <signal.h>
#include <termios.h>
#include <thread>
#include <atomic>

namespace flexKeyboard{
    extern const char KEYCODE_SPACE = 0x20;
    /////////////////////////////////////////
    struct termios cooked, raw;
    char key = 0;
    char* address = &key;
    int delayTime = 100000;
    int kfd = 0;
    /////////////////////////////////////////
    void quit(int sig)
    {
        (void)sig;
        tcsetattr(kfd, TCSANOW, &cooked);
        exit(0);
    }
    /////////////////////////////////////////
    void getchat()
    {
        signal(SIGINT, &quit);
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
        while(1)
        {
            key = 0;
            if(read(kfd, &key, 1) < 0)
            {
                perror("read():");
                exit(-1);
            }
            usleep(100000);
        }
    }
    /////////////////////////////////////////
    std::thread keyBoard(getchat);
    bool keyBoardFlag = false;

    /////////////////////////////////////////
    class flexKeyboard{
    public:
        //bind key and response time, for better experiences, please be sure no more than 100Hz
        flexKeyboard(char*& key_, int response_time = -1){
            key_ = address;
            delayTime = response_time==-1?100000:(1000000/response_time);
            if(!keyBoardFlag){
                keyBoardFlag = true;
                keyBoard.detach();
            }
        }
    };
}



