#include <iostream> 
#include <opencv2/opencv.hpp>

#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#pragma comment(lib, "WS2_32.lib")
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <malloc.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <fcntl.h>
#include <fcntl.h>
#endif

using namespace cv;
using namespace std;

#define SERVER_PORT 8000

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    	int clientSocket;
    	if ((clientSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    	{
        	perror("socket");
        	return 1;
    	}
    	
    	struct sockaddr_in serverAddr;
    	serverAddr.sin_family = AF_INET;
    	serverAddr.sin_port = htons(SERVER_PORT);
    	serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    	int len = sizeof(serverAddr);
	Mat image;
    	std::vector<uchar> data_encode(100000, 0);
    	std::vector<int> quality {cv::ImwriteFlags::IMWRITE_JPEG_QUALITY, 50};
    	
    	cout << "init" << endl;

    	VideoCapture capture;
    	capture.open(0);
	capture.set(CAP_PROP_FRAME_WIDTH, 640);
	capture.set(CAP_PROP_FRAME_HEIGHT, 480);

    	while(1)
    	{
    		
    		capture >> image;
    		if (image.empty())
		{
		    	printf("empty image\n\n");
		    	return -1;
		}
		imshow("Input video", image);
		waitKey(100);
		imencode(".jpg", image, data_encode, quality);
		auto n = sendto(clientSocket, data_encode.data(), data_encode.size(), 0, (struct sockaddr *)&serverAddr, len);
		if(n < 0)
        	{
            		perror("Send: ");
            		return 1;
        	}
        	data_encode.clear();        	
    	}
    	capture.release();
	return 0;
}
