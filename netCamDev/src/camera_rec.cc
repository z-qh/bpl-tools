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
    	
    	if(bind(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    	{
        	perror("bind error:");
        	exit(1);
    	}
    	
    	int recv_num;
	Mat image;
	
    	std::vector<uchar> decode(100000, 0);
    	std::vector<int> quality {cv::ImwriteFlags::IMWRITE_JPEG_QUALITY, 50};
    	
    	while(1)
    	{
    		recv_num = recvfrom(clientSocket, decode.data(), 100000, 0, (struct sockaddr *)&serverAddr, (socklen_t *)&len);
    		if(recv_num < 0)
    		{
    			perror("recvfrom error:");
    			return 1;
    		}
    		if(recv_num == 0) continue;	
    		decode.resize(recv_num);
    		image = imdecode(decode, cv::ImreadModes::IMREAD_COLOR); 
    		if (!image.empty())
        	{
            		imshow("image", image);
            		waitKey(100);
        	}
    	}
	return 0;
}
