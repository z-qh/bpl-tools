#include <ros/ros.h>
#include <iostream>
#include <Base_control/ControlCmd.h>

void speed_Cast(Base_control::ControlCmd &cmd)
{
  cmd.request.xx = cmd.request.xx>200?200:cmd.request.xx;
  cmd.request.xx = cmd.request.xx<-200?-200:cmd.request.xx;
  cmd.request.yy = cmd.request.yy>200?200:cmd.request.yy;
  cmd.request.yy = cmd.request.yy<-200?-200:cmd.request.yy;
//  if(cmd.request.xx >= 200)
//  {
//    cmd.request.xx = 200;
//  }
//  if(cmd.request.xx <= -200)
//  {
//    cmd.request.xx = -200;
//  }
//  if(cmd.request.yy >= 100)
//  {
//    cmd.request.yy = 100;
//  }
//  if(cmd.request.yy <= -100)
//  {
//    cmd.request.yy = -100;
//  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "qt_joy");
  ros::NodeHandle n;
  ros::ServiceClient Client = n.serviceClient<Base_control::ControlCmd>("/Controler");
  std::cout << "hello world!" << std::endl;
  Base_control::ControlCmd cmd;
  cmd.request.xx = 0;
  cmd.request.yy - 0;
  ros::Rate loop(1);
  while(ros::ok())
  {
    cmd.request.xx += 20;
    cmd.request.yy += 10;
    speed_Cast(cmd);
    Client.call(cmd);
    loop.sleep();
  }
  return 0;
}