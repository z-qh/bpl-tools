#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "gnss_driver/gps_navi_msg.h"
#include "tf/transform_datatypes.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include "../include/imu_gnss/gps_ins_msg.h"
#include "sensor_msgs/Imu.h"

#include "flexKeyboard.hpp"

using namespace std;

bool getOdomFronFile(vector<nav_msgs::Odometry>& odom, string filePath)
{
    ifstream file;
    file.open(filePath, ios::in);
    if(!file.good())
        return false;
    int odomSize = 0;
    file >> odomSize;
    odom.resize(odomSize);
    for(int i = 0; i < odomSize; i++){
        double time = 0, roll = 0, pitch = 0, yaw = 0, temp;
        file >> time >> odom[i].pose.pose.position.x
        >>odom[i].pose.pose.position.y
        >>odom[i].pose.pose.position.z
        >> roll >> pitch >> yaw >> temp;
        auto q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        odom[i].header.frame_id = "map";
        odom[i].header.stamp = ros::Time().fromSec(time);
        odom[i].pose.pose.orientation = q;
    }
    file.close();
    return true;
}

string realPath = "/home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/odomReal.txt";
string simPath = "/home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/odomSim.txt";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imuWatcher");
    ros::NodeHandle nh;
    char* key;
    flexKeyboard::flexKeyboard FK(key);

    ros::Publisher pubSim = nh.advertise<nav_msgs::Odometry>("/odomSim", 100);
    ros::Publisher pubReal = nh.advertise<nav_msgs::Odometry>("/odomReal", 100);

    vector<nav_msgs::Odometry> globalOdomSim;
    vector<nav_msgs::Odometry> globalOdomReal;

    getOdomFronFile(globalOdomSim, simPath);
    getOdomFronFile(globalOdomReal, realPath);

    tf::StampedTransform aftMappedTrans;
    tf::TransformBroadcaster tfBroadcaster;


    aftMappedTrans.frame_id_ = "map";
    aftMappedTrans.child_frame_id_ = "aft_mapped";

    ros::Rate* loop;
    loop = new ros::Rate(30);
    int fps = 30;
    bool state = false;
    int simCount = 0;
    int realCount = 0;
    while(ros::ok())
    {
        if(*key == '1' || *key == '!'){
            if(simCount<globalOdomSim.size()){
                if(simCount<globalOdomSim.size()) {
                    aftMappedTrans.stamp_ = globalOdomSim[simCount].header.stamp;
                    aftMappedTrans.setRotation(tf::Quaternion(-globalOdomSim[simCount].pose.pose.orientation.y,
                                                              -globalOdomSim[simCount].pose.pose.orientation.z,
                                                              globalOdomSim[simCount].pose.pose.orientation.x,
                                                              globalOdomSim[simCount].pose.pose.orientation.w));
                    aftMappedTrans.setOrigin(tf::Vector3(globalOdomSim[simCount].pose.pose.position.x,
                                                         globalOdomSim[simCount].pose.pose.position.y,
                                                         globalOdomSim[simCount].pose.pose.position.z));
                    tfBroadcaster.sendTransform(aftMappedTrans);
                    pubSim.publish(globalOdomSim[simCount++]);
                }
            }
        }else if(*key == '2' || *key == '@'){
            if(realCount<globalOdomReal.size()) {
                aftMappedTrans.stamp_ = globalOdomReal[realCount].header.stamp;
                aftMappedTrans.setRotation(tf::Quaternion(-globalOdomReal[realCount].pose.pose.orientation.y,
                                                          -globalOdomReal[realCount].pose.pose.orientation.z,
                                                          globalOdomReal[realCount].pose.pose.orientation.x,
                                                          globalOdomReal[realCount].pose.pose.orientation.w));
                aftMappedTrans.setOrigin(tf::Vector3(globalOdomReal[realCount].pose.pose.position.x,
                                                     globalOdomReal[realCount].pose.pose.position.y,
                                                     globalOdomReal[realCount].pose.pose.position.z));
                tfBroadcaster.sendTransform(aftMappedTrans);
                pubReal.publish(globalOdomReal[realCount++]);
            }
        }else if(*key == flexKeyboard::KEYCODE_SPACE){
            state = !state;
        }else if(*key == '-' || *key == '_'){
            delete loop;
            fps = (fps-5)<5?5:(fps-5);
            cout << "fps: " << fps << endl;
            loop = new ros::Rate(fps);
        }else if(*key == '=' || *key == '+'){
            delete loop;
            fps = (fps+5)>50?50:(fps+5);
            cout << "fps: " << fps << endl;
            loop = new ros::Rate(fps);
        }else;
        if(state)
        {
            if(simCount<globalOdomSim.size()) {
                aftMappedTrans.stamp_ = globalOdomSim[simCount].header.stamp;
                aftMappedTrans.setRotation(tf::Quaternion(-globalOdomSim[simCount].pose.pose.orientation.y,
                                                          -globalOdomSim[simCount].pose.pose.orientation.z,
                                                          globalOdomSim[simCount].pose.pose.orientation.x,
                                                          globalOdomSim[simCount].pose.pose.orientation.w));
                aftMappedTrans.setOrigin(tf::Vector3(globalOdomSim[simCount].pose.pose.position.x,
                                                     globalOdomSim[simCount].pose.pose.position.y,
                                                     globalOdomSim[simCount].pose.pose.position.z));
                tfBroadcaster.sendTransform(aftMappedTrans);
                pubSim.publish(globalOdomSim[simCount++]);
            }
            if(false&&realCount<globalOdomReal.size()) {
                aftMappedTrans.stamp_ = globalOdomReal[realCount].header.stamp;
                aftMappedTrans.setRotation(tf::Quaternion(-globalOdomReal[realCount].pose.pose.orientation.y,
                                                          -globalOdomReal[realCount].pose.pose.orientation.z,
                                                          globalOdomReal[realCount].pose.pose.orientation.x,
                                                          globalOdomReal[realCount].pose.pose.orientation.w));
                aftMappedTrans.setOrigin(tf::Vector3(globalOdomReal[realCount].pose.pose.position.x,
                                                     globalOdomReal[realCount].pose.pose.position.y,
                                                     globalOdomReal[realCount].pose.pose.position.z));
                tfBroadcaster.sendTransform(aftMappedTrans);
                pubReal.publish(globalOdomReal[realCount++]);
            }
        }
        loop->sleep();
    }
    return 0;
}