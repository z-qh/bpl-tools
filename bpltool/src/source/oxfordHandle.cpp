#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "opencv2/opencv.hpp"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"

#include "gnss_driver/gps_navi_msg.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"

#include "pcl_conversions/pcl_conversions.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"


using namespace std;

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                           (float, z, z) (float, intensity, intensity)
                                           (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                           (double, time, time)
)

struct PointXYZIL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
	uint32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,
                                   (float, x, x)
								   (float, y, y)
                                   (float, z, z)
								   (float, intensity, intensity)
                                   (uint32_t, label, label)
)

typedef PointXYZIL PointSemantic;
typedef PointXYZIL PointType;


class oxford2Bag{
public:
    static double getTime(string& timeStamp){
        double thisTime = 0;
        while(timeStamp.front() == '0'){
            timeStamp.erase(timeStamp.begin());
        }
        if(timeStamp.size() != 16){
            cerr << timeStamp << " time stamp error " << endl;
            getchar();
            exit(0);
        }
        string secStr(timeStamp.begin(),timeStamp.end()-6);
        string msecStr(timeStamp.end()-6, timeStamp.end());
        thisTime = stod(secStr) + stod(msecStr) / 1000000.0;
        return thisTime;
    }
    static string getTimeStamp(double time){
        long long sec = int(time);
        long long msec = int((time - sec) * 1000000.0);
        stringstream ss;
        ss << fixed << sec;
        ss << fixed << setw(6) << setfill('0') << msec;
        return ss.str();
    }

private:
    string lidarTimesPath;
    string leftLidarPath;
    string leftLabelPath;
    string lidarPosesPath;

    vector<string> lidarTimes;
    // 获取所有lidar时间戳，并检测对应的Bin文件和Label文件
    void getLidarTimes(){
        ifstream lidarTimesFile(lidarTimesPath);
        while(lidarTimesFile.good()){
            string timeStamp;
            int noneValue;
            lidarTimesFile >> timeStamp >> noneValue;
            if(lidarTimesFile.eof()) break;
            lidarTimes.push_back(timeStamp);
            stringstream ssbin,sslabel;
            // continue;//ForTest No Check
            sslabel << leftLabelPath << "/" << fixed << timeStamp << ".label";
            ssbin   << leftLidarPath << "/" << fixed << timeStamp << ".bin";
            if( 0 != access(ssbin.str().c_str(), 0) || 0 != access(sslabel.str().c_str(), 0) ){
                cerr << " file not exist " << timeStamp << endl;
                lidarTimesFile.close();
                exit(0);
            }
        }
        lidarTimesFile.close();
        cout << "get lidar Times " << lidarTimes.size() << endl;
    }

    vector<nav_msgs::Odometry> GTOdom;
    vector<PointXYZIRPYT> GTPoses;
    vector<Eigen::Isometry3d> transPoses;
    vector<string> posesTimes;
    Eigen::Vector3d it;
    Eigen::Quaterniond iQ;
    Eigen::Isometry3d rader_to_lidar = Eigen::Isometry3d::Identity();
    // 获取GT位姿与其时间戳 ，保存成T-X-Y-Z-R-P-Y的形式
    void getLidarPose(){
        ifstream lidarPosesFile(lidarPosesPath);
        string head;
        getline(lidarPosesFile, head);
        Eigen::Isometry3d nowPose = Eigen::Isometry3d::Identity();
        nowPose.rotate(iQ);
        nowPose.pretranslate(it);
        while(lidarPosesFile.good()){
            string poseStr;
            getline(lidarPosesFile, poseStr);
            string time, noneTime;
            string dX,dY,dZ,dR,dP,dYaw;
            istringstream iss(poseStr);
            getline(iss, time, ',');
            getline(iss, noneTime, ',');
            getline(iss, dX, ',');
            getline(iss, dY, ',');
            getline(iss, dZ, ',');
            getline(iss, dR, ',');
            getline(iss, dP, ',');
            getline(iss, dYaw, ',');
            if(lidarPosesFile.eof()) break;
            Eigen::Vector3d dt(stod(dX),stod(dY),stod(dZ));
            auto temQ = tf::createQuaternionFromRPY(stod(dR), stod(dP), stod(dYaw));
            Eigen::Quaterniond dQ(temQ.w(), temQ.x(), temQ.y(), temQ.z());
            Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
            dT.rotate(dQ);
            dT.pretranslate(dt);
            dT = rader_to_lidar.inverse() * dT * rader_to_lidar;
            nowPose = nowPose * dT;
            transPoses.push_back(nowPose);
            posesTimes.push_back(time);
            // Odom
            nav_msgs::Odometry odom;
            odom.pose.pose.position.x = nowPose(0,3);
            odom.pose.pose.position.y = nowPose(1,3);
            odom.pose.pose.position.z = nowPose(2,3);
            Eigen::Quaterniond odomQ(nowPose.matrix().block<3,3>(0,0));
            odom.pose.pose.orientation.x = odomQ.x();
            odom.pose.pose.orientation.y = odomQ.y();
            odom.pose.pose.orientation.z = odomQ.z();
            odom.pose.pose.orientation.w = odomQ.w();
            odom.header.frame_id = "camera_init";
            odom.header.stamp = ros::Time().fromSec(getTime(time));
            GTOdom.push_back(odom);
            // TXYZRPY
            PointXYZIRPYT pp;
            pp.time = getTime(time);
            pp.x = nowPose(0,3);
            pp.y = nowPose(1,3);
            pp.z = nowPose(2,3);
            pp.yaw = nowPose.matrix().block<3,3>(0,0).eulerAngles(2,1,0)(0);
            pp.pitch = nowPose.matrix().block<3,3>(0,0).eulerAngles(2,1,0)(1);
            pp.roll = nowPose.matrix().block<3,3>(0,0).eulerAngles(2,1,0)(2);
            GTPoses.push_back(pp);
        }
        lidarPosesFile.close();
        cout << "get gt " << transPoses.size() << endl;
    }

    vector<gnss_driver::gps_navi_msg> GPSData;
    string GPSDataPath;
    // 获取GPSINS的数据
    void getGPS(){
        ifstream GPSFile(GPSDataPath);
        string header;
        getline(GPSFile, header);
        while(GPSFile.good()){
            string gpsstr;
            getline(GPSFile, gpsstr);
            istringstream iss(gpsstr);
            gnss_driver::gps_navi_msg tmpMsg;
            string time;
            string state, none;
            string lat,lon,ele;
            string north,east,down;
            string roll,pitch,yaw;
            string northspeed, eastspeed, downspeed;
            getline(iss, time, ',');
            getline(iss, state, ',');
            getline(iss, lat, ',');
            getline(iss, lon, ',');
            getline(iss, ele, ',');
            getline(iss, north, ',');//northing
            getline(iss, east, ',');//easting
            getline(iss, down, ',');//down
            getline(iss, none, ',');//utmzone
            getline(iss, northspeed, ',');//vel north
            getline(iss, eastspeed, ',');//vel east
            getline(iss, downspeed, ',');//vel donw
            getline(iss, roll, ',');
            getline(iss, pitch, ',');
            getline(iss, yaw, ',');
            if(GPSFile.eof()) break;
            tmpMsg.latitude = stod(lat);
            tmpMsg.longitude = stod(lon);
            tmpMsg.elevation = stod(ele);
            tmpMsg.InsWorkStatus = state;
            tmpMsg.rollAngle = stod(roll);
            tmpMsg.pitchAngle = stod(pitch);
            tmpMsg.yawAngle = stod(yaw);
            //NED 2 ENU
            tmpMsg.rollAngle = tmpMsg.rollAngle;
            tmpMsg.pitchAngle = -tmpMsg.pitchAngle;
            tmpMsg.yawAngle = -tmpMsg.yawAngle;
            tmpMsg.latitude = stod(east);
            tmpMsg.longitude = stod(north);
            tmpMsg.elevation = -stod(down);


            tmpMsg.northspeed = stod(northspeed);
            tmpMsg.eastspeed = stod(eastspeed);
            tmpMsg.skyspeed = -stod(downspeed);
            tmpMsg.header.frame_id = "camera_init";
            tmpMsg.header.stamp = ros::Time().fromSec(getTime(time));
            GPSData.push_back(tmpMsg);
        }
        GPSFile.close();
        cout << "get GPS " << GPSData.size() << endl;
    }

    string saveBagPath;

    bool LoadBinAndLabel(string& timeStamp, pcl::PointCloud<PointXYZIL>::Ptr cloud){
        cloud->clear();
        stringstream ssbin, sslabel;
        sslabel << leftLabelPath << "/" << fixed << timeStamp << ".label";
        ssbin << leftLidarPath << "/" << fixed << timeStamp << ".bin";
        if( 0 != access(ssbin.str().c_str(), 0) || 0 != access(sslabel.str().c_str(), 0) ) return false;
        ifstream binfile,labelfile;
        labelfile.open(sslabel.str(), ios::in | ios::binary);
        binfile.open(ssbin.str(), ios::in | ios::binary);
        // Check the file correction
        binfile.seekg(0, ios::end);
        labelfile.seekg(0, ios::end);
        int pointNums = binfile.tellg()/sizeof(float)/4;
        int labelNums = labelfile.tellg()/sizeof(int32_t);
        if(pointNums != labelNums+1){
            cout << sslabel.str() << ": " << labelNums << endl;
            cout << ssbin.str() << ": " << pointNums << endl;
            binfile.close();
            labelfile.close();
            return false;
        }
        binfile.seekg(0, ios::beg);
        labelfile.seekg(0, ios::beg);
        // load
        for (int j = 0; j < pointNums; j++) {
            PointXYZIL point;
            binfile.read((char *) &point.x, 3*sizeof(float));
            binfile.read((char *) &point.intensity, sizeof(float));
            uint32_t labelValue = 0;
            if(j!=0) labelfile.read((char *) &labelValue, sizeof(uint32_t));
            point.label = labelValue;
            point.x = -point.x;
            point.y = point.y;
            point.z = -point.z;
            cloud->push_back(point);
        }
        binfile.close(); 
        labelfile.close();
        return true;
    }

    void saveBagData(){
        rosbag::Bag BagWithGPS;
        BagWithGPS.open(saveBagPath, rosbag::bagmode::Write);
        cout << " Start Save Data to Bag !" << endl;
        for(int i = 0; i < lidarTimes.size(); ++i){
            string& cloudstr = lidarTimes[i];
            double thisTime = getTime(cloudstr);
            pcl::PointCloud<PointXYZIL>::Ptr cloudDest(new pcl::PointCloud<PointXYZIL>());
            if(!LoadBinAndLabel(cloudstr, cloudDest)) {
                cout << " File Get Wrong!" << cloudstr << endl;
                break;
            }
            cout << "load file " << cloudstr << " " << cloudDest->size() << endl;
            sensor_msgs::PointCloud2 msgTemp;
            pcl::toROSMsg(*cloudDest, msgTemp);
            msgTemp.header.frame_id = "camera_init";
            msgTemp.header.stamp = ros::Time().fromSec(thisTime);
            if(msgTemp.header.stamp.toSec() > endTime) break;
            BagWithGPS.write("/laser", msgTemp.header.stamp, msgTemp);
        }
        for(auto& gps : GPSData){
            if(gps.header.stamp.toSec() > endTime) break;
            BagWithGPS.write("/gps", gps.header.stamp, gps);
        }
        for(auto& gt : GTOdom){
            if(gt.header.stamp.toSec() > endTime) break;
            BagWithGPS.write("/gt", gt.header.stamp, gt);
        }
        BagWithGPS.close();

    }
    double endTime;
public:
    oxford2Bag() = delete;
    oxford2Bag(string base, string save, string end,
               Eigen::Vector3d it_=Eigen::Vector3d::Zero(),
               Eigen::Quaterniond iQ_=Eigen::Quaterniond(1,0,0,0)){
        it = it_;
        iQ = iQ_;
        rader_to_lidar(0,0) = -1;
        rader_to_lidar(1,1) = -1;

        endTime = getTime(end);
        leftLidarPath = base + "/left_bin";
        leftLabelPath = base + "/left_labels";
        lidarTimesPath = base + "/velodyne_left_timestamps";
        getLidarTimes();

        lidarPosesPath = base + "/gt.csv";
        getLidarPose();

        GPSDataPath = base + "/ins.csv";
        getGPS();

        saveBagPath = save;
        saveBagData();
    }
};

class enlargeGT{
public:
    static double getTime(string& timeStamp){
        double thisTime = 0;
        while(timeStamp.front() == '0'){
            timeStamp.erase(timeStamp.begin());
        }
        if(timeStamp.size() != 16){
            cerr << timeStamp << " time stamp error " << endl;
            getchar();
            exit(0);
        }
        string secStr(timeStamp.begin(),timeStamp.end()-6);
        string msecStr(timeStamp.end()-6, timeStamp.end());
        thisTime = stod(secStr) + stod(msecStr) / 1000000.0;
        return thisTime;
    }
    static vector<Eigen::Isometry3d> slerp(Eigen::Isometry3d&start,Eigen::Isometry3d&end,double time,double fre){
        int count = (int)(time * fre)+1;
        vector<Eigen::Isometry3d> result;
        Eigen::Quaterniond startQ(start.matrix().block<3,3>(0,0));
        Eigen::Quaterniond endQ(end.matrix().block<3,3>(0,0));
        Eigen::Vector3d Deltat = end.matrix().block<3,1>(0,3)-start.matrix().block<3,1>(0,3);
        cout << Deltat << endl;
        for(int i = 1; i < count; ++i){
            double thisTime = (double)i/(double)count;
            Eigen::Quaterniond tmpQ = startQ.slerp(thisTime, endQ);
            Eigen::Vector3d tmpt = start.matrix().block<3,1>(0,3);
            // Rot The Point
            Eigen::Isometry3d tmpT = Eigen::Isometry3d::Identity();
            tmpT.matrix().block<3,3>(0,0) = Eigen::Matrix3d(tmpQ);
            tmpT.matrix().block<3,1>(0,3) = tmpt;
            result.push_back(tmpT);
        }
        return result;
    }
    double newFre;
    string gtFile;
    vector<Eigen::Isometry3d> transPoses;
    vector<string> posesTimestamp;
    vector<double> posesTimes;
    // Read All Pose
    void readGT(){
        ifstream lidarPosesFile(gtFile);
        string head;
        getline(lidarPosesFile, head);
        Eigen::Isometry3d nowPose = Eigen::Isometry3d::Identity();
        while(lidarPosesFile.good()){
            string poseStr;
            getline(lidarPosesFile, poseStr);
            string time, noneTime;
            string dX,dY,dZ,dR,dP,dYaw;
            istringstream iss(poseStr);
            getline(iss, time, ',');
            getline(iss, noneTime, ',');
            getline(iss, dX, ',');
            getline(iss, dY, ',');
            getline(iss, dZ, ',');
            getline(iss, dR, ',');
            getline(iss, dP, ',');
            getline(iss, dYaw, ',');
            if(lidarPosesFile.eof()) break;
            Eigen::Vector3d dt(stod(dX),stod(dY),stod(dZ));
            auto temQ = tf::createQuaternionFromRPY(stod(dR), stod(dP), stod(dYaw));
            Eigen::Quaterniond dQ(temQ.w(), temQ.x(), temQ.y(), temQ.z());
            Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
            dT.rotate(dQ);
            dT.pretranslate(dt);
            nowPose = nowPose * dT;
            transPoses.push_back(nowPose);
            posesTimestamp.push_back(time);
            posesTimes.push_back(getTime(time));
        }
        lidarPosesFile.close();
        cout << "get gt " << transPoses.size() << endl;
    }
    //4Hz TO 20Hz, Spherical linear interpolationv
    vector<Eigen::Isometry3d> fixedPose;
    void enlarge(){
        for(int i = 0; i < transPoses.size()-1; ++i){
            auto&now=transPoses[i];
            auto&next=transPoses[i+1];
            double nowTime = posesTimes[i];
            double nextTime = posesTimes[i+1];
            auto mid = slerp(now, next, nextTime-nowTime, newFre);
            fixedPose.push_back(now);
            for(auto&p:mid) fixedPose.push_back(p);
        }
        fixedPose.push_back(transPoses.back());
    }

    void saveGT(){
        for(auto&p:fixedPose){
            double x,y,z,roll,pitch,yaw;
            x = p.matrix()(0,3);
            y = p.matrix()(1,3);
            z = p.matrix()(2,3);
            yaw = p.matrix().block<3,3>(0,0).eulerAngles(2,1,0)(0);
            pitch = p.matrix().block<3,3>(0,0).eulerAngles(2,1,0)(1);
            roll = p.matrix().block<3,3>(0,0).eulerAngles(2,1,0)(2);
            cout << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << endl;
        }
    }
    //For test
    enlargeGT(){
        Eigen::Isometry3d p1,p2;
        double time1=0,time2=1;
        newFre = 10;
        p1= Eigen::Quaterniond (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
                                                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
        p1.matrix().block<3,1>(0,3) = Eigen::Vector3d(3,1,0);

        p2 = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) *
                                                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
        p2.matrix().block<3,1>(0,3) = Eigen::Vector3d(6,4,0);
        posesTimes.push_back(time1);
        posesTimes.push_back(time2);
        transPoses.push_back(p1);
        transPoses.push_back(p2);
        enlarge();
        saveGT();
    }
    enlargeGT(string filePath, double destFrequence=20){
        newFre = destFrequence;
        gtFile = filePath;
        readGT();
        enlarge();
        saveGT();
    }
};

//目前这个数据集数据如下
// Oxford1
// lidar_PNG    44414   20Hz	timestamp+32*900点云
// GPS          11102    5Hz	timestamp+经纬高
// INS          111003  50Hz	timestamp+经纬高+欧拉角
// GT           8866     4Hz	source_timestamp+destination_timestamp+相对位姿+欧拉角
// 原始每个GT位姿两个时间戳之前对应了5帧点云，
// 1.取第一帧点云，剩余4帧抛弃掉，是做placerecog的思路
// 2.将20Hz的点云不利用位姿进行二合一，按照1800分辨率去重，得到32*1350点云，二合一后的点云时间戳同第一个，得到10Hz点云，每个GT两帧时间戳内取第一帧，后面的抛弃，这是为了跑suma的思路
// 后来想想，1方式就会更稀疏也无法分割，2方式可以分割但是点仍旧不够密集，而且畸变严重，怎么处理才比较合理呢，
// 这里“按照1800分辨率去重”的意思是，同loam中确定点的columnIdn的方式一样，
// 点云二合一势必出现columnIdn相同的点，这个在rangenet分割时会报错“XX不能除以0”，
// 仿照loam中程序抛弃第二个点然后得到大概32*1350左右的分辨率
// 其中一个解决方法是令GT插值至20HZ


void handleO1(){
    string endtime = "1547123004245831";
    string o1Source = "/home/qh/oxford1";
    string o1Save = "/home/qh/oxford1/o1.bag";
    // o1 X 5736002.726355 Y 619888.767784 Taw 2.645384
    Eigen::Vector3d o1t(2.726355, 8.767784, 0);
    auto o1tmpQ = tf::createQuaternionFromRPY(0,0,2.645384);
    Eigen::Quaterniond o1Q(o1tmpQ.w(), o1tmpQ.x(), o1tmpQ.y(), o1tmpQ.z());
    oxford2Bag o1(o1Source, o1Save, endtime);
}

int main(int argc, char** argv){
//    ros::init(argc, argv, "oxfordHandle");
//    ros::NodeHandle nh;
    handleO1();
//    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/ox", 100);
//    enlargeGT o1;
//    vector<nav_msgs::Odometry> odoms;
//    for(auto&p:o1.fixedPose){
//        nav_msgs::Odometry to;
//        to.header.frame_id = "map";
//        to.pose.pose.position.x = p.matrix()(0,3);
//        to.pose.pose.position.y = p.matrix()(1,3);
//        to.pose.pose.position.z = p.matrix()(2,3);
//        Eigen::Quaterniond tq(p.matrix().block<3,3>(0,0));
//        to.pose.pose.orientation.x = tq.x();
//        to.pose.pose.orientation.y = tq.y();
//        to.pose.pose.orientation.z = tq.z();
//        to.pose.pose.orientation.w = tq.w();
//        odoms.push_back(to);
//    }
//    int odomI = 0;
//    cout << endl;
//    ros::Rate loop(2);
//    while(ros::ok()){
//        if(odomI < odoms.size()) {
//            cout << odoms[odomI].pose.pose.position.x << " " << odoms[odomI].pose.pose.position.y << " " << odoms[odomI].pose.pose.position.z << endl;
//            pub.publish(odoms[odomI++]);
//        }
//        loop.sleep();
//    }
}