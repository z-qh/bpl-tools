#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QSpinBox>
#include <QSlider>
#include <QLabel>

#include <QDesktopWidget>

#include <QtDebug>
#include <QEvent>
#include <QMouseEvent>

#include <QHBoxLayout>
#include <QVBoxLayout>

#include "QFileDialog"

#include "QPushButton"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "iostream"
#include "string"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "Eigen/Eigen"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


extern QVector<double> cloudShift;
extern int lenSize;
extern int widthSize;
extern float mDis;

class node{
private:
    pcl::PointCloud<pcl::PointXYZI> cloud;
    Eigen::MatrixXd SC[5];
    void createSC(){
        for(int ishift = 0; ishift < cloudShift.size(); ishift++){
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZI>());
            tmpCloud->clear();
            for(auto point:cloud.points)
            {
                point.z += cloudShift[ishift];
                tmpCloud->push_back(point);
            }
            SC[ishift] = makeScancontext_B(tmpCloud, lenSize, widthSize, mDis);
        }
    }
    static float xy2theta(const float&_x, const float& _y){
        if ( _x >= 0 & _y >= 0)
            return (180/M_PI) * atan(_y / _x);

        if ( _x < 0 & _y >= 0)
            return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

        if ( _x < 0 & _y < 0)
            return 180 + ( (180/M_PI) * atan(_y / _x) );

        if ( _x >= 0 & _y < 0)
            return 360 - ( (180/M_PI) * atan((-_y) / _x) );
    }
    template<typename T>
    static Eigen::MatrixXd makeScancontext_B(T &_scan, int PC_NUM_RING, int PC_NUM_SECTOR, double PC_MAX_RADIUS ){
        double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
        double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);
        int num_pts_scan_number = _scan->points.size();
        // main
        const int NO_POINT = -1000;
        //矩阵的维度为20*60
        Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);//PC_NUM_RING = 20, PC_NUM_SECTOR = 60 Ones为全1矩阵
        float value_angle,value_range;
        int ring_idx, sctor_idx;
        // std::cout<<"points size is "<<_scan->points.size()<<std::endl;
        #pragma omp parallel for num_threads(3)
        for(auto data:_scan->points){
            auto point = data;
            point.y += 20;   //保证z值大于0
            value_range = std::sqrt(point.x*point.x + point.z*point.z);
            value_angle = xy2theta(point.x,point.z);
            if(value_range > PC_MAX_RADIUS){
                continue;
            }
            ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((value_range/PC_MAX_RADIUS)*PC_NUM_RING))),1);//0还是1
            sctor_idx = std::max( std::min(PC_NUM_SECTOR, int(ceil((value_angle / 360.0) * PC_NUM_SECTOR ))), 1);
            if(desc(ring_idx-1,sctor_idx-1)<point.y)
                desc(ring_idx-1,sctor_idx-1) = point.y;
        }
        for(int row_idx = 0; row_idx < desc.rows(); ++row_idx)
            for(int col_idx = 0; col_idx < desc.cols(); ++col_idx)
                if(desc(row_idx, col_idx) == NO_POINT )
                    desc(row_idx, col_idx) = 0;
        return desc;
    }

    static Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd& _desc){
        /*
         * summary: columnwise mean vector
        */
        Eigen::MatrixXd variant_key(1, _desc.cols());
        for(int col_idx = 0; col_idx < _desc.cols(); col_idx++){
            Eigen::MatrixXd curr_col = _desc.col(col_idx);
            variant_key(0, col_idx) = curr_col.mean();
        }

        return variant_key;
    }

    static Eigen::MatrixXd circshift(Eigen::MatrixXd &mat, int _shift){
        if(_shift<0){
            return mat;
        }
        if(_shift == 0){
            Eigen::MatrixXd afterCircShift(mat);
            return afterCircShift;
        }
        Eigen::MatrixXd afterCircShift = Eigen::MatrixXd::Zero(mat.rows(),mat.cols());
        for(int col_idx = 0; col_idx<mat.cols(); ++col_idx){
            int newLocation = (col_idx+_shift)%mat.cols();
            afterCircShift.col(newLocation) = mat.col(col_idx);
        }
        return afterCircShift;
    }

    static int fastAlignUsingVKey(Eigen::MatrixXd& _vkey1, Eigen::MatrixXd& _vkey2){
        int argmin_vkey_shift = 0;
        double min_veky_diff_norm = 10000000;
        //shift_idx 为平移的量
        // int min_shift_idx = 0, max_shift_idx =
        for(int shift_idx = 0; shift_idx<_vkey1.cols(); ++shift_idx){

            Eigen::MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);//将一位矩阵平移
            Eigen::MatrixXd vkey_diff = _vkey1 - vkey2_shifted;//平移后的向量，将两者相减
            double cur_diff_norm = vkey_diff.norm();//norm为计算二范数

            if(cur_diff_norm<min_veky_diff_norm){
                argmin_vkey_shift = shift_idx;
                min_veky_diff_norm = cur_diff_norm;
            }
        }
        return argmin_vkey_shift;
    }

    static double distDirectSC(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2){
        int num_effective_cols = 0;
        double sum_similarity = 0;
        for(int col_idx = 0; col_idx<_sc1.cols(); ++col_idx){
            Eigen::VectorXd col_sc1 = _sc1.col(col_idx);
            Eigen::VectorXd col_sc2 = _sc2.col(col_idx);
            //norm返回向量的二范数
            if(col_sc1.norm() == 0 || col_sc2.norm()==0){
                continue;
            }
            double sector_similarity = col_sc1.dot(col_sc2)/(col_sc1.norm()*col_sc2.norm()); //1-sector_similarity为余弦相似度
            sum_similarity = sum_similarity + sector_similarity;
            num_effective_cols++;
        }
        double sc_sim = sum_similarity/(num_effective_cols*1.0);
        //double sc_sim = sum_similarity/num_effective_cols;
        return std::move(1.0-sc_sim);
    }

    static std::pair<double,int>distanceBtnScanContext(Eigen::MatrixXd &sc_1, Eigen::MatrixXd &sc_2){
        // 1. fast align using variant key (not in original IROS18)
        //快速配准 判断平移多少，
        Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(sc_1); //输出1*n维矩阵
        Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(sc_2);
        int argmin_vkey_shift = fastAlignUsingVKey(vkey_sc1,vkey_sc2);      //查找两个描述子之间最小平移量
        const int SEARCH_RADIUS = round(0.5*0.1*sc_1.cols());      //0.5*0.1*列数
        std::vector<int>shift_idx_search_space{argmin_vkey_shift};
        for(int i=0;i<SEARCH_RADIUS+1;++i){
            shift_idx_search_space.push_back((argmin_vkey_shift+i+sc_1.cols())%sc_1.cols());
            shift_idx_search_space.push_back((argmin_vkey_shift-i+sc_1.cols())%sc_1.cols());
        }
        std::sort(shift_idx_search_space.begin(),shift_idx_search_space.end());
        //2. 余弦距离匹配
        int argmin_shift = 0;
        double minScanContextDist = 10000000;
        for(int shift:shift_idx_search_space){
            Eigen::MatrixXd sc2_shifted = circshift(sc_2, shift);
            double curScanContextDist = distDirectSC(sc_1,sc2_shifted);
            if(curScanContextDist<minScanContextDist){
                argmin_shift = shift;
                minScanContextDist = curScanContextDist;
            }
        }
        return std::make_pair(minScanContextDist,argmin_shift);
    }

public:
    node() = delete;
    explicit node(pcl::PointCloud<pcl::PointXYZI>& cloud_){
        cloud = cloud_;
        createSC();
    }
    void updateSc(){
        createSC();
    }
    static float getScore(node&nodeOld, node&nodeNew){
        double min_dist = 10000000;
        int nn_align = 0, nn_idx = 0;   //nn_align为描述子旋转的角度值， nn_idx为匹配成功的索引值
        int loop_index = -1;

        for(int i = 0; i < 5; i++){
            Eigen::MatrixXd currentContextShift = nodeNew.SC[i];
            Eigen::MatrixXd scanContextCandidate = nodeOld.SC[2];
            std::pair<double, int> sc_dist_result = distanceBtnScanContext(currentContextShift, scanContextCandidate);
            double candidate_dist = sc_dist_result.first;       //余弦距离
            if(candidate_dist < min_dist)
            {
                min_dist = candidate_dist;                      //两个描述子之间的余弦距离
            }
        }
        return min_dist;
    }

};

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    ros::NodeHandle* nh;
    int pubQueueSize = 1;                               //ros publisher q size

private://sliderbar
    int appWidth = 800;//windows size
    int appHeight = 600;

    void initSliderBar();                               //init func
    void deInitSliderBar();
    bool eventFilter(QObject *obj, QEvent *event);      //mouse catach func

    QSpinBox *rollSpinBox, *pitchSpinBox,*yawSpinBox;   //adjust box
    QSlider *rollSlider, *pitchSlider, *yawSlider;      //slider
    QLabel *rollT, *pitchT, *yawT;                      //show value
    QLabel *rollTitle, *pitchTitle, *yawTitle;
    double roll = 0, pitch = 0, yaw = 0;                //var
    double rollLast = 0, pitchLast = 0, yawLast = 0;
    ros::Publisher *rollPub, *pitchPub, *yawPub;        //ros publisher to debug
    std::string rollTopic = "/R";                       //ros pub topics
    std::string pitchTopic = "/P";
    std::string yawTopic = "/Y";

private://button control
    ros::Publisher *keyControlPub;
    QPushButton *activeStopButton;
    bool activeStopButtonState = false;
    QLabel *buttonRunState;
    void buttonControlInit();
    void buttonControlDeInit();

private://cloud to scancontext
    QPushButton* openFileCloud;
    QPushButton* updateSc;

    QVector<QString> pathList;
    QVector<pcl::PointCloud<pcl::PointXYZI>> cloudList;
    QVector<node> nodeList;

    void initButtonCloud();
    void deinitButtonCloud();
    void handleFileOpen();
    void handleUpdate();

};
#endif // MAINWINDOW_H
