#include "ros/ros.h"

#include "iostream"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/core/auto_differentiation.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"

#include <opencv2/highgui/highgui.hpp>
#include "random"

using namespace std;
using namespace g2o;

void genereteData(vector<double>&dataX_, vector<double>&dataY_){
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::normal_distribution<double> gauss(0, 1);
    dataX_.clear();
    dataY_.clear();
    for(int i = 0; i < 1000; i++){
        double x = i / 50.0;
        double err = gauss(gen);
        double y = 3 * x * x + 4 * x + 5 + err;
        dataX_.push_back(x);
        dataY_.push_back(y);
    }
}

class fitvertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }
    void oplusImpl(const double* update) override {
        _estimate += Eigen::Vector3d(update);
    }
    bool read(std::istream& is) override {
        is >> _estimate[0] >> _estimate[1] >> _estimate[2];
        return true;
    }
    bool write(std::ostream& os) const override{
        os << _estimate[0] << _estimate[1] << _estimate[2];
        return os.good();
    }
};

class fitedge: public g2o::BaseUnaryEdge<1, double, fitvertex>
{
public:
    double x;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit fitedge(double x_): g2o::BaseUnaryEdge<1, double, fitvertex>(), x(x_){}
    // 计算模型误差
    void computeError() override{
        // 一元边 即上面定义的边
        const fitvertex* v = dynamic_cast<const fitvertex*>(_vertices[0]);
        // _error 是误差向量 维度由构建 fitedge 定义
        const Eigen::Vector3d& param = v->estimate();
        _error(0,0) = _measurement - (param[0] * x * x + param[1] * x + param[2]);
    }
    bool read(std::istream &is) override {
        is >> x;
        return true;
    }
    bool write(std::ostream &os) const override {
        os << x;
        return os.good();
    }
};



int main2(int argc, char** argv){
    vector<double> dataX,dataY;
    // 生成数据
    genereteData(dataX, dataY);
    // H 矩阵
    using BlockSolverTpye = g2o::BlockSolverPL<3, 1>;
    using LinearSolverType = g2o::LinearSolverDense<BlockSolverTpye::PoseMatrixType>;
    // LM 算法
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverTpye>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);
    auto* v = new fitvertex();
    v->setEstimate(Eigen::Vector3d(0,0,0));
    v->setId(0);
    optimizer.addVertex(v);
    for(size_t i = 0; i < dataX.size(); i++){
        auto* edge = new fitedge(dataX[i]);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(dataY[i]);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        optimizer.addEdge(edge);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    Eigen::Vector3d result = v->estimate();
    cout << "a:" << result[0] << " b:" << result[1] << " c:" << result[2] << endl;
    return 0;
}

using namespace std;




int main(){
    // 三帧的位姿，准确
    Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R2(Eigen::AngleAxisd(20.0/180.0*M_PI, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d R3(Eigen::AngleAxisd(40.0/180.0*M_PI, Eigen::Vector3d::UnitZ()));
    // 三帧的平移，准确
    Eigen::Vector3d t1(0, 0, 0);
    Eigen::Vector3d t2(5, 1, 0);
    Eigen::Vector3d t3(10, 3, 0);
    // 地图点的全局位姿，准确（全局坐标为第一帧位姿）
    Eigen::Vector3d map_point1 = Eigen::Vector3d(3,5,0);
    Eigen::Vector3d map_point2 = Eigen::Vector3d(7,5,0);
    Eigen::Vector3d map_point3 = Eigen::Vector3d(5,8,0);
    // 随机噪声参数
    std::default_random_engine eng;
    std::normal_distribution<double> gauss1(0, 0.1);
    std::normal_distribution<double> gauss2(0, 0.1);
    // 后面的假设都建立在观测是有噪声的情况下
    // t1 += Eigen::Vector3d(gauss2(eng), gauss2(eng), gauss2(eng));
    // t2 += Eigen::Vector3d(gauss2(eng), gauss2(eng), gauss2(eng));
    // t3 += Eigen::Vector3d(gauss2(eng), gauss2(eng), gauss2(eng));
    // 每一帧下对应的观测，是边数据，此处位姿是有误差的，但是每一帧观测是比较准确的
    Eigen::Vector3d mesaure_1_1 = R1.inverse() * map_point1 - R1.inverse() * t1;
    Eigen::Vector3d mesaure_1_2 = R1.inverse() * map_point2 - R1.inverse() * t1;
    Eigen::Vector3d mesaure_1_3 = R1.inverse() * map_point3 - R1.inverse() * t1;
    Eigen::Vector3d mesaure_2_1 = R2.inverse() * map_point1 - R2.inverse() * t2;
    Eigen::Vector3d mesaure_2_2 = R2.inverse() * map_point2 - R2.inverse() * t2;
    Eigen::Vector3d mesaure_2_3 = R2.inverse() * map_point3 - R2.inverse() * t2;
    Eigen::Vector3d mesaure_3_1 = R3.inverse() * map_point1 - R3.inverse() * t3;
    Eigen::Vector3d mesaure_3_2 = R3.inverse() * map_point2 - R3.inverse() * t3;
    Eigen::Vector3d mesaure_3_3 = R3.inverse() * map_point3 - R3.inverse() * t3;
    // 情况2，观测本身是有误差的
    map_point1 += Eigen::Vector3d(gauss1(eng), gauss1(eng), gauss1(eng));
    map_point2 += Eigen::Vector3d(gauss1(eng), gauss1(eng), gauss1(eng));
    map_point3 += Eigen::Vector3d(gauss1(eng), gauss1(eng), gauss1(eng));

    // pose维度6 landmark维度3
    using BlockSolverTpye = g2o::BlockSolverPL<6, 3>;
    using LinearSolverType = g2o::LinearSolverDense<BlockSolverTpye::PoseMatrixType>;
    // LM 算法
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverTpye>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    cout << " optimizer  Levenberg " << endl;
    g2o::ParameterSE3Offset* cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setOffset();
    cameraOffset->setId(0);
    optimizer.addParameter(cameraOffset);
    
    // 位姿是顶点
    // 第一帧位姿
    g2o::VertexSE3 * vSE3_1 = new g2o::VertexSE3();
    vSE3_1->setEstimate(g2o::SE3Quat(R1,t1));
    vSE3_1->setId(0);
    vSE3_1->setFixed(false);
    optimizer.addVertex(vSE3_1);

    // 第二帧位姿
    g2o::VertexSE3 * vSE3_2 = new g2o::VertexSE3();
    vSE3_2->setEstimate(g2o::SE3Quat(R2,t2));
    vSE3_2->setId(1);
    vSE3_2->setFixed(false);
    optimizer.addVertex(vSE3_2);

    // 第三帧位姿
    g2o::VertexSE3 * vSE3_3 = new g2o::VertexSE3();
    vSE3_3->setEstimate(g2o::SE3Quat(R3,t3));
    vSE3_3->setId(2);
    vSE3_3->setFixed(false);
    optimizer.addVertex(vSE3_3);

    // 路标点是顶点
    // 第一个路标点
    g2o::VertexPointXYZ * vMP3_1 = new g2o::VertexPointXYZ();
    vMP3_1->setEstimate(map_point1);
    vMP3_1->setId(3);
    vMP3_1->setFixed(false);
    optimizer.addVertex(vMP3_1);

    // 第二个路标点
    g2o::VertexPointXYZ * vMP3_2 = new g2o::VertexPointXYZ();
    vMP3_2->setEstimate(map_point2);
    vMP3_2->setId(4);
    vMP3_2->setFixed(false);
    optimizer.addVertex(vMP3_2);

    // 第三个路标点
    g2o::VertexPointXYZ * vMP3_3 = new g2o::VertexPointXYZ();
    vMP3_3->setEstimate(map_point3);
    vMP3_3->setId(5);
    vMP3_3->setFixed(false);
    optimizer.addVertex(vMP3_3);


    cout << " Frame 1" << endl;
    // 第一帧数据 三条二元边
    // 第一条
    g2o::EdgeSE3PointXYZ * edge1_1 = new EdgeSE3PointXYZ();
    edge1_1->setId(6);
    edge1_1->setParameterId(0, 0);
    edge1_1->setVertex(0, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(0)));
    edge1_1->setVertex(1, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(3)));
    edge1_1->setMeasurement(mesaure_1_1);
    optimizer.addEdge(edge1_1);

    // 第二条
    EdgeSE3PointXYZ* edge1_2 = new EdgeSE3PointXYZ();
    edge1_2->setId(7);
    edge1_2->setParameterId(0, 0);
    edge1_2->setVertex(0, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(0)));
    edge1_2->setVertex(1, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(4)));
    edge1_2->setMeasurement(mesaure_1_2);
    optimizer.addEdge(edge1_2);

    // 第三条
    EdgeSE3PointXYZ* edge1_3 = new EdgeSE3PointXYZ();
    edge1_3->setId(8);
    edge1_3->setParameterId(0, 0);
    edge1_3->setVertex(0, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(0)));
    edge1_3->setVertex(1, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(5)));
    edge1_3->setMeasurement(mesaure_1_3);
    optimizer.addEdge(edge1_3);


    cout << " Frame 2" << endl;
    // 第二帧数据 三条二元边
    // 第一条
    EdgeSE3PointXYZ* edge2_1 = new EdgeSE3PointXYZ();
    edge2_1->setId(9);
    edge2_1->setParameterId(0, 0);
    edge2_1->setVertex(0, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(1)));
    edge2_1->setVertex(1, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(3)));
    edge2_1->setMeasurement(mesaure_2_1);
    optimizer.addEdge(edge2_1);

    // 第二条
    EdgeSE3PointXYZ* edge2_2 = new EdgeSE3PointXYZ();
    edge2_2->setId(10);
    edge2_2->setParameterId(0, 0);
    edge2_2->setVertex(0, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(1)));
    edge2_2->setVertex(1, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(4)));
    edge2_2->setMeasurement(mesaure_2_2);
    optimizer.addEdge(edge2_2);

    // 第三条
    EdgeSE3PointXYZ* edge2_3 = new EdgeSE3PointXYZ();
    edge2_3->setId(11);
    edge2_3->setParameterId(0, 0);
    edge2_3->setVertex(0, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(1)));
    edge2_3->setVertex(1, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(5)));
    edge2_3->setMeasurement(mesaure_2_3);
    optimizer.addEdge(edge2_3);


    cout << " Frame 3" << endl;
    // 第三帧数据 三条二元边
    // 第一条
    EdgeSE3PointXYZ* edge3_1 = new EdgeSE3PointXYZ();
    edge3_1->setId(12);
    edge3_1->setParameterId(0, 0);
    edge3_1->setVertex(0, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(2)));
    edge3_1->setVertex(1, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(3)));
    edge3_1->setMeasurement(mesaure_3_1);
    optimizer.addEdge(edge3_1);

    // 第二条
    EdgeSE3PointXYZ* edge3_2 = new EdgeSE3PointXYZ();
    edge3_2->setId(13);
    edge3_2->setParameterId(0, 0);
    edge3_2->setVertex(0, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(2)));
    edge3_2->setVertex(1, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(4)));
    edge3_2->setMeasurement(mesaure_3_2);
    optimizer.addEdge(edge3_2);

    // 第三条
    EdgeSE3PointXYZ* edge3_3 = new EdgeSE3PointXYZ();
    edge3_3->setId(14);
    edge3_3->setParameterId(0, 0);
    edge3_3->setVertex(0, dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(2)));
    edge3_3->setVertex(1, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(5)));
    edge3_3->setMeasurement(mesaure_3_3);
    optimizer.addEdge(edge3_3);
    

    optimizer.initializeOptimization();
    cout << " start optimizer " << endl;
    optimizer.optimize(10);

    g2o::VertexPointXYZ* mapp1_res = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(3));
    g2o::VertexPointXYZ* mapp2_res = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(4));
    g2o::VertexPointXYZ* mapp3_res = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(5));
    g2o::VertexSE3* T1_res = static_cast<g2o::VertexSE3*>(optimizer.vertex(0));
    g2o::VertexSE3* T2_res = static_cast<g2o::VertexSE3*>(optimizer.vertex(1));
    g2o::VertexSE3* T3_res = static_cast<g2o::VertexSE3*>(optimizer.vertex(2));

    auto mapp1_res_vec = mapp1_res->estimate();
    auto mapp2_res_vec = mapp2_res->estimate();
    auto mapp3_res_vec = mapp3_res->estimate();
    auto t1_res_vec = T1_res->estimate().translation();
    auto t2_res_vec = T2_res->estimate().translation();
    auto t3_res_vec = T3_res->estimate().translation();

    printf("map point 1: before -> after  |2: before -> after  |3: before -> after\n");
    printf("         x : %05.2f  -> %05.2f  |   %05.2f  -> %05.2f  |   %05.2f  -> %05.2f\n", map_point1[0], mapp1_res_vec[0], map_point2[0], mapp2_res_vec[0], map_point3[0], mapp3_res_vec[0]);
    printf("         y : %05.2f  -> %05.2f  |   %05.2f  -> %05.2f  |   %05.2f  -> %05.2f\n", map_point1[1], mapp1_res_vec[1], map_point2[1], mapp2_res_vec[1], map_point3[1], mapp3_res_vec[1]);
    printf("         z : %05.2f  -> %05.2f  |   %05.2f  -> %05.2f  |   %05.2f  -> %05.2f\n", map_point1[2], mapp1_res_vec[2], map_point2[2], mapp2_res_vec[2], map_point3[2], mapp3_res_vec[2]);

    printf("translate 1: before -> after  |2: before -> after  |3: before -> after\n");
    printf("         x : %05.2f  -> %05.2f  |   %05.2f  -> %05.2f  |   %05.2f  -> %05.2f\n", t1[0], t1_res_vec[0], t2[0], t2_res_vec[0], t3[0], t3_res_vec[0]);
    printf("         y : %05.2f  -> %05.2f  |   %05.2f  -> %05.2f  |   %05.2f  -> %05.2f\n", t1[1], t1_res_vec[1], t2[1], t2_res_vec[1], t3[1], t3_res_vec[1]);
    printf("         z : %05.2f  -> %05.2f  |   %05.2f  -> %05.2f  |   %05.2f  -> %05.2f\n", t1[2], t1_res_vec[2], t2[2], t2_res_vec[2], t3[2], t3_res_vec[2]);

    return 0;
}
