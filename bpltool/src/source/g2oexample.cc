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
    Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R2(Eigen::AngleAxisd(20.0/180.0*M_PI, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d R3(Eigen::AngleAxisd(40.0/180.0*M_PI, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d t1(0, 0, 0);
    Eigen::Vector3d t2(5, 1, 0);
    Eigen::Vector3d t3(10, 3, 0);

    std::default_random_engine eng;
    std::normal_distribution<double> gauss(0, 1);

    Eigen::Vector3d map_point1 = Eigen::Vector3d(3,5,0);
    Eigen::Vector3d map_point2 = Eigen::Vector3d(7,5,0);
    Eigen::Vector3d map_point3 = Eigen::Vector3d(5,8,0);


    Eigen::Vector3d mesaure_1_1 = R1.inverse() * map_point1 - R1.inverse() * t1;
    Eigen::Vector3d mesaure_1_2 = R1.inverse() * map_point2 - R1.inverse() * t1;
    Eigen::Vector3d mesaure_1_3 = R1.inverse() * map_point3 - R1.inverse() * t1;
    Eigen::Vector3d mesaure_2_1 = R2.inverse() * map_point1 - R2.inverse() * t2;
    Eigen::Vector3d mesaure_2_2 = R2.inverse() * map_point2 - R2.inverse() * t2;
    Eigen::Vector3d mesaure_2_3 = R2.inverse() * map_point3 - R2.inverse() * t2;
    Eigen::Vector3d mesaure_3_1 = R3.inverse() * map_point1 - R3.inverse() * t3;
    Eigen::Vector3d mesaure_3_2 = R3.inverse() * map_point2 - R3.inverse() * t3;
    Eigen::Vector3d mesaure_3_3 = R3.inverse() * map_point3 - R3.inverse() * t3;

    map_point1 += Eigen::Vector3d(gauss(eng), gauss(eng), gauss(eng));
    map_point2 += Eigen::Vector3d(gauss(eng), gauss(eng), gauss(eng));
    map_point3 += Eigen::Vector3d(gauss(eng), gauss(eng), gauss(eng));

    cout << " mappoint  " << endl << map_point1 << endl << map_point2 << endl << map_point3 << endl;

    cout << " measurement 1 " << endl << mesaure_1_1 << endl << mesaure_1_2 << endl << mesaure_1_3 << endl;
    cout << " measurement 2 " << endl << mesaure_2_1 << endl << mesaure_2_2 << endl << mesaure_2_3 << endl;
    cout << " measurement 3 " << endl << mesaure_3_1 << endl << mesaure_3_2 << endl << mesaure_3_3 << endl;


    // pose维度6 landmark维度3


    using BlockSolverTpye = g2o::BlockSolverPL<6, 3>;
    using LinearSolverType = g2o::LinearSolverDense<BlockSolverTpye::PoseMatrixType>;
    // LM 算法
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverTpye>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // typedef g2o::BlockSolverX Block;
    // typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,1> > Block;
    // std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());
    // std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));
    // g2o::SparseOptimizer optimizer;     // 图模型
    // optimizer.setAlgorithm( solver );   // 设置求解器
    // optimizer.setVerbose( true );       // 打开调试输出
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
    vSE3_1->setFixed(true);
    optimizer.addVertex(vSE3_1);

    // 第二帧位姿
    g2o::VertexSE3 * vSE3_2 = new g2o::VertexSE3();
    vSE3_2->setEstimate(g2o::SE3Quat(R2,t2));
    vSE3_2->setId(1);
    vSE3_2->setFixed(true);
    optimizer.addVertex(vSE3_2);

    // 第三帧位姿
    g2o::VertexSE3 * vSE3_3 = new g2o::VertexSE3();
    vSE3_3->setEstimate(g2o::SE3Quat(R3,t3));
    vSE3_3->setId(2);
    vSE3_3->setFixed(true);
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

    auto mapp1_res_vec = mapp1_res->estimate();
    auto mapp2_res_vec = mapp2_res->estimate();
    auto mapp3_res_vec = mapp3_res->estimate();

    cout << " mapp 1 " << endl << mapp1_res_vec << endl;
    cout << " mapp 2 " << endl << mapp2_res_vec << endl;
    cout << " mapp 3 " << endl << mapp3_res_vec << endl;

    return 0;
}
