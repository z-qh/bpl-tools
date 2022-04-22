#include "ros/ros.h"

#include "iostream"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"

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

int main(int argc, char** argv){
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
