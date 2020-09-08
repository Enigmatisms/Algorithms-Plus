#ifndef __EKF__
#define __EKF__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/core.hpp>

template<typename Ty, int size>
class EKF{
using Matrix = Eigen::Matrix<Ty, size, size>;
using Vector = Eigen::Matrix<Ty, size, 1>;
public:
    /**
     * @brief 初始化
     * @param ctrl_setup 设置初始控制误差对角阵值
     * @param obs_setup 设置初始观测误差对角阵值
     */
    EKF(double ctrl_setup = 1.0, double obs_setup = 1.0);
    ~EKF(){;}
public:
    void predict(double dt);
    void update(double x, double y, double dt);
    /// 以上还处于滤波阶段
    void predictMove(double dt, double &x, double &y);              // 预测delta_t 时间后的位置
private:
    /**
     * @brief 获取观测雅可比 
     * @param measure 观测量 一个向量
     * @return 在观测 measure 处的观测方程雅可比
     */
    Matrix getObsMatrix(double x, double y, double dt) const;      // 系统观测展开

    /**
     * @brief 获取控制模型在先验处的展开
     * @param ctrl 控制量输入
     * @return 控制模型雅可比
     */
    Matrix getCtrlMatrix(double dt) const;     // 系统控制展开

    /**
     * @brief 观测模型
     * @param measure 输入一个观测 输出观测模型的输出
     * @return 观测模型输出
     */
    Vector measureModel(double xm, double y, double dt) const;     

    /**
     * @brief 控制模型
     * @param ctrl 输入一个控制 输出控制模型的输出
     * @return 控制模型输出
     */
    Vector ctrlModel(double dt) const;
public:
    Vector statePre;            // 先验状态
    Vector statePost;           // 后验状态
    Matrix stateCovPre;         // 状态先验协方差
    Matrix stateCovPost;        // 状态后验协方差
private:
    Matrix ctrlErrorCov;        // 控制误差协方差
    Matrix obsErrorCov;         // 观测误差协方差
    Matrix temp;                // 临时
    Matrix K;                   // Kalman增益
};

template<typename Ty, int size>
EKF<Ty, size>::EKF(double ctrl_setup, double obs_setup){
    ctrlErrorCov    = ctrl_setup * Matrix::Identity();
    obsErrorCov     = obs_setup * Matrix::Identity();
    stateCovPre     = Matrix::Identity() + ctrlErrorCov;
    K.setIdentity();
    statePost << 100, 0, 5.8;
    statePre.setIdentity();
    stateCovPost.setIdentity();
}

template<typename Ty, int size>
void EKF<Ty, size>::predict(double dt){
    statePre = ctrlModel(dt);
    Matrix jacobi = getCtrlMatrix(dt);
    std::cout << "Jacobi for ctrl:" << std::endl;
    std::cout << jacobi << std::endl;
    stateCovPre = jacobi * stateCovPost * jacobi.transpose() + ctrlErrorCov;
    // std::cout << "state pre:" << std::endl;
    std::cout << "Prediction result:" << statePre(0) << ", " << statePre(1) << ", " << statePre(2) << std::endl;
}

template<typename Ty, int size>
void EKF<Ty, size>::update(double x, double y, double dt){
    Vector model = measureModel(x, y, dt);
    Matrix jacobi = getObsMatrix(x, y, dt);
    std::cout << "Measurement model: " << model.transpose() << std::endl;
    // std::cout << "Jacobi:" << jacobi << std::endl;

    Eigen::JacobiSVD<Matrix> svd(jacobi * stateCovPre * jacobi.transpose() + obsErrorCov, 
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    // SVD求逆矩阵
    std::cout << stateCovPre << std::endl;
    temp = svd.matrixV() * svd.singularValues().asDiagonal().toDenseMatrix().inverse() * svd.matrixU().transpose();
    K = stateCovPre * jacobi.transpose() * temp;                        // 计算增益
    std::cout << "K (gain):" << std::endl << K << std::endl;
    statePost = statePre + K * (model - statePre);                       // 计算后验状态
    stateCovPost = (Matrix::Identity() - K * jacobi) * stateCovPre;     // 计算后验协方差
    std::cout << "Update result x, y, w: " << stateCovPost(0) << ", " << stateCovPost(1) << ", " << stateCovPost(2) << std::endl;
}

template<typename Ty, int size>
void EKF<Ty, size>::predictMove(double dt, double &x, double &y){
    // 在每次的correct做完后，我们获得了较为准确的 w 则可以预测
    Vector result = ctrlModel(dt);
    x = result(0);
    y = result(1);
}

///======================模拟测试========================///
/// 由于Eigen没有办法自动求导 我们需要自己定义状态转移 观测 以及他们相应的雅可比计算方法///
/// 此处构建一个含有噪声的 绕原点旋转的角速度存在噪声的圆形
template<typename Ty, int size>
Eigen::Matrix<Ty, size, size> EKF<Ty, size>::getObsMatrix(double x, double y, double dt) const{
    // 观测雅可比
    double r0 = sqrt(pow(statePre(0), 2) + pow(statePre(1), 2)),
            r2 = pow(x, 2) + pow(y, 2), pro = x * statePre(0) + y * statePre(1);
    double dwx = - (statePre(0) / r0 / sqrt(r2) - x * pro / r0 / pow(r2, 1.5)) / sqrt(1 - pow(pro, 2) / pow(r0, 2) / r2);
    double dwy = - (statePre(1) / r0 / sqrt(r2) - y * pro / r0 / pow(r2, 1.5)) / sqrt(1 - pow(pro, 2) / pow(r0, 2) / r2);
    Matrix result;
    result << 1, 0, 0, 0, 1, 0, 0.5 * dwx / dt, 0.5 * dwy / dt, 0.5;
    return result;
}

template<typename Ty, int size>
Eigen::Matrix<Ty, size, size> EKF<Ty, size>::getCtrlMatrix(double dt) const{
    Matrix result;
    double x = statePost(0), y = statePost(1), w = statePost(2);
    std::cout << "w * dt result: " << w * dt << std::endl;
    result << cos(w * dt), - sin(w * dt), - dt * x * sin(w * dt) - dt * y * cos(w * dt),
        sin(w * dt), cos(w * dt), - dt * y * sin(w * dt) + dt * x * cos(w * dt),
        0, 0, 1;
    return result;
}

template<typename Ty, int size>
Eigen::Matrix<Ty, size, 1> EKF<Ty, size>::ctrlModel(double dt) const{
    Vector result;
    double x = statePost(0), y = statePost(1), w = statePost(2);
    result << x * cos(w * dt) - y * sin(w * dt), y * cos(w * dt) + x * sin(w * dt), w;
    return result;
}

template<typename Ty, int size>
Eigen::Matrix<Ty, size, 1> EKF<Ty, size>::measureModel(double x, double y, double dt) const{
    Vector result;
    double x0 = statePost(0), y0 = statePost(1), r0 = sqrt(pow(x0, 2) + pow(y0, 2)), r = sqrt(pow(x, 2) + pow(y, 2));
    result << x, y, acos((x * x0 + y * y0) / r / r0) / dt;      // 使用内积法求角速度
    return result;
}

#endif  // __EKF__