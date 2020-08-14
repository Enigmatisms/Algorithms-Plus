#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#define DEG2RAD 0.017453

union Quaternion
{
    float data[4];
    struct{
        float w;
        float x;
        float y;
        float z;
    }coeffs;
};

// 四元数转旋转矩阵(3*3)
void Qt2Matrix(Quaternion *q1, float *Mat)
{
    float x2 = pow(q1->coeffs.x, 2);
    float y2 = pow(q1->coeffs.y, 2);
    float z2 = pow(q1->coeffs.z, 2);
    float xy = q1->coeffs.x * q1->coeffs.y;
    float xz = q1->coeffs.x * q1->coeffs.z;
    float xw = q1->coeffs.x * q1->coeffs.w;
    float yz = q1->coeffs.y * q1->coeffs.z;
    float yw = q1->coeffs.y * q1->coeffs.w;
    float zw = q1->coeffs.z * q1->coeffs.w;
    Mat[0] = 1 - 2 * y2 - 2 * z2;
    Mat[1] = 2 * xy - 2 * zw;
    Mat[2] = 2 * xz + 2 * yw;
    Mat[3] = 2 * xy + 2 * zw;
    Mat[4] = 1 - 2 * x2 - 2 * z2;
    Mat[5] = 2 * yz - 2 * xw;
    Mat[6] = 2 * xz - 2 * yw;
    Mat[7] = 2 * yz + 2 * xw;
    Mat[8] = 1 - 2 * x2 - 2 * y2;
}

// 就地转换四元数
void Euler2QtInPlace(float y, float p, Quaternion *qt)
{
    qt->coeffs.w = cosf(y / 2) * cosf(p / 2);
    qt->coeffs.x = -sinf(y / 2) * sinf(p / 2);
    qt->coeffs.y = cosf(y / 2) * sinf(p / 2);
    qt->coeffs.z = sinf(y / 2) * cosf(p / 2);
}

void euler2Quat(float r, float p, float y, Quaternion *qt, int is_rad){
    if (is_rad == 0){
        r *= 0.0174533;       // pi / 180 = 0.0174533
        p *= 0.0174533;
        y *= 0.0174533;     
    }
    qt->coeffs.w = cosf(r / 2) * cosf(p / 2) * cosf(y / 2) + sinf(r / 2) * sinf(p / 2) * sinf(y / 2);  // w
    qt->coeffs.x = sinf(r / 2) * cosf(p / 2) * cosf(y / 2) - cosf(r / 2) * sinf(p / 2) * sinf(y / 2);  // x
    qt->coeffs.y = cosf(r / 2) * sinf(p / 2) * cosf(y / 2) + sinf(r / 2) * cosf(p / 2) * sinf(y / 2);  // y
    qt->coeffs.z = cosf(r / 2) * cosf(p / 2) * sinf(y / 2) - sinf(r / 2) * sinf(p / 2) * cosf(y / 2);  // z
}

// 求四元数模长
float QtNorm(Quaternion *qt)
{
    float res = 0.0;
    for (int i = 0; i < 4; i++)
    {
        res += powf(qt->data[i], 2);
    }
    return sqrt(res);
}

// 四元数共轭
void QtConj(Quaternion *in, Quaternion *out)
{
    out->coeffs.w = in->coeffs.w;
    for (int i = 1; i < 4; i++)
    {
        out->data[i] = -in->data[i];
    }
}

// 四元数乘法 (checked)
void QtMul(Quaternion *q1, Quaternion *q2, Quaternion *out)
{
    out->coeffs.w = q1->data[0] * q2->data[0] - q1->data[1] * q2->data[1] - q1->data[2] * q2->data[2] - q1->data[3] * q2->data[3];
    out->coeffs.x = q1->data[0] * q2->data[1] + q1->data[1] * q2->data[0] + q1->data[2] * q2->data[3] - q1->data[3] * q2->data[2];
    out->coeffs.y = q1->data[0] * q2->data[2] - q1->data[1] * q2->data[3] + q1->data[2] * q2->data[0] + q1->data[3] * q2->data[1];
    out->coeffs.z = q1->data[0] * q2->data[3] + q1->data[1] * q2->data[2] - q1->data[2] * q2->data[1] + q1->data[3] * q2->data[0];
}

// 四元数归一化
void QtNormalize(Quaternion *in)
{
    float norm = QtNorm(in);
    for (int i = 0; i < 4; i++)
    {
        in->data[i] /= norm;
    }
}

// 初始化qt
void QtSet(float w, float x, float y, float z, Quaternion *qt)
{
    qt->coeffs.w = w;
    qt->coeffs.x = x;
    qt->coeffs.y = y;
    qt->coeffs.z = z;
}

// 四元数点积
float Qtdot(Quaternion *q1, Quaternion *q2)
{
    float res = 0.0;
    for (int i = 0; i < 4; i++)
    {
        res += q1->data[i] * q2->data[i];
    }
    return res;
}

// 输入的两个四元数必须是归一化的，否则点积得到的不是cosw 而是|模|^2cosw
// 球面线性插值
void slerp(Quaternion *q1, Quaternion *q2, Quaternion *res, float r)
{
    float cosw = Qtdot(q1, q2);
    if (cosw < 0.0f)
    { // 当形成钝角时，插值长度会比对应的锐角插值大，转换到锐角情况下(取反)
        for (int i = 0; i < 4; i++)
        { // 四元数取反表示的旋转不变
            q2->data[i] *= -1.0f;
        }
    }
    float c1 = 0.0f, c2 = 0.0;
    if (cosw > 0.9995f)
    { // 两四元数夹角过小，使用线性插值
        c1 = 1.0f - r;
        c2 = r;
    }
    else
    {
        float w = acosf(cosw); // 角度
        c1 = sinf((1.0f - r) * w) / sinf(w);
        c2 = sinf(r * w) / sinf(w);
    }
    for (int i = 0; i < 4; i++)
    {
        res->data[i] = c1 * q1->data[i] + c2 * q2->data[i];
    }
}
// 三个四元数相乘 (checked)
void QtMul3(Quaternion *q1, Quaternion *q2, Quaternion *q3, Quaternion *res)
{
    Quaternion tmp;
    QtMul(q1, q2, &tmp);
    QtMul(&tmp, q3, res);
}

// 设置单位四元数
void QtIdentity(Quaternion *in)
{
    in->data[0] = 1.0;
    for (int i = 1; i < 4; i++)
    {
        in->data[i] = 0.0;
    }
}

// 就地共轭 (checked)
void QtConjInPlace(Quaternion *in)
{
    for (int i = 1; i < 4; i++)
    {
        in->data[i] = -in->data[i];
    }
}

// 四元数转向量(mat(4, 1)) 使用Hamilton表述(w, x, y, z)
void Qt2Vector(Quaternion* qt, float* Mat){
    for (int i = 0; i < 4; i++){
        Mat[i] = qt->data[i];
    }
}

// 角速度转四元数
/// @param in 当前的角位置四元数
/// @param out 角速度四元数
/// @param wx wy wz RPY 三轴旋转角速度 x 对应 roll/y 对应 pitch/z 对应 yaw
void angularVel2Quat(float wx, float wy, float wz, Quaternion* out){
    float t = sqrtf(powf(wx, 2) + powf(wy, 2) + powf(wz, 2));
    if (t < 0.0001f){               // 模值过小保护
        QtIdentity(out);
    }
    else{
        wx /= t;
        wy /= t;
        wz /= t;
        out->data[0] = cosf(t / 2);
        out->data[1] = sinf(t / 2) * wx;
        out->data[2] = sinf(t / 2) * wy;
        out->data[3] = sinf(t / 2) * wz;
    }
}

void Euler2QtFull(float r, float p, float y, Quaternion *qt)
{
    qt->coeffs.w = cosf(r / 2) * cosf(p / 2) * cosf(y / 2) + sinf(r / 2) * sinf(p / 2) * sinf(y / 2);
    qt->coeffs.x = sinf(r / 2) * cosf(p / 2) * cosf(y / 2) - cosf(r / 2) * sinf(p / 2) * sinf(y / 2);
    qt->coeffs.y = cosf(r / 2) * sinf(p / 2) * cosf(y / 2) + sinf(r / 2) * cosf(p / 2) * sinf(y / 2);
    qt->coeffs.z = cosf(r / 2) * cosf(p / 2) * sinf(y / 2) - sinf(r / 2) * sinf(p / 2) * cosf(y / 2);
}

void eigen2Quat(const Eigen::Quaterniond& qt, Quaternion* out){
    Eigen::Vector4d coe = qt.coeffs();
    for (int i = 1; i < 4; i++){
        out->data[i] = coe(i - 1);
    }
    out->data[0] = coe(3);
}

#define DELTA_TIME 0.01136            // 1
Eigen::Quaterniond angularVel2Quat(const Eigen::Vector3d& avel)
{
    float t = avel.norm() * DELTA_TIME;
    if (t < 0.0001){
        return Eigen::Quaterniond::Identity();
    }
    Eigen::Vector3d direction = avel.normalized();
    return Eigen::Quaterniond(
        cos(t / 2), 
        sin(t / 2) * direction(0),
        sin(t / 2) * direction(1),
        sin(t / 2) * direction(2)
    );
}

/// 欧拉角速率转角速度 最后一个参数是roll轴的欧拉角速率
/// @param x y z 输入 输出 输入RPY 当前角度 输出 空间 w(x, y, z)方向角速度分量
void ratio2Velocity(float rp, float ry, Eigen::Vector3d &w, float rr = 0.0){
    // 全部预先转为弧度
    // 电控提供的rr, rp, ry 全是弧度
    Eigen::Vector3d ratio(rr, rp, ry);
    Eigen::Matrix3d transform;
    // 位置角单位是角度 需要转换
    w *= 0.0174533;
    transform << cosf(w(1)) * cosf(w(2)), - sinf(w(2)), 0,
        cosf(w(1)) * sinf(w(2)), cosf(w(2)), 0,
        -sinf(w(1)), 0, 1;
    w = transform * ratio;
    // 输出的角速度单位是弧度/每DELTA_TIME
}

void printQuat(const Eigen::Quaterniond& qt){
    Eigen::Vector4d coe = qt.coeffs();
    std::cout << "Eigen result:";
    for (int i = 0; i < 3; i++){
        std::cout << coe(i) << ", ";
    }
    std::cout << coe(3) << std::endl;
}