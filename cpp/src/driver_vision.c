#include "driver_vision.h"
/***********
			该程序从视觉处拷来，具体源代码参照视觉组
			*********************************/
///
#define OFFSET_X (0)
#define OFFSET_Y (145.5)
#define OFFSET_Z (61.95)
#if 1 //如果使用的是大子弹
#define VISION_K (0.00831)
#else //如果使用的是小子弹
#define VISION_K (0.01949)
#endif

#define VISION_V (15) //速度偏置
#define VISION_YAW (0.6)
#define VISION_PITCH (0)

#define VISION_PITCH_KP (0.0)
#define VISION_PITCH_KI (0.0)
#define VISION_PITCH_KD (0.0)
#define VISION_PITCH_AP (0.25)
#define VISION_PITCH_BP (0)
#define VISION_PITCH_CP (0.0)

#define VISION_YAW_KP (0.0)
#define VISION_YAW_KI (0.0)
#define VISION_YAW_KD (2.0)
#define VISION_YAW_AP (0.07)
#define VISION_YAW_BP (0)
#define VISION_YAW_CP (1)

GimbalCtrl VisionGimbalCtrl;
MatStruct world_motion;
MatStruct car_motion;
mat car_dt, car_ddt, w_dt, w_ddt, w_rt, temp, mult, arm_t;

void tempInit(void)
{
    mat_init(&car_dt, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&car_ddt, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&w_dt, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&w_ddt, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&temp, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&mult, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&arm_t, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&w_rt, 3, 3, (float *)malloc(9 * sizeof(float)));

    mat_init(&car_motion.r, 3, 3, (float *)malloc(9 * sizeof(float)));
    mat_init(&car_motion.t, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&car_motion.dt, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&car_motion.ddt, 3, 1, (float *)malloc(3 * sizeof(float)));

    mat_init(&world_motion.r, 3, 3, (float *)malloc(9 * sizeof(float)));
    mat_init(&world_motion.t, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&world_motion.dt, 3, 1, (float *)malloc(3 * sizeof(float)));
    mat_init(&world_motion.ddt, 3, 1, (float *)malloc(3 * sizeof(float)));
}

/**
 * @brief Init the Transformation matrix from camera to ballistic //TODO: write in ros tf
 * @param x Translate x, 单位mm
 * @param y Translate y, 单位mm
 * @param z Translate z, 单位mm
 * @param pitch 摄像头与枪管的pitch角度差, 单位角度（deg）
 * @param yaw 摄像头与枪管的yaw角度差, 单位角度（deg）
 * @param init_v 初速度，单位m/s
 * @param init_k 空气摩擦因数，默认为0.1
 */
PID VisionPitchIncreasement = {0}, VisionYawIncreasement = {0};
extern Motion car, wm;
void VisionInit(void)
{
    VisionGimbalCtrl.offset_x = OFFSET_X / 1000;
    VisionGimbalCtrl.offset_y = OFFSET_Y / 1000;
    VisionGimbalCtrl.offset_z = OFFSET_Z / 1000;
    VisionGimbalCtrl.init_k = VISION_K;
    VisionGimbalCtrl.init_v = VISION_V;
    VisionGimbalCtrl.offset_yaw = VISION_YAW;
    VisionGimbalCtrl.offset_pitch = VISION_PITCH;

    VisionPitchIncreasement.Kp = VISION_PITCH_KP;
    VisionPitchIncreasement.Ki = VISION_PITCH_KI;
    VisionPitchIncreasement.Kd = VISION_PITCH_KD;
    VisionPitchIncreasement.Ap = VISION_PITCH_AP;
    VisionPitchIncreasement.Bp = VISION_PITCH_BP;
    VisionPitchIncreasement.Cp = VISION_PITCH_CP;
    VisionPitchIncreasement.OutMax = 3;
    VisionPitchIncreasement.OutMin = -3;
    VisionPitchIncreasement.calc = &PidCalc;
    VisionPitchIncreasement.clear = &PidClear;
    VisionPitchIncreasement.clear(&VisionPitchIncreasement);

    VisionYawIncreasement.Kp = VISION_YAW_KP;
    VisionYawIncreasement.Ki = VISION_YAW_KI;
    VisionYawIncreasement.Kd = VISION_YAW_KD;
    VisionYawIncreasement.Ap = VISION_YAW_AP;
    VisionYawIncreasement.Bp = VISION_YAW_BP;
    VisionYawIncreasement.Cp = VISION_YAW_CP;
    VisionYawIncreasement.OutMax = 3;
    VisionYawIncreasement.OutMin = -3;
    VisionYawIncreasement.calc = &PidCalc;
    VisionYawIncreasement.clear = &PidClear;
    VisionYawIncreasement.clear(&VisionPitchIncreasement);
    tempInit();
}

/**
 * @fn BulletModel:定义了SIGNGLE_DIRECTION是，定义是单向空气阻力模型，否则为双向空气阻力模型
 * @brief 完全的弹道模型（考虑x与y方向上的空气阻力）
 * @param x x方向上的位移（敌方装甲板到枪口的距离，tVec的[0]位置）
 * @param v 子弹出射初速度
 * @param angle 迭代当前pitch角度
 * @return 本次迭代所计算的pitch实际值（位置）
 */
float BulletModel(float x, float v, float angle)
{
    return x * GRAVITY / (VisionGimbalCtrl.init_k * v * cosf(angle)) + tanf(angle) * x +
           1 / (VisionGimbalCtrl.init_k * VisionGimbalCtrl.init_k) * GRAVITY * logf(1 - VisionGimbalCtrl.init_k * x / (v * cosf(angle)));
}

float calcTime(float x, float v, float angle)
{
    return (-1 / (VisionGimbalCtrl.init_k) * logf(1 - VisionGimbalCtrl.init_k * x / (v * cosf(angle))));
}

// 四元数转旋转矩阵(3*3)
void QtMatrix(Quaternion *q1, mat *Mat)
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
    mat->pData[0] = 1 - 2 * y2 - 2 * z2;
    mat->pData[1] = 2 * xy - 2 * zw;
    mat->pData[2] = 2 * xz + 2 * yw;
    mat->pData[3] = 2 * xy + 2 * zw;
    mat->pData[4] = 1 - 2 * x2 - 2 * z2;
    mat->pData[5] = 2 * yz - 2 * xw;
    mat->pData[6] = 2 * xz - 2 * yw;
    mat->pData[7] = 2 * yz + 2 * xw;
    mat->pData[8] = 1 - 2 * x2 - 2 * y2;
}

// 就地转换四元数
void Euler2QtInPlace(float y, float p, Quaternion *qt)
{
    qt->coeffs.w = cosf(y / 2) * cosf(p / 2);
    qt->coeffs.x = -sinf(y / 2) * sinf(p / 2);
    qt->coeffs.y = cosf(y / 2) * sinf(p / 2);
    qt->coeffs.z = sinf(y / 2) * cosf(p / 2);
}

void w2QtInPlace(float wp, float wq, Quaternion *qt);

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

// 四元数乘法
void QtMul(Quaternion *q1, Quaternion *q2, Quaternion *out)
{
    out->coeffs.w = q1->data[0] * q2->data[0] - q1->data[1] * q2->data[1] - q1->data[2] * q2->data[2] - q1->data[3] * q2->data[3];
    out->coeffs.x = q1->data[0] * q2->data[1] + q1->data[1] * q2->data[0] + q1->data[3] * q2->data[2] - q1->data[2] * q2->data[3];
    out->coeffs.y = q1->data[0] * q2->data[2] + q1->data[2] * q2->data[0] + q1->data[1] * q2->data[3] - q1->data[3] * q2->data[1];
    out->coeffs.z = q1->data[0] * q2->data[3] + q1->data[3] * q2->data[0] + q1->data[2] * q2->data[1] - q1->data[1] * q2->data[2];
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
// 三个四元数相乘
void QtMul3(Quaternion *q1, Quaternion *q2, Quaternion *q3, Quaternion *res)
{
    Quaternion tmp;
    QtMul(q1, q2, &tmp);
    QtMul(&tmp, q3, res);
}

// 设置单位四元数
void QtIdentity(Quaternion *in)
{
    for (int i = 0; i < 3; i++)
    {
        in->data[i] = 0.0;
    }
    in->data[3] = 1.0;
}

// 就地共轭
void QtConjInPlace(Quaternion *in)
{
    for (int i = 1; i < 4; i++)
    {
        in->data[i] = -in->data[i];
    }
}

// 预测后取得装甲板相对相机的最终位置结果
void getTarget(void);

#define DELTA_TIME 0.01136 /// DELTA_TIME 需要修改 之后再确定
// 这个需要电控改
// 预测车在delta_t时间内的运动
/**
 * @param delta_t 是弹道解算时间(输入)
 * @param pre_t 需要预测的敌方车辆平移(输入 / 输出)
 * @param pre_r 需要预测的敌方车辆旋转(输入 / 输出)
 * @param wm 由云台以及底盘解算所得的自身运动状态(相机云台旋转和平移)(输入)
 * @note delta_time 标量 除了与它相关的乘法 * 以外，其他乘法 * 均为矩阵乘法
 */

void predict(float delta_t, Motion *const car, Motion *const wm, float *armor)
{
    if (delta_t == 0.0f)
    { // 初始化时delta_t 此时无需预测
        QtMatrix(&car->r, &car_motion.r);
        mat_mult(&car_motion.r, &arm_t, &mult);
        mat_add(&mult, &car_motion.t, &mult);
        armor[0] = mult.pData[0] - VisionGimbalCtrl.offset_x;
        armor[1] = mult.pData[1] - VisionGimbalCtrl.offset_y;
        armor[2] = mult.pData[2] - VisionGimbalCtrl.offset_z;
    }
    else
    {
        QtMatrix(&car->r, &car_motion.r);
        mat_mult(&world_motion.r, &car_motion.t, &mult);
        mat_add(&mult, &world_motion.t, &mult);

        mat_scale(&car_motion.dt, delta_t, &car_dt);
        mat_add(&mult, &car_dt, &mult);
        mat_scale(&car_motion.ddt, powf(delta_t, 2) / 2, &car_ddt);
        mat_add(&mult, &car_ddt, &mult);
        mat_trans(&world_motion.r, &w_rt);

        mat_sub(&mult, &world_motion.t, &temp);
        mat_scale(&world_motion.dt, delta_t, &w_dt);
        mat_sub(&temp, &w_dt, &temp);
        mat_scale(&world_motion.ddt, powf(delta_t, 2) / 2, &w_ddt);
        mat_sub(&temp, &w_ddt, &temp);

        Quaternion back2cam, ind, res, rot, pre_r;
        QtIdentity(&ind);
        slerp(&ind, &wm->dr, &res, delta_t / DELTA_TIME); // 相机角速度slerp
        QtMul(&wm->r, &res, &back2cam);                   // 相机位置 + 角速度 * t
        QtConj(&back2cam);                                // 原地转置

        QtMatrix(&back2cam, &w_rt);
        mat_mult(&w_rt, &temp, &mult); // 车相对位移预测结果

        slerp(&ind, &car->dr, &res, delta_t / DELTA_TIME); // 由车的角速度转化为角度
        QtMul3(&back2cam, &wm->r, &car->r, &rot);          // pre_r的结果
        QtMul(&rot, &res, &pre_r);

        QtMatrix(&pre_r, &w_rt);
        mat_mult(&w_rt, &arm_t, &temp);
        mat_add(&mult, &temp, &mult);

        // 装甲板的相机坐标系位置 -> 转换到枪管坐标系位置（减offset）
        armor[0] = mult.pData[0] - VisionGimbalCtrl.offset_x;
        armor[1] = mult.pData[1] - VisionGimbalCtrl.offset_y;
        armor[2] = mult.pData[2] - VisionGimbalCtrl.offset_z;
    }
}

void getDistPos(float* const t, float angle, float *dist, float *pos)
{
    *dist = sqrtf(powf(t[2] * cosf(angle) + t[1] * sinf(angle), 2) + powf(t[0], 2));
    *pos = t[2] * sinf(angle) - t[1] * cosf(angle);
}

/** 
 * @brief 弹道模型预测解算
 * @param car 敌方车辆运动状态
 * @param wm 我方车辆运动状态 
 * @param now_pit now_yaw vel 云台当前角度 子弹速度
 * @param pit, yaw (输入输出) 云台运动增量以及子弹时间
 */
void solve(Motion *const car, Motion *const wm, float now_pit, float vel, float *pit, float *yaw)
{
    float y_temp, y_act, dy, delta_t = 0.0, old_delta = 0.0, pre_t[3];
    float angle = now_pit / RAD2DEG, start_angle = angle, dist, y_pos;
    predict(0.0f, car, wm, pre_t);
    getDistPos(pre_t, start_angle, &dist, &y_pos); // 获取当前距离以及目标位置计算值
    y_temp = dist * tanf(angle);                   // y_temp 为枪管指向的y位置, y_pos 为目标所在的y位置
    for (int i = 0; i < 25; i++)
    {
        angle = atan2f(y_temp, dist);
        y_act = BulletModel(dist, vel, angle);
        delta_t = calcTime(dist, vel, angle);
        predict(delta_t, car, wm, pre_t);              // 预测对方位置，更新距离(dist) y_pos
        getDistPos(pre_t, start_angle, &dist, &y_pos); // 对方位置每改变一次，需要重新计算dist y_pos, 云台角度不变
        dy = y_pos - y_act;
        y_temp += dy;

        if (fabsf(delta_t - old_delta) < 0.01 && fabsf(dy) < 0.001)
        {
            break;
        }
        old_delta = delta_t;
    }
    *pit = angle * RAD2DEG - start_angle * RAD2DEG + VisionGimbalCtrl.offset_pitch; // 增量
    *yaw = -atan2(pre_t[0] - VisionGimbalCtrl.offset_x, pre_t[2] - VisionGimbalCtrl.offset_z) * RAD2DEG + VisionGimbalCtrl.offset_yaw;
}

/// 弹道模型

///===================== 以下两个信息封装函数 由电控写 ===============================///
// 云台封装自身运动数据
// 通过云台信息获得对应的四元数以及ARM库下的3*1矩阵
// @note 参数的形式可以更改，你怎么方便怎么来
void getSelfMotion(VisionReceiveDataStruct vision, Motion *motion)
{
    /// TODO: 读取一次时间 (可能需要进行0.375的转换)-----0.0改为读取的系统时间
    float delta_time = 0.0;     
    world_motion.t.pData[0] = vision.position_integral_x;
    world_motion.t.pData[1] = vision.position_integral_y;
    world_motion.t.pData[2] = vision.position_integral_z;
    world_motion.dt.pData[0] = vision.speed_x;
    world_motion.dt.pData[1] = vision.speed_y;
    world_motion.dt.pData[2] = vision.speed_z;
    world_motion.ddt.pData[0] = vision.accelerated_x;
    world_motion.ddt.pData[1] = vision.accelerated_y;
    world_motion.ddt.pData[2] = vision.accelerated_z;
    Euler2QtInPlace(vision.spinquaternion1_yaw / RAD2DEG, vision.spinquaternion2_pitch / RAD2DEG, &motion->r);
    QtMatrix(&motion->r, &world_motion.r);

    /// TODO: 以下是dr的计算
    /// TODO: Motion 定义需要改变，需要增加last_r的记录

    // 可能无法直接利用云台的角速度信息
    Quaternion ind, rconj, temp;
    QtIdentity(&ind);

    /// TODO: motion->last_r暂时不存在
    QtConj(&motion->last_r, &rconj);
    QtMul(&rconj, &motion->r, &temp);
    slerp(&ind, &temp, &rconj, DELTA_TIME / delta_time);             // 插值
    slerp(&motion->dr, &rconj, &motion->dr, 0.3);
}

// 云台封装视觉消息数据
// 云台接收到视觉消息 将敌方车辆的运动要素 t, dt, ddt, r, dr, armt 转换为四元数与矩阵
// 输入参数 1: 消息解码联合体 可以做成常量的指针如 Quaternion * const qt
// 输入参数 2(也是输出): Motion* 提前定义好一个 Motion, Motion 里存除了armt外的全部信息
// 输入参数 3(也是输出): armt(装甲板位置)的mat(3, 1)给后续计算使用
void visionInfoPackup(VisionReceiveDataStruct VisionReceiveData, Motion *car)
{
    car_motion.t.pData[0] = VisionReceiveData.position_integral_x;
    car_motion.t.pData[1] = VisionReceiveData.position_integral_y;
    car_motion.t.pData[2] = VisionReceiveData.position_integral_z;
    car_motion.dt.pData[0] = VisionReceiveData.speed_x;
    car_motion.dt.pData[1] = VisionReceiveData.speed_y;
    car_motion.dt.pData[2] = VisionReceiveData.speed_z;
    car_motion.ddt.pData[0] = VisionReceiveData.accelerated_x;
    car_motion.ddt.pData[1] = VisionReceiveData.accelerated_y;
    car_motion.ddt.pData[2] = VisionReceiveData.accelerated_z;
    QtSet(VisionReceiveData.spinquaternion1_yaw,
          VisionReceiveData.spinquaternion2_pitch,
          VisionReceiveData.spinquaternion3_ser,
          VisionReceiveData.spinquaternion4,
          &car->r);
    QtSet(VisionReceiveData.anglequaternion1_yawspeed,
          VisionReceiveData.anglespinquaternion2_pitchspeed,
          VisionReceiveData.anglespinquaternion3,
          VisionReceiveData.anglespinquaternion4,
          &car->dr);
}
