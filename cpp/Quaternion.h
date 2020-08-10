/**
 * quaternion and rotation matrix
 * relative calclation and conversion
 * by Sentinel
 * 2020.8.9
 */

#include <math.h>

// 从四元数联合体中，可以使用data按下标取值 也可以使用变量名按变量取值
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



// 就地转换四元数
void Euler2QtInPlace(float p, float y, Quaternion* qt){
    qt->coeffs.w = cosf(p/2) * cosf(y/2);
    qt->coeffs.x = -sinf(p/2) * sinf(y/2);
    qt->coeffs.y = sinf(p/2) * cosf(y/2);
    qt->coeffs.z = cosf(p/2) * sinf(y/2);
}


// 求四元数模长
float QtNorm(Quaternion* qt){
    float res = 0.0;
    for (int i = 0; i < 4; i++){
        res += powf(qt->data[i], 2);
    }
    return sqrt(res);
}

// 四元数共轭
void QtConj(Quaternion* in, Quaternion* out){
    out->coeffs.w = in->coeffs.w;
    for (int i = 1; i < 4; i++){
        out->data[i] = - in->data[i];
    }
}

// 四元数乘法
void QtMul(Quaternion* q1, Quaternion* q2, Quaternion* out){
    out->coeffs.w = q1->data[0] * q2->data[0] - q1->data[1] * q2->data[1] - q1->data[2] * q2->data[2] - q1->data[3] * q2->data[3];
    out->coeffs.x = q1->data[0] * q2->data[1] + q1->data[1] * q2->data[0] + q1->data[3] * q2->data[2] - q1->data[2] * q2->data[3];
    out->coeffs.y = q1->data[0] * q2->data[2] + q1->data[2] * q2->data[0] + q1->data[1] * q2->data[3] - q1->data[3] * q2->data[1];
    out->coeffs.z = q1->data[0] * q2->data[3] + q1->data[3] * q2->data[0] + q1->data[2] * q2->data[1] - q1->data[1] * q2->data[2];
}

// 四元数归一化
void QtNormalize(Quaternion* in){
    float norm = QtNorm(in);
    for (int i = 0; i < 4; i++){
        in->data[i] /= norm;
    }
}

// 四元数求逆
void QtInv(Quaternion* in, Quaternion* out){
    QtConj(in, out);
    QtNormalize(out);
}

// 旋转三维点(感觉这个实现有点复杂)
void QtRotate(float* x, float* y, float* z, Quaternion* in){
    Quaternion p;
    p.coeffs.w = 0;
    p.coeffs.x = *x;
    p.coeffs.y = *y;
    p.coeffs.z = *z;
    Quaternion tmp, inv;
    QtMul(in, &p, &tmp);
    QtInv(in, &inv);
    QtMul(&tmp, &inv, &p);
    *x = p.coeffs.x;
    *y = p.coeffs.y;
    *z = p.coeffs.z;
}

// 四元数点积
float Qtdot(Quaternion* q1, Quaternion* q2){
    float res = 0.0;
    for (int i = 0; i < 4; i++){
        res += q1->data[i] * q2->data[i];
    }
    return res;
}

// 输入的两个四元数必须是归一化的，否则点积得到的不是cosw 而是|模|^2cosw
// 球面线性插值
void slerp(Quaternion* q1, Quaternion* q2, Quaternion* res, float r){
    float cosw = Qtdot(q1, q2);
    if (cosw < 0.0f){           // 当形成钝角时，插值长度会比对应的锐角插值大，转换到锐角情况下(取反)
        for(int i = 0; i < 4; i++){ // 四元数取反表示的旋转不变
            q2->data[i] *= -1.0f;
        }
    }
    float c1 = 0.0f, c2 = 0.0;
    if (cosw > 0.9995f){        // 两四元数夹角过小，使用线性插值
        c1 = 1.0f - r;
        c2 = r;
    }
    else{
        float w = acosf(cosw);  // 角度
        c1 = sinf((1 - r) * w) / sin(w);
        c2 = sin(r * w) / sin(w);
    }
    for (int i = 0; i < 4; i++){
        res->data[i] = c1 * q1->data[i] + c2 * q2->data[i];
    }
}


// 四元数转旋转矩阵(3*3)
void QtMatrix(Quaternion* q1, float* mat){
    ;// 这个需要电控写
}

// 三个四元数相乘
void QtMul3(Quaternion* q1, Quaternion* q2, Quaternion* q3, Quaternion* res){
    Quaternion tmp;
    QtMul(q1, q2, &tmp);
    QtMul(&tmp, q3, res);
}

// 设置单位四元数
void QtIdentity(Quaternion* in){
    for (int i = 0; i < 3; i++){
        in->data[i] = 0.0;
    }
    in->data[3] = 1.0;
}

// 就地共轭
void QtConjInPlace(Quaternion* in){
    for (int i = 1; i < 4; i++){
        in->data[i] = - in->data[i];
    }
}

// 初始化qt
void QtSet(float w, float x, float y, float z, Quaternion* qt){
    qt->coeffs.w = w;
    qt->coeffs.x = x;
    qt->coeffs.y = y;
    qt->coeffs.z = z;
}



