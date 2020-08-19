#include "stdio.h"
#include "math.h"
#include "stdlib.h"

/// 一维Kalman实现
/// 子弹速度Kalman
/// (先验预测) 状态转移: 本次速度vk = 0.5 xk-1 + 0.5 * uk + ek
/// 其中, uk 为控制量,一个非线性关系式,与摩擦轮速度有关 加权平均
/// (后验修正) 观测: 直接观测: 裁判系统返回速度

typedef struct{
    float trsf;                 // 状态转移(A)
    float ctrl;                 // 控制(B)
    float obs;                  // 观测(H)
    float ctrl_var;             // 控制方差(Q)大
    float obs_var;              // 观测方差(R)小
    union{
        float post;             // 状态后验
        float pre;              // 状态先验
    }state;
    union{
        float post;             // 方差后验
        float pre;              // 方差先验
    }var;
}Kalman;    


float KalmanCtrlFunc(float x){
    /// 需要补充 摩擦轮速度与射速公式
    return x; 
}

void KalmanInit(Kalman * kf, float init_vel){
    kf->trsf = 0.5;
    kf->ctrl = 0.5;
    kf->obs = 1;
    kf->ctrl_var = 0.25;
    kf->obs_var = 0.04;
    kf->state.post = init_vel;
    kf->var.post = 0.5;
    kf->state.pre = 1.0;
    kf->var.pre = 1.0;
}

float KalmanPredict(Kalman * kf, float u){
    kf->state.pre = kf->trsf * kf->state.post + kf->ctrl * KalmanCtrlFunc(u);
    kf->var.pre = kf->trsf * kf->trsf * kf->var.post + kf->ctrl_var;
    return kf->state.pre;
}

float KalmanCorrect(Kalman * kf, float obs){
    float gain = kf->var.pre * kf->obs / (kf->obs * kf->obs * kf->var.pre + kf->obs_var);
    kf->state.post = kf->state.pre + gain * (obs - kf->obs * kf->state.pre);
    kf->var.post = (1.0f - gain * kf->obs) * kf->var.pre;
    return kf->state.post; 
}

/// 中值滤波配上Kalman滤波,过滤高斯 + 椒盐噪声

/// 双向链表 中值滤波
typedef struct link{
    float data;
    struct link* next;
    struct link* prev;
}link;

typedef struct Deque{
    link* head;     // 头
    link* tail;     // 尾
    int max_size;
    int size;
    link* med;      // 中部
}deque;

float FindMed(deque* que){
    link* now = que->head;
    for (int i = 0; i < (int)((que->size - 1) / 2); i++){
        now = now->next;
    }
    que->med = now;
    return now->data;
}

void SearchLeft(link* new_link, deque* que, float val){
    link* start = que->med;
    if (que->size < que->max_size){
        start = que->head;
    }
    while(start->data > val){
        if (start->prev != NULL && start->data > val)
            start = start->prev;
        else break;
    }
    if (start->prev == NULL){
        start->prev = new_link;
        new_link->next = start;
        que->head = new_link;   
    }
    else{
        new_link->prev = start;         
        start->next->prev = new_link;
        new_link->next = start->next;
        start->next = new_link;
    }
    que->size ++;
    if (que->size > que->max_size){
        que->tail = que->tail->prev;
        free(que->tail->next);
        que->tail->next = NULL;
        que->size --;
    }
    FindMed(que);
}

void SearchRight(link* new_link, deque* que, float val){
    link* start = que->med;
    if (que->size < que->max_size){
        start = que->head;
    }
    while(start->data < val){
        if (start->next != NULL)
            start = start->next;
        else break;
    }
    if (start->next == NULL && start->data < val){
        start->next = new_link;
        new_link->prev = start;
        que->tail = new_link;   
    }
    else{
        new_link->next = start;         
        start->prev->next = new_link;
        new_link->prev = start->prev;
        start->prev = new_link;
    }
    que->size ++;
    if (que->size > que->max_size){
        que->head = que->head->next;
        free(que->head->prev);
        que->head->prev = NULL;
        que->size --;
    }
    FindMed(que);
}

void LinkAppend(deque* que, float val){
    link* new_link = (link*)malloc(sizeof(link));
    new_link->data = val;
    link* start = que->med;
    if (que->size < que->max_size){
        start = que->head;
    }
    if (start->data <= val){
        SearchRight(new_link, que, val);
    }
    else {
        SearchLeft(new_link, que, val);
    }
}

void InitDeque(deque* que, int max_size){
    que->head = (link*)malloc(sizeof(link));
    que->tail = que->head;
    que->head->next = NULL;
    que->head->prev = NULL;
    que->head->data = 15.0;
    que->med = que->head;
    que->max_size = max_size;
    que->size = 1;
}

void DestroyDeque(deque* que){
    link* now = que->head;
    while (now != que->tail){
        now = now->next;
        free(now->prev);
        now->prev = NULL;
    }
    free(now);
    que->head = NULL;
    que->med = NULL;
    que->tail= NULL;
    que->size = 0;
    que->max_size = 0;
}

void DisplayDeque(deque* que){
    link* now = que->head;
    printf("Deque([");
    while (now->next != NULL && now != que->tail){
        if(now == que->med){
            printf("[%f], ", now->data);
        }
        else{
            printf("%f, ", now->data);
        }
        now = now->next;
    }
    printf("%f])\n\n", que->tail->data);
}