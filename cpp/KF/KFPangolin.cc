// 双线程Pangolin
// 绘制一个随着时间来回运动的方块
#include <pangolin/pangolin.h>
#include <iostream>
#include <memory>
#include <thread>
#include "include/Predict.hpp"

const std::string window = "display";

void drawCubeFrame(Eigen::Vector3d p, cv::Scalar color, GLfloat l){
    Eigen::Vector3d offset = p / 2;
    glLineWidth(0.005);
    // glBegin(GL_LINES);
    GLfloat frame_vex[72] = {
        0, 0, 0,  l, 0, 0,  0, l, 0,  l, l, 0,  0, l, l,  l, l, l,  0, 0, l,  l, 0, l,
        0, 0, 0,  0, 0, l,  l, 0, 0,  l, 0, l,  0, l, 0,  0, l, l,  l, l, 0,  l, l, l,
        0, 0, 0,  0, l, 0,  l, 0, 0,  l, l, 0,  0, 0, l,  0, l, l,  l, 0, l,  l, l, l
    };
    for (int i = 0; i < 72; i++) {
        frame_vex[i] -= l / 2;
    }
    for (int i = 0; i < 24; i++) {
        frame_vex[3 * i] += p(0);
        frame_vex[3 * i + 1] += p(1);
        frame_vex[3 * i + 2] += p(2);
    }
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, frame_vex);
    for (int d = 0; d < 3; d++){
        glColor3ub(color[0], color[1], color[2]);
        glDrawArrays(GL_LINES, d * 8, 8);
    }
}


int main(){
    int width = 1440, height = 1080;
    pangolin::CreateWindowAndBind(window, width, height);
    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(width, height, 1776.67168581218, 1778.59375346543, (int)(width / 2), (int)(height / 2), 0.2, 100),
        pangolin::ModelViewLookAtRDF(0, 0, 0, 0, 0, 1, 0, -1, 0)            // 相机空间位置 + 相机看向的空间点 + 不明作用的参数
    );
    std::unique_ptr<pangolin::Handler3D> handler(new pangolin::Handler3D(camera));

    pangolin::View& inter = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -(float)width / (float)height)
            .SetHandler(handler.get());                  // 设置一个交互视图（做什么的？）

    Predict pre;
    Msg msg(0, 0, 15);
    while (!pangolin::ShouldQuit()){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        inter.Activate(camera);
        
        Eigen::Vector3d sim = pre.simulateTarget(8000, Tanh), pred;
        cv::Point3d cam_p(sim(0), sim(1), sim(2));
        pre.translatePredict(cam_p, msg, pred);
        drawCubeFrame(sim / 1000, cv::Scalar(0, 255, 0), 0.04);
        drawCubeFrame(pred / 1000, cv::Scalar(0, 255, 255), 0.04);
        pangolin::FinishFrame();
    }
    
    return 0;
}