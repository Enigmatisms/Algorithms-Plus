// PANGOLIN学习
#include <pangolin/pangolin.h>
#include <iostream>
#include <memory>
typedef unsigned char uchar;

uchar RED[3] = {255, 0, 0};

// 绘制正方形框架
void drawCubeFrame(GLfloat linew, GLfloat l){
    glLineWidth(linew);
    // glBegin(GL_LINES);
    GLfloat frame_vex[] = {
        0, 0, 0,  l, 0, 0,  0, l, 0,  l, l, 0,  0, l, l,  l, l, l,  0, 0, l,  l, 0, l,
        0, 0, 0,  0, 0, l,  l, 0, 0,  l, 0, l,  0, l, 0,  0, l, l,  l, l, 0,  l, l, l,
        0, 0, 0,  0, l, 0,  l, 0, 0,  l, l, 0,  0, 0, l,  0, l, l,  l, 0, l,  l, l, l
    };
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, frame_vex);
    for (int d = 0; d < 3; d++){
        glColor3ub(255, 0, 0);
        glDrawArrays(GL_LINES, d * 8, 8);
    }
}


int main(int argc, char** argv){
    int width = 640, height = 480;
    pangolin::CreateWindowAndBind("display", width, height);     // 创建OpenGL窗口
    glEnable(GL_DEPTH_TEST);                                // 意义不明，创建深度测试

    // 懂了，设置的就是朝上的那个轴是哪个轴（我们还是习惯z朝上）
    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(width, height, 420, 420, 320, 320, 0.2, 100),     // 图像宽高 + 内参（uv轴焦距 + uv轴原点偏移） + 最近最远视距
        pangolin::ModelViewLookAt(2, 2, 2, 0, 0, 0, pangolin::AxisZ)            // 相机空间位置 + 相机看向的空间点 + 不明作用的参数
    );
    // 光轴朝向的点位置（从本身到朝向点就是光轴所在直线），up轴的定义等一下调参
    std::unique_ptr<pangolin::Handler3D> handler(new pangolin::Handler3D(camera));
    // pangolin::Handler3D* handler = new pangolin::Handler3D(camera);        // 交互句柄

    // 输出视图的样式（相当于上面就是在设置3D位置，此处设置2D投影）
    pangolin::View& inter = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -(float)width / (float)height)
        .SetHandler(handler.get());                  // 设置一个交互视图（做什么的？）
    // SetBounds最后的参数表示的是画面比例（宽高比，同时还限制了画面占比）

    while (!pangolin::ShouldQuit()){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        inter.Activate(camera);

        drawCubeFrame(4, 2);

        pangolin::FinishFrame();
    }
    // delete handler;
    return 0;
}