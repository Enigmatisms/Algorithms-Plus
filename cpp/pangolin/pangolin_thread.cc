// 双线程Pangolin
// 绘制一个随着时间来回运动的方块
#include <pangolin/pangolin.h>
#include <iostream>
#include <memory>
#include <thread>
typedef unsigned char uchar;

uchar RED[3] = {255, 100, 100};

const std::string window = "display";
const std::string win2 = "main";
GLfloat px = -3.0f, py = -3.0f, pz = -3.0f, l = 2.0f;
GLfloat vx = -0.01f, vy = -0.01f, vz = -0.01f;

void pangolinInit(std::string win_name, int w = 1000, int h = 800, bool remove = true){
    pangolin::CreateWindowAndBind(win_name, w, h);
    glEnable(GL_DEPTH_TEST);
    if (remove == true){
        pangolin::GetBoundWindow()->RemoveCurrent();
    }
}

void drawCubeFrame(){
    GLfloat frame_vex[] = {
        px, py, pz,  l + px, py, pz,  px, l + py, pz,
        l + px, l + py, pz,  px, l + py, l + pz,  l + px, l + py, l + pz,  px, py, l + pz,  l + px, py, l + pz,
        px, py, pz,  px, py, l + pz,  l + px, py, pz,  l + px, py, l + pz,
        px, l + py, pz,  px, l + py, l + pz,  l + px, l + py, pz,  l + px, l + py, l + pz,
        px, py, pz,  px, l + py, pz,  l + px, py, pz,  l + px, l + py, pz,
        px, py, l + pz,  px, l + py, l + pz,  l + px, py, l + pz,  l + px, l + py, l + pz
    };
    glLineWidth(4);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, frame_vex);
    for (int d = 0; d < 3; d++){
        glColor3ub(RED[d], RED[(d + 1) % 3], RED[(d + 2) % 3]);
        glDrawArrays(GL_LINES, d * 8, 8);
    }
    if (px < -3.0f || px > 3.0f){
        vx = -vx;
    }
    px += vx;

    if (py < -3.0f || py > 3.0f){
        vy = -vy;
    }
    py += vy;

    if (pz < -3.0f || pz > 3.0f){
        vz = -vz;
    }
    pz += vz;
    glBegin(GL_LINES);
    glColor3ub(255, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(5, 0, 0);
    glColor3ub(0, 255, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 5, 0);
    glColor3ub(0, 0, 255);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 5);
    glEnd();
}

void display(int w, int h, std::string win_name){
    pangolin::BindToContext(win_name);            // 相当于找到当前已经设置好名字的Window，重新bind
    glEnable(GL_DEPTH_TEST);
    pangolin::OpenGlRenderState cam(
        pangolin::ProjectionMatrix(w, h, 450, 450, (int)(w / 2), (int)(h / 2), 0.2, 120),
        pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisZ)
    );
    std::unique_ptr<pangolin::Handler3D> handler(new pangolin::Handler3D(cam));
    pangolin::View& disp = pangolin::CreateDisplay()
            .SetBounds(0.0f, 1.0f, 0.0f, 1.0f, -(float)(w / h))
            .SetHandler(handler.get());
    
    while (!pangolin::ShouldQuit()){
        // 清除屏幕颜色以及深度buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        disp.Activate(cam);
        drawCubeFrame();
        pangolin::FinishFrame();
    }
    pangolin::GetBoundWindow()->RemoveCurrent();
}

int main(){
    int width = 1000, height = 800;
    pangolinInit(window);
    pangolinInit(win2, width, height, false);
    std::thread render(display, width, height, window);
    render.detach();

    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(width, height, 420, 420, (int)(width / 2), (int)(height / 2), 0.2, 100),
        pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisZ)            // 相机空间位置 + 相机看向的空间点 + 不明作用的参数
    );
    std::unique_ptr<pangolin::Handler3D> handler(new pangolin::Handler3D(camera));

    pangolin::View& inter = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -(float)width / (float)height)
            .SetHandler(handler.get());                  // 设置一个交互视图（做什么的？）

    while (!pangolin::ShouldQuit()){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        inter.Activate(camera);
        pangolin::glDrawColouredCube();         // 绘制一个正方体
        glLineWidth(3);     // 线宽
        glBegin(GL_LINES);
        glColor3ub(255, 0, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(2, 0, 0);

        glColor3ub(0, 255, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 2, 0);

        glColor3ub(0, 0, 255);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 2);

        glEnd();
        // printf("Main thread on the run.\n");
        pangolin::FinishFrame();
    }
    
    return 0;
}