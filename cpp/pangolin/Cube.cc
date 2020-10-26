#include "include/Cube.hpp"

uchar RED[3] = {255, 100, 100};

int main(){
    int width = 1000, height = 800;
    pangolin::CreateWindowAndBind("cube", width, height);
    glEnable(GL_DEPTH_TEST);
    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(width, height, 420, 420, (int)(width / 2), (int)(height / 2), 0.2, 100),     
        pangolin::ModelViewLookAt(4, 4, 4, 0, 0, 0, pangolin::AxisZ)           
    );
    std::unique_ptr<pangolin::Handler3D> handler(new pangolin::Handler3D(camera));
    Cube cb(1, 1, 1, 2);

    pangolin::View& disp = pangolin::CreateDisplay()
            .SetBounds(0.0f, 1.0f, 0.0f, 1.0f, -(float)(width / height))
            .SetHandler(handler.get());

    pangolin::CreatePanel("ctrl")
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UINT_WIDTH));

    pangolin::Var<std::function<void()>> reset("ctrl.reset", std::bind(&Cube::reset, &cb));
    pangolin::RegisterKeyPressCallback('w', std::bind(&Cube::move, &cb, AXIS_X, FORWARD));
    pangolin::RegisterKeyPressCallback('a', std::bind(&Cube::move, &cb, AXIS_Y, FORWARD));
    pangolin::RegisterKeyPressCallback('s', std::bind(&Cube::move, &cb, AXIS_X, BACKWARD));
    pangolin::RegisterKeyPressCallback('d', std::bind(&Cube::move, &cb, AXIS_Y, BACKWARD));
    pangolin::RegisterKeyPressCallback(' ', std::bind(&Cube::move, &cb, AXIS_Z, FORWARD));
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL, std::bind(&Cube::move, &cb, AXIS_Z, BACKWARD));
    pangolin::RegisterKeyPressCallback('q', std::bind(&Cube::rotate, &cb, AXIS_Z, FORWARD));
    pangolin::RegisterKeyPressCallback('r', std::bind(&Cube::rotate, &cb, AXIS_Z, BACKWARD));

    double *pts = cb.getCurrentFrame();    
    while (!pangolin::ShouldQuit()){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        disp.Activate(camera);
        // pts = cb.getCurrentFrame();             
        cb.getCurrentFrame();
        glLineWidth(4);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_DOUBLE, 0, pts);
        for (int d = 0; d < 3; d++){
            glColor3ub(RED[d], RED[(d + 1) % 3], RED[(d + 2) % 3]);
            glDrawArrays(GL_LINES, d * 8, 8);
        }
        pangolin::FinishFrame();
    }
    return 0;
}