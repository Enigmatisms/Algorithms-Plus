#include "../include/Volume.hpp"

const cv::Point dirs[4] = {cv::Point(1, 0), cv::Point(-1, 0), cv::Point(0, 1), cv::Point(0, -1)};

void Edge::drawSelf(cv::Mat& src) const {
    if (this->veritical == true){
        cv::line(src, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(0, 0, 255), 3);
    }
    else {
        cv::line(src, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(255, 0, 0), 3);
    }
}

void Edge::edgeAngles(const cv::Point& pos, float& a1, float& a2) const {
    ;
}

/// 使用的应该是光源在图像上的位置
template<typename Ty>
octOrient Edge::calcOrient(const cv::Point& light, const cv::Point_<Ty>& cen){
    int dx = cen.x - light.x,
        dy = cen.y - light.y,
        adx = std::abs(dx), ady = std::abs(dy);
    // 正方向判定
    if (adx < BLOCK_SIZE / 2 || ady < BLOCK_SIZE / 2){
        if (adx < 0.5){
            return dy > 0 ? DB : DT;
        }
        else {          // 如果adx不小于0.5 说明必然有 ady小于0.5 (两者不会同时小于0.5)
            return dx > 0 ? DR : DL;
        }
    }
    else {      // 非正方向
        if (dx > 0){
            return dy > 0 ? BR : TR;
        }
        else {
            return dy > 0 ? BL : TL;
        }
    }
}

// ================================= Volume2D ====================================

Volume::Volume(int stone_num)
{
    pos_occ = cv::Point(1, 1);
    pos_map = cv::Point(1.5 * BLOCK_SIZE, 1.5 * BLOCK_SIZE);
    occ = new uchar* [VISUAL_Y];                // occ是行优先的
    for (int i = 0; i < VISUAL_Y; i++){
        occ[i] = new uchar [VISUAL_X];
        for (int j = 0; j < VISUAL_X; j++){
            occ[i][j] = 1;
        }
    }
    map.create(cv::Size(VISUAL_X * BLOCK_SIZE, VISUAL_Y * BLOCK_SIZE), CV_8UC3);
    std::chrono::system_clock clk;
    uint64_t t = clk.now().time_since_epoch().count();
    rng = new cv::RNG(t);
    generateMap(stone_num);
}

Volume::~Volume(){
    delete rng;
    for (int i = 0; i < VISUAL_Y; i++){
        delete [] occ[i];
    }
    delete [] occ;
}

void Volume::generateMap(int stone_num){
    // 中空化
    #pragma omp parallel for num_threads(8)
    for (int i = 1; i < VISUAL_Y - 1; i++){
        for (int j = 1; j < VISUAL_X - 1; j++){
            occ[i][j] = 0;
        }
    }
    // 随机生成部分石块
    for (int k = 0; k < stone_num; k++){
        int x = 0, y = 0;
        do {
            x = rng->uniform(1, VISUAL_X - 2);
            y = rng->uniform(1, VISUAL_Y - 2);
        } while(x == 1 && y == 1);
        occ[y][x] = 1;
    }
}

void Volume::getOutSideEdgeByBlock(){
    int cx = VISUAL_X / 2, cy = VISUAL_Y / 2;
    std::vector<std::vector<Edge> > es(4);
    #pragma omp sections
    {
        #pragma omp section
        {
            pushBackEdges(es[0], 0, 0);
        }
        #pragma omp section
        {
            pushBackEdges(es[1], 0, cy);
        }
        #pragma omp section
        {
            pushBackEdges(es[2], cx, 0);
        }
        #pragma omp section
        {
            pushBackEdges(es[3], cx, cy);
        }
    }
    std::cout << "Edges are all pushed.\n";
    for (int i = 0; i < 4; i++){
        const std::vector<Edge>& tmp = es[i];
        for (const Edge& eg: tmp){
            all_edges.emplace_back(eg);
        }  
    }
    for (size_t i = 0; i < all_edges.size(); i++){
        Edge* ptr = &all_edges[i];
        edges.emplace(ptr);
    }
}

void Volume::pushBackEdges(std::vector<Edge>& egs, int startx, int starty) const{
    int hx = VISUAL_X / 2,
        hy = VISUAL_Y / 2,
        endy = starty + hy,
        endx = startx + hx;
    for (int i = starty; i < endy; i++){
        for (int j = startx; j < endx; j++){
            if (occ[i][j] == 0 || (i == pos_occ.y && j == pos_occ.x)) continue;
            if (i != 0 && j != 0 && i != VISUAL_Y - 1 && i != VISUAL_X - 1){  
                octOrient block = Edge::calcOrient<float>(pos_map, BLOCK_SIZE * cv::Point2f(float(j) + 0.5, float(i) + 0.5));
                if (occ[i][j - 1] == 0 && (block & BLOCK_RIGHT)){           // left block在右侧
                    cv::Point p1(j * BLOCK_SIZE, BLOCK_SIZE * i);
                    cv::Point p2(j * BLOCK_SIZE, BLOCK_SIZE * (i + 1));
                    egs.emplace_back(p1, p2, pos_map, true);
                }
                if (occ[i][j + 1] == 0 && (block & BLOCK_LEFT) == 0){       // right
                    cv::Point p1((j + 1) * BLOCK_SIZE, BLOCK_SIZE * i);
                    cv::Point p2((j + 1) * BLOCK_SIZE, BLOCK_SIZE * (i + 1));
                    egs.emplace_back(p1, p2, pos_map, true);
                }
                if (occ[i - 1][j] == 0 && (block & BLOCK_DOWN)){            // up
                    cv::Point p1(j * BLOCK_SIZE, BLOCK_SIZE * i);
                    cv::Point p2((j + 1) * BLOCK_SIZE, BLOCK_SIZE * i);
                    egs.emplace_back(p1, p2, pos_map, false);
                }
                if (occ[i + 1][j] == 0 && (block & BLOCK_UP) == 0){         // down
                    cv::Point p1(j * BLOCK_SIZE, BLOCK_SIZE * (i + 1));
                    cv::Point p2((j + 1) * BLOCK_SIZE, BLOCK_SIZE * (i + 1));
                    egs.emplace_back(p1, p2, pos_map, false);
                }
            }
            else{                   // 边界上的
                if (i == 0){        // 在0行边界上
                    if (occ[i + 1][j] == 0){
                        cv::Point p1(j * BLOCK_SIZE, BLOCK_SIZE);
                        cv::Point p2((j + 1) * BLOCK_SIZE, BLOCK_SIZE);
                        egs.emplace_back(p1, p2, pos_map, false);
                    }
                }
                else if (j == 0) {
                    if (occ[i][j + 1] == 0){    // 在0列边界上的
                        cv::Point p1(BLOCK_SIZE, BLOCK_SIZE * i);
                        cv::Point p2(BLOCK_SIZE, BLOCK_SIZE * (i + 1));
                        egs.emplace_back(p1, p2, pos_map, true);
                    }
                }
                else if (i == VISUAL_Y - 1) {   // 最底部列
                    if (occ[i - 1][j] == 0){
                        cv::Point p1(BLOCK_SIZE * j, BLOCK_SIZE * i);
                        cv::Point p2(BLOCK_SIZE * (j + 1), BLOCK_SIZE * i);
                        egs.emplace_back(p1, p2, pos_map, false);
                    }
                }
                else {      // j == VISUAL_X-1
                    if (occ[i][j - 1] == 0){    // 在0列边界上的
                        cv::Point p1(BLOCK_SIZE * j, BLOCK_SIZE * i);
                        cv::Point p2(BLOCK_SIZE * j, BLOCK_SIZE * (i + 1));
                        egs.emplace_back(p1, p2, pos_map, true);
                    }
                }
            }
        }
    }
}

void Volume::move(DIRECT mv){
    switch (mv)
    {
    case UP:
        if (pos_occ.y > 0) {
            pos_occ.y -= 1;
            if (occ[pos_occ.y][pos_occ.x] == 1){
                pos_occ.y++;
                return;
            }
            pos_map.y -= BLOCK_SIZE;
        }
        break;
    case DOWN:
        if (pos_occ.y < VISUAL_Y) {
            pos_occ.y += 1;
            if (occ[pos_occ.y][pos_occ.x] == 1){
                pos_occ.y--;
                return;
            }
            pos_map.y += BLOCK_SIZE;
        }
        break;
    case LEFT:
        if (pos_occ.x > 0) {
            pos_occ.x -= 1;
            if (occ[pos_occ.y][pos_occ.x] == 1){
                pos_occ.x++;
                return;
            }
            pos_map.x -= BLOCK_SIZE;
        }
        break;
    case RIGHT:
        if (pos_occ.x < VISUAL_X) {
            pos_occ.x += 1;
            if (occ[pos_occ.y][pos_occ.x] == 1){
                pos_occ.x--;
                return;
            }
            pos_map.x += BLOCK_SIZE;
        }
        break;
    }
}

void Volume::debugDisplay(){
    for (int i = 0; i < VISUAL_Y; i++){
        for (int j = 0; j < VISUAL_X; j++){
            if (occ[i][j] == 1){
                cv::rectangle(map, cv::Rect(j * BLOCK_SIZE, i * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE), cv::Scalar(0), -1);
            }
            else {
                cv::rectangle(map, cv::Rect(j * BLOCK_SIZE, i * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE), cv::Scalar(255, 255, 255), -1);
            }
        }
    }
    for (size_t i = 0; i < edges.size(); i++){
        all_edges[i].drawSelf(map);
    }
    cv::rectangle(map, cv::Rect(pos_map.x - BLOCK_SIZE / 2 + 2,  pos_map.y - BLOCK_SIZE / 2 + 2,
        BLOCK_SIZE - 4, BLOCK_SIZE - 4), cv::Scalar(50, 50, 50), -1);
    cv::imshow("disp", map);
}