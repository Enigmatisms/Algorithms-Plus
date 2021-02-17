#include "../include/Volume.hpp"

const int steps = BLOCK_SIZE / 2;
const int luminous_range = BLOCK_SIZE * BLOCK_SIZE * 225;

const std::map<octOrient, cv::Scalar> colors = {
    {TL, cv::Scalar(50, 50, 255)}, {DT, cv::Scalar(50, 255, 50)}, {TR, cv::Scalar(255, 50, 50)},
    {DL, cv::Scalar(50, 255, 255)}, {DR, cv::Scalar(255, 50, 255)}, {BL, cv::Scalar(255, 255, 50)},
    {DB, cv::Scalar(255, 100, 0)}, {BR, cv::Scalar(0, 100, 255)}
};

void Edge::drawSelf(cv::Mat& src) const {
    cv::line(src, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), colors.at(orient), 3);
}

void Edge::edgeVector(const cv::Point& pos, cv::Point2f& v1, cv::Point2f& v2) const {
    cv::Point2f tmp(pos.x, pos.y);
    v1 = p1 - tmp, v2 = p2 - tmp;
    double n1 = std::sqrt(v1.ddot(v1)), n2 = std::sqrt(v2.ddot(v2));
    v1 /= n1;
    v2 /= n2;
}

void Edge::rebaseVertex(const cv::Point& light, uchar** const occ, const cv::Point2f& v1, const cv::Point2f& v2, bool rebase_p1){
    if (vertical){
        float dx = centroid.x - light.x, target_pos = light.y + dx * v1.y / v1.x;
        int occ_posy = int(target_pos / BLOCK_SIZE), occ_posx = int(centroid.x / BLOCK_SIZE);
        if (occ[occ_posy][occ_posx - 1] == occ[occ_posy][occ_posx]) {        // 端点更新到墙内部或空气内部
            target_pos = light.y + dx * v2.y / v2.x;
        }
        if (rebase_p1)
            p1.y = target_pos;
        else
            p2.y = target_pos;
        centroid.y = (p1.y + p2.y) / 2;
    }
    else {
        float dy = centroid.y - light.y, target_pos = light.x + dy * v1.x / v1.y;
        int occ_posx = int(target_pos / BLOCK_SIZE), occ_posy = int(centroid.y / BLOCK_SIZE);
        if (occ[occ_posy - 1][occ_posx] == occ[occ_posy][occ_posx]) {
            target_pos = light.x + dy * v2.x / v2.y;
        }
        if (rebase_p1)
            p1.x = target_pos;
        else
            p2.x = target_pos;
        centroid.x = (p1.x + p2.x) / 2;
    }
}

/// 使用的应该是光源在图像上的位置
template<typename Ty>
octOrient Edge::calcOrient(const cv::Point& light, const cv::Point_<Ty>& cen, bool vert){
    int dx = cen.x - light.x,
        dy = cen.y - light.y,
        adx = std::abs(dx), ady = std::abs(dy);
    // 正方向判定
    int half_size = BLOCK_SIZE / 2;
    if (adx < half_size || ady < half_size){
        if (adx < half_size && vert == false){           // 水平的
            return dy > 0 ? DB : DT;
        }
        else if (ady < half_size || vert == true){          // 如果adx不小于0.5 说明必然有 ady小于0.5 (两者不会同时小于0.5)
            return dx > 0 ? DR : DL;
        }
    }
    if (dx > 0){
        return dy > 0 ? BR : TR;
    }
    else {
        return dy > 0 ? BL : TL;
    }
}

void Edge::updateSelf(const cv::Point& light, uchar** const occ, const cv::Point2f& v1, const cv::Point2f& v2) {
    cv::Point2f self_v1, self_v2;
    edgeVector(light, self_v1, self_v2);        // v1, v2, self_v1 v2都是归一化后的
    float a11 = self_v1.dot(v1),      // 各个相对角度(cos是减函数，也就是dot结果越大越好)
          a12 = self_v1.dot(v2), 
          a21 = self_v2.dot(v1), 
          a22 = self_v2.dot(v2),
          aa  = v1.dot(v2);
    bool flag1 = (a11 >= aa && a12 >= aa),        // 是否在内部
        flag2 = (a21 >= aa && a22 >= aa);
    if (flag1 && flag2) {   // 全部遮挡
        valid = false;
    }
    else if (flag1)         // 向量1在内部 （也就是更新端点1位置）
    {   
        if (std::abs(a11 - aa) < NUMERICAL_ZERO || std::abs(a12 - aa) < NUMERICAL_ZERO) return;
        if (a11 > a12){     // 离向量1近
            rebaseVertex(light, occ, v1, v2, true);
        }
        else {
            rebaseVertex(light, occ, v2, v1, true);
        }
    }
    else if (flag2)         // 向量2在内部 （更新端点2位置）
    {
        if (std::abs(a21 - aa) < NUMERICAL_ZERO || std::abs(a22 - aa) < NUMERICAL_ZERO) return;
        if (a21 > a22){     // 离向量1近
            rebaseVertex(light, occ, v1, v2, false);
        }
        else {
            rebaseVertex(light, occ, v2, v1, false);
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
    #ifdef RECORD
	    outputVideo.open(outPath, DIVX, 60.0, sWH);		
    #endif
}

Volume::~Volume(){
    #ifdef RECORD
        outputVideo.release();
    #endif
    delete rng;
    for (int i = 0; i < VISUAL_Y; i++){
        delete [] occ[i];
    }
    delete [] occ;
}

void Volume::generateMap(int stone_num){
    // 中空化
    #pragma omp parallel for num_threads(4)
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
    #pragma omp parallel sections
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
    for (int i = 0; i < 4; i++){
        const std::vector<Edge>& tmp = es[i];
        for (const Edge& eg: tmp){
            all_edges.emplace_back(eg);
        }  
    }
    for (size_t i = 0; i < all_edges.size(); i++){
        edges.emplace(&all_edges[i]);
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

void Volume::moveStep(DIRECT mv){
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

void Volume::moveSmooth(DIRECT mv, std::string win_name){
    switch (mv)
    {
        case UP: 
            if (occ[pos_occ.y - 1][pos_occ.x] == 0) pos_occ.y --;
            else return; break;
        case DOWN:
            if (occ[pos_occ.y + 1][pos_occ.x] == 0) pos_occ.y ++;
            else return; break;
        case LEFT:
            if (occ[pos_occ.y][pos_occ.x - 1] == 0) pos_occ.x --;
            else return; break;
        case RIGHT:
            if (occ[pos_occ.y][pos_occ.x + 1] == 0) pos_occ.x ++;
            else return; break;
        default:
            break;
    }
    for (int k = 0; k < steps; k++){
        switch (mv)
        {
        case UP:
            if (pos_occ.y > 0) pos_map.y -= 2;
            break;
        case DOWN:
            if (pos_occ.y < VISUAL_Y) pos_map.y += 2;
            break;
        case LEFT:
            if (pos_occ.x > 0) pos_map.x -= 2;
            break;
        case RIGHT:
            if (pos_occ.x < VISUAL_X) pos_map.x += 2;
            break;
        default:
            break;
        }
        reset();
        getOutSideEdgeByBlock();
        edgeTranverse();
        debugDisplay(win_name, true);
        #ifdef RECORD
            outputVideo.write(map);
        #endif
    }
}


void Volume::debugDisplay(std::string win_name, bool render_flag){
    if (render_flag == true) {
        cv::rectangle(map, cv::Rect(0, 0, VISUAL_X * BLOCK_SIZE, VISUAL_Y * BLOCK_SIZE), cv::Scalar(255, 255, 255), -1);
        edgeTranverse();
        render();
    }
    for (int i = 0; i < VISUAL_Y; i++){
        for (int j = 0; j < VISUAL_X; j++){
            if (occ[i][j] == 1){
                cv::rectangle(map, cv::Rect(j * BLOCK_SIZE, i * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE), cv::Scalar(0, 0, 0), -1);
            }
        }
    }
    map.forEach<cv::Vec3b>(
        [&](cv::Vec3b& pix, const int* pos){
            if (pix[0] == 0xff){        // 为白色
                double dist = std::pow(pos[1] - pos_map.x, 2) + std::pow(pos[0] - pos_map.y, 2);
                uchar res = std::max(20.0, 255.0 - 235.0 / luminous_range * dist);
                pix[0] = pix[1] = pix[2] = res;
            }
        }
    );
    for (size_t i = 0; i < all_edges.size(); i++){
        if (all_edges[i].isValid() == true){
            all_edges[i].drawSelf(map);
        }
    }
    cv::rectangle(map, cv::Rect(pos_map.x - BLOCK_SIZE / 2 + 4,  pos_map.y - BLOCK_SIZE / 2 + 4,
        BLOCK_SIZE - 8, BLOCK_SIZE - 8), cv::Scalar(0, 200, 200), -1);
    cv::rectangle(map, cv::Rect(pos_map.x - BLOCK_SIZE / 2 + 2,  pos_map.y - BLOCK_SIZE / 2 + 2,
        BLOCK_SIZE - 4, BLOCK_SIZE - 4), cv::Scalar(0, 0, 200), 2);
    cv::imshow(win_name, map);
    cv::waitKey(1);
}

void Volume::edgeTranverse(){
    while (edges.empty() == false){
        Edge* ptr = edges.top();
        edges.pop();
        cv::Point2f vec1, vec2;
        ptr->edgeVector(pos_map, vec1, vec2);           // 
        const octOrient orient = ptr->getOrient();
        bool due_flag = false;                          // 是否处于正方向
        if (orient & 0x01 || orient & 0x04){            // 边处于正方向，需要搜索三个方向
            due_flag = true;
        }
        for (size_t i = 0; i < all_edges.size(); i++){
            if (all_edges[i].isValid() == true){
                octOrient cur = all_edges[i].getOrient();
                if (due_flag == true){
                     if (orientTagDueJudge(orient, cur) == false) continue;     // 无需搜索的方向
                }
                else{
                    if (cur != orient) continue;
                }
                if (&all_edges[i] == ptr) continue;
                all_edges[i].updateSelf(pos_map, occ, vec1, vec2);    // 可能修改valid选项或者其centroid / 端点
            }
        }
        ptr->setValid(false);
        std::vector<cv::Point> contour;
        getRenderFrame(ptr, vec1, vec2, contour);
        contours.emplace_back(contour);
    }
}

void Volume::getRenderFrame(const Edge* const eg, const cv::Point2f& v1, const cv::Point2f& v2, std::vector<cv::Point>& contour) const
{
    cv::Point s1, s2;               // boundary solutions
    float ubx = VISUAL_X * BLOCK_SIZE - 1, uby = VISUAL_Y * BLOCK_SIZE - 1;
    getBoundarySolution(eg->p1, v1, s1);
    getBoundarySolution(eg->p2, v2, s2);
    contour.emplace_back(s1);
    contour.emplace_back(eg->p1);
    contour.emplace_back(eg->p2);
    contour.emplace_back(s2);
    cv::Point dd = s1 - s2, sump = (s1 + s2) / 2;
    dd.x = std::abs(dd.x);
    dd.y = std::abs(dd.y);
    if (dd.x == 0 || dd.y == 0)             // 不加入点 (个人很怕一些不稳定的表现)
        return;
    else if (dd.x >= ubx){                  // 加入两个点
        if (sump.y < eg->centroid.y) {
            if (s1.x == 0){
                contour.emplace_back(ubx, 0);
                contour.emplace_back(0, 0);
            }
            else{
                contour.emplace_back(0, 0);
                contour.emplace_back(ubx, 0);
            }
        }
        else{
            if (s1.x == 0){
                contour.emplace_back(ubx, uby);
                contour.emplace_back(0, uby);
            }
            else{
                contour.emplace_back(0, uby);
                contour.emplace_back(ubx, uby);
            }
        }
    }
    else if (dd.y >= uby){                  // 同上
        if (sump.x < eg->centroid.x) {
            if (s1.y == 0){
                contour.emplace_back(0, uby);
                contour.emplace_back(0, 0);
            }
            else{
                contour.emplace_back(0, 0);
                contour.emplace_back(0, uby);
            }
        }
        else{
            if (s1.y == 0){
                contour.emplace_back(ubx, uby);
                contour.emplace_back(ubx, 0);
            }
            else{
                contour.emplace_back(ubx, 0);
                contour.emplace_back(ubx, uby);
            }
        }
    }
    else{                                   // 加入一个角点
        if ((s1.x == 0 && s2.y == 0) || (s1.y == 0 && s2.x == 0)) contour.emplace_back(0, 0);
        else if ((s1.x == 0 && s2.y == uby) || (s1.y == uby && s2.x == 0)) contour.emplace_back(0, uby);
        else if ((s1.x == ubx && s2.y == 0) || (s1.y == 0 && s2.x == ubx)) contour.emplace_back(ubx, 0);
        else contour.emplace_back(ubx, uby);
    }
}

void Volume::getBoundarySolution(const cv::Point2f& eg, const cv::Point2f& v, cv::Point& s) {
    float ubx = VISUAL_X * BLOCK_SIZE - 1, uby = VISUAL_Y * BLOCK_SIZE - 1;
    if (std::abs(v.x) < NUMERICAL_ZERO) s = v.y > 0 ? cv::Point(eg.x, uby) : cv::Point(eg.x, 0);
    else if (std::abs(v.y) < NUMERICAL_ZERO) s = v.x > 0 ? cv::Point(ubx, eg.y) : cv::Point(0, eg.y);
    else{
        if (v.x > 0 && v.y > 0){          // I
            float xp = eg.x + v.x / v.y * (uby - eg.y), yp = eg.y + v.y / v.x * (ubx - eg.x);
            s = xp < ubx ? cv::Point(xp, uby) : cv::Point(ubx, yp);
        }
        else if (v.x < 0 && v.y > 0){     // II
            float xp = eg.x + v.x / v.y * (uby - eg.y), yp = eg.y - v.y / v.x * eg.x;
            s = xp < 0 ? cv::Point(0, yp) : cv::Point(xp, uby);
        }
        else if (v.x < 0 && v.y < 0){          // III
            float xp = eg.x - v.x / v.y * eg.y, yp = eg.y - v.y / v.x * eg.x;
            s = xp < 0 ? cv::Point(0, yp) : cv::Point(xp, 0);
        }
        else{   // IV
            float xp = eg.x - v.x / v.y * eg.y, yp = eg.y + v.y / v.x * (ubx - eg.x);
            s = xp > ubx ? cv::Point(ubx, yp) : cv::Point(xp, 0);
        }
    }
}

void Volume::render(){
    #pragma omp parallel for num_threads(4)
    for (size_t i = 0; i < contours.size(); i++){
        cv::fillConvexPoly(map, contours[i], cv::Scalar(20, 20, 20));
    }
    contours.clear();
}