#include "../include/SDF.hpp"

void SDF::mergeMesh(const Edges& eg1, const Edges& eg2, double _gsize, Edges& dst) {
    grid_size = _gsize;                                     // step 1. 设置grid大小
    Eigen::Vector2d tl;
    Eigen::Vector2i grids;
    meshBoundingBox(eg1, eg2, tl, grids);                   // step 2. 求出两个mesh的bounding box 有一定的裕量
    Eigen::MatrixXd sdf1, sdf2, alpha1, alpha2;
    Eigen::MatrixXi idx1, idx2;
    #pragma omp parallel sections                           // step 3. 三通道输出，输出sdf，计算的权重以及到mesh哪一段距离最小
    {
        #pragma omp section
        {
            singleMeshSDF(eg1, tl, grids, sdf1, alpha1, idx1);    
        }
        #pragma
        {
            singleMeshSDF(eg2, tl, grids, sdf2, alpha2, idx2);
        }
    }
    Eigen::MatrixXd sdf = Eigen::MatrixXd(grids(0), grids(1));
    Eigen::MatrixXd weights = Eigen::MatrixXd(grids(0), grids(1));
    Eigen::MatrixXd alpha = Eigen::MatrixXd(grids(0), grids(1));
    sdf.setZero();
    #pragma omp parallel for num_threads(4)
    for (int i = 0; i < grids(0); i++) {                    // step 4. 按照alpha + weight进行加权融合 sdf / 权重选择最大值
        for (int j = 0; j < grids(1); j++) {
            int index1 = idx1(i, j), index2 = idx2(i, j);
            double v1 = sdf1(i, j), v2 = sdf2(i, j);
            double wt1 = eg1[index1].weight, wt2 = eg2[index2].weight;
            double u1 = alpha1(i, j) * wt1, u2 = alpha2(i, j) * wt2;
            alpha(i, j) = (u1 + u2) / (wt1 + wt2);
            sdf(i, j) = (v1 * 2 * u1 + v2 * u2) / (2 * u1 + u2);
            weights(i, j) = std::max(eg1[index1].weight, eg2[index2].weight);
        }
    }
    cv::Mat image, img1, img2, img3, img4, img5, img6, img7;
    visualizeValues(sdf, tl, image);
    visualizeValues(sdf2, tl, img7);
    visualizeEdges(eg1, color_g, image);
    visualizeEdges(eg2, color_k, image);
    cv::imshow("disp", image);
    visualizeAlpha(alpha1, tl, img1);
    visualizeAlpha(alpha2, tl, img2);
    visualizeAlpha(alpha, tl, img5);
    visualizeBelongs(idx2, tl, img4);
    visualizeCombinedAlpha(alpha1, alpha2, tl, img3);
    visualizeEdges(eg1, color_g, img4);
    cv::imshow("combined", img3);
    cv::imshow("sdf1", img1);
    cv::imshow("sdf2", img2);
    cv::imshow("belong", img4);
    cv::imshow("alpha", img5);
    cv::imshow("sdf", img7);
    cv::waitKey(0);
    
    EdgeMap emap;   
    marchingSquare(sdf, weights, emap);                     // step 5. marching cubes 2D 算法
    visualizeValues(sdf, tl, img6);
    visualizeMarchingSquare(emap, tl, color_k, img6);
    searchSerialize(tl, alpha,emap, dst);                         // step 6. DFS搜索哈希表 得到顺序化的edges
    // visualizeEdges(dst, color_b, img6);
    cv::imshow("viz", img6);
    cv::waitKey(0);
}

void SDF::meshBoundingBox(const Edges& m1, const Edges& m2, Eigen::Vector2d& tl, Eigen::Vector2i& rnc) const {
    double min_x = 1e10, max_x = 0.0, min_y = 1e10, max_y = 0.0;
    std::array<const Edges* const, 2> refs = {&m1, &m2}; 
    for (const Edges* const eg: refs) {
        for (size_t i = 0; i < eg->size() - 1; i++) {
            double x = eg->at(i).sp.x(), y = eg->at(i).sp.y();
            if (x < min_x)      min_x = x;
            else if (x > max_x) max_x = x;
            if (y < min_y)      min_y = y;
            else if (y > max_y) max_y = y;
        }
        double x = eg->back().ep.x(), y = eg->back().ep.y();
        if (x < min_x)      min_x = x;
        else if (x > max_x) max_x = x;
        if (y < min_y)      min_y = y;
        else if (y > max_y) max_y = y;
    }
    tl(0) = min_x - 2 * grid_size;
    tl(1) = min_y - 2 * grid_size;
    rnc(0) = std::ceil((max_y - min_y + 32 * grid_size) / grid_size);
    rnc(1) = std::ceil((max_x - min_x + 32 * grid_size) / grid_size);
}

void SDF::singleMeshSDF(
    const Edges& edges, const Eigen::Vector2d& tl, const Eigen::Vector2i& grids,
    Eigen::MatrixXd& sdf, Eigen::MatrixXd& alpha, Eigen::MatrixXi& idx
) const {
    sdf = Eigen::MatrixXd(grids(0), grids(1));
    alpha = Eigen::MatrixXd(grids(0), grids(1));
    idx = Eigen::MatrixXi(grids(0), grids(1));
    idx.setZero();
    alpha.setZero();
    singleMeshPieceSDF<START>(edges.front(), tl, sdf, alpha);
    size_t last_index = edges.size() - 1, i = 1;
    for (i = 1; i < last_index; i++) {     // no head and tail
        Eigen::MatrixXd tmp(grids(0), grids(1));
        singleMeshPieceSDF<NORMAL>(edges[i], tl, tmp, alpha);
        bool acute = edges[i].isAngleAcute(edges[i - 1]);
        bool normal_same = edges[i].sameDirectionAsNormal(edges[i - 1]);
        #pragma omp parallel num_threads(4)
        for (int j = 0; j < grids(0); j++) {
            for (int k = 0; k < grids(1); k++) {
                double abs_sd = std::abs(tmp(j, k));
                double abs_origin = std::abs(sdf(j, k));
                if (abs_origin > abs_sd) {
                    idx(j, k) = i;
                    sdf(j, k) = tmp(j, k);
                }
                else if (abs_origin == abs_sd && acute == false) {
                    if (normal_same) {
                        if (tmp(j, k) < 0 && sdf(j, k) > 0) {
                            sdf(j, k) = tmp(j, k);
                            idx(j, k) = i;
                        }
                    }
                    else {
                        if (tmp(j, k) > 0 && sdf(j, k) < 0) {
                            sdf(j, k) = tmp(j, k);
                            idx(j, k) = i;
                        }
                    }
                } 
                double distance = std::abs(tmp(j, k));
                double new_alpha = 1.0 / (distance + 1.0);
                if (new_alpha > alpha(j, k))
                    alpha(j, k) = new_alpha;
            }
        }
    }
    Eigen::MatrixXd tmp(grids(0), grids(1));
    singleMeshPieceSDF<END>(edges.back(), tl, tmp, alpha);
    bool acute = true, normal_same = true;
    if (last_index == 0)
        i = 0;
    else {
        acute = edges[last_index].isAngleAcute(edges[last_index - 1]);
        normal_same = edges[last_index].sameDirectionAsNormal(edges[last_index - 1]);
    }
    #pragma omp parallel num_threads(4)
    for (int j = 0; j < grids(0); j++) {
        for (int k = 0; k < grids(1); k++) {
            double abs_sd = std::abs(tmp(j, k));
            double abs_origin = std::abs(sdf(j, k));
            if (abs_origin > abs_sd) {
                idx(j, k) = i;
                sdf(j, k) = tmp(j, k);
            }
            else if (abs_origin == abs_sd && acute == false) {
                if (normal_same) {
                    if (tmp(j, k) < 0 && sdf(j, k) > 0) {
                        sdf(j, k) = tmp(j, k);
                        idx(j, k) = i;
                    }
                }
                else {
                    if (tmp(j, k) > 0 && sdf(j, k) < 0) {
                        sdf(j, k) = tmp(j, k);
                        idx(j, k) = i;
                    }
                }
            } 
        }
    }
    alphaCalculation(edges.front(), tl, alpha, true);
    alphaCalculation(edges.back(), tl, alpha, false);
}

template<SDF::calcType type>
void SDF::singleMeshPieceSDF(const Edge& eg, const Eigen::Vector2d& tl, Eigen::MatrixXd& sdf, Eigen::MatrixXd& alpha) const {
    const Eigen::Vector2d s2c = tl - eg.sp, e2c = tl - eg.ep;
    Eigen::Vector2d direct = eg.ep - eg.sp;
    const Eigen::Vector2d normal = Eigen::Vector2d(-direct.y(), direct.x()).normalized();
    Eigen::Vector2d truc = direct / 4.0;
    if (type == END) truc *= 3.0;
    direct.normalize();
    for (int i = 0; i < sdf.rows(); i++) {
        for (int j = 0; j < sdf.cols(); j++) {
            Eigen::Vector2d s2p = s2c + Eigen::Vector2d(j, i);
            Eigen::Vector2d e2p = e2c + Eigen::Vector2d(j, i);
            double sdot = s2p.dot(direct), edot = e2p.dot(direct);
            bool positive = (s2p.dot(normal) > 0.0);
            bool over_s = (sdot <= 0.0), over_e = (edot >= 0.0), within = ((!over_s) & (!over_e));
            if (type == NORMAL) {
                if (within) {
                    double dist = std::sqrt(s2p.squaredNorm() - std::pow(sdot, 2));
                    sdf(i, j) = positive ? dist : -dist;
                }
                else if (over_e) {
                    sdf(i, j) = positive ? e2p.norm() : - e2p.norm();
                }
                else if (over_s) {
                    sdf(i, j) = positive ? s2p.norm() : - s2p.norm();
                }
            }
            else if (type == START) {
                if (over_e)
                    sdf(i, j) = positive ? e2p.norm() : - e2p.norm();
                else {
                    double dist = std::sqrt(s2p.squaredNorm() - std::pow(sdot, 2));
                    sdf(i, j) = positive ? dist : -dist;
                }
                Eigen::Vector2d t2p = s2p - truc;
                bool over_truc = (t2p.dot(direct) <= 0.0);       // 是否超过边界截断位置
                if (over_truc == false) {                                   // 未超过
                    double distance = std::abs(sdf(i, j));
                    double new_alpha = 1.0 / (distance + 1.0);
                    if (new_alpha > alpha(i, j))
                        alpha(i, j) = new_alpha;
                }
            }
            else {
                if (over_s) 
                    sdf(i, j) = positive ? s2p.norm() : - s2p.norm();
                else {
                    double dist = std::sqrt(s2p.squaredNorm() - std::pow(sdot, 2));
                    sdf(i, j) = positive ? dist : -dist;
                }
                Eigen::Vector2d t2p = s2p - truc;
                bool over_truc = (t2p.dot(direct) > 0.0);
                if (over_truc == false) {
                    double distance = std::abs(sdf(i, j));
                    double new_alpha = (1.0 / (distance + 1.0));
                    if (new_alpha > alpha(i, j))
                        alpha(i, j) = new_alpha;
                }
            }
        }
    }
}
 
void SDF::alphaCalculation(const Edge& eg, const Eigen::Vector2d& tl, Eigen::MatrixXd& alpha, bool truc_start) const {
    const Eigen::Vector2d s2c = tl - eg.sp, e2c = tl - eg.ep;
    Eigen::Vector2d direct = eg.ep - eg.sp;
    Eigen::Vector2d truc = direct / 4.0 * 3.0;
    for (int i = 0; i < alpha.rows(); i++) {
        for (int j = 0; j < alpha.cols(); j++) {
            Eigen::Vector2d t2p(0.0, 0.0);
            bool over_truc = false;
            if (truc_start == false) {
                Eigen::Vector2d s2p = s2c + Eigen::Vector2d(j, i);
                t2p = s2p - truc;
                over_truc = (t2p.dot(direct) >= 0.0);
            }
            else {
                Eigen::Vector2d e2p = e2c + Eigen::Vector2d(j, i);
                t2p = e2p + truc;
                over_truc = (t2p.dot(direct) <= 0.0);
            }
            if (over_truc == false) continue; 
            double norm = t2p.norm();
            double new_alpha = (1.0 / (norm + 1.0));
            if (new_alpha > alpha(i, j))
                alpha(i, j) = new_alpha;
        }
    }
}

void SDF::marchingSquare(const Eigen::MatrixXd& lut, const Eigen::MatrixXd& weights, EdgeMap& dst) const {
    MatrixXu tags;
    gridTagsCalculation(lut, tags);
    int cols = lut.cols(), rows = lut.rows();
    for (int i = 0; i < rows - 1; i++) {
        for (int j = 0; j < cols - 1; j++) {
            Edges result;
            uchar tag = tags(i, j);
            const Vertex& vtx = tb[tag];
            Eigen::Vector4d vals, wghts;
            vals(0) = lut(i, j);
            vals(1) = lut(i, j + 1);
            vals(2) = lut(i + 1, j + 1);
            vals(3) = lut(i + 1, j);
            wghts(0) = weights(i, j);
            wghts(1) = weights(i, j + 1);
            wghts(2) = weights(i + 1, j + 1);
            wghts(3) = weights(i + 1, j);
            linearInterp(vtx, wghts, vals, result);
            std::pair<int, int> pr = {i, j};
            for (const Edge& eg: result)                        // 保存所有边
                dst[pr].push_back(eg);
        }
    }
}

void SDF::gridTagsCalculation(const Eigen::MatrixXd& sdf, MatrixXu& tags) const {
    int rows = sdf.rows(), cols = sdf.cols();
    tags = MatrixXu(rows, cols);
    for (int i = 0; i < rows - 1; i++) {
        for (int j = 0; j < cols - 1; j++) {
            Eigen::Vector4d vals;
            vals << sdf(i + 1, j), sdf(i + 1, j + 1), sdf(i, j + 1), sdf(i, j);
            uchar tag = 0x0f, and_elem = 0xfe;
            for (int k = 0; k < 4; k++) {
                if (vals(k) < 0)
                    tag &= and_elem;
                and_elem <<= 1;
                and_elem |= 0x01;
            }
            tags(i, j) = tag;
        }
    }
}

void SDF::linearInterp(const Vertex& vtx, const Eigen::Vector4d& wghts, const Eigen::Vector4d& vals, Edges& edges) const {
    Eigen::Vector4d abs_vals = vals.cwiseAbs();
    for (size_t i = 0; i < vtx.size(); i++) {
        int v11 = vtx[i].first, v21 = vtx[i].second;
        int v12 = (v11 + 1) % 4, v22 = (v21 + 1) % 4;
        double off1 = abs_vals(v11) / (abs_vals(v11) + abs_vals(v12)), off2 = abs_vals(v21) / (abs_vals(v21) + abs_vals(v22));
        Edge edge;
        edge.sp = (vertices[v12] - vertices[v11]) * off1;
        edge.ep = (vertices[v22] - vertices[v21]) * off2;
        edge.weight = Eigen::Vector4d(wghts[v11], wghts[v12], wghts[21], wghts[22]).maxCoeff();     // 4选3 或 4选4
        edges.push_back(edge);
    }
}

void SDF::searchSerialize(const Eigen::Vector2d& tl, const Eigen::MatrixXd& alpha, EdgeMap& emap, Edges& result) const {
    std::stack<EdgeMap::const_iterator> its;
    EdgeBool bools;
    for (EdgeMap::const_iterator cit = emap.cbegin(); cit != emap.cend(); cit++) {
        bools[std::make_pair(cit->first.first, cit->first.second)] = false;
    }
    std::deque<Edge> edges;
    IntPrs prs; 
    its.push(emap.cbegin());
    bool push_back = true;
    int not_found_cnt = 0;
    while (its.empty() == false) {
        EdgeMap::const_iterator it = its.top();
        int row = it->first.first, col = it->first.second;
        std::pair<int, int> pr = {row, col};
        bools[pr] = true;
        its.pop();
        for (const Edge& eg: it->second) {
            Edge new_eg;
            new_eg.sp = Eigen::Vector2d(col, row) * grid_size + eg.sp + tl;
            new_eg.ep = Eigen::Vector2d(col, row) * grid_size + eg.ep + tl;
            new_eg.weight = eg.weight;
            int tlx = tl.y() / grid_size, tly = tl.x() / grid_size;
            if (push_back) {
                edges.push_back(new_eg);
                prs.emplace_back(row + tly, col + tlx);
            }
            else {
                edges.push_front(new_eg);
                prs.emplace_front(row + tly, col + tlx);
            }
        }
        bool find_none = true;
        for (const std::pair<int, int>& offset: neighbor_4) {
            std::pair<int, int> pr(row + offset.first, col + offset.second);
            EdgeBool::const_iterator bool_it = bools.find(pr);
            if (bool_it != bools.end()) {
                bool used = (*bool_it).second;
                if (used == true) continue;
                EdgeMap::const_iterator cit = emap.find(pr);
                if (cit != emap.end()) {
                    its.push(cit);
                    find_none = false;
                }
            }
        }
        if (find_none == true) {
            not_found_cnt ++;
            push_back = !push_back;
        }
    }
    printf("Not found counter: %d\n", not_found_cnt);
    for (IntPrs::const_iterator cit = prs.cbegin(); edges.size() > 1; cit++) {
        if (alpha(cit->first, cit->second) < threshold) {
            edges.pop_front();
        }
        else break;
    }
    for (IntPrs::const_reverse_iterator rit = prs.crbegin(); edges.size() > 1; rit++) {
        if (alpha(rit->first, rit->second) < threshold)
            edges.pop_back();
        else break;
    }
    result.assign(edges.begin(), edges.end());
}

// ======================= DEBUG =========================
#ifdef DEBUG        // DEBUG 使用
void SDF::visualizeValues(const Eigen::MatrixXd& vals, const Eigen::Vector2d& tl, cv::Mat& dst) const {
    dst.create(vals.rows() + int(tl(1) / grid_size), vals.cols() + int(tl(0) / grid_size), CV_8UC3);
    double maxi = std::abs(vals.maxCoeff()), mini = vals.minCoeff();
    #pragma omp parallel for num_threads(8)
    for (int i = 0; i < vals.rows(); i++) {
        for (int j = 0; j < vals.cols(); j++) {
            cv::Vec3b color(255, 255, 255);
            if (vals(i, j) > 1e-5) {
                uchar channels = std::max((maxi - vals(i, j)) / maxi * 254, 0.0);
                color = cv::Vec3b(channels, channels, 255);
            }
            else if (vals(i, j) < -1e-5){
                uchar channels = std::max((vals(i, j) - mini) / (-mini) * 254, 0.0);
                color = cv::Vec3b(255, channels, channels);
            }
            else {
                color = cv::Vec3b(255, 255, 0);
            }
            dst.at<cv::Vec3b>(i + int(tl(1)), j + int(tl(0))) = color;
        }
    }
}

void SDF::visualizeMesh(const Mesh& mesh, const cv::Vec3b& color, cv::Mat& dst) const {
    /// @todo 为了方便起见，默认所有点都 > 0 否则无法可视化
    for (size_t i = 0; i < mesh.size() - 1; i++) {
        const Eigen::Vector2d& pt1 = mesh[i];
        const Eigen::Vector2d& pt2 = mesh[i + 1];
        cv::Point p1(pt1.x() / grid_size, pt1.y() / grid_size);
        cv::Point p2(pt2.x() / grid_size, pt2.y() / grid_size);
        cv::line(dst, p1, p2, color);
    }
}

void SDF::visualizeEdges(const Edges& edges, const cv::Vec3b& color, cv::Mat& dst) const {
    for (const Edge& eg: edges) {
        cv::Point p1(eg.sp.x() / grid_size, eg.sp.y() / grid_size);
        cv::Point p2(eg.ep.x() / grid_size, eg.ep.y() / grid_size);
        cv::line(dst, p1, p2, color);
    }
}

void SDF::visualizeMarchingSquare(const EdgeMap& emap, const Eigen::Vector2d& tl, const cv::Vec3b& color, cv::Mat& dst) const {
    double tlx = tl.x() / grid_size, tly = tl.y() / grid_size;
    for (const auto& pr: emap) {
        for (const Edge& eg: pr.second) {
            cv::Point p1(eg.sp.x() / grid_size + pr.first.second + tlx, eg.sp.y() / grid_size + pr.first.first + tly);
            cv::Point p2(eg.ep.x() / grid_size + pr.first.second + tlx, eg.ep.y() / grid_size + pr.first.first + tly);
            cv::line(dst, p1, p2, color);
        }
    }
}

void SDF::visualizeAlpha(const Eigen::MatrixXd& alpha, const Eigen::Vector2d& tl, cv::Mat& dst) const {
    int tlx = int(tl(0) / grid_size), tly = int(tl(1) / grid_size);
    dst.create(tly + alpha.rows(), tlx + alpha.cols(), CV_8UC3);
    double max_val = alpha.maxCoeff();
    #pragma omp parallel for num_threads(8)
    for (int i = 0; i < alpha.rows(); i++) {
        for (int j = 0; j < alpha.cols(); j++) {
            uchar val = alpha(i, j) / max_val * 255;
            dst.at<cv::Vec3b>(i + tly, j + tlx) = cv::Vec3b(val, val, val);
        }
    }
}

void SDF::visualizeCombinedAlpha(const Eigen::MatrixXd& a1, const Eigen::MatrixXd& a2, const Eigen::Vector2d& tl, cv::Mat& dst) const {
    Eigen::MatrixXd a = a1 + a2;
    int tlx = int(tl(0) / grid_size), tly = int(tl(1) / grid_size);
    dst.create(tly + a.rows(), tlx + a.cols(), CV_8UC3);
    double max_val = a.maxCoeff();
    #pragma omp parallel for num_threads(8)
    for (int i = 0; i < a.rows(); i++) {
        for (int j = 0; j < a.cols(); j++) {
            uchar val = a(i, j) / max_val * 255;
            dst.at<cv::Vec3b>(i + tly, j + tlx) = cv::Vec3b(val, val, val);
        }
        // uchar val = 0;
        //     if (alpha(i, j) > threshold) {
        //         val = 255;
        //     }
        //     dst.at<cv::Vec3b>(i + tly, j + tlx) = cv::Vec3b(val, val, val);
    }
}

void SDF::visualizeBelongs(const Eigen::MatrixXi& belong, const Eigen::Vector2d& tl, cv::Mat& dst) const {
    int tlx = int(tl(0) / grid_size), tly = int(tl(1) / grid_size);
    dst.create(tly + belong.rows(), tlx + belong.cols(), CV_8UC3);
    int max_val = std::max(belong.maxCoeff(), 1);
    #pragma omp parallel for num_threads(8)
    for (int i = 0; i < belong.rows(); i++) {
        for (int j = 0; j < belong.cols(); j++) {
            uchar val = 255 * belong(i, j) / max_val;
            dst.at<cv::Vec3b>(i + tly, j + tlx) = cv::Vec3b(val, val, val);
        }
    }
}
#endif