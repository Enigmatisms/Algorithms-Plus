#include "../include/SDF.hpp"

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
    rnc(0) = std::ceil((max_y - min_y + 4 * grid_size) / grid_size);
    rnc(1) = std::ceil((max_x - min_x + 4 * grid_size) / grid_size);
}

void SDF::singleMeshSDF(
    const Edges& edges, const Eigen::Vector2d& tl, const Eigen::Vector2i& grid_size,
    Eigen::MatrixXd& sdf, Eigen::MatrixXd& alpha, Eigen::MatrixXi& idx
) const {
    sdf = Eigen::MatrixXd(grid_size(0), grid_size(1));
    alpha = Eigen::MatrixXd(grid_size(0), grid_size(1));
    idx = Eigen::MatrixXi(grid_size(0), grid_size(1));
    Eigen::MatrixXd ones = Eigen::MatrixXd(grid_size(0), grid_size(1));
    ones.setOnes();
    alpha.setZero();
    singleMeshPieceSDF<START>(edges.front(), tl, sdf, alpha);
    size_t last_index = edges.size() - 1, i = 1;
    for (i = 1; i < last_index; i++) {     // no head and tail
        Eigen::MatrixXd tmp(grid_size(0), grid_size(1));
        singleMeshPieceSDF<NORMAL>(edges[i], tl, tmp, alpha);
        bool acute = edges[i].isAngleAcute(edges[i - 1]);
        bool normal_same = edges[i].sameDirectionAsNormal(edges[i - 1]);
        printf("Line %d, acute: %d, normal_same: %d\n", i, acute, normal_same);
        for (int j = 0; j < grid_size(0); j++) {
            for (int k = 0; k < grid_size(1); k++) {
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
                double new_alpha = 1.0 / (abs_sd + 1.0);
                if (alpha(j, k) < new_alpha)
                    alpha(j, k) = new_alpha;
            }
        }
    }
    Eigen::MatrixXd tmp(grid_size(0), grid_size(1));
    singleMeshPieceSDF<END>(edges.back(), tl, tmp, alpha);
    bool acute = edges[last_index].isAngleAcute(edges[last_index - 1]);
    bool normal_same = edges[last_index].sameDirectionAsNormal(edges[last_index - 1]);
    for (int j = 0; j < grid_size(0); j++) {
        for (int k = 0; k < grid_size(1); k++) {
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

void SDF::doubleMeshSDF(const Edges& eg1, const Edges& eg2, Eigen::MatrixXd& sdf) const {
    Eigen::Vector2d tl;
    Eigen::Vector2i grid_size;
    meshBoundingBox(eg1, eg2, tl, grid_size);                  // 求出两个mesh的范围 有一定的裕量
    Eigen::MatrixXd sdf1, sdf2, alpha1, alpha2;
    Eigen::MatrixXi idx1, idx2;
    singleMeshSDF(eg1, tl, grid_size, sdf1, alpha1, idx1);    // 三通道输出，输出sdf，计算的权重以及到mesh哪一段距离最小
    singleMeshSDF(eg2, tl, grid_size, sdf2, alpha2, idx2);
    sdf = Eigen::MatrixXd(grid_size(0), grid_size(1));
    sdf.setZero();
    for (int i = 0; i < grid_size(0); i++) {
        for (int j = 0; j < grid_size(1); j++) {
            int index1 = idx1(i, j), index2 = idx2(i, j);
            double v1 = sdf1(i, j), v2 = sdf2(i, j);
            double w1 = alpha1(i, j) * eg1[index1].weight, w2 = alpha2(i, j) * eg2[index2].weight;
            sdf(i, j) = (v1 * w1 + v2 * w2) / (w1 + w2);
        }
    }
    cv::Mat image, img1, img2, img3;
    visualizeValues(sdf, tl, image);
    visualizeEdges(eg1, color_g, image);
    visualizeEdges(eg2, color_k, image);
    cv::imshow("disp", image);

    visualizeAlpha(alpha1, tl, img1);
    visualizeAlpha(alpha2, tl, img2);
    visualizeCombinedAlpha(alpha1, alpha2, tl, img3);
    cv::imshow("combined", img3);
    cv::imshow("sdf1", img1);
    cv::imshow("sdf2", img2);
    cv::waitKey(0);



    // visualizeEdges(eg1, color_g, image);
    // visualizeEdges(eg2, color_k, image);

    // visualizeValues(sdf1, tl, img1);
    // visualizeValues(sdf2, tl, img2);
    // visualizeEdges(eg1, color_k, img1);
    // visualizeEdges(eg2, color_k, img2);
    // cv::imshow("disp", image);
    // cv::imshow("sdf1", img1);
    // cv::imshow("sdf2", img2);
    // cv::waitKey(0);
    EdgeMap emap;
    marchingSquare(sdf, emap);
    Mesh mesh;
    visualizeValues(sdf, tl, image);
    visualizeMarchingSquare(emap, tl, color_b, image);
    searchSerialize(tl, emap, mesh);
    visualizeMesh(mesh, color_k, image);
    cv::imshow("march", image);
    cv::waitKey(0);
}

template<SDF::calcType type>
void SDF::singleMeshPieceSDF(const Edge& eg, const Eigen::Vector2d& tl, Eigen::MatrixXd& sdf, Eigen::MatrixXd& alpha) const {
    const Eigen::Vector2d s2c = tl - eg.sp, e2c = tl - eg.ep;
    Eigen::Vector2d direct = eg.ep - eg.sp;
    const Eigen::Vector2d normal = Eigen::Vector2d(-direct.y(), direct.x()).normalized();
    Eigen::Vector2d truc = direct / 4.0;
    if (type == END)
        truc *= 3.0;
    direct.normalize();
    for (int i = 0; i < sdf.rows(); i++) {
        for (int j = 0; j < sdf.cols(); j++) {
            Eigen::Vector2d s2p = s2c + Eigen::Vector2d(j, i);
            Eigen::Vector2d e2p = e2c + Eigen::Vector2d(j, i);
            double sdot = s2p.dot(direct), edot = e2p.dot(direct);
            bool positive = (s2p.dot(normal) > 0.0);
            bool antis = (sdot <= 0.0), antie = (edot <= 0.0);
            bool over_s = antis & antie, over_e = (!antis) & (!antie);
            if (type == NORMAL) {
                if (over_s) 
                    sdf(i, j) = positive ? s2p.norm() : - s2p.norm();
                else if (over_e)
                    sdf(i, j) = positive ? e2p.norm() : - e2p.norm();
                else {
                    double dist = std::sqrt(s2p.squaredNorm() - std::pow(sdot, 2));
                    sdf(i, j) = positive ? dist : -dist;
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
                bool over_truc = ((t2p.dot(direct) <= 0.0) & antie);       // 是否超过边界截断位置
                if (over_truc == false) {                                   // 未超过
                    double new_alpha = 1.0 / (std::abs(sdf(i, j)) + 1.0);
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
                bool over_truc = ((t2p.dot(direct) > 0.0) & ~antis);
                if (over_truc == false) {
                    double new_alpha = 1.0 / (std::abs(sdf(i, j)) + 1.0);
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
    const Eigen::Vector2d normal = Eigen::Vector2d(-direct.y(), direct.x()).normalized();
    Eigen::Vector2d truc = direct / 4.0 * 3.0;
    for (int i = 0; i < alpha.rows(); i++) {
        for (int j = 0; j < alpha.cols(); j++) {
            Eigen::Vector2d t2p(0.0, 0.0);
            bool over_truc = false;
            if (truc_start == false) {
                Eigen::Vector2d s2p = s2c + Eigen::Vector2d(j, i);
                t2p = s2p - truc;
                over_truc = (s2p.dot(direct) >= 0.0) & (t2p.dot(direct) >= 0.0);
            }
            else {
                Eigen::Vector2d e2p = e2c + Eigen::Vector2d(j, i);
                t2p = e2p + truc;
                over_truc = (e2p.dot(direct) <= 0.0) & (t2p.dot(direct) <= 0.0);
            }
            if (over_truc == false) continue; 
            double new_alpha = 1.0 / (t2p.norm() + 1.0);
            if (new_alpha > alpha(i, j))
                alpha(i, j) = new_alpha;
        }
    }
}

void SDF::marchingSquare(const Eigen::MatrixXd& lut, EdgeMap& dst) const {
    MatrixXu tags;
    gridTagsCalculation(lut, tags);
    int cols = lut.cols(), rows = lut.rows();
    for (int i = 0; i < rows - 1; i++) {
        for (int j = 0; j < cols - 1; j++) {
            Edges result;
            uchar tag = tags(i, j);
            const Vertex& vtx = tb[tag];
            Eigen::Vector4d vals;
            vals(0) = lut(i, j);
            vals(1) = lut(i, j + 1);
            vals(2) = lut(i + 1, j + 1);
            vals(3) = lut(i + 1, j);
            linearInterp(vtx, vals, result);
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

void SDF::linearInterp(const Vertex& vtx, const Eigen::Vector4d& vals, Edges& edges) const {
    Eigen::Vector4d abs_vals = vals.cwiseAbs();
    for (size_t i = 0; i < vtx.size(); i++) {
        int v11 = vtx[i].first, v21 = vtx[i].second;
        int v12 = (v11 + 1) % 4, v22 = (v21 + 1) % 4;
        double off1 = abs_vals(v11) / (abs_vals(v11) + abs_vals(v12)), off2 = abs_vals(v21) / (abs_vals(v21) + abs_vals(v22));
        Edge edge;
        edge.sp = (vertices[v12] - vertices[v11]) * off1;
        edge.ep = (vertices[v22] - vertices[v21]) * off2;
        edge.weight = 1.0;
        edges.push_back(edge);
    }
}

void SDF::searchSerialize(const Eigen::Vector2d& tl, EdgeMap& emap, Mesh& mesh) const {
    std::stack<EdgeMap::const_iterator> its;
    std::deque<Edge> edges;
    its.push(emap.cbegin());
    bool push_back = true;
    while (its.empty() == false) {
        EdgeMap::const_iterator it = its.top();
        int row = it->first.first, col = it->first.second;
        emap.erase(it);
        its.pop();
        for (const Edge& eg: it->second) {
            Edge new_eg;
            new_eg.sp = Eigen::Vector2d(col, row) + eg.sp + tl;
            new_eg.ep = Eigen::Vector2d(col, row) + eg.ep + tl;
            new_eg.weight = eg.weight;
            if (push_back)
                edges.push_back(new_eg);
            else
                edges.push_front(new_eg);
        }
        bool find_none = true;
        for (const std::pair<int, int>& offset: neighbor_4) {
            std::pair<int, int> pr(row + offset.first, col + offset.second);
            EdgeMap::const_iterator cit = emap.find(pr);
            if (cit != emap.end()) {
                its.push(cit);
                find_none = false;
            }
        }
        if (find_none == true) {
            push_back = false;
            printf("PUSH_BACK = FALSE\n");
        }
    }
    for (size_t i = 0; i < edges.size() - 1; i++)
        mesh.push_back(edges[i].sp);
    mesh.push_back(edges.back().sp);
    mesh.push_back(edges.back().ep);
}

// ======================= DEBUG =========================
void SDF::visualizeValues(const Eigen::MatrixXd& vals, const Eigen::Vector2d& tl, cv::Mat& dst) const {
    dst.create(vals.rows() + int(tl(1) / grid_size), vals.cols() + int(tl(0) / grid_size), CV_8UC3);
    double maxi = std::abs(vals.maxCoeff()), mini = vals.minCoeff();
    #pragma omp parallel for num_threads(8)
    for (int i = 0; i < vals.rows(); i++) {
        for (int j = 0; j < vals.cols(); j++) {
            cv::Vec3b color(255, 255, 255);
            if (vals(i, j) >= 0.0) {
                uchar channels = std::max((maxi - vals(i, j)) / maxi * 254, 0.0);
                color = cv::Vec3b(channels, channels, 255);
            }
            else {
                uchar channels = std::max((vals(i, j) - mini) / (-mini) * 254, 0.0);
                color = cv::Vec3b(255, channels, channels);
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
    }
}

void SDF::visualizeBelongs(const Eigen::MatrixXi& belong, const Eigen::Vector2d& tl, cv::Mat& dst) const {
    int tlx = int(tl(0) / grid_size), tly = int(tl(1) / grid_size);
    dst.create(tly + belong.rows(), tlx + belong.cols(), CV_8UC3);
    int max_val = belong.maxCoeff();
    #pragma omp parallel for num_threads(8)
    for (int i = 0; i < belong.rows(); i++) {
        for (int j = 0; j < belong.cols(); j++) {
            uchar val = 255 * belong(i, j) / max_val;
            dst.at<cv::Vec3b>(i + tly, j + tlx) = cv::Vec3b(val, val, val);
        }
    }
}
