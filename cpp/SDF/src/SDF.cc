#include "../include/SDF.hpp"

void SDF::meshBoundingBox(const Edges& m1, const Edges& m2, Eigen::Vector2d& tl, Eigen::Vector2i& rnc) const {
    // 输入两个Mesh，输出Mesh对应的ROI（有裕量）的左上角点 以及纵横grid数量
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

    singleMeshPieceSDF<START>(edges.front(), tl, sdf);
    size_t last_index = edges.size() - 1;
    for (size_t i = 1; i < last_index; i++) {     // no head and tail
        Eigen::MatrixXd tmp;
        singleMeshPieceSDF<NORMAL>(edges[i], tl, tmp);
        for (int j = 0; j < grid_size(0); j++) {
            for (int k = 0; k < grid_size(1); k++) {
                double abs_sd = std::abs(tmp(j, k));
                if (std::abs(sdf(j, k)) > abs_sd) {
                    idx(j, k) = i;
                    sdf(j, k) = tmp(j, k);
                }
                double new_alpha = 1.0 / (abs_sd + 1.0);
                if (alpha(j, k) < new_alpha)
                    alpha(j, k) = new_alpha;
            }
        }
    }
    Eigen::MatrixXd tmp;
    singleMeshPieceSDF<END>(edges.back(), tl, tmp);
    for (int j = 0; j < grid_size(0); j++) {
        for (int k = 0; k < grid_size(1); k++) {
            double abs_sd = std::abs(tmp(j, k));
            if (std::abs(sdf(j, k)) > abs_sd) {
                idx(j, k) = last_index;
                sdf(j, k) = tmp(j, k);
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
}

template<SDF::calcType type>
void SDF::singleMeshPieceSDF(const Edge& eg, const Eigen::Vector2d& tl, Eigen::MatrixXd& sdf) const {
    const Eigen::Vector2d c2s = tl - eg.sp, c2e = tl - eg.sp;
    Eigen::Vector2d direct = (eg.ep - eg.sp).normalized();
    const Eigen::Vector2d normal = Eigen::Vector2d(-direct.y(), direct.x()).normalized();
    for (int i = 0; i < sdf.rows(); i++) {
        for (int j = 0; j < sdf.cols(); j++) {
            Eigen::Vector2d p2s = c2s + Eigen::Vector2d(j, i);
            Eigen::Vector2d p2e = c2e + Eigen::Vector2d(j, i);
            double sdot = p2s.dot(direct), edot = p2e.dot(direct);
            double snorm = p2s.dot(normal), enorm = p2e.norm(normal);
            bool antis = (sdot <= 0.0), antie = (edot <= 0.0);
            bool over_s = antis & antie, over_e = (~antis) & (~antie);
            bool nears = (snorm > enorm), positive = (snorm > 0.0);
            if (type == NORMAL) {
                if (over_s) 
                    sdf(i, j) = positive ? p2s.norm() : - p2s.norm();
                else if (over_e)
                    sdf(i, j) = positive ? p2e.norm() : - p2e.norm();
                else {
                    double dist = std::sqrt(p2s.squaredNorm() - std::pow(sdot, 2));
                    sdf = positive ? dist : -dist;
                }
            }
            else if (type == START) {
                if (over_e)
                    sdf(i, j) = positive ? p2e.norm() : - p2e.norm();
                else {
                    double dist = std::sqrt(p2s.squaredNorm() - std::pow(sdot, 2));
                    sdf = positive ? dist : -dist;
                }
            }
            else {
                if (over_s) 
                    sdf(i, j) = positive ? p2s.norm() : - p2s.norm();
                else {
                    double dist = std::sqrt(p2s.squaredNorm() - std::pow(sdot, 2));
                    sdf = positive ? dist : -dist;
                }
            }
        }
    }
}

void SDF::marchingSquare(const Eigen::MatrixXd& sdf1, const Eigen::MatrixXd& sdf2, const Eigen::Vector4d& tlbr, EdgeMap& dst) const {
    Eigen::MatrixXd lut = sdf1 + sdf2;          // SDF look up table
    MatrixXu tags;
    gridTagsCalculation(lut, tags);
    int cols = lut.cols(), rows = lut.rows();
    double sx = tlbr.w(), sy = tlbr.x(), ex = tlbr.y(), ey = tlbr.z();
    for (int i = 0; i < rows - 1; i++) {
        for (int j = 0; j < cols - 1; j++) {
            Edges result;
            uchar tag = tags(i, j);
            const Vertex& vtx = tb[tag];
            Eigen::Vector4d vals;
            vals(0) = lut(i, j);
            vals(1) = lut(i + 1, j);
            vals(2) = lut(i, j + 1);
            vals(3) = lut(i + 1, j + 1);
            linearInterp(vals, vtx, (tag & 0x08), result);
            std::pair<int, int> pr = {i, j};
            for (const Edge& eg: result)                        // 保存所有边
                dst[pr].push_back(eg);
        }
    }
    /// ================= 直接可视化 ====================
    /// @todo
}