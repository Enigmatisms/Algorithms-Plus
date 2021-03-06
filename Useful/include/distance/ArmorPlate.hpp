/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.2.9 00:27
 * 主要思路：设置几个判断条件：（2020.1.18日写）
 * 9/2/2020 修改：
 *      drawArmorPlate需要通过数字判断其是否valid
 */
#ifndef _ARMOR_PLATE_HPP
#define _ARMOR_PLATE_HPP

#include <fstream>
#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../aim_deps/LOG.hpp"
#include "../aim_deps/AimDeps.hpp"
//#define ARMORPLATE_DEBUG                      //无需灯条配对信息输出时注释此行
#ifdef ARMORPLATE_DEBUG
    #define amp_debug rmlog::LOG::printc        //彩色输出
#else
    #define amp_debug(...)
#endif

class ArmorPlate{
public:
    ArmorPlate(double distance, double angle);         
    ~ArmorPlate();
public:

    /**
     * @brief 根据与匹配情况匹配所有的灯条
     * @param matches cv::Point2f 与匹配对，两个值是两个灯条在lights中的索引
     * @param lights 灯条容器，由LightMatch类传入
     * @param tar_list 入参/输出，由更高层的AimDistance类传入
     */
    void matchAll(
        const std::vector<cv::Point>& matches,
        std::vector<aim_deps::Light> &lights, 
        std::vector<aim_deps::Armor> &tar_list);
    /**
     * @brief 绘制装甲板于图像
     * @param src 输入/输出,用于绘制的图像
     * @param tar_list 装甲板列表
     * @param optimal 最优装甲板的位置(使用绿色绘制)
     */
    void drawArmorPlates(cv::Mat &src, 
        const std::vector<aim_deps::Armor>& tar_list, const int optimal) const;     
                                               //消息发布
    void postProcess(std::vector<aim_deps::Armor> &tar_list);
private:
    bool isRatioValid(float l1, float l2) const;                //中点连线（平方）的比是否合适
    bool isEdgesValid() const;                                                //两对边(平方)的比是否合适

    /**
     * @brief 对两个灯条的RotatedRect进行匹配，直接在tar_list中emplace_back符合条件的装甲板
     * @param l1 灯条1
     * @param l2 灯条2
    */
    bool isMatch(const aim_deps::Light &l1, const aim_deps::Light &l2);
    bool getArmorPlate(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2);         //灯条匹配装甲板
    void filter(std::vector<aim_deps::Armor> &tar_list,
            std::vector<aim_deps::Light> &lights);                      //进一步过滤装甲板容器
    bool isAreaGood() const;                                            //面积是否正确：面积过小的装甲板将会被过滤

    inline bool isAngleMatch(float ang1, float ang2) const;          

    /** @brief 计算共灯条时，装甲板长边与共灯条边形成的夹角，夹角接近90度为真，同时考虑灯条夹角
     * @param pts 装甲板的四个点列表
     * @param start 从第start点开始计算角度（只需要算两个角度即可）
     * @return 角度的cos值之和，越小说明所有角度越接近90度
     */             
    inline static float cornerAngle(const cv::Point2f *pts,
                    const int start);
    inline static float lengthThreshold(const float l);                 //计算自适应装甲板长宽比
    inline static float compensation(const float mean);                 //根据平均灯条长度计算灯条长度补偿值
    static int lightCompensate(aim_deps::LightBox &l1, 
            aim_deps::LightBox &l2, aim_deps::Armor *_a = nullptr);     //灯条补偿
    inline static float rebuildRatio(const cv::Point2f *pts);           //灯条重建系数

    ///=============================临时=================================///

	/// 卷积操作
	void convolution(const cv::Mat &src, bool left = true);

	/// 质心求取
	cv::Point2f momentCenter(const cv::Mat &src, const cv::Point2f &offset);

	/// 由double 以及 angle 计算输出路径并按照格式输出contain中的信息到某个csv文件
	void output(bool left = true);

    /// 提取信息的处理

    static bool angleCmp(const aim_deps::Armor &arm1, const aim_deps::Armor &arm2){
        double ang1 = std::abs(arm1.left_light.box.angle - _angle) +
                std::abs(arm1.right_light.box.angle - _angle),
                ang2 = std::abs(arm2.left_light.box.angle - _angle) +
            std::abs(arm2.right_light.box.angle - _angle);
        return (ang1 < ang2);       // 比较哪个角离期望角接近
    }
public:
    int center_cnt;             // 取得中心次数的计数器（大于7后不再增加）
    static double _angle;
private:
    double dist;				// 输出训练数据使用
	double ang;					// 输出训练数据使用	
    cv::Point2f m_ctr;          // 中心
	std::vector<double> contain;										//输出暂存容器

    bool _is_enemy_blue;                                                //敌人颜色
    cv::Point2f points[4];                                              //装甲板点列的临时容器
    aim_deps::Distance_Params params;                                   //装甲板匹配参数

};
#endif     //_ARMOR_PLATE_HPP


