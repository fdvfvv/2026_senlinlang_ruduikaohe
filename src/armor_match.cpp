#include "armor_match.h"
#include <cmath>
#include <algorithm>
#include <vector> 
#include<iostream>
using namespace std;

// 辅助函数：计算两点间欧氏距离
static double calcDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// 辅助函数：将minAreaRect的points数组转换为vector<Point>
static std::vector<cv::Point> rectPointsToVector(const cv::RotatedRect& rrect) {
    cv::Point2f pts[4];
    rrect.points(pts); // 获取旋转矩形的4个顶点
    std::vector<cv::Point> contour;
    for (int i = 0; i < 4; ++i) {
        contour.push_back(cv::Point(static_cast<int>(pts[i].x), static_cast<int>(pts[i].y)));
    }
    return contour;
}

// 检查两个灯条是否符合装甲板匹配条件
bool Armor::isMatch(const LightBar& l1, const LightBar& l2) {
    // 1. 计算灯条核心参数（长宽比、面积、旋转角度）
    double l1_w = l1.rotated_rect.size.width;
    double l1_h = l1.rotated_rect.size.height;
    double l1_aspect = std::max(l1_w, l1_h) / std::min(l1_w, l1_h);

    // 修复：使用辅助函数转换为正确的轮廓点集
    std::vector<cv::Point> l1_contour = rectPointsToVector(l1.rotated_rect);
    double l1_area = cv::contourArea(l1_contour);

    double l2_w = l2.rotated_rect.size.width;
    double l2_h = l2.rotated_rect.size.height;
    double l2_aspect = std::max(l2_w, l2_h) / std::min(l2_w, l2_h);

    std::vector<cv::Point> l2_contour = rectPointsToVector(l2.rotated_rect);
    double l2_area = cv::contourArea(l2_contour);

    // 2. 多维度阈值筛选（剔除误识别）
    // 角度筛选：灯条均为竖直，旋转角度接近
    double angle_diff = std::abs(l1.rotated_rect.angle - l2.rotated_rect.angle);
    if (angle_diff > ANGLE_THRESH) return false;
    //printf("灯条是竖直的\n");

    // 长宽比筛选：避免形状差异过大
    double aspect_diff = std::abs(l1_aspect - l2_aspect);
    if (aspect_diff > ASPECT_RATIO_DIFF_THRESH) return false;
    //printf("灯条形状差异没有过大\n");

    // 面积筛选：避免影子/噪声与真实灯条匹配（增加防除零判断）
    if (std::max(l1_area, l2_area) < 1e-6) return false; // 防除零
    double area_ratio = std::min(l1_area, l2_area) / std::max(l1_area, l2_area);
    //printf("%f %f %f\n",area_ratio,l1_area,l2_area);
    if (area_ratio < AREA_RATIO_THRESH) return false;
    //printf("灯条面积合格\n");

    // 距离筛选：符合装甲板灯条固定间距
    //double center_dist = calcDistance(l1.center, l2.center);
    //if (center_dist < DISTANCE_MIN || center_dist > DISTANCE_MAX) return false;
    //我新写的：
    double center_dist = calcDistance(l1.center, l2.center);
    //printf("%f\n",center_dist/l1_h);
    //printf("%f %f %f\n",center_dist,l1_h,l2_h);
    //printf("%f %f\n",l1_w,l2_w);
    if (center_dist > (l1_h+l2_h)/2*11) return false;
    //printf("灯条间距合格\n");

    //新增
    if (center_dist > (l1_w+l2_w)/2*4.5) return false;

    // 上下对齐筛选：y坐标偏差不能太大
    //double y_diff = std::abs(l1.center.y - l2.center.y);
    //printf("%f\n",y_diff);
    //if (y_diff > Y_DIFF_THRESH) return false;
    //printf("y坐标偏差合格\n");
    //我新写的
    double y_diff = std::abs(l1.center.y - l2.center.y);
    //printf("%f\n",y_diff/l1_w);
    //printf("%f %f %f\n",y_diff,l1_w,l2_w);
    if (y_diff > l1_w*0.5 || y_diff > l2_w*0.5) return false;
    //printf("y坐标偏差合格\n");

    // 左右分布筛选：x坐标差异足够（避免前后灯条误匹配）事实证明这个判断好像并没什么卵用
    //double x_diff = std::abs(l1.center.x - l2.center.x);
    //if (x_diff < X_DIFF_MIN) return false;
    //printf("x坐标偏差合格\n\n\n");

    //printf("\n\n");

    return true;
}

// 核心函数：从灯条列表中匹配目标装甲板（中间两个灯条）
std::optional<Armor> Armor::matchLights(const std::vector<LightBar>& lights, const cv::Size& frame_size) {
    std::vector<Armor> possible_armors;

    // 第一步：二次筛选有效灯条（剔除影子/小噪声）
    std::vector<LightBar> valid_lights;
    for (const auto& light : lights) {
        double w = light.rotated_rect.size.width;
        double h = light.rotated_rect.size.height;
        double aspect = std::max(w, h) / std::min(w, h);
        // 修复：计算面积时同样转换为正确的轮廓点集
        std::vector<cv::Point> contour = rectPointsToVector(light.rotated_rect);
        double area = cv::contourArea(contour);
        
        // 进一步严格筛选：长宽比≥2、面积≥30（适配暗画面影子过滤）
        valid_lights.push_back(light);
    }

    // 灯条数量不足2个，直接返回空
    if (valid_lights.size() < 2) return std::nullopt;

    // 第二步：遍历所有灯条对，筛选符合条件的装甲板
    for (size_t i = 0; i < valid_lights.size(); ++i) {
        for (size_t j = i + 1; j < valid_lights.size(); ++j) {
            LightBar l1 = valid_lights[i];
            LightBar l2 = valid_lights[j];

            if (isMatch(l1, l2)) {
                // 确定左右灯条（x小为左，x大为右）
                if (l1.center.x < l2.center.x) {
                    possible_armors.emplace_back(l1, l2);
                } else {
                    possible_armors.emplace_back(l2, l1);
                }
            }
        }
    }

    // 无匹配到装甲板，返回空
    if (possible_armors.empty()) return std::nullopt;

    // 第三步：筛选中间的装甲板（目标是4个灯条中中间的两个）
    double frame_center_x = frame_size.width / 2.0f; // 画面水平中心
    Armor target_armor = possible_armors[0];
    double min_dist_to_center = std::abs(target_armor.center.x - frame_center_x);

    // 选择中心最接近画面水平中心的装甲板
    for (const auto& armor : possible_armors) {
        double dist = std::abs(armor.center.x - frame_center_x);
        if (dist < min_dist_to_center) {
            min_dist_to_center = dist;
            target_armor = armor;
        }
    }

    return target_armor;
}
