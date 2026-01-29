#ifndef COMMON_H
#define COMMON_H

#include <opencv2/opencv.hpp>
#include <cmath>

// 通用常量
const int MAX_LIGHT_BAR_NUM = 4;  // 最大灯条数量（1-4）
const cv::Scalar COLOR_RED = cv::Scalar(0,0,255);    // 灯条1颜色
const cv::Scalar COLOR_GREEN = cv::Scalar(0,255,0);  // 灯条2颜色
const cv::Scalar COLOR_BLUE = cv::Scalar(255,0,0);   // 灯条3颜色
const cv::Scalar COLOR_YELLOW = cv::Scalar(255,255,0);// 灯条4颜色

// 通用工具函数声明
// 计算两点间欧氏距离
inline float calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// 角度归一化到[-90,90]
inline float normalizeAngle(float angle) {
    while (angle < -90) angle += 180;
    while (angle > 90) angle -= 180;
    return angle;
}

// 绘制旋转矩形（通用函数）
inline void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness = 2) {
    cv::Point2f pts[4];
    rect.points(pts);
    for (int i = 0; i < 4; i++) {
        cv::line(img, pts[i], pts[(i+1)%4], color, thickness);
    }
}

#endif // COMMON_H
