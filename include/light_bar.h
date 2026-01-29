#ifndef LIGHT_BAR_H
#define LIGHT_BAR_H

#include <opencv2/opencv.hpp>
#include <vector>

// 复刻basic_armor的类结构，定义灯条对象
class LightBar {
public:
    // 灯条的旋转矩形（包含位置、角度、尺寸）
    cv::RotatedRect rotated_rect;
    // 灯条的轴对齐矩形
    cv::Rect rect;
    // 灯条中心点
    cv::Point2f center;

    // 构造函数：由旋转矩形初始化灯条
    explicit LightBar(const cv::RotatedRect& rrect) 
        : rotated_rect(rrect),
          rect(rrect.boundingRect()),
          center(rrect.center) {}

    // 静态方法：从输入帧中检测蓝色灯条（复刻basic_armor的检测逻辑）
    static std::vector<LightBar> detect(const cv::Mat& frame);
};

#endif // LIGHT_BAR_H
