#ifndef KALMAN_H
#define KALMAN_H

#include <opencv2/opencv.hpp>

class ArmorKalmanFilter {
public:
    // 构造函数：初始化卡尔曼滤波器，参数为初始坐标(x, y)
    ArmorKalmanFilter(float init_x = 0.0f, float init_y = 0.0f);

    // 预测下一帧的坐标，返回预测的(x, y)
    cv::Point2f predict();

    // 用实际检测到的坐标修正滤波器
    void correct(const cv::Point2f& actual_point);

private:
    cv::KalmanFilter kf;  // OpenCV卡尔曼滤波器
    cv::Mat measurement;  // 观测矩阵 [x, y]
};

#endif // KALMAN_H
