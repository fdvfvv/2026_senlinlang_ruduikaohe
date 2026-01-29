#include "kalman.h"

ArmorKalmanFilter::ArmorKalmanFilter(float init_x, float init_y) {
    // 初始化卡尔曼滤波器：4个状态量(x,y,dx,dy)，2个观测量(x,y)，0个控制量
    kf = cv::KalmanFilter(4, 2, 0);

    // 状态转移矩阵 A [4x4]：匀速运动模型，时间步长=1
    kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    // 观测矩阵 H [2x4]：仅观测位置(x,y)
    kf.measurementMatrix = (cv::Mat_<float>(2, 4) << 
        1, 0, 0, 0,
        0, 1, 0, 0);

    // 过程噪声协方差 Q：控制模型不确定性（越小越信任模型）
    cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));

    // 测量噪声协方差 R：控制观测不确定性（越小越信任检测结果）
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));

    // 后验误差协方差 P：初始值
    cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

    // 初始化状态：初始位置(init_x, init_y)，初始速度(0,0)
    kf.statePost = (cv::Mat_<float>(4, 1) << init_x, init_y, 0, 0);

    // 初始化观测矩阵
    measurement = cv::Mat::zeros(2, 1, CV_32F);
}

cv::Point2f ArmorKalmanFilter::predict() {
    // 预测下一状态
    cv::Mat prediction = kf.predict();
    // 返回预测的(x,y)坐标
    return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
}

void ArmorKalmanFilter::correct(const cv::Point2f& actual_point) {
    // 设置观测值为实际检测到的坐标
    measurement.at<float>(0) = actual_point.x;
    measurement.at<float>(1) = actual_point.y;
    // 用观测值修正卡尔曼滤波器
    kf.correct(measurement);
}
