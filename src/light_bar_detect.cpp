#include "sanweizuobiao.h"
#include "armor_match.h"
#include "light_bar.h"
#include "kalman.h" 
#include <opencv2/opencv.hpp>
#include <iostream>

// 蓝色灯条的HSV颜色阈值（适配暗画面）
const cv::Scalar LOWER_BLUE = cv::Scalar(80, 215, 40);
const cv::Scalar UPPER_BLUE = cv::Scalar(110, 235, 255);
// 灯条轮廓筛选条件
const double ASPECT_RATIO_THRESH = 1.0;
const double MIN_AREA_THRESH = 10.0;

// 灯条检测核心函数
std::vector<LightBar> LightBar::detect(const cv::Mat& frame) {
    std::vector<LightBar> light_bars;
    cv::Mat hsv, blue_mask;

    // 颜色空间转换+阈值分割
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, LOWER_BLUE, UPPER_BLUE, blue_mask);
    
    // 形态学闭运算消除小噪声
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);

    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 筛选灯条轮廓
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < MIN_AREA_THRESH) continue;

        cv::RotatedRect rrect = cv::minAreaRect(contour);
        double aspect_ratio = std::max(rrect.size.width, rrect.size.height) 
                            / std::min(rrect.size.width, rrect.size.height);
        
        if (aspect_ratio > ASPECT_RATIO_THRESH) {
            light_bars.emplace_back(rrect);
        }
    }

    return light_bars;
}

int main() {
    // 视频路径
    const std::string VIDEO_PATH = "/home/fdvfvv/project/src/test.mp4"; //这里改成你自己的视频路径
    cv::VideoCapture cap(VIDEO_PATH);
    if (!cap.isOpened()) {
        std::cerr << "[Error] 无法打开视频文件: " << VIDEO_PATH << std::endl;
        return -1;
    }

    // 初始化三维坐标转换器
    SanWeiZuoBiao coordConverter;
    CameraParam camParam;

    // 初始化卡尔曼滤波器
    ArmorKalmanFilter kalman_filter(0, 0);
    bool first_detect = true;
    cv::Rect last_armor_rect;  // 保存上一帧装甲板尺寸

    cv::Mat frame;
    while (cap.read(frame)) {
        if (frame.empty()) break;

        // 裁剪画面下方区域（避开顶部干扰）
        int h = frame.rows;
        int w = frame.cols;
        cv::Rect roi(0, static_cast<int>(h * 0.4), w, static_cast<int>(h * 0.6));
        cv::Mat cropped_frame = frame(roi);

        // 动态更新相机主点坐标
        camParam.cx = cropped_frame.cols / 2.0;
        camParam.cy = cropped_frame.rows / 2.0;
        SanWeiZuoBiao tempConverter(camParam);

        // 可视化HSV分割和轮廓 
        cv::Mat hsv, blue_mask;
        cv::cvtColor(cropped_frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, LOWER_BLUE, UPPER_BLUE, blue_mask);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);

        cv::imshow("1. Blue Mask (HSV分割结果)", blue_mask);
        cv::Mat contour_show = cropped_frame.clone();
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(contour_show, contours, -1, cv::Scalar(0, 0, 255), 1);
        cv::imshow("2. All Contours (所有轮廓)", contour_show);

        // 检测灯条
        std::vector<LightBar> light_bars = LightBar::detect(cropped_frame);

        // 装甲板匹配+卡尔曼预测 
        auto armor_opt = Armor::matchLights(light_bars, cropped_frame.size());
        if (armor_opt.has_value()) {
            Armor target_armor = armor_opt.value();

            // 卡尔曼滤波：修正+预测
            if (first_detect) {
                kalman_filter = ArmorKalmanFilter(target_armor.center.x, target_armor.center.y);
                first_detect = false;
            } else {
                kalman_filter.correct(target_armor.center);
            }
            cv::Point2f predict_center = kalman_filter.predict();

            // 1. 绘制当前装甲板（蓝色框）
            cv::Rect armor_rect;
            armor_rect.x = std::min(target_armor.left_light.rect.x, target_armor.right_light.rect.x);
            armor_rect.y = std::min(target_armor.left_light.rect.y, target_armor.right_light.rect.y);
            armor_rect.width = std::max(target_armor.left_light.rect.x + target_armor.left_light.rect.width,
                                        target_armor.right_light.rect.x + target_armor.right_light.rect.width) - armor_rect.x;
            armor_rect.height = std::max(target_armor.left_light.rect.y + target_armor.left_light.rect.height,
                                        target_armor.right_light.rect.y + target_armor.right_light.rect.height) - armor_rect.y;
            cv::rectangle(cropped_frame, armor_rect, cv::Scalar(255, 0, 0), 2);
            last_armor_rect = armor_rect;  // 保存尺寸

            // 2. 绘制当前中心点（红色）
            cv::circle(cropped_frame, target_armor.center, 5, cv::Scalar(0, 0, 255), -1);

            // 3. 绘制预测装甲板（黄色粗框）
            cv::Rect predict_armor_rect;
            predict_armor_rect.width = armor_rect.width;
            predict_armor_rect.height = armor_rect.height;
            predict_armor_rect.x = static_cast<int>(predict_center.x - armor_rect.width / 2);
            predict_armor_rect.y = static_cast<int>(predict_center.y - armor_rect.height / 2);
            cv::rectangle(cropped_frame, predict_armor_rect, cv::Scalar(0, 255, 255), 3);

            // 4. 绘制预测中心点（粉色）
            cv::circle(cropped_frame, predict_center, 5, cv::Scalar(255, 0, 255), -1);

            // 5. 三维坐标转换+绘制
            cv::Vec3d cameraCoord = tempConverter.pixelToCamera3D(
                target_armor.left_light.center,
                target_armor.right_light.center,
                target_armor.center
            );
            char coordText[128];
            sprintf(coordText, "3D Coord: X=%.3fm, Y=%.3fm, Z=%.3fm",
                    cameraCoord[0], cameraCoord[1], cameraCoord[2]);
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(coordText, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
            cv::rectangle(frame, 
                        cv::Point(10, 30 - textSize.height - baseline),
                        cv::Point(10 + textSize.width, 30),
                        cv::Scalar(0, 0, 0), -1);
            cv::putText(frame, coordText, cv::Point(10, 30 - baseline),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

        } else if (!first_detect) {
            // 未检测到装甲板时仍预测（黄色虚线框）
            cv::Point2f predict_center = kalman_filter.predict();
            if (last_armor_rect.width > 0) {
                cv::Rect predict_armor_rect;
                predict_armor_rect.width = last_armor_rect.width;
                predict_armor_rect.height = last_armor_rect.height;
                predict_armor_rect.x = static_cast<int>(predict_center.x - last_armor_rect.width / 2);
                predict_armor_rect.y = static_cast<int>(predict_center.y - last_armor_rect.height / 2);
                cv::rectangle(cropped_frame, predict_armor_rect, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
        }

        // 绘制灯条（绿色框）
        for (const auto& lb : light_bars) {
            cv::rectangle(cropped_frame, lb.rect, cv::Scalar(0, 255, 0), 2);
        }

        // 显示完整画面
        cropped_frame.copyTo(frame(roi));
        cv::imshow("Blue Light Bar Detection", frame);

        // 按q退出
        if (cv::waitKey(30) == 'q') break;
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
