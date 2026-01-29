#ifndef SANWEIZUO_BIAO_H
#define SANWEIZUO_BIAO_H

#include <opencv2/opencv.hpp>

// 相机参数结构体（用户需根据实际相机校准结果修改）
struct CameraParam {
    // 内参矩阵：fx/fy为焦距（像素单位），cx/cy为主点坐标（像素单位）
    double fx = 650.0;   // 示例焦距（需根据实际校准调整）
    double fy = 650.0;
    double cx = 320.0;   // 默认主点x（动态适配帧尺寸，此处为初始值）
    double cy = 240.0;   // 默认主点y（动态适配帧尺寸，此处为初始值）
    
    // 畸变系数（默认无畸变，有校准数据可修改）
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    
    // 装甲板实际宽度：两灯条中心的实际距离（单位：米，必须根据实际装甲板修改！）
    double armorActualWidth = 0.1357;  //装甲板实际尺寸
};

class SanWeiZuoBiao {
public:
    // 构造函数（默认使用默认相机参数）
    SanWeiZuoBiao(const CameraParam& param = CameraParam());
    
    // 核心函数：像素坐标转相机三维坐标
    // 输入：左右灯条中心点（像素坐标）、装甲板中心点（像素坐标）
    // 输出：相机坐标系三维坐标 (X: 水平左右, Y: 垂直上下, Z: 深度（相机到装甲板距离）)
    cv::Vec3d pixelToCamera3D(const cv::Point2f& leftLightCenter, 
                              const cv::Point2f& rightLightCenter, 
                              const cv::Point2f& armorCenter);

private:
    CameraParam camParam;  // 相机参数
};

#endif // SANWEIZUO_BIAO_H
