#include "sanweizuobiao.h"
#include <cmath>

// 构造函数初始化相机参数
SanWeiZuoBiao::SanWeiZuoBiao(const CameraParam& param) : camParam(param) {}

// 像素坐标转三维坐标实现（针孔相机模型）
cv::Vec3d SanWeiZuoBiao::pixelToCamera3D(const cv::Point2f& leftLightCenter, 
                                          const cv::Point2f& rightLightCenter, 
                                          const cv::Point2f& armorCenter) {
    // 1. 计算两灯条在图像上的像素距离
    double pixelDistance = cv::norm(leftLightCenter - rightLightCenter);
    if (pixelDistance < 1e-6) {  // 避免除零错误
        return cv::Vec3d(0.0, 0.0, 0.0);
    }

    // 2. 计算深度Z：Z = (焦距 * 实际宽度) / 像素距离（相似三角形原理）
    double Z = (camParam.fx * camParam.armorActualWidth) / pixelDistance;

    // 3. 计算X（水平方向）和Y（垂直方向）坐标
    // 像素坐标 -> 相机坐标公式：X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy
    double X = (armorCenter.x - camParam.cx) * Z / camParam.fx;
    double Y = (armorCenter.y - camParam.cy) * Z / camParam.fy;

    // 可选：若需Y轴向上（默认相机坐标系Y轴向下），取消下方注释
    // Y = -Y;

    return cv::Vec3d(X, Y, Z);
}
