#ifndef ARMOR_MATCH_H
#define ARMOR_MATCH_H

#include "light_bar.h"
#include <vector>
#include <optional>

// 装甲板类：封装左右灯条、中心点及匹配逻辑
class Armor {
public:
    LightBar left_light;  // 左灯条（x坐标更小）
    LightBar right_light; // 右灯条（x坐标更大）
    cv::Point2f center;   // 装甲板中心点（两灯条中心中点）

    // 构造函数：通过左右灯条初始化，自动计算中心点
    Armor(const LightBar& left, const LightBar& right)
        : left_light(left), right_light(right) {
        center.x = (left.center.x + right.center.x) / 2.0f;
        center.y = (left.center.y + right.center.y) / 2.0f;
    }

    // 静态方法：从灯条列表中匹配目标装甲板（中间两个灯条组成）
    // 入参：检测到的灯条列表、裁剪后画面尺寸（用于筛选中间位置）
    static std::optional<Armor> matchLights(const std::vector<LightBar>& lights, const cv::Size& frame_size);

private:
    // 匹配阈值（可根据视频实际情况微调）
    static constexpr double ANGLE_THRESH = 10.0;          // 两灯条旋转角度差（≤10°）括号里的这些都是一些大概的默认值
    static constexpr double ASPECT_RATIO_DIFF_THRESH = 2.0; // 长宽比差值（≤0.5）
    static constexpr double AREA_RATIO_THRESH = 0.45;      // 面积比（最小/最大 ≥0.5）
    static constexpr int DISTANCE_MIN = 40;               // 中心最小距离（≥50像素）
    static constexpr int DISTANCE_MAX = 200;              // 中心最大距离（≤200像素）
    static constexpr int Y_DIFF_THRESH = 7;              // 中心y坐标差（≤20像素）
    static constexpr int X_DIFF_MIN = 20;                 // 中心x坐标差（≥30像素）
    //这里的数据我是针对test.mp4视频调的，可以根据不同视频进行调整

    // 内部函数：检查两个灯条是否符合装甲板匹配条件
    static bool isMatch(const LightBar& l1, const LightBar& l2);
};

#endif // ARMOR_MATCH_H
