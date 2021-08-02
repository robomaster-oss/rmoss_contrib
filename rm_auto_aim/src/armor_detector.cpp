/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/

#include "rm_auto_aim/armor_detector.hpp"

#include <cmath>
#include <rm_util/math.hpp>
#include <rm_util/types.hpp>
#include <rm_util/debug.hpp>


using namespace std;
using namespace cv;
using namespace rm_auto_aim;

ArmorDetector::ArmorDetector() { target_is_red_ = true; }
ArmorDetector::~ArmorDetector() { }

//颜色和亮度做与运算
int ArmorDetector::preImg(Mat src, Mat& dst)
{
    Mat bgr[3];
    Mat img_b, img_g, img_r;
    split(src, bgr); //将三个通道的像素值分离
    img_b = bgr[0];
    img_g = bgr[1];
    img_r = bgr[2];
    //RM_DEBUG(imshow("b", img_b));
    //RM_DEBUG(imshow("g", img_g));
    //RM_DEBUG(imshow("r", img_r));
    Mat imgTargetColor, imgBrightness;
    // default taget is red
    if (target_is_red_) {
        imgTargetColor = img_r - 0.2 * img_g - 0.8 * img_b;
        imgBrightness = img_r;
    } else {
        imgTargetColor = img_b - 0.8 * img_r - 0.2 * img_g;
        imgBrightness = img_b;
    }
    //目标颜色区域
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
    dilate(imgTargetColor, imgTargetColor, element);
    threshold(imgTargetColor, imgTargetColor, 72, 255, cv::THRESH_BINARY);
    RM_DEBUG(imshow("target_dialte_thre", imgTargetColor));
    //亮度区域
    GaussianBlur(imgBrightness, imgBrightness, Size(3, 3), 1);
    threshold(imgBrightness, imgBrightness, 150, 255, cv::THRESH_BINARY);
    RM_DEBUG(imshow("bright_thre", imgBrightness));
    //逻辑与，求交集
    bitwise_and(imgTargetColor, imgBrightness, dst);
    RM_DEBUG(imshow("dst", dst));
    return 0;
}

//通过图片上拟合的矩形轮廓，获取灯条，满足灯条要求的返回0,否则返回1
int ArmorDetector::getLightDescriptor(cv::RotatedRect r,
    LightDescriptor& light)
{
    light.lightArea = r.size.area();
    if (light.lightArea < 10 || light.lightArea > 3000) {
        return 1; //区域大小不满足
    }
    light.r = r;
    Point2f rect_points[4];
    r.points(rect_points);
    // line1:point0-point1
    float lineLen1 = cv::norm(rect_points[0] - rect_points[1]);
    // line2:point1-point2
    float lineLen2 = cv::norm(rect_points[1] - rect_points[2]);
    ;
    Point2f endPoint1, endPoint2; //直线条的端点
    if (lineLen1 > lineLen2) {
        // line1为最长边
        endPoint1 = (rect_points[0] + rect_points[3]) / 2;
        endPoint2 = (rect_points[1] + rect_points[2]) / 2;
        light.lightHight = lineLen1;
        light.lightWidth = lineLen2;
    } else {
        // line2为最长边
        endPoint1 = (rect_points[0] + rect_points[1]) / 2;
        endPoint2 = (rect_points[2] + rect_points[3]) / 2;
        light.lightHight = lineLen2;
        light.lightWidth = lineLen1;
    }
    light.lightRatioWH = light.lightWidth / light.lightHight;
    /*
    area:       195  133    65    41   44      1025   2970
    height:     25   21     13    10   8.8     73     124
    wh:         0.3  0.27   0.3   0.5  0.57    0.19   0.19
    sqrt(h)*wh  1.5  1.23   1.1   1.58 1.69    1.65   2.11    取3
    */
    if (light.lightRatioWH > 3 / sqrt(light.lightHight)) {
        return 2; //宽高比不符合，与区域面积（高度）有关，正相关
    }
    if (light.lightRatioWH > 1) {
        return 2; //宽高比不符合，与区域面积（高度）有关，正相关
    }

    //直线的上下端点
    if (endPoint1.y > endPoint2.y) {
        light.endPointYMax = endPoint1;
        light.endPointYMin = endPoint2;
    } else {
        light.endPointYMax = endPoint2;
        light.endPointYMin = endPoint1;
    }
    //求解灯条angel与x轴的夹角
    light.lightAngle = rm_util::calcInclineAngle(light.endPointYMin, light.endPointYMax);

    if (abs(light.lightAngle - 90) > 45) {
        return 3; //灯条倾斜度不符合
    }
    light.centerPoint = r.center;
    return 0;
}

int ArmorDetector::lightsMatch(LightDescriptor l1, LightDescriptor l2,
    ArmorDescriptor& armor)
{
    if (l1.centerPoint.x > l2.centerPoint.x) {
        armor.lightR = l1;
        armor.lightL = l2;
    } else {
        armor.lightR = l2;
        armor.lightL = l1;
    }
    armor.parallelErr = abs(armor.lightL.lightAngle - armor.lightR.lightAngle);
    armor.armorWidth = cv::norm(armor.lightL.centerPoint - armor.lightR.centerPoint);
    armor.armorHight = armor.lightL.lightHight > armor.lightR.lightHight
        ? armor.lightL.lightHight
        : armor.lightR.lightHight; //取两灯条最大值
    armor.armorRatioWH = armor.armorWidth / armor.armorHight;
    //条件1
    if (armor.armorHight < 60) {
        if (armor.parallelErr > 4) {
            return 1; //平行太大误差，否定
        }
    } else {
        if (armor.parallelErr > armor.armorHight / 15) {
            return 1; //平行太大误差，否定
        }
    }
    //条件2
    if (armor.armorRatioWH < 1 || armor.armorRatioWH > 5) {
        return 2; //宽高比不符合，否定
    }

    float innerAngle;
    if (armor.lightL.lightHight > armor.lightR.lightHight) {
        innerAngle = rm_util::calcInnerAngle(armor.lightL.centerPoint,
            armor.lightL.endPointYMin, armor.lightR.centerPoint); //取值0-180
    } else {
        innerAngle = rm_util::calcInnerAngle(armor.lightR.centerPoint,
            armor.lightR.endPointYMin, armor.lightL.centerPoint); //取值0-180
    }
    armor.innerAngleErr = abs(innerAngle - 90);
    //条件3
    if (armor.innerAngleErr > 20) {
        return 3; //装甲板正度（矩形内角=90）不符合，否定
    }
    armor.horizonLineAngle = rm_util::calcInclineAngle(armor.lightL.centerPoint, armor.lightR.centerPoint);
    //条件4
    if (abs(armor.horizonLineAngle - 90) < 60) {
        return 4; //装甲板倾斜不符合，否定
    }
    //修补
    if (armor.lightL.lightHight > armor.lightR.lightHight) {
        lightHightLong(armor.lightR, armor.armorHight); //修补右灯条
    } else {
        lightHightLong(armor.lightL, armor.armorHight); //补偿左灯条
    }
    //计算中心点
    armor.centerPoint = (armor.lightR.centerPoint + armor.lightL.centerPoint) / 2;
    //装甲板四个点
    armor.points[0] = armor.lightR.endPointYMin;
    armor.points[1] = armor.lightL.endPointYMin;
    armor.points[2] = armor.lightL.endPointYMax;
    armor.points[3] = armor.lightR.endPointYMax;
    return 0;
}

int ArmorDetector::process(cv::Mat img)
{
    mLights.clear();
    mArmors.clear();
    RM_DEBUG(waitKey(1));
    //预处理
    Mat grayImg;
    preImg(img, grayImg);
    //提取灯条
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(grayImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE,
        Point(0, 0)); //只检测外轮廓，保存轮廓上的所有点
    int ret;
    for (const auto& contour : contours) {
        if (contour.size() > 5) { //满足最小轮廓点数
            RotatedRect lightRect = fitEllipse(contour); //椭圆拟合
            LightDescriptor light;
            ret = getLightDescriptor(lightRect, light);
            if (ret == 0) {
                mLights.push_back(light); //加入灯条列表
            }
            // cout<<"light ret:"<<ret<<endl;
        }
    }
    // cout<<"light num:"<<mLights.size()<<endl;
    Mat drawing = Mat::zeros(img.size(), CV_8UC3);
    for (size_t i = 0; i < mLights.size(); i++){
        RM_DEBUG(rm_util::drawRotatedRect(drawing, mLights[i].r));
    }
    RM_DEBUG(imshow("light", drawing));
    //赋予id
    for (size_t i = 0; i < mLights.size(); i++)
        mLights[i].id = i;
    //匹配灯条
    if (mLights.size() < 2) {
        return 1;
    }
    //遍历匹配
    for (size_t i = 0; i < mLights.size(); i++) {
        for (size_t j = i + 1; j < mLights.size(); j++) {
            ArmorDescriptor armor;
            ret = lightsMatch(mLights[i], mLights[j], armor);
            if (ret == 0) {
                mArmors.push_back(armor);
                // printArmorDescriptor(armor);
            }
            // cout<<"match ret:"<<ret<<endl;
        }
    }
    if (mArmors.size() <= 0) {
        return 2;
    }
    for (size_t i = 0; i < mArmors.size(); i++) {
        RM_DEBUG(rm_util::draw4Point4f(img, mArmors[i].points));
    }
    RM_DEBUG(imshow("result", img));
    //返回处理结果
    return 0;
}

void ArmorDetector::lightHightLong(LightDescriptor& light,
    float height)
{ //灯条高度补偿
    float coff; //补偿比例系数
    coff = (float)((height - light.lightHight) / height * 0.45);
    cv::Point2f dVec = coff * (light.endPointYMax - light.endPointYMin);
    light.endPointYMax = light.endPointYMax + dVec;
    light.endPointYMin = light.endPointYMin - dVec;
}

vector<ArmorDescriptor> ArmorDetector::getArmorVector() { return mArmors; }

void ArmorDetector::printArmorDescriptor(ArmorDescriptor armor)
{
    cout << "-----------------------------------" << endl;
    cout << "装甲板高度" << armor.armorHight << endl;
    cout << "装甲板宽度" << armor.armorWidth << endl;
    cout << "装甲板宽高比" << armor.armorRatioWH << endl;
    cout << "装甲板灯条平行角度误差" << armor.parallelErr << endl;
    cout << "装甲板内角误差" << armor.innerAngleErr << endl;
    cout << "装甲板水平角度" << armor.horizonLineAngle << endl;
    cout << "灯条大小：" << armor.lightL.lightArea << "  "
         << armor.lightR.lightArea << endl;
    cout << "灯条高度：" << armor.lightL.lightHight << "  "
         << armor.lightR.lightHight << endl;
    cout << "灯条W/H：" << armor.lightL.lightRatioWH << "  "
         << armor.lightR.lightRatioWH << endl;
    cout << "-----------------------------------" << endl;
}

void ArmorDetector::setTargetColor(bool is_red)
{
    if (target_is_red_ != is_red) {
        target_is_red_ = is_red;
    }
}
