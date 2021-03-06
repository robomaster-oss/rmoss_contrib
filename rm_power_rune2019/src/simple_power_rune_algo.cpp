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
#include "rm_power_rune2019/simple_power_rune_algo.hpp"
#include "rm_common/debug.hpp"

using namespace std;
using namespace cv;
using namespace rm_power_rune2019;

SimplePowerRuneAlgo::SimplePowerRuneAlgo(std::shared_ptr<rm_common::MonoMeasureTool> mono_location_tool)
{
    //装甲板参数
    float realWidth, realHeight, half_x, half_y;
    realHeight = 15.1;
    realWidth = 26;
    half_x = realWidth / 2;
    half_y = realHeight / 2;
    //按坐标系顺序取
    realpoints_.emplace_back(half_x, half_y, 0);
    realpoints_.emplace_back(-half_x, half_y, 0);
    realpoints_.emplace_back(-half_x, -half_y, 0);
    realpoints_.emplace_back(half_x, -half_y, 0);
    is_reshoot_ = false;
    roi_ = Rect(0, 0, 1280, 512);
    roi_offset_ = Point2f(0, 0);
    //
    mono_location_tool_ = mono_location_tool;
    rune_state_machine_.clear();
    points_are_enough_ = false;
}

SimplePowerRuneAlgo::~SimplePowerRuneAlgo() { }

int SimplePowerRuneAlgo::process4large(cv::Mat& img)
{
    Mat roi_img = img(roi_);
    //识别
    int ret = rune_detector_.process(roi_img);
    if (ret != 0) {
        return -1; //未识别到，忽略跳过
    }
    std::vector<ArmorDescriptor> armors = rune_detector_.getAllArmors();
    //store center point of detected armors and fit circle
    if (!points_are_enough_) {
        for (size_t i = 0; i < armors.size(); i++) {
            stored_points_.push_back(armors[i].center);
        }
        if (stored_points_.size() > 50) {
            if (RunePredictionTool::fitCircle(stored_points_, rune_center_, rune_radius_)) {
                points_are_enough_ = true;
            } else {
                //re-store and re-fit if it's failed to fit circle
                stored_points_.clear();
            }
        }
    }
    //debug info
    for (size_t i = 0; i < armors.size(); i++) {
        rm_common::draw4Point4f(roi_img, armors[i].points);
    }
    RM_DEBUG(imshow("result", roi_img));
    // update FMS of power rune
    auto code = rune_state_machine_.update(armors);
    if (code == FmsResultCode::wait0) {
        if (is_reshoot_) {
            code = FmsResultCode::shoot;
        }
    }
    if ((code != FmsResultCode::shoot) || (!points_are_enough_)) {
        //状态机不正常状态,或存储的点不够导致未拟合出圆
        return -2;
    }
    //目标装甲板
    ArmorDescriptor armor = rune_state_machine_.getTargetArmor();
    //预测2d (旋转模型)
    Point2f pred_point = RunePredictionTool::rotatePoint2D(rune_center_, armor.center, 30 * CV_PI / 180);
    //3D target (get 3d point by using PnP,then pred 3d point by using 2d rotation model and tri)
    Point3f position;
    vector<Point2f> target_points;
    for (int i = 0; i < 4; i++) {
        target_points.push_back(armor.points[i] + roi_offset_);
    }
    mono_location_tool_->solvePnP4Points(target_points, realpoints_, position);
    //project pred_point to 3D
    target_ = mono_location_tool_->unproject(pred_point + roi_offset_, position.z);
    is_reshoot_ = false;
    //debug info
    rune_detector_.printArmorDescriptor(armor);
    printf("target point:(%f,%f,%f)..predict point(%f,%f,%f)\n", position.x, position.y, position.z, target_.x, target_.y, target_.z);
    //for prediction circle
    RM_DEBUG(cv::circle(roi_img, rune_center_, rune_radius_,cv::Scalar(0, 255, 0), 1, 8));
    RM_DEBUG(cv::circle(roi_img, rune_center_, 5, cv::Scalar(0, 255, 0), -1, 8));
    //for target armor
    RM_DEBUG(cv::circle(roi_img, armor.center, 5, rm_common::blue, -1, 8));
    rm_common::draw4Point4f(roi_img, armor.points);
    RM_DEBUG(cv::circle(roi_img, pred_point, 5, rm_common::red, -1, 8));
    RM_DEBUG(imshow("shoot_result", roi_img));
    RM_DEBUG(waitKey(1));
    return 0;
}

int SimplePowerRuneAlgo::process4small(cv::Mat& img)
{
    int ret;
    Mat roi_img = img(roi_);
    //识别
    ret = rune_detector_.process(roi_img);
    if (ret != 0) {
        return -1; //识别原因，不打击
    }
    std::vector<ArmorDescriptor> armors = rune_detector_.getAllArmors();
    //img debug
    for (size_t i = 0; i < armors.size(); i++) {
        rm_common::draw4Point4f(roi_img, armors[i].points);
    }
    RM_DEBUG(imshow("result", roi_img));
    // update FMS of power rune
    auto code = rune_state_machine_.update(armors);
    if (code != FmsResultCode::shoot) {
        //没有发生状态变化，无须打击
        return -2;
    }
    //目标装甲板
    ArmorDescriptor armor = rune_state_machine_.getTargetArmor();
    // PNP解算
    Point3f position;
    vector<Point2f> target_points;
    for (int i = 0; i < 4; i++) {
        target_points.push_back(armor.points[i] + roi_offset_);
    }
    mono_location_tool_->solvePnP4Points(target_points, realpoints_, position);
    target_ = position;
    is_reshoot_ = false;
    // debug
    // rune_detector_.printArmorDescriptor(armor);
    RM_DEBUG(cv::circle(roi_img, armor.center, 5, rm_common::blue, -1, 8));
    rm_common::draw4Point4f(roi_img, armor.points);
    RM_DEBUG(imshow("shoot_result", roi_img));
    RM_DEBUG(waitKey(1));

    return 0;
}

void SimplePowerRuneAlgo::clear4small()
{
    rune_state_machine_.clear();
}

void SimplePowerRuneAlgo::clear4large()
{
    rune_state_machine_.clear();
    stored_points_.clear();
    points_are_enough_ = false;
}

void SimplePowerRuneAlgo::setTargetColor(ArmorColor color)
{
    rune_detector_.setTargetColor(color);
}

cv::Point3f SimplePowerRuneAlgo::getShootTarget()
{
    return target_ / 100;
}

void SimplePowerRuneAlgo::setReShoot() { is_reshoot_ = true; }
