/*
      ___           ___           ___           ___           ___           ___           ___
     /\  \         /\__\         /\  \         /\  \         /\__\         /\  \         /\  \
    /::\  \       /:/  /         \:\  \       /::\  \       /::|  |       /::\  \        \:\  \
   /:/\:\  \     /:/  /           \:\  \     /:/\:\  \     /:|:|  |      /:/\:\  \        \:\  \
  /::\~\:\  \   /:/  /  ___       /::\  \   /:/  \:\  \   /:/|:|  |__   /::\~\:\  \       /::\  \
 /:/\:\ \:\__\ /:/__/  /\__\     /:/\:\__\ /:/__/ \:\__\ /:/ |:| /\__\ /:/\:\ \:\__\     /:/\:\__\
 \/__\:\/:/  / \:\  \ /:/  /    /:/  \/__/ \:\  \ /:/  / \/__|:|/:/  / \:\~\:\ \/__/    /:/  \/__/
      \::/  /   \:\  /:/  /    /:/  /       \:\  /:/  /      |:/:/  /   \:\ \:\__\     /:/  /
      /:/  /     \:\/:/  /     \/__/         \:\/:/  /       |::/  /     \:\ \/__/     \/__/
     /:/  /       \::/  /                     \::/  /        /:/  /       \:\__\              2020
     \/__/         \/__/                       \/__/         \/__/         \/__/  by Vasily Yuryev

 Robot lane recognition system
 Python version: https://github.com/vas0x59/autonet_r1/blob/master/src/lane_detector/reg_line1_oneL.py

 */
#include "LaneDetector.h"

using namespace  std;

double get_clockwise_angle(cv::Point2i p)
{
    double angle = 0.0;
    angle = atan2(p.y,p.x);
    return angle;
}

LaneDetector::LaneDetector() {

}

void LaneDetector::setParameters(LaneDetectorParams params) {
    has_params_  = true;
    params.lane_roi = sort_src(params.lane_roi, params.image_w, params.image_h);
    params_ = params;
}

LaneDetectorOut LaneDetector::detect(cv::Mat image_in, cv::Mat &image_out) {
    if (has_params_) {
        LaneDetectorOut out;
        out.success = true;
        draw_roi(image_out);
        draw_points(image_out);
        detect_(image_in.clone());
        return out;
    }
    else {
        return LaneDetectorOut();
    }
}

void LaneDetector::detect_(cv::Mat image_in){

    cv::Mat per_transform = cv::getPerspectiveTransform(cv::Mat(params_.lane_roi), cv::Mat(gen_dst(image_in.cols, image_in.rows)));
    cv::warpPerspective(image_in, image_in, per_transform, cv::Size(image_in.cols, image_in.rows), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    cv::imshow("warped", image_in);
    cv::Mat filtered;
    cv::cvtColor(image_in, filtered, cv::COLOR_BGR2GRAY);

    cv::Mat mask=filtered < params_.thresh_val;
    filtered.setTo(cv::Scalar(params_.thresh_val),mask);
    cv::medianBlur(filtered, filtered, 5);

    cv::imshow("filtered", filtered);
    cv::adaptiveThreshold(filtered, filtered, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 5, 2);
    cv::medianBlur(filtered, filtered, 3);
    cv::imshow("filtered2", filtered);
    cv::waitKey(1);
}
bool compair_dist(cv::Point2f a,  cv::Point2f b, cv::Point2f c){
    return hypot(a.x - c.x, a.y - c.y) < hypot(b.x - c.x, b.y - c.y);
}

vector<cv::Point2f> LaneDetector::sort_src(vector<cv::Point2f> src, int w, int h){
//    vector<cv::Point2f> dst = {cv::Point2f(0, h), cv::Point2f(w, h), cv::Point2f(w, 0), cv::Point2f(0, 0)};
    cv::Point2f lb = *std::min_element(src.begin(), src.end(), [h](cv::Point2f a , cv::Point2f b){return compair_dist(a, b, cv::Point2f(0, (float)h));});
    cv::Point2f lu = *std::min_element(src.begin(), src.end(), [](cv::Point2f a , cv::Point2f b){return compair_dist(a, b, cv::Point2f(0, 0));});
    cv::Point2f rb = *std::min_element(src.begin(), src.end(), [w, h](cv::Point2f a , cv::Point2f b){return compair_dist(a, b, cv::Point2f((float)w, (float)h));});
    cv::Point2f ru = *std::min_element(src.begin(), src.end(), [w](cv::Point2f a , cv::Point2f b){return compair_dist(a, b, cv::Point2f((float)w, 0));});
    src[0] = lb; src[1] = rb; src[2] = ru; src[3] = lu;
    return src;
}

vector<cv::Point2f> LaneDetector::gen_dst(int w, int h){
    vector<cv::Point2f> dst = {cv::Point2f(0, h), cv::Point2f(w, h), cv::Point2f(w, 0), cv::Point2f(0, 0)};
    return dst;
}


void LaneDetector::draw_roi(cv::Mat &image_out) {
    cv::Mat points;
    cv::Mat(params_.lane_roi).convertTo(points, CV_32S);
    cv::polylines(image_out, points, true, cv::Scalar(0, 240, 0), 3);
}

void LaneDetector::draw_points(cv::Mat &image_out) {
//    cv::polylines(image_out, params_.lane_roi, true, cv::Scalar(0, 240, 0), 3);
    cv::circle(image_out, params_.lane_setpoint, 5, cv::Scalar(0, 180, 0), -1);
}

