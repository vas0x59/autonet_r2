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

 Robot lane recogintion system
 Python version: https://github.com/vas0x59/autonet_r1/blob/master/src/lane_detector/reg_line1_oneL.py

 */
#include "LaneDetector.h"

using namespace  std;


LaneDetector::LaneDetector() {

}

void LaneDetector::setParameters(LaneDetectorParams params) {
    has_params_  = true;
    params_ = params;
}

LaneDetectorOut LaneDetector::detect(cv::Mat image_in, cv::Mat &image_out) {
    if (has_params_) {
        LaneDetectorOut out;
        out.success = true;
        draw_roi(image_out);
        return out;
    }
    else {
        return LaneDetectorOut();
    }
}

void LaneDetector::draw_roi(cv::Mat &image_out) {
    cv::polylines(image_out, params_.lane_roi, true, cv::Scalar(0, 255, 0));
}

