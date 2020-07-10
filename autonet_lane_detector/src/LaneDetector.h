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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#ifndef SRC_LANEDETECTOR_H
#define SRC_LANEDETECTOR_H

using namespace std;

/*
 * Parameter list:
 *
 * lane_set_point   -  Lane image setpoint (px)
 * stop_line_roi    -
 * window_size      -
 * window_count     -
 * window_thresh    -
 * thresh_val       - threshold for image filter
 * lane_roi         - roi to crop
 *
 */

struct LaneDetectorParams {
    int window_size;
    int window_count;
    int window_thresh;
    int thresh_val;
    int image_w;
    int image_h;
    vector<cv::Point2f> lane_roi;
    cv::Point2i  lane_setpoint;
//    vector<cv::Point2f> stop_line_roi;
    cv::Point2i  stop_line_setpoint;
};

struct LaneDetectorOut {
    float e1;
    float e2;
    cv::Point2i p1;
    cv::Point2i p2;
    cv::Point2i stop_line;
    bool stop_line_bool;
    bool success = false;
};


class LaneDetector {
public:
    LaneDetector();
//    LaneDetector();
    void setParameters(LaneDetectorParams params);
    LaneDetectorOut detect(cv::Mat image_in, cv::Mat &image_out);
private:
    void draw_roi(cv::Mat &image_out);
    void draw_points(cv::Mat &image_out);
    vector<cv::Point2f> gen_dst(int w, int h);
    vector<cv::Point2f> sort_src(vector<cv::Point2f> src, int w, int h);
    LaneDetectorParams params_;
    void detect_(cv::Mat image_in);
    bool has_params_ = false;
};


#endif //SRC_LANEDETECTOR_H
