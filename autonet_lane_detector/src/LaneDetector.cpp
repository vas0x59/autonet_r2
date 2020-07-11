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

 */
#include "LaneDetector.h"

using namespace std;

double get_clockwise_angle(cv::Point2i p) {
    double angle = 0.0;
    angle = atan2(p.y, p.x);
    return angle;
}

bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
    double i = fabs(contourArea(cv::Mat(contour1)));
    double j = fabs(contourArea(cv::Mat(contour2)));
    return (i < j);
}

cv::Rect border_rect(cv::Rect rect, cv::Size bounds) {
    cv::Rect rect2 = rect;
    if (rect2.x < 0) {
        rect2.x = 0;
    }
    if (rect2.y < 0) {
        rect2.y = 0;
    }
    if (rect2.x + rect2.width > bounds.width) {
        rect2.x = bounds.width - rect2.width;
    }
    if (rect2.y + rect2.height > bounds.height) {
        rect2.y = bounds.height - rect2.height;
    }
    return rect2;
}

LaneDetector::LaneDetector() {

}

void LaneDetector::setParameters(LaneDetectorParams params) {
    has_params_ = true;
    params.lane_roi = sort_src(params.lane_roi, params.image_w, params.image_h);
    params_ = params;
    if (params_.window_count % 2 == 1) {
        params_.window_count += 1;
    }
}

LaneDetectorOut LaneDetector::detect(cv::Mat image_in, cv::Mat &image_out, cv::Mat cameraMatrix, cv::Mat distCoeffs) {
    if (has_params_) {

        return detect_(image_in, image_out, cameraMatrix, distCoeffs);
    } else {
        return LaneDetectorOut();
    }
}

LaneDetectorOut LaneDetector::detect_(cv::Mat image_in_or, cv::Mat &image_out, cv::Mat cameraMatrix, cv::Mat distCoeff) {
    LaneDetectorOut out;
    out.success = true;
    draw_roi(image_out);
    draw_points(image_out);
//    vector<cv::Point2f> window_means_out;
//    cv::Mat per_transform_inv = cv::getPerspectiveTransform(cv::Mat(gen_dst(image_in.cols, image_in.rows, size_red_)),
//                                                            cv::Mat(params_.lane_roi));
//    detect_(image_in.clone(), window_means_out);
//    if (window_means_out.size() > 0) {
//        vector<cv::Point2f> points_un;
//        cv::perspectiveTransform(window_means_out, points_un, per_transform_inv);
//        draw_line_points(image_out, points_un);
//    }
    cv::Mat per_transform_inv = cv::getPerspectiveTransform(
            cv::Mat(gen_dst(image_in_or.cols, image_in_or.rows, size_red_)),
            cv::Mat(params_.lane_roi));
    cv::Mat per_transform = cv::getPerspectiveTransform(cv::Mat(params_.lane_roi),
                                                        cv::Mat(gen_dst(image_in_or.cols, image_in_or.rows,
                                                                        size_red_)));
    cv::Mat image_warped;
    cv::warpPerspective(image_in_or, image_warped, per_transform,
                        cv::Size(image_in_or.cols * size_red_, image_in_or.rows * size_red_), cv::INTER_LINEAR,
                        cv::BORDER_REPLICATE);
//    cv::imshow("warped", image_in);
    cv::Mat filtered;
    cv::cvtColor(image_warped, filtered, cv::COLOR_BGR2GRAY);

    cv::Mat mask = filtered < params_.thresh_val;
    filtered.setTo(cv::Scalar(params_.thresh_val), mask);
    cv::medianBlur(filtered, filtered, 5);

//    cv::imshow("filtered", filtered);
    cv::adaptiveThreshold(filtered, filtered, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 5, 2);
//    cv::medianBlur(filtered, filtered, 3);
    cv::dilate(filtered, filtered, cv::Mat::ones(cv::Size(2, 2), CV_8U));;
//    cv::imshow("filtered2", filtered);
    vector<cv::Point2f> set_point_warped(1);
    cv::perspectiveTransform(vector<cv::Point2f>({params_.lane_setpoint}), set_point_warped, per_transform);


    cv::Moments m = cv::moments(filtered(cv::Range(filtered.rows / 4 * 3, filtered.rows / 4 * 4),
                                         cv::Range(0, filtered.cols)), true);
    int IndWhitestColumnR = int(m.m10 / (m.m00 + 1e-7));
    int window_height = filtered.rows / params_.window_count;
    int XCenterRightWindow = IndWhitestColumnR;
    vector<cv::Point2f> window_means(params_.window_count);
    int detected_windows = 0;
    for (int window = 0; window < params_.window_count; window++) {
        int win_y1 = filtered.rows - (window + 1) * window_height;
        int win_y2 = filtered.rows - (window) * window_height;
        int right_win_x1 = XCenterRightWindow - params_.window_size;
        int right_win_x2 = XCenterRightWindow + params_.window_size;
        cv::Rect roi = border_rect(cv::Rect(right_win_x1, win_y1, right_win_x2 - right_win_x1, win_y2 - win_y1),
                                   cv::Size(filtered.cols, filtered.rows));
//        cv::rectangle(image_warped, roi, cv::Scalar(0, 0, window * (255 / params_.window_count)), 2, 0);
        if (0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= filtered.cols && 0 <= roi.y && 0 <= roi.height &&
            roi.y + roi.height <= filtered.rows) {
            cv::Mat croped = filtered(roi);
            cv::Point2f mean_pnt;

            int cc = 0;
            if ((cv::sum(croped)[0] / 255) > params_.window_thresh) {
                cv::Moments m = cv::moments(croped, true);
//                        m.
                cv::Point2f temp;
                temp.x = m.m10 / (m.m00 + 1e-7);
                temp.y = m.m01 / (m.m00 + 1e-7);
                mean_pnt = temp;
                if (mean_pnt.x == NAN || mean_pnt.y == NAN || mean_pnt.x == -NAN || mean_pnt.y == -NAN) {
                    mean_pnt.x = roi.x + roi.width / 2;
                    mean_pnt.y = roi.y + roi.height / 2;
//                        cout << "PROBLEM" << endl;
                } else {
                    mean_pnt.x += roi.x;
                    mean_pnt.y += roi.y;
                    window_means[window] = mean_pnt;
                    detected_windows += 1;
                }
            } else {
                mean_pnt.x = roi.x + roi.width / 2;
                mean_pnt.y = roi.y + roi.height / 2;
            }
//                window_means[window] = mean_pnt;
//                cout << "window: " << window << " pnt:" << mean_pnt << endl;
            XCenterRightWindow = mean_pnt.x;
            window_means[window] = mean_pnt;
        }
    }
//    cout << "detected_windows" << detected_windows << endl;
//    draw_line_points(image_warped, window_means, cv::Scalar(0, 120, 220));
//    for (int i = 0; )
//    cv::circle(image_warped, cv::Point2i(set_point_warped[0]), 5, cv::Scalar(120, 150, 0), -1);
    cv::Scalar line;
    cv::fitLine(window_means, line, cv::DIST_L2, 0, 0.01, 0.01);
    //    lefty = int((-x*vy/vx) + y)
//    righty = int(((cols-x)*vy/vx)+y)
    float x1 = float((-line[3] * line[0] / line[1]) + line[2]);
    float x0 = float(((image_warped.rows - line[3]) * line[0] / line[1]) + line[2]);
    vector<cv::Point2f> itog_points(3);
    itog_points[2] = cv::Point2f(x0, image_warped.rows - 1);
    itog_points[1] = cv::Point2f(line[2], line[3]);
    itog_points[0] = cv::Point2f(x1, 0);
//    cv::circle(image_warped, cv::Point2i(x0, image_warped.rows - 1), 5, cv::Scalar(0, 0, 255), -1);
//    cv::circle(image_warped, cv::Point2i(line[2], line[3]), 5, cv::Scalar(0, 50, 255), -1);
//    cv::circle(image_warped, cv::Point2i(x1, 0), 5, cv::Scalar(0, 100, 250), -1);

//    cv::imshow("point", image_warped);


//    cv::waitKey(1);

    vector<cv::Point2f> itog_points_solve(3);
//    itog_points_solve[0]
    float vect_A = atan2(itog_points[0].y - itog_points[2].y, itog_points[0].x - itog_points[2].x);
    cv::Point2f p2 = cv::Point2f(line[2] + cos(vect_A) * (filtered.cols / 2), line[3] + sin(vect_A) * (filtered.cols / 2));
    cv::Point2f p1 = cv::Point2f(line[2] - cos(vect_A) * (filtered.cols / 2), line[3] - sin(vect_A) * (filtered.cols / 2));
    if (p2.y > p1.y)
        itog_points_solve[2] = p2;
    else
        itog_points_solve[2] = p1;
    itog_points_solve[1] = itog_points[1];
    if (p1.y < p2.y)
        itog_points_solve[0] = p1;
    else
        itog_points_solve[0] = p2;

    vector<cv::Point2f> points_un;
    cv::perspectiveTransform(window_means, points_un, per_transform_inv);
    draw_line_points(image_out, points_un, cv::Scalar(0, 120, 220));
    vector<cv::Point2f> itog_unwarp(3);
    cv::perspectiveTransform(itog_points, itog_unwarp, per_transform_inv);
    draw_line_points(image_out, itog_unwarp, cv::Scalar(0, 0, 255));
    cv::circle(image_out, cv::Point2i(itog_unwarp[0]), 5, cv::Scalar(0, 150, 80), -1);
    vector<cv::Point2f> itog_points_solve_unwrap(3);
    cv::perspectiveTransform(itog_points_solve, itog_points_solve_unwrap, per_transform_inv);
    out.angle = vect_A;
    out.points_img_flat = itog_points;
    out.points_img = itog_unwarp;
    out.points_img_norm = itog_points_solve_unwrap;
    out.err_x_img_flat = (set_point_warped[0].x - itog_points[1].x) / float(filtered.cols / 2);
    out.err_diff_x_img_flat = (itog_points[2].x - itog_points[0].x) / float(filtered.cols / 2);
    return out;
}

bool compair_dist(cv::Point2f a, cv::Point2f b, cv::Point2f c) {
    return hypot(a.x - c.x, a.y - c.y) < hypot(b.x - c.x, b.y - c.y);
}

vector<cv::Point2f> LaneDetector::sort_src(vector<cv::Point2f> src, int w, int h) {
//    vector<cv::Point2f> dst = {cv::Point2f(0, h), cv::Point2f(w, h), cv::Point2f(w, 0), cv::Point2f(0, 0)};
    cv::Point2f lb = *std::min_element(src.begin(), src.end(), [h](cv::Point2f a, cv::Point2f b) {
        return compair_dist(a, b, cv::Point2f(0, (float) h));
    });
    cv::Point2f lu = *std::min_element(src.begin(), src.end(), [](cv::Point2f a, cv::Point2f b) {
        return compair_dist(a, b, cv::Point2f(0, 0));
    });
    cv::Point2f rb = *std::min_element(src.begin(), src.end(), [w, h](cv::Point2f a, cv::Point2f b) {
        return compair_dist(a, b, cv::Point2f((float) w, (float) h));
    });
    cv::Point2f ru = *std::min_element(src.begin(), src.end(), [w](cv::Point2f a, cv::Point2f b) {
        return compair_dist(a, b, cv::Point2f((float) w, 0));
    });
    src[0] = lb;
    src[1] = rb;
    src[2] = ru;
    src[3] = lu;
    return src;
}

vector<cv::Point2f> LaneDetector::gen_dst(int w, int h, float size_red) {
    vector<cv::Point2f> dst = {cv::Point2f(0, h * size_red), cv::Point2f(w * size_red, h * size_red),
                               cv::Point2f(w * size_red, 0), cv::Point2f(0, 0)};
    return dst;
}


void LaneDetector::draw_roi(cv::Mat &image_out) {
    cv::Mat points;
    cv::Mat(params_.lane_roi).convertTo(points, CV_32S);
    cv::polylines(image_out, points, true, cv::Scalar(0, 100, 255), 4);
}

void LaneDetector::draw_points(cv::Mat &image_out) {
//    cv::polylines(image_out, params_.lane_roi, true, cv::Scalar(0, 240, 0), 3);
    cv::circle(image_out, cv::Point2i(params_.lane_setpoint), 6, cv::Scalar(120, 150, 0), -1);
}

void LaneDetector::draw_line_points(cv::Mat &image_out, vector<cv::Point2f> points, cv::Scalar color) {
    if (points.size() > 0) {
        for (int i = 0; i < points.size(); i++) {
            cv::circle(image_out, cv::Point2i(points[i]), 6, color, -1);
        }
    }
}

