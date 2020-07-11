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

using namespace std;

double get_clockwise_angle(cv::Point2i p) {
    double angle = 0.0;
    angle = atan2(p.y, p.x);
    return angle;
}

bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}
cv::Rect border_rect(cv::Rect rect, cv::Size bounds){
    cv::Rect rect2 = rect;
    if (rect2.x < 0){
        rect2.x = 0;
    }
    if (rect2.y < 0){
        rect2.y = 0;
    }
    if (rect2.x + rect2.width > bounds.width){
        rect2.x = bounds.width - rect2.width;
    }
    if (rect2.y + rect2.height > bounds.height){
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

LaneDetectorOut LaneDetector::detect(cv::Mat image_in, cv::Mat &image_out) {
    if (has_params_) {
        LaneDetectorOut out;
        out.success = true;
        draw_roi(image_out);
        draw_points(image_out);
        vector<cv::Point2f> window_means_out;
        cv::Mat per_transform_inv = cv::getPerspectiveTransform(cv::Mat(gen_dst(image_in.cols, image_in.rows, size_red_)),
                                                                cv::Mat(params_.lane_roi));
        detect_(image_in.clone(), window_means_out);
        if (window_means_out.size() > 0) {
            vector<cv::Point2f> points_un;
            cv::perspectiveTransform(window_means_out, points_un, per_transform_inv);
            draw_line_points(image_out, points_un);
        }
        return out;
    } else {
        return LaneDetectorOut();
    }
}

void LaneDetector::detect_(cv::Mat image_in, vector<cv::Point2f> &window_means_out, vector<cv::Point2f> &itog) {

    cv::Mat per_transform = cv::getPerspectiveTransform(cv::Mat(params_.lane_roi),
                                                        cv::Mat(gen_dst(image_in.cols, image_in.rows, size_red_)));

    cv::warpPerspective(image_in, image_in, per_transform, cv::Size(image_in.cols*size_red_, image_in.rows*size_red_), cv::INTER_LINEAR,
                        cv::BORDER_REPLICATE);
//    cv::imshow("warped", image_in);
    cv::Mat filtered;
    cv::cvtColor(image_in, filtered, cv::COLOR_BGR2GRAY);

    cv::Mat mask = filtered < params_.thresh_val;
    filtered.setTo(cv::Scalar(params_.thresh_val), mask);
    cv::medianBlur(filtered, filtered, 5);

//    cv::imshow("filtered", filtered);
    cv::adaptiveThreshold(filtered, filtered, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 5, 2);
//    cv::bi
//    filtered = 255 - filtered;
//    cv::medianBlur(filtered, filtered, 3);
    cv::dilate(filtered, filtered, cv::Mat::ones(cv::Size(2, 2), CV_8U));
//    cv::erode(filtered, filtered, cv::Mat::ones(cv::Size(2, 2), CV_8U));
//    cv::adaptiveThreshold(filtered, filtered, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
//                          cv::THRESH_BINARY_INV, 5, 2);
    cv::imshow("filtered2", filtered);
    vector<cv::Point2f> set_point_warped(1);
    cv::perspectiveTransform(vector<cv::Point2f>({params_.lane_setpoint}), set_point_warped, per_transform);

//    vector<vector<cv::Point>> contours;
//    vector<cv::Vec4i> hierarchy;
//    cv::findContours( filtered, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
//    cout << "size " << contours.size() << endl;
//    if (contours.size() > 0){
//        cv::drawContours(image_in, contours, -1, cv::Scalar(0, 180, 255), 2);
//        vector<vector<cv::Point2i>> all_points_windowed(params_.window_count);
//        int window_height = filtered.rows / params_.window_count;
//        vector<cv::Scalar> windows_rois;
//        for (int window = 0; window < params_.window_count; window++) {
//            int win_y1 = filtered.rows - (window + 1) * window_height;
//            int win_y2 = filtered.rows - (window) * window_height;
//            windows_rois.push_back(cv::Scalar(win_y1, win_y2));
//        }
//        cout << "Reshape" << windows_rois.size() << endl;
//        for (int f = 0; f < contours.size(); f++) {
//            for (int t = 0; t < contours[f].size(); t++) {
//                for (int i = 0; i < windows_rois.size(); i++) {
//                    cv::Point2i j = contours[f][t];
//                    if (j.y >= min(windows_rois[i][0], windows_rois[i][1]) && j.y <= max(windows_rois[i][0], windows_rois[i][1])){
//                        all_points_windowed[i].push_back(j);
////                        break;
//                    }
//                }
//
//            }
//        }
////        cv::fitLine();
//        cout << "size2 " << all_points_windowed.size() << endl;
//        vector<cv::Point2f> window_means;
//        for (int i = 0; i < params_.window_count; i++){
//            cv::Point2i zero(0.0f, 0.0f);
//            cv::Point2f sum  = cv::Point2f(std::accumulate(all_points_windowed[i].begin(), all_points_windowed[i].end(), zero));
//            cv::Point2f mean_point(sum.x / all_points_windowed[i].size(), filtered.rows - (i) * window_height);
////            cv::Point2f mean_point = cv::mean(all_points_windowed[i]);
//            window_means.push_back(mean_point);
//            cout << "window: " << i << " point: " <<  mean_point << endl;
//        }
//        window_means_out = window_means;
//        draw_line_points(image_in, window_means);
////        vector<cv::Point2f> points_means(2);
////        for (int i = 0; i < params_.window_count; i++){
////
////            if (i < params_.window_count / 2) {
////
////            }
////        }
////        cout << "size2 " << all_points_windowed.size() << endl;
//
//
//    }
//    cv::imshow("drawContours", image_in);



//    cv::Mat histogram;
//    vector<float> histogram_v;
//    histogram.zeros(cv::Size(filtered.cols, 1), CV_8U);
//    cv::Rect roi = cv::Rect(0, , w, h);
//    cv::Mat to_hist = filtered(cv::Range(filtered.rows / 4 * 2, filtered.rows / 4 * 4),
//                               cv::Range(0, filtered.cols));
//    cv::imshow("to_hist", to_hist);
//    cv::reduce(to_hist, histogram_v, 0, cv::REDUCE_AVG, CV_32F);
//    cv::imshow("histogram", histogram_v);
//
//    cv::waitKey(1);
//    histogram.
//    = histogram;
//    int IndWhitestColumnR =
//            std::max_element(histogram_v.begin(), histogram_v.end()) - histogram_v.begin();

//    cout << IndWhitestColumnR << endl;
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
        cv::Rect roi = border_rect(cv::Rect(right_win_x1, win_y1, right_win_x2 - right_win_x1, win_y2 - win_y1), cv::Size(filtered.cols, filtered.rows));
        cv::rectangle(image_in, roi, cv::Scalar(window * (255 / params_.window_count)), 2, 0);
        if (0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= filtered.cols && 0 <= roi.y && 0 <= roi.height &&
            roi.y + roi.height <= filtered.rows) {
            cv::Mat croped = filtered(roi);
//            cv::reduce(filtered(roi), histogram_v, 0, cv::REDUCE_AVG, CV_32F);
//            XCenterRightWindow =
//                    std::max_element(histogram_v.begin(), histogram_v.end()) - histogram_v.begin() + right_win_x1;
            vector<vector<cv::Point>> contours;
            vector<cv::Vec4i> hierarchy;
//            cv::findContours(filtered(roi), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//            cout << "window: " << window <<  "contours_size" << contours.size() << endl;
            cv::Point2f mean_pnt;
            if (1 > 0) {
//                std::sort(contours.begin(), contours.end(), compareContourAreas);
//                cout << contours.size() << endl;
                int cc = 0;
//                for (int i = 0; i < contours.size() && i < 2; i++){
//                    if (cv::contourArea(contours[i]) > params_.window_thresh) {
                    if ((cv::sum(croped)[0] / 255) > params_.window_thresh)
                    {
                        cv::Moments m = cv::moments(croped, true);
//                        m.
                        cv::Point2f temp;
                        temp.x = m.m10 / (m.m00 + 1e-7);
                        temp.y = m.m01 / (m.m00 + 1e-7);
                        mean_pnt += temp;
                        cc++;
                    }
//                }
                if (cc > 0) {
                    mean_pnt /= cc;
                    if (mean_pnt.x == NAN || mean_pnt.y == NAN || mean_pnt.x == -NAN || mean_pnt.y == -NAN) {
                        mean_pnt.x = roi.x + roi.width / 2;
                        mean_pnt.y = roi.y + roi.height / 2;
//                        cout << "PROBLEM" << endl;
                    } else {
                        mean_pnt.x += roi.x;
                        mean_pnt.y += roi.y;
                        window_means[window] = mean_pnt;
                        detected_windows+=1;
                    }
                }
                else {
                    mean_pnt.x = roi.x + roi.width / 2;
                    mean_pnt.y = roi.y + roi.height / 2;
                }
//                window_means[window] = mean_pnt;
//                cout << "window: " << window << " pnt:" << mean_pnt << endl;
                XCenterRightWindow = mean_pnt.x;

            }
            else {
                mean_pnt.x = roi.x + roi.width / 2;
                mean_pnt.y = roi.y + roi.height / 2;
            }
            window_means[window] = mean_pnt;
        }
//        cout << "window: " << window << " XCenterRightWindow: " << XCenterRightWindow << endl;
    }
    cout << "detected_windows" << detected_windows << endl;
    draw_line_points(image_in, window_means);
//    for (int i = 0; )
    cv::circle(image_in, cv::Point2i(set_point_warped[0]), 5, cv::Scalar(150, 180, 0), -1);
    window_means_out = window_means;
    cv::Scalar line;
    cv::fitLine(window_means, line, cv::DIST_L2, 0, 0.01, 0.01);
    //    lefty = int((-x*vy/vx) + y)
//    righty = int(((cols-x)*vy/vx)+y)
    float x1 = float((-line[3]*line[0]/line[1]) + line[2]);
    float x0 = float(((image_in.rows-line[3])*line[0]/line[1]) + line[2]);
//    cv::line(image_in,cv::Point2i(rightx, image_in.rows-1),cv::Point2i(leftx,0),cv::Scalar(200,0,200),2);
    cv::circle(image_in, cv::Point2i(x0, image_in.rows-1), 5, cv::Scalar(0, 0, 0), -1);
    cv::circle(image_in, cv::Point2i(line[2], line[3]), 5, cv::Scalar(0, 0, 150), -1);
    cv::circle(image_in, cv::Point2i(x1,0), 5, cv::Scalar(0, 0, 250), -1);

    cv::imshow("point", image_in);
    cv::waitKey(1);

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
    vector<cv::Point2f> dst = {cv::Point2f(0, h * size_red), cv::Point2f(w * size_red, h * size_red), cv::Point2f(w * size_red, 0), cv::Point2f(0, 0)};
    return dst;
}


void LaneDetector::draw_roi(cv::Mat &image_out) {
    cv::Mat points;
    cv::Mat(params_.lane_roi).convertTo(points, CV_32S);
    cv::polylines(image_out, points, true, cv::Scalar(0, 240, 0), 3);
}

void LaneDetector::draw_points(cv::Mat &image_out) {
//    cv::polylines(image_out, params_.lane_roi, true, cv::Scalar(0, 240, 0), 3);
    cv::circle(image_out, cv::Point2i(params_.lane_setpoint), 5, cv::Scalar(0, 180, 0), -1);
}

void LaneDetector::draw_line_points(cv::Mat &image_out, vector<cv::Point2f> points) {
    if (points.size() > 0) {
        for (int i = 0; i < points.size(); i++) {
            cv::circle(image_out, cv::Point2i(points[i]), 5, cv::Scalar(0, 150, 220), -1);
        }
    }
}

