//
// Created by vasily on 10.07.2020.
//

#ifndef SRC_UTILS_H
#define SRC_UTILS_H

static void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr& cinfo, cv::Mat& matrix, cv::Mat& dist)
{
    for (unsigned int i = 0; i < 3; ++i)
        for (unsigned int j = 0; j < 3; ++j)
            matrix.at<double>(i, j) = cinfo->K[3 * i + j];
    dist = cv::Mat(cinfo->D, true);
}


#endif //SRC_UTILS_H
