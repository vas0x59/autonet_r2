#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/features2d/features2d.hpp"

#include <xmlrpcpp/XmlRpcValue.h>
using namespace cv;
using namespace std;

void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{
    vector<float> err;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++)
    {  Point2f pt = points2.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
            if((pt.x<0)||(pt.y<0))	{
                status.at(i) = 0;
            }
            points1.erase (points1.begin() + (i - indexCorrection));
            points2.erase (points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }

    }

}


void featureDetection(Mat img_1, vector<Point2f>& points1)	{   //uses FAST as of now, modify parameters as necessary
    vector<KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}

class VisualOdometry{
public:
    void calcDelta(Mat frame_now, Mat &res, Mat cameraMatrix){
        cvtColor(frame_now, frame_now_gray, COLOR_BGR2GRAY);
        imshow("frame_now_gray", frame_now_gray);
        if (first_run){
            frame_prev = frame_now_gray;
            first_run = false;

        }

        if (p_prev.size() > 0)
        {
//            calcOpticalFlowPyrLK(frame_prev, frame_now_gray, p_prev, p_now, flow_status, flow_errs, Size(21, 21), 4);
            featureTracking(frame_prev, frame_now_gray, p_prev, p_now, flow_status);
            Mat mask;
            Mat tvec;
            Mat rvec;
            if (p_now.size() > 5 && p_prev.size() > 5) {
                E = findEssentialMat(p_now, p_prev, cameraMatrix, RANSAC, 0.999, 1.0, mask);
                recoverPose(E, p_now, p_prev, cameraMatrix, rvec, tvec, mask);
                cout << "rvec: " << rvec << "\n";
                cout << "tvec:" << tvec << endl;
                cout << "R_f: " << R_f << "\n";
                cout << "t_f:" << t_f << endl;

//                Point3f pnt = Point3f(tvec);
//                pose_h = pose_h + pnt;
                t_f = t_f + 1.0 * (R_f*tvec);
                cout <<"first\n";
                R_f = rvec*R_f;

            }
            cout << "p_now_o " << p_now.size() << " p_prev " << p_prev.size() << "\n";
            size_t i, k;
            for (i = k = 0; i < p_now.size(); i++)
            {
                if (!flow_status[i])
                    continue;
                p_now[k++] = p_now[i];
            }
            p_now.resize(k);
            cout << "\n";
            cout << "p_now " << p_now.size() << "\n";
            for (size_t i = 0; i < p_prev.size(); i++)
            {
                circle(res, p_prev[i], 3, Scalar(0, 0, 255), -1, 8);
            }
            for (size_t i = 0; i < p_now.size(); i++)
            {
                circle(res, p_now[i], 2, Scalar(255, 0, 0), -1, 8);
            }

            p_prev.resize(p_now.size());
//        p_prev = p_now;
            swap(p_prev, p_now);

        }
        if (p_prev.size() < MIN_POINTS_) // if enough tracking indices are dropped, calculate a new set
        {
            cout << "refound\n";
            featureDetection(frame_prev,p_prev);
            p_now = p_prev;
            if (p_prev.size() > 0) {
                featureTracking(frame_prev, frame_now_gray, p_prev, p_now, flow_status);
                cout << "refound 0000\n";
            }
            cout << "refound ok\n";
        }
        cout << "t_f " << t_f << "\n";
        frame_prev = frame_now_gray.clone();
    }
private:
    vector<Point2f> p_prev, p_now;
    Mat frame_prev, frame_now_gray, E;
    bool first_run = true;
    float MIN_POINTS_ = 20;
    int MAX_POINTS_ = 75;
    float MIN_DIST_POINTS_ = 5;
    vector<float> flow_errs;
    vector<unsigned char> flow_status;
    Point3f pose_h = Point3f(0, 0, 0);
    Mat R_f = Mat::zeros(3, 3, CV_64F) + 0.01;
    Mat t_f = Mat::zeros(3, 1, CV_64F);
};


Mat cameraMatrix, distCoeffs;
bool has_camera_info = false;

static void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo,
                            Mat &matrix, Mat &dist)
{
    for (unsigned int i = 0; i < 3; ++i)
        for (unsigned int j = 0; j < 3; ++j)
            matrix.at<double>(i, j) = cinfo->K[3 * i + j];

    for (unsigned int k = 0; k < cinfo->D.size(); k++)
        dist.at<double>(k) = cinfo->D[k];
}
void cinfoCallback(const sensor_msgs::CameraInfoConstPtr &cinfo)
{
    if (has_camera_info != true)
    {
        parseCameraInfo(cinfo, cameraMatrix, distCoeffs);
        has_camera_info = true;
        std::cout << "Cam OK: " << cameraMatrix << "dist: " << distCoeffs << std::endl;
    }
    // solver.se
    //  cout << "hello2";
}


VisualOdometry vio_calc;

void imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        // ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (has_camera_info == true)
    {
        Mat res;
        res = cv_ptr->image;
        vio_calc.calcDelta(cv_ptr->image, res, cameraMatrix);
        imshow("res", res);
        waitKey(1);
    }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "vio");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);


    cameraMatrix = Mat::zeros(3, 3, CV_64F);
    distCoeffs = Mat::zeros(8, 1, CV_64F);
    ros::Subscriber cinfo_sub = nh.subscribe("camera_info", 1, cinfoCallback);
    image_transport::Subscriber image_sub = it.subscribe("image_raw", 1, imageCb);
    ros::spin();
    return 0;
}
