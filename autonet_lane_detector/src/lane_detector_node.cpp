//
// Created by vasily on 10.07.2020.
//
#include <math.h>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <ros/ros.h>
//#include <nodelet/nodelet.h>
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "LaneDetector.h"
#include "utils.h"

using cv::Mat;

struct LaneDetectorNodeParams {

};

class LaneDetectorNode{
private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    image_transport::Publisher debug_pub_;
    image_transport::CameraSubscriber img_sub_;
    ros::Publisher markers_pub_, vis_markers_pub_;
    ros::Subscriber map_markers_sub_;
    Mat camera_matrix_, dist_coeffs_;
public:
    void onInit() {
        ros::NodeHandle nh_;
        image_transport::ImageTransport it(nh_);
//        image_transport::ImageTransport it_priv(nh_priv_);
        img_sub_ = it.subscribeCamera("image_raw", 1, &LaneDetectorNode::imageCallback, this);
        debug_pub_ = it.advertise("debug", 1);

    }
private:
    void getParameters(){

    }
    void projectCoordinates(){

    }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr &cinfo)
    {
        parseCameraInfo(cinfo, camera_matrix_, dist_coeffs_);
        Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat out_image = image.clone();



        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id = msg->header.frame_id;
        out_msg.header.stamp = msg->header.stamp;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = out_image;

        debug_pub_.publish(out_msg.toImageMsg());
    }
};


int main(int argc, char **argv) {

    ros::init(argc, argv, "autonet_lane_detector");


    return 0;
}




