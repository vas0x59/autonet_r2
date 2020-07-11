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

#include <cmath>
#include <memory>
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

#include <xmlrpcpp/XmlRpcValue.h>

#include "LaneDetector.h"
#include "utils.h"

#include <autonet_lane_detector/LaneRes.h>

using cv::Mat;
using namespace std;

struct LaneDetectorNodeParams {
    geometry_msgs::Polygon lane_roi;
    string camera_frame;
    string lane_roi_frame;
    geometry_msgs::Point lane_setpoint;
    geometry_msgs::Point stop_setpoint;
    LaneDetectorParams detector_p;
};

class LaneDetectorNode {
private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    image_transport::Publisher debug_pub_;
    image_transport::CameraSubscriber img_sub_;
    ros::Publisher lane_roi_pub_, lane_res_pub_;
    Mat camera_matrix_, dist_coeffs_;
    LaneDetectorNodeParams params_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_ = ros::NodeHandle("~");
    bool first_time = true;
    bool has_parameters = false;
    LaneDetector ldetector;
public:
    void onInit() {
        ROS_INFO("LaneDetectorNode");
//        nh_priv_ = ros::NodeHandle("~");
        lane_roi_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/lane_detector/lane_roi", 2);
        lane_res_pub_ = nh_.advertise<autonet_lane_detector::LaneRes>("/lane_detector/lane_res", 2);
        br_ = std::make_unique<tf2_ros::TransformBroadcaster>();
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, nh_);

        image_transport::ImageTransport it(nh_);
//        image_transport::ImageTransport it_priv(nh_priv_);
        img_sub_ = it.subscribeCamera("image_raw", 1, &LaneDetectorNode::imageCallback, this);
        debug_pub_ = it.advertise("/lane_detector/debug", 1);

        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
        getParameters();
//        projectCoordinates();

    }

private:
    void pubLaneROI() {
        if (has_parameters && lane_roi_pub_.getNumSubscribers() > 0) {
            geometry_msgs::PolygonStamped polygon;
            polygon.header.frame_id = params_.lane_roi_frame;
            polygon.header.stamp = ros::Time::now();
            polygon.polygon = params_.lane_roi;
            lane_roi_pub_.publish(polygon);
        }
    }

    void getParameters() {
        std::vector<std::string> keys;
        nh_priv_.param("lane_roi_frame", params_.lane_roi_frame, string("base_link"));
        nh_priv_.param("camera_frame", params_.camera_frame, string("camera_frame"));
        nh_priv_.param("window_size", params_.detector_p.window_size, 10);
        nh_priv_.param("window_count", params_.detector_p.window_count, 5);
        nh_priv_.param("window_thresh", params_.detector_p.window_thresh, 50);
        nh_priv_.param("thresh_val", params_.detector_p.thresh_val, 110);

        XmlRpc::XmlRpcValue xml_lane_roi_temp;
        nh_priv_.getParam("lane_roi", xml_lane_roi_temp);
        if (xml_lane_roi_temp.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("param 'xml_lane_roi_temp' is not a list");
        }
        else if (xml_lane_roi_temp.size() != 4){
            ROS_ERROR("len of  'xml_lane_roi_temp' is not 4");
        }
        else {
            for (int i = 0; i < xml_lane_roi_temp.size(); ++i) {
                if (xml_lane_roi_temp[i].getType() != XmlRpc::XmlRpcValue::TypeArray) {
                    ROS_ERROR("xml_lane_roi_temp[%d] is not a list", i);
                } else {
                    if (xml_lane_roi_temp[i].size() != 3) {
                        ROS_ERROR("xml_lane_roi_temp[%d] size not 3 ", i);
                    } else if (
                            xml_lane_roi_temp[i][0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
                            xml_lane_roi_temp[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
                            xml_lane_roi_temp[i][2].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
                        ROS_ERROR("xml_lane_roi_temp[%d] is not double", i);
                    } else {
                        geometry_msgs::Point32 pnt;

                        pnt.x = static_cast<double>(xml_lane_roi_temp[i][0]);
                        pnt.y = static_cast<double>(xml_lane_roi_temp[i][1]);
                        pnt.z = static_cast<double>(xml_lane_roi_temp[i][2]);
                        params_.lane_roi.points.push_back(pnt);
                    }
                }
            }
        }
        cout << params_.lane_roi << endl;
        vector<double> lane_setpoint_t;
        nh_priv_.getParam("lane_setpoint", lane_setpoint_t);
        params_.lane_setpoint.x = lane_setpoint_t[0];
        params_.lane_setpoint.y = lane_setpoint_t[1];
        params_.lane_setpoint.z = lane_setpoint_t[2];
        vector<double> stop_setpoint_t;
        nh_priv_.getParam("stop_setpoint", stop_setpoint_t);
        params_.stop_setpoint.x = stop_setpoint_t[0];
        params_.stop_setpoint.y = stop_setpoint_t[1];
        params_.stop_setpoint.z = stop_setpoint_t[2];
        has_parameters = true;
        cout << "has_parameters" << endl;

    }

    void projectCoordinates() {
        cout << params_.camera_frame << " " << params_.lane_roi_frame << endl;
        geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(params_.camera_frame,
                                                                                params_.lane_roi_frame, ros::Time(0),
                                                                                ros::Duration(0.5));
        params_.detector_p.lane_roi = vector<cv::Point2f>(params_.lane_roi.points.size());
        vector<cv::Point3f> lane_roi_points_local(params_.lane_roi.points.size());
//        vector<cv::Point2f> lane_roi_points_img(params_.lane_roi.points.size());
        for (int i = 0; i < params_.lane_roi.points.size(); i++) {
            geometry_msgs::Point pnt_local;
            geometry_msgs::Point pnt_nlocal;
            pnt_nlocal.x = params_.lane_roi.points[i].x;
            pnt_nlocal.y = params_.lane_roi.points[i].y;
            pnt_nlocal.z = params_.lane_roi.points[i].z;
            tf2::doTransform(pnt_nlocal, pnt_local, transform);
            lane_roi_points_local[i].x = pnt_local.x;
            lane_roi_points_local[i].y = pnt_local.y;
            lane_roi_points_local[i].z = pnt_local.z;
        }
//        cout << "projectPoints" << endl;
        cv::Vec3f vec(0, 0, 0);
        cv::projectPoints(lane_roi_points_local, vec, vec, camera_matrix_, dist_coeffs_, params_.detector_p.lane_roi);

//        for (int i = 0; i < lane_roi_points_img.size(); i++)
//            params_.detector_p.lane_roi[i] = cv::Point2i(lane_roi_points_img[i]);

        geometry_msgs::Point pnt_local;
        vector<cv::Point3f> points_local(2);
        vector<cv::Point2f> points_img(2);
        tf2::doTransform(params_.lane_setpoint, pnt_local, transform);
        points_local[0].x = pnt_local.x;
        points_local[0].y = pnt_local.y;
        points_local[0].z = pnt_local.z;
        tf2::doTransform(params_.stop_setpoint, pnt_local, transform);
        points_local[1].x = pnt_local.x;
        points_local[1].y = pnt_local.y;
        points_local[1].z = pnt_local.z;
        cv::projectPoints(points_local, vec, vec, camera_matrix_, dist_coeffs_, points_img);
        params_.detector_p.lane_setpoint = points_img[0];
        params_.detector_p.stop_line_setpoint = points_img[1];
        ldetector.setParameters(params_.detector_p);
    }

//    geometry_msgs::Point getLanePoint(LaneDetectorOut res){
////        params_.lane_roi
//        vector<cv::Point2f> roi_2d;
//        for (auto pnt : params_.lane_roi.points){
//            roi_2d.push_back(cv::Point2f(pnt.x*100, pnt.y*100));
//        }
//        cv::Rect rect = cv::boundingRect(roi_2d);
//        float vect_l_w = max(rect.width, rect.height) / 100.0;
//        vector<cv::Point3f> obj_points(3);
//        obj_points[0] = cv::Point3f(vect_l_w / 2.0, 0, 0);
//        obj_points[1] = cv::Point3f(0, 0, 0);
//        obj_points[2] = cv::Point3f(-vect_l_w / 2.0, 0, 0);
//        vector<cv::Point3f> tvecs, rvecs;
//        cv::solveP3P(obj_points, res.points_img_norm, camera_matrix_, dist_coeffs_, rvecs, tvecs, cv::SOLVEPNP_P3P);
//        for (int i = 0; i < tvecs.size(); i++){
//            cout << "sol: " << i << " tvec: " << tvecs[i] << endl;
//        }
//        if (tvecs.size() > 0){
//            tf2::Quaternion q;
//            q.setRPY(0, 0, res.angle);
//            cv::Point3f pnt = tvecs[0];
//            geometry_msgs::TransformStamped transformStamped;
//            transformStamped.header.stamp = ros::Time::now();
//            transformStamped.header.frame_id = params_.camera_frame;
//            transformStamped.child_frame_id = "lane_estimated";
//            transformStamped.transform.translation.x = pnt.x;
//            transformStamped.transform.translation.y = pnt.y;
//            transformStamped.transform.translation.z = pnt.z;
//            transformStamped.transform.rotation.x = q.x();
//            transformStamped.transform.rotation.y = q.y();
//            transformStamped.transform.rotation.z = q.z();
//            transformStamped.transform.rotation.w = q.w();
//            br_->sendTransform(transformStamped);
//        }
//
//    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &cinfo) {
//        cout << "parseCameraInfo" << endl;
        parseCameraInfo(cinfo, camera_matrix_, dist_coeffs_);
        if (first_time) {
//            cout << "first time" << endl;
            params_.detector_p.image_w = msg->width;
            params_.detector_p.image_h = msg->height;
            projectCoordinates();
            first_time = false;
        } else {
//            cout << "IMAGE" << endl;
            Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            Mat out_image = image.clone();
            ros::Time st_t = ros::Time::now();
            LaneDetectorOut out = ldetector.detect(image, out_image, camera_matrix_, dist_coeffs_);
            ros::Time st_t2 = ros::Time::now();
            cout << "Detector time: " << (st_t2 - st_t).toSec() * 1000 << endl;

            autonet_lane_detector::LaneRes res_msg;
            res_msg.angle = out.angle;
            res_msg.roi = params_.lane_roi.points;
            res_msg.err_x_img_flat = out.err_x_img_flat;
            res_msg.set_point = params_.lane_setpoint;
            res_msg.set_point_frame = params_.lane_roi_frame;
            res_msg.err_diff_x_img_flat = out.err_diff_x_img_flat;

            cv_bridge::CvImage debug_msg;
            debug_msg.header.frame_id = msg->header.frame_id;
            debug_msg.header.stamp = msg->header.stamp;
            debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
            debug_msg.image = out_image;

            debug_pub_.publish(debug_msg.toImageMsg());
            lane_res_pub_.publish(res_msg);
            pubLaneROI();
        }
    }
};


int main(int argc, char **argv) {

    ros::init(argc, argv, "autonet_lane_detector");
    LaneDetectorNode ldn;
    ldn.onInit();
    ros::spin();
    return 0;
}




