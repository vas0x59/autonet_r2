//
// Created by vasily on 10.07.2020.
//
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
    LaneDetectorNodeParams params_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_ = ros::NodeHandle("~");
    bool first_time = true;
public:
    void onInit() {
        ROS_INFO("LaneDetectorNode");
//        nh_priv_ = ros::NodeHandle("~");

        br_ = std::make_unique<tf2_ros::TransformBroadcaster>();
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, nh_);

        image_transport::ImageTransport it(nh_);
//        image_transport::ImageTransport it_priv(nh_priv_);
        img_sub_ = it.subscribeCamera("image_raw", 1, &LaneDetectorNode::imageCallback, this);
        debug_pub_ = it.advertise("debug", 1);
        getParameters();
        projectCoordinates();

    }
private:
    void getParameters(){
        std::vector<std::string> keys;
        nh_priv_.param("lane_roi_frame", params_.lane_roi_frame, string("base_link"));
        nh_priv_.param("camera_frame", params_.camera_frame, string("camera_frame"));
        nh_priv_.param("window_size", params_.detector_p.window_size, 10);
        nh_priv_.param("window_count", params_.detector_p.window_count, 5);
        nh_priv_.param("window_thresh", params_.detector_p.window_thresh, 50);
        nh_priv_.param("thresh_val", params_.detector_p.thresh_val, 110);

        XmlRpc::XmlRpcValue xml_lane_roi_temp;
        nh_priv_.getParam("lane_roi", xml_lane_roi_temp);
        if( xml_lane_roi_temp.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
            ROS_ERROR("param 'xml_lane_roi_temp' is not a list");
        } else {
            for( int i=0; i<xml_lane_roi_temp.size(); ++i ) {
                if( xml_lane_roi_temp[i].getType() != XmlRpc::XmlRpcValue::TypeArray ) {
                    ROS_ERROR("xml_lane_roi_temp[%d] is not a list", i);
                } else {
                    if( xml_lane_roi_temp[i].size() != 3 ) {
                        ROS_ERROR("xml_lane_roi_temp[%d] size not 3 ", i);
                    }
                    else if(
                            xml_lane_roi_temp[i][0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
                            xml_lane_roi_temp[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
                            xml_lane_roi_temp[i][2].getType() != XmlRpc::XmlRpcValue::TypeDouble ){
                        ROS_ERROR("xml_lane_roi_temp[%d] is not double", i);
                    }
                    else {
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

    }
    void projectCoordinates(){
        geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(params_.camera_frame, params_.lane_roi_frame, ros::Time(0), ros::Duration(0.5));
        params_.detector_p.lane_roi = vector<cv::Point2i>(params_.lane_roi.points.size());
        vector<cv::Point3f> lane_roi_points_local(params_.lane_roi.points.size());
        for (int i = 0; i < params_.lane_roi.points.size(); i++){
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
        cv::Vec3f vec(0, 0, 0);
        cv::projectPoints(lane_roi_points_local, vec, vec, camera_matrix_, dist_coeffs_, params_.detector_p.lane_roi);

        geometry_msgs::Point pnt_local;
        vector<cv::Point3f> points_local(2);
        vector<cv::Point2i> points_img(2);
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
    }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr &cinfo)
    {
        parseCameraInfo(cinfo, camera_matrix_, dist_coeffs_);
        if (first_time){
            projectCoordinates();
        }
        else {
            Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            Mat out_image = image.clone();


            cv_bridge::CvImage out_msg;
            out_msg.header.frame_id = msg->header.frame_id;
            out_msg.header.stamp = msg->header.stamp;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = out_image;

            debug_pub_.publish(out_msg.toImageMsg());
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




