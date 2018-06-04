#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <fstream>
#include <boost/assign/list_of.hpp>
#include "weitu.h"


// [[5.54893554e+03 0.00000000e+00 1.21751967e+03]
//  [0.00000000e+00 5.54163511e+03 1.18197642e+03]
//  [0.00000000e+00 0.00000000e+00 1.00000000e+00]]

sensor_msgs::CameraInfo get_default_camera_info_from_image(sensor_msgs::ImagePtr img){
    double fx = 5.54893554e+03;
    double fy = 5.54163511e+03;
    double tx = 1.21751967e+03;
    double ty = 1.18197642e+03;

    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = img->header.frame_id;
    // Fill image size
    cam_info_msg.height = img->height;
    cam_info_msg.width = img->width;
    ROS_INFO_STREAM("The image width is: " << img->width);
    ROS_INFO_STREAM("The image height is: " << img->height);
    // Add the most common distortion model as sensor_msgs/CameraInfo says
    cam_info_msg.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    cam_info_msg.D.resize(5, 0.0);
    // Give a reasonable default intrinsic camera matrix
    cam_info_msg.K = boost::assign::list_of(fx) (0.0) (tx)
                                           (0.0) (fy) (ty)
                                           (0.0) (0.0) (1.0);
    // Give a reasonable default rectification matrix
    cam_info_msg.R = boost::assign::list_of (1.0) (0.0) (0.0)
                                            (0.0) (1.0) (0.0)
                                            (0.0) (0.0) (1.0);
    // Give a reasonable default projection matrix
    cam_info_msg.P = boost::assign::list_of (fx) (0.0) (tx) (0.0)
                                            (0.0) (fy) (ty) (0.0)
                                            (0.0) (0.0) (1.0) (0.0);
    return cam_info_msg;
}

int main(int argc, char** argv){
    weitu::Camera camera;
    camera.open(0);
    int width_target = 0; 
    int height_target = 0;
    while (1) {
       cv::Mat rgb = camera.get();
       if(!rgb.empty()){
            width_target = rgb.cols;
            height_target = rgb.rows;
            break;
       }else{
           ROS_INFO("no img");
       }
    }


    ros::init(argc, argv, "weitu_image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~"); // to get the private params
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("weitu_cam", 1);

    std::string camera_name;
    _nh.param("camera_name", camera_name, std::string("weitu_cam"));
    ROS_INFO_STREAM("Camera name: " << camera_name);

    std::string frame_id;
    _nh.param("frame_id", frame_id, std::string("weitu_cam"));
    ROS_INFO_STREAM("Publishing with frame_id: " << frame_id);

    _nh.param("width", width_target, 0);
    _nh.param("height", height_target, 0);
    if (width_target != 0 && height_target != 0){
        ROS_INFO_STREAM("Forced image width is: " << width_target);
        ROS_INFO_STREAM("Forced image height is: " << height_target);
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    sensor_msgs::CameraInfo cam_info_msg;
    std_msgs::Header header;
    header.frame_id = frame_id;
    // camera_info_manager::CameraInfoManager cam_info_manager(nh, camera_name, "");
    // // Get the saved camera info if any
    // cam_info_msg = cam_info_manager.getCameraInfo();
    cam_info_msg.header = header;

    ros::Rate r(20);
    while (nh.ok()) {
        frame = camera.get();
        // if (pub.getNumSubscribers() > 0)
        {
            // Check if grabbed frame is actually filled with some content
            if(!frame.empty()) {
                // Flip the image if necessary
                msg = cv_bridge::CvImage(header, "mono8", frame).toImageMsg();
                // Create a default camera info if we didn't get a stored one on initialization
                if (cam_info_msg.distortion_model == ""){
                    // ROS_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
                    cam_info_msg = get_default_camera_info_from_image(msg);
                    // cam_info_manager.setCameraInfo(cam_info_msg);
                }
                // The timestamps are in sync thanks to this publisher
                pub.publish(*msg, cam_info_msg, ros::Time::now());
            }else{
                ROS_INFO("empty img!");
            }
            ros::spinOnce();
        }
        r.sleep();
    }
}