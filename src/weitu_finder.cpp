#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "weitu.h"
#include <tf/transform_broadcaster.h>

double qr_dist = 0;
std::string camera_link = "camera_link";
std::string camera_marker = "camera_marker";
std::string path_to_cam_info = "";

sensor_msgs::CameraInfo get_default_camera_info_from_image(sensor_msgs::ImagePtr img)
{
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
    cam_info_msg.K = boost::assign::list_of(fx)(0.0)(tx)(0.0)(fy)(ty)(0.0)(0.0)(1.0);
    // Give a reasonable default rectification matrix
    cam_info_msg.R = boost::assign::list_of(1.0)(0.0)(0.0)(0.0)(1.0)(0.0)(0.0)(0.0)(1.0);
    // Give a reasonable default projection matrix
    cam_info_msg.P = boost::assign::list_of(fx)(0.0)(tx)(0.0)(0.0)(fy)(ty)(0.0)(0.0)(0.0)(1.0)(0.0);
    return cam_info_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "weitu_finder_publisher");
    ros::NodeHandle _nh("~"); // to get the private params

    std::string camera_name;
    _nh.param("camera_name", camera_name, std::string("camera"));
    ROS_INFO_STREAM("Camera name: " << camera_name);

    std::string frame_id;
    _nh.param("frame_id", frame_id, std::string("camera_link"));
    ROS_INFO_STREAM("Publishing with frame_id: " << frame_id);

    sensor_msgs::CameraInfo cam_info_msg;
    std_msgs::Header header;
    header.frame_id = frame_id;
    camera_info_manager::CameraInfoManager cam_info_manager(_nh, camera_name, path_to_cam_info);
    // Get the saved camera info if any
    cam_info_msg = cam_info_manager.getCameraInfo();
    cam_info_msg.header = header;

    cv::Mat K_pnp;
    cv::Mat D_pnp;
    weitu::Camera camera;
    camera.open(0);
    while (1)
    {
        cv::Mat rgb = camera.get();
        if (!rgb.empty())
        {
            // camera info
            if (cam_info_msg.distortion_model == "")
            {
                sensor_msgs::ImagePtr msg;
                msg = cv_bridge::CvImage(header, "mono8", frame).toImageMsg();
                cam_info_msg = get_default_camera_info_from_image(msg);
                cam_info_manager.setCameraInfo(cam_info_msg);
            }
            K_pnp = cv::Mat(3,3,CV_32FC1,cam_info_msg.K);
            D_pnp = cv::Mat(1,5,CV_32FC1,cam_info_msg.D);
            break;
        }
        else
        {
            ROS_INFO("no img");
        }
    }

    tf::TransformBroadcaster br;
    ros::Rate r(20);
    while (_nh.ok())
    {
        cv::Mat frame = camera.get();
        // if (pub.getNumSubscribers() > 0)
        {
            if (!frame.empty())
            {
                std::vector<FinderPattern> pattern = qr_pattern::find(frame);
                if (pattern.size() != 3)
                {
                    continue;
                }

                std::vector<cv::Point2f> p4(4);
                for (int i = 0; i < pattern.size(); i++)
                {
                    int centerRow = int(pattern[i].getY());
                    int centerCol = int(pattern[i].getX());
                    p4[i] = cv::Point2f(centerCol, centerRow);
                }

                // find top left
                std::vector<std::vector<cv::Point2f>> v3;
                for (int i = 0; i < pattern.size(); i++)
                {
                    auto p_check = p4[i];
                    std::vector<cv::Point2f> v;
                    for (int j = 0; j < pattern.size(); j++)
                    {
                        if (j != i)
                        {
                            v.push_back(p4[j] - p_check);
                        }
                    }
                    v3.push_back(v);
                }
                double min_cos = std::numeric_limits<double>::max();
                int topleft_i = 0;
                for (int i = 0; i < v3.size(); i++)
                {
                    auto &v = v3[i];
                    double cos_value = v[0].dot(v[1]) / v[0].norm() / v[1].norm();
                    if (cos_value < min_cos)
                    {
                        min_cos = cos_value;
                        topleft_i = min_cos;
                    }
                }
                auto &topleft_v = v3[topleft_i];

                // find top right and bottom left
                int bottomleft_i = 0;
                int topright_i = 0;
                // in order
                for (int i = 0; i < 3; i++)
                {
                    if (i != topleft_i)
                    {
                        bottomleft_i = i;
                        break;
                    }
                }
                for (int i = 0; i < 3; i++)
                {
                    if (i != topleft_i && i != bottomleft_i)
                    {
                        topright_i = i;
                        break;
                    }
                }
                // counter clock
                if (topleft_v[0].cross(topleft_v[1]) > 0)
                {
                    int temp = bottomleft_i;
                    bottomleft_i = topright_i;
                    topright_i = temp;
                }

                // img points
                std::vector<cv::Point2f> p4_ordered(4);
                p4_ordered[0] = p4[topleft_i];
                p4_ordered[1] = p4[topright_i];
                p4_ordered[2] = p4[bottomleft_i];
                p4_ordered[3] = p4[1] + p4[2] - p4[0];

                //obj points
                std::vector<cv::Point3f> p4w_ordered(4);
                p4w_ordered[0] = cv::Point2d(-1, -1, 0);
                p4w_ordered[1] = cv::Point2d(1, -1, 0);
                p4w_ordered[2] = cv::Point2d(-1, 1, 0);
                p4w_ordered[3] = cv::Point2d(1, 1, 0);
                for(auto& p: p4w_ordered){
                    p *= qr_dist/2;
                }

                std::vector<float> rvec, tvec, rotM; // use vector for easy access
                cv::solvePnP(cv::Mat(p4w_ordered), cv::Mat(p4_ordered), K_pnp, D_pnp, rvec, tvec, false, SOLVEPNP_P3P);
                cv::Rodrigues(rvec, rotM);

                tf::Transform transform;
                transform.setOrigin(tf::Vector3(tvec[0], tvec[1], tvec[2]));
                tf::Matrix3x3 tf_rotM(rotM[0], rotM[1], rotM[2], rotM[3], rotM[4], rotM[5], rotM[6], rotM[7], rotM[8]);
                transform.setBasis(tf_rotM);
                // transform = transform.inverse();
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), camera_link, camera_marker));
            }
            else
            {
                ROS_INFO("empty img!");
            }
            ros::spinOnce();
        }
        r.sleep();
    }
}
