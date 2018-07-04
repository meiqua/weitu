#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "weitu.h"
#include <tf/transform_broadcaster.h>

double qr_dist = 0.031;
// std::string camera_link = "camera_link";
std::string camera_link = "robot1_camera_eef_link";

std::string camera_marker = "camera_marker";
// std::string path_to_cam_info = "/tmp/calibrationdata/ost.yaml";

float K_data[] = {
5502.067210, 0.000000, 1184.077158,
0.000000, 5481.872057, 1098.097571,
0.000000, 0.000000, 1.000000
};
float D_data[] = {
-0.338621, 0.168072, -0.001953, -0.001642, 0.000000
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "weitu_finder_publisher");
    ros::NodeHandle _nh("~"); // to get the private params

    cv::Mat K_pnp = cv::Mat(3,3,CV_32FC1,K_data);
    cv::Mat D_pnp = cv::Mat(1,5,CV_32FC1,D_data);
    weitu::Camera camera;
    camera.open(0);

    tf::TransformBroadcaster br;
    ros::Rate r(20);

    // Timer timer;
    while (_nh.ok())
    {
        r.sleep();
        cv::Mat frame = camera.get();

        // if(!frame.empty()){
        // cv::Mat frame_;
        // cv::pyrDown(frame, frame_);
        // cv::pyrDown(frame_,frame_);
        // cv::imshow("frame", frame_);
        // cv::waitKey(1);
        // }

        // if (pub.getNumSubscribers() > 0)
        // if(false)
        // timer.reset();
        {
            if (!frame.empty())
            {
                std::vector<FinderPattern> pattern = qr_pattern::find(frame);

               std::vector<cv::Point2f> p4(4);
                cv::Mat to_show = frame.clone();
                cv::cvtColor(to_show, to_show, CV_GRAY2BGR);
                for (int i = 0; i < pattern.size(); i++)
                {
                    if(i<3){
                    int centerRow = int(pattern[i].getY());
                    int centerCol = int(pattern[i].getX());
                    p4[i] = cv::Point2f(centerCol, centerRow);

                cv::line(to_show, cv::Point(centerCol, centerRow-80), cv::Point(centerCol, centerRow+80)
                        , cv::Scalar(0, 0, 255), 2);
                cv::line(to_show, cv::Point(centerCol-80, centerRow), cv::Point(centerCol+80, centerRow)
                        , cv::Scalar(0, 0, 255), 2);
                    }
                }
                cv::pyrDown(to_show, to_show);
                cv::pyrDown(to_show, to_show);
                cv::imshow("test", to_show);
                cv::waitKey(1);

                // timer.out("1 time");

                if (pattern.size() < 3){
                    // ROS_INFO("pattern not found!");
                    continue;
                }
                if(pattern[2].count < pattern[0].count/2){
                    continue;
                }


                // find top left
                std::vector<std::vector<cv::Point2f>> v3;
                for (int i = 0; i < 3; i++)
                {
                    auto p_check = p4[i];
                    std::vector<cv::Point2f> v;
                    for (int j = 0; j < 3; j++)
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
                    double cos_value = v[0].dot(v[1]) / std::sqrt(v[0].x*v[0].x+v[0].y*v[0].y) / std::sqrt(v[1].x*v[1].x+v[1].y*v[1].y);
                    if (cos_value < min_cos)
                    {
                        min_cos = cos_value;
                        topleft_i = i;
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
                std::vector<cv::Point2d> p4_ordered(4);
                p4_ordered[0] = p4[topleft_i];
                p4_ordered[1] = p4[topright_i];
                p4_ordered[2] = p4[bottomleft_i];
                p4_ordered[3] = p4[1] + p4[2] - p4[0];

                //obj points
                std::vector<cv::Point3d> p4w_ordered(4);
                p4w_ordered[0] = cv::Point3d(-1, -1, 0);
                p4w_ordered[1] = cv::Point3d(1, -1, 0);
                p4w_ordered[2] = cv::Point3d(-1, 1, 0);
                p4w_ordered[3] = cv::Point3d(1, 1, 0);
                for(auto& p: p4w_ordered){
                    p *= qr_dist/2;
                }

                cv::Mat rvec, tvec, rotM; // use vector for easy access
                cv::Mat p4w_mat, p4_mat;
                cv::Mat(p4w_ordered).convertTo(p4w_mat, CV_64F);
                cv::Mat(p4_ordered).convertTo(p4_mat, CV_64F);

                // timer.out("2 time");
                cv::solvePnP(p4w_mat, p4_mat, K_pnp, D_pnp, rvec, tvec, false, cv::SOLVEPNP_P3P);
                // ROS_INFO("here we are!");
                std::cout << "z: " << tvec.at<double>(2,0) << std::endl;
                cv::Rodrigues(rvec, rotM);
                // rotM.convertTo(rotM, CV_64F);
                // tvec.convertTo(rvec, CV_64F);

                tf::Transform transform;
                transform.setOrigin(tf::Vector3(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0)));
                tf::Matrix3x3 tf_rotM(
                rotM.at<double>(0,0), rotM.at<double>(0,1), rotM.at<double>(0,2),
                rotM.at<double>(1,0), rotM.at<double>(1,1), rotM.at<double>(1,2),
                rotM.at<double>(2,0), rotM.at<double>(2,1), rotM.at<double>(2,2));
                transform.setBasis(tf_rotM);
                // transform = transform.inverse();
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), camera_link, camera_marker));
            }
            else
            {
                ROS_INFO("empty img!");
            }
            // ros::spinOnce();
        }
    }
}
