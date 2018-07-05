#include "ros/ros.h"
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <ur_msgs/IOStates.h>
#include <ur_msgs/Digital.h>
#include <ur_msgs/SetIO.h>

#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "weitu.h"

geometry_msgs::Point robot1_wait_point;
geometry_msgs::Quaternion robot1_wait_quat;

geometry_msgs::Point robot2_wait_point;
geometry_msgs::Quaternion robot2_wait_quat;

int total_holes = 2;
std::vector<geometry_msgs::Point> hole_loc_record(total_holes);
std::vector<geometry_msgs::Quaternion> hole_quat_record(total_holes);

std::vector<geometry_msgs::Point> hole_loc_record2(total_holes);
std::vector<geometry_msgs::Quaternion> hole_quat_record2(total_holes);

bool use_cam = true;
double cam_z = 1.211108 - 1.211108 + 0.227 + 0.007;
std::vector<std::vector<double>> hole_detect_record_base(total_holes, {0, 0});
std::vector<std::vector<double>> cam_xy(total_holes);

bool data_from_file = true;
bool init_data_cali = true;
bool linear_cali = true;
double linear_cali_mat[4] = {1, 0, 0, 1};

bool init_success = false;
void record_init()
{
    // robot1_wait_point.x = 0.393060;
    // robot1_wait_point.y = -0.311998;
    // robot1_wait_point.z =  1.295736;
    // robot1_wait_quat = tf::createQuaternionMsgFromRollPitchYaw(3.14,0.0,0.798839);

    robot1_wait_point.x = 0.301654;
    robot1_wait_point.y = -0.536764;
    robot1_wait_point.z = 1.245408;
    robot1_wait_quat = tf::createQuaternionMsgFromRollPitchYaw(-0.283075, 1.544032, 2.879499);

    robot2_wait_point.x = 1.276787;
    robot2_wait_point.y = -0.163849;
    robot2_wait_point.z = 1.156983;
    robot2_wait_quat = tf::createQuaternionMsgFromRollPitchYaw(2.061372, 1.551123, -0.402698);

    // hole_loc_record[0].x =  0.686415;
    // hole_loc_record[0].y = -0.200301;
    // hole_loc_record[0].z =  1.211108;
    // hole_quat_record[0] = tf::createQuaternionMsgFromRollPitchYaw(3.14,-0.0,0.716477);

    // hole_loc_record[1].x = 0.805855;
    // hole_loc_record[1].y = -0.306341;
    // hole_loc_record[1].z = 1.211448;
    // hole_quat_record[1] = tf::createQuaternionMsgFromRollPitchYaw(3.14,0.0,0.716489);

    hole_loc_record[0].x = 0.688733;
    hole_loc_record[0].y = -0.193154;
    hole_loc_record[0].z = 1.183054;
    hole_quat_record[0] = tf::createQuaternionMsgFromRollPitchYaw(-0.132105, 1.553939, 3.032758);

    hole_loc_record[1].x = 0.818764;
    hole_loc_record[1].y = -0.308212;
    hole_loc_record[1].z = 1.190902;
    hole_quat_record[1] = tf::createQuaternionMsgFromRollPitchYaw(-0.002487, 1.551592, -3.122236);

    hole_loc_record2[0].x = 0.697769;
    hole_loc_record2[0].y = -0.148696;
    hole_loc_record2[0].z = 1.022616 + 0.004;
    hole_quat_record2[0] = tf::createQuaternionMsgFromRollPitchYaw(2.275835, 1.551514, -0.194501);

    hole_loc_record2[1].x = 0.812843;
    hole_loc_record2[1].y = -0.299524;
    hole_loc_record2[1].z = 1.026773 + 0.004;
    hole_quat_record2[1] = tf::createQuaternionMsgFromRollPitchYaw(2.276561, 1.554767, -0.194700);

    // hole_detect_record_base[0][0] = -0.010891;
    // hole_detect_record_base[0][1] = 0.006826;

    // hole_detect_record_base[1][0] = -0.005617;
    // hole_detect_record_base[1][1] = 0.002216;

    hole_detect_record_base[0][0] = 0.000337;
    hole_detect_record_base[0][1] = 0.005204;

    hole_detect_record_base[1][0] = 0.010629;
    hole_detect_record_base[1][1] = 0.008875;
}

void save_point_quat(geometry_msgs::Point &robot1_wait_point, std::string str_robot1_wait_point,
                     geometry_msgs::Quaternion &robot1_wait_quat,
                     std::string str_robot1_wait_quat, cv::FileStorage &fs)
{
    fs << (str_robot1_wait_point + "_x") << robot1_wait_point.x;
    fs << (str_robot1_wait_point + "_y") << robot1_wait_point.y;
    fs << (str_robot1_wait_point + "_z") << robot1_wait_point.z;
    fs << (str_robot1_wait_quat + "_x") << robot1_wait_quat.x;
    fs << (str_robot1_wait_quat + "_y") << robot1_wait_quat.y;
    fs << (str_robot1_wait_quat + "_z") << robot1_wait_quat.z;
    fs << (str_robot1_wait_quat + "_w") << robot1_wait_quat.w;
}

void load_point_quat(geometry_msgs::Point &robot1_wait_point, std::string str_robot1_wait_point,
                     geometry_msgs::Quaternion &robot1_wait_quat,
                     std::string str_robot1_wait_quat, cv::FileStorage &fs)
{
    robot1_wait_point.x = (double)(fs)[str_robot1_wait_point + "_x"];
    robot1_wait_point.y = (double)(fs)[str_robot1_wait_point + "_y"];
    robot1_wait_point.z = (double)(fs)[str_robot1_wait_point + "_z"];
    robot1_wait_quat.x = (double)(fs)[str_robot1_wait_quat + "_x"];
    robot1_wait_quat.y = (double)(fs)[str_robot1_wait_quat + "_y"];
    robot1_wait_quat.z = (double)(fs)[str_robot1_wait_quat + "_z"];
    robot1_wait_quat.w = (double)(fs)[str_robot1_wait_quat + "_w"];
}
void load_point_quat(geometry_msgs::Point &robot1_wait_point, std::string str_robot1_wait_point,
                     geometry_msgs::Quaternion &robot1_wait_quat,
                     std::string str_robot1_wait_quat, cv::FileNode &it)
{
    robot1_wait_point.x = (double)(it)[str_robot1_wait_point + "_x"];
    robot1_wait_point.y = (double)(it)[str_robot1_wait_point + "_y"];
    robot1_wait_point.z = (double)(it)[str_robot1_wait_point + "_z"];
    robot1_wait_quat.x = (double)(it)[str_robot1_wait_quat + "_x"];
    robot1_wait_quat.y = (double)(it)[str_robot1_wait_quat + "_y"];
    robot1_wait_quat.z = (double)(it)[str_robot1_wait_quat + "_z"];
    robot1_wait_quat.w = (double)(it)[str_robot1_wait_quat + "_w"];
}

void save_data()
{
    cv::FileStorage fs("data.yaml", cv::FileStorage::WRITE);
    save_point_quat(robot1_wait_point, "robot1_wait_point", robot1_wait_quat, "robot1_wait_quat", fs);
    save_point_quat(robot2_wait_point, "robot2_wait_point", robot2_wait_quat, "robot2_wait_quat", fs);

    fs << "hole_record"
       << "[";
    for (int i = 0; i < hole_loc_record.size(); i++)
    {
        fs << "{";
        save_point_quat(hole_loc_record[i], "loc", hole_quat_record[i], "quat", fs);
        save_point_quat(hole_loc_record2[i], "loc2", hole_quat_record2[i], "quat2", fs);
        fs << "}";
    }
    fs << "]";

    fs << "hole_detect_record_base"
       << "[";
    for (auto &p : hole_detect_record_base)
    {
        fs << "[";
        for (auto v : p)
        {
            fs << v;
        }
        fs << "]";
    }
    fs << "]";

    fs << "linear_cali_mat"
       << "[";
    fs << linear_cali_mat[0] << linear_cali_mat[1] << linear_cali_mat[2] << linear_cali_mat[3];
    fs << "]";
}

void load_data()
{
    cv::FileStorage fs("data.yaml", cv::FileStorage::READ);
    load_point_quat(robot1_wait_point, "robot1_wait_point", robot1_wait_quat, "robot1_wait_quat", fs);
    load_point_quat(robot2_wait_point, "robot2_wait_point", robot2_wait_quat, "robot2_wait_quat", fs);

    {
        cv::FileNode hole_record = fs["hole_record"];
        cv::FileNodeIterator it = hole_record.begin(), it_end = hole_record.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            auto fn = *it;
            load_point_quat(hole_loc_record[i], "loc", hole_quat_record[i], "quat", fn);
            load_point_quat(hole_loc_record2[i], "loc2", hole_quat_record2[i], "quat2", fn);
        }
    }

    {
        cv::FileNode hole_detect_record_base_ = fs["hole_detect_record_base"];
        cv::FileNodeIterator it = hole_detect_record_base_.begin(), it_end = hole_detect_record_base_.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            auto inner_seq = *it;
            auto inner_it = inner_seq.begin();
            auto inner_it_end = inner_seq.end();
            for (int j = 0; inner_it != inner_it_end; inner_it++, j++)
            {
                hole_detect_record_base[i][j] = (double)(*inner_it);
            }
        }
    }

    {
        cv::FileNode linear_cali_mat_ = fs["linear_cali_mat"];
        cv::FileNodeIterator it = linear_cali_mat_.begin(), it_end = linear_cali_mat_.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            linear_cali_mat[i] = (double)(*it);
        }
    }
}

const double jump_threshold = 5;
const double eef_step = 0.005;

bool robot1_go(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface &robot1_move_group)
{
    double fraction = 0;
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose robot1_pose1;
    robot1_pose1.orientation = q;
    robot1_pose1.position = p;
    waypoints.push_back(robot1_pose1);
    moveit_msgs::RobotTrajectory trajectory1;

    fraction = robot1_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
    if (fraction < 0.9)
    {
        return false;
    }

    ROS_WARN("Robot1 cartesian path. (%.2f%% achieved)", fraction * 100.0);
    ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan robot1_plan1;
    robot1_plan1.trajectory_ = trajectory1;
    robot1_move_group.execute(robot1_plan1);
    // ROS_INFO("robot1 is ready");
    // ros::Duration(1.0).sleep();
    return true;
}

bool robot2_go(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface &robot2_move_group)
{
    double fraction;
    std::vector<geometry_msgs::Pose> waypoints;

    robot2_move_group.setMaxVelocityScalingFactor(0.1);
    waypoints.clear();

    // geometry_msgs::Pose robot2_current_pose;
    // robot2_current_pose = robot2_move_group.getCurrentPose().pose;
    // robot2_current_pose.position.z += 0.1;
    // waypoints.push_back(robot2_current_pose);

    geometry_msgs::Pose screw_hole_pose1;
    screw_hole_pose1.position = p;
    screw_hole_pose1.orientation = q;
    geometry_msgs::Pose mid_pose;
    mid_pose = screw_hole_pose1;
    mid_pose.position.z += 0.1;
    waypoints.push_back(mid_pose);
    waypoints.push_back(screw_hole_pose1);
    moveit_msgs::RobotTrajectory trajectory3;

    fraction = robot2_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory3);
    if (fraction < 0.9)
    {
        return false;
    }

    ROS_WARN("Robot2 cartesian path. (%.2f%% achieved)", fraction * 100.0);
    ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan robot2_plan1;
    robot2_plan1.trajectory_ = trajectory3;
    robot2_move_group.execute(robot2_plan1);
    // ROS_INFO("robot2 is ready");
    // ros::Duration(1.0).sleep();
    return true;
}

bool robot2_go2(geometry_msgs::Point p, geometry_msgs::Quaternion q, moveit::planning_interface::MoveGroupInterface &robot2_move_group)
{
    double fraction;
    std::vector<geometry_msgs::Pose> waypoints;

    robot2_move_group.setMaxVelocityScalingFactor(0.1);
    waypoints.clear();
    geometry_msgs::Pose robot2_current_pose;
    robot2_current_pose = robot2_move_group.getCurrentPose().pose;

    geometry_msgs::Pose mid_pose;
    mid_pose = robot2_current_pose;
    mid_pose.position.z += 0.1;
    waypoints.push_back(mid_pose);
    geometry_msgs::Pose robot2_wait_pose;
    robot2_wait_pose.position = p;
    robot2_wait_pose.orientation = q;
    waypoints.push_back(robot2_wait_pose);
    moveit_msgs::RobotTrajectory trajectory3;

    fraction = robot2_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory3);
    if (fraction < 0.9)
    {
        return false;
    }

    ROS_WARN("Robot2 cartesian path. (%.2f%% achieved)", fraction * 100.0);
    ros::Duration(1.0).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan robot2_plan1;
    robot2_plan1.trajectory_ = trajectory3;
    robot2_move_group.execute(robot2_plan1);
    // ROS_INFO("robot2 is ready");
    // ros::Duration(1.0).sleep();
    return true;
}

std::vector<double> get_xy(double cam_z, ros::ServiceClient &robot2_io_states_client)
{
    ur_msgs::SetIO robot2_io_states_srv;
    robot2_io_states_srv.request.fun = 1;
    robot2_io_states_srv.request.pin = 7;
    robot2_io_states_srv.request.state = 1.0;
    robot2_io_states_client.call(robot2_io_states_srv);
    ROS_INFO("OPEN LIGHT");

    std::vector<double> result;

    auto detect_xy = hole_detect::find_hole(cam_z);
    if (detect_xy.size() > 0)
    {
        result = detect_xy;
    }

    robot2_io_states_srv.request.state = 0.0;
    robot2_io_states_client.call(robot2_io_states_srv);

    if (result.size() > 0)
    {
        ROS_INFO("base x : %lf", result[0]);
        ROS_INFO("base y : %lf", result[1]);
    }

    return result;
}

const double PI = 3.14159265359;
static const std::string ROBOT1_PLANNING_GROUP = "robot1_manipulator";
static const std::string ROBOT2_PLANNING_GROUP = "robot2_manipulator";
static const std::string ARMS_PLANNING_GROUP = "dual_arms";
std::vector<ur_msgs::Digital> robot1_digital_in, robot2_digital_in;
bool robot1_digital_in_flag = false;
bool robot2_digital_in_flag = false;

void robot1_io_callback(const ur_msgs::IOStates::ConstPtr &msg)
{
    robot1_digital_in = msg->digital_in_states;
    if (robot1_digital_in.size())
    {
        robot1_digital_in_flag = true;
    }
    else
    {
        ROS_INFO("robot1_digital_in is null!");
        robot1_digital_in_flag = false;
    }
}

void robot2_io_callback(const ur_msgs::IOStates::ConstPtr &msg)
{
    robot2_digital_in = msg->digital_in_states;
    if (robot2_digital_in.size())
    {
        robot2_digital_in_flag = true;
    }
    else
    {
        ROS_INFO("robot2_digital_in is null!");
        robot2_digital_in_flag = false;
    }
}

void printCurrentState(geometry_msgs::Pose current_pose)
{
    ROS_INFO("current pose x : %lf", current_pose.position.x);
    ROS_INFO("current pose y : %lf", current_pose.position.y);
    ROS_INFO("current pose z : %lf", current_pose.position.z);
}

void printCurrentState(geometry_msgs::Point current_pose)
{
    ROS_INFO("current pose x : %lf", current_pose.x);
    ROS_INFO("current pose y : %lf", current_pose.y);
    ROS_INFO("current pose z : %lf", current_pose.z);
}

void screw_hole(ros::ServiceClient &robot2_io_states_client)
{
    int setio_fun = 1;
    int setio_pin = 2;
    double setio_state = 1.0;
    ur_msgs::SetIO robot2_io_states_srv;
    robot2_io_states_srv.request.fun = setio_fun;
    robot2_io_states_srv.request.pin = setio_pin;
    robot2_io_states_srv.request.state = setio_state;
    robot2_io_states_client.call(robot2_io_states_srv);
    ros::Duration(1.0).sleep();
    robot2_io_states_srv.request.state = 0.0;
    robot2_io_states_client.call(robot2_io_states_srv);
    //2.吹气 DO1
    setio_fun = 1;
    setio_pin = 1;
    setio_state = 1.0;
    robot2_io_states_srv.request.fun = setio_fun;
    robot2_io_states_srv.request.pin = setio_pin;
    robot2_io_states_srv.request.state = setio_state;
    robot2_io_states_client.call(robot2_io_states_srv);
    ros::Duration(2.0).sleep();
    robot2_io_states_srv.request.state = 0.0;
    robot2_io_states_client.call(robot2_io_states_srv);
    //3.下压 DO3
    setio_fun = 1;
    setio_pin = 3;
    setio_state = 1.0;
    robot2_io_states_srv.request.fun = setio_fun;
    robot2_io_states_srv.request.pin = setio_pin;
    robot2_io_states_srv.request.state = setio_state;
    robot2_io_states_client.call(robot2_io_states_srv);
    ros::Duration(1.5).sleep();
    robot2_io_states_srv.request.state = 0.0;
    robot2_io_states_client.call(robot2_io_states_srv);
    //等待完成信号
    // while (1)
    // {
    //     if(robot2_digital_in_flag){
    //         if(robot2_digital_in.at(1).state == 1){
    //             ROS_INFO("screwing finished!");
    //             break;
    //         }
    //     }
    //     ros::Duration(1.0).sleep();
    // }
}

int main(int argc, char *argv[])
{
    // at least open one window for waitkey
    auto show_1 = cv::imread("1.png");
    cv::imshow("1", show_1);

    if (data_from_file)
    {
        load_data();
        init_success = true;
    }
    else
    {
        // record_init();
        // init_success = false;
    }

    ros::init(argc, argv, "dual_arm");
    ros::NodeHandle nh;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface robot1_move_group(ROBOT1_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface robot2_move_group(ROBOT2_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface arms_move_group(ARMS_PLANNING_GROUP);

    ROS_INFO("Reference frame of robot1: %s", robot1_move_group.getPlanningFrame().c_str());
    robot1_move_group.setEndEffectorLink("robot1_camera_eef_link");
    ROS_INFO("End effector link of robot1: %s", robot1_move_group.getEndEffectorLink().c_str());
    ROS_INFO("End effector link of robot2: %s", robot2_move_group.getEndEffectorLink().c_str());
    ros::Duration(1.0).sleep();
    tf::TransformListener listener;
    std::string target_frame = "table_ground";
    ros::ServiceClient robot1_io_states_client, robot2_io_states_client;
    ur_msgs::SetIO robot1_io_states_srv, robot2_io_states_srv;
    std::string robot1_io_states_srv_name, robot2_io_states_srv_name;
    robot1_io_states_srv_name = "robot1/ur_driver/set_io";
    robot2_io_states_srv_name = "robot2/ur_driver/set_io";
    robot1_io_states_client = nh.serviceClient<ur_msgs::SetIO>(robot1_io_states_srv_name);
    robot2_io_states_client = nh.serviceClient<ur_msgs::SetIO>(robot2_io_states_srv_name);
    int setio_fun;
    int setio_pin;
    double setio_state;
    std::string robot1_io_states_topic, robot2_io_states_topic;
    ros::Subscriber robot1_io_states_sub, robot2_io_states_sub;
    robot1_io_states_topic = "robot1/ur_driver/io_states";
    robot2_io_states_topic = "robot2/ur_driver/io_states";
    robot1_io_states_sub = nh.subscribe<ur_msgs::IOStates>(robot1_io_states_topic, 10, robot1_io_callback);
    robot2_io_states_sub = nh.subscribe<ur_msgs::IOStates>(robot2_io_states_topic, 10, robot2_io_callback);
    ros::Publisher robot1_urscript_pub_;
    std::string robot1_urscript_topic;
    robot1_urscript_topic = "robot1/ur_driver/URScript";
    robot1_urscript_pub_ = nh.advertise<std_msgs::String>(robot1_urscript_topic, 1);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    init_data_cali = false;
    if (init_data_cali)
    {
        std::cout << "init calibrating..." << std::endl;

        bool cam_bot_cali = true;
        if (cam_bot_cali)
        {
            ROS_INFO("OPEN LIGHT");
            robot2_io_states_srv.request.fun = 1;
            robot2_io_states_srv.request.pin = 7;
            robot2_io_states_srv.request.state = 1.0;
            robot2_io_states_client.call(robot2_io_states_srv);

            for (int current_hole = 0; current_hole < total_holes; current_hole++)
            {
                ROS_INFO(("calibrating hole " + std::to_string(current_hole) + ", cam_robot go").c_str());
                geometry_msgs::Pose robot1_current_pose;
                bool ret1 = true;

                if (init_success)
                {
                    ret1 = robot1_go(hole_loc_record[current_hole], hole_quat_record[current_hole], robot1_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "please align hole to center, \npress w a s d, q to break, \n1-9 for steps， others to refresh" << std::endl;

                {
                    weitu::Camera camera;
                    camera.open(0);

                    float bot_steps = 0.003;
                    while (1)
                    {
                        cv::Mat rgb = camera.get();
                        if (!rgb.empty())
                        {
                            cv::Mat smaller_rgb;
                            cv::pyrDown(rgb, smaller_rgb);
                            auto p = hole_detect::find(smaller_rgb);
                        }
                        char key = cv::waitKey(0);

                        robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                        auto temp = robot1_current_pose.position;

                        if (key == '1')
                        {
                            bot_steps = 0.001;
                        }
                        else if (key == '2')
                        {
                            bot_steps = 0.002;
                        }
                        else if (key == '3')
                        {
                            bot_steps = 0.003;
                        }
                        else if (key == '4')
                        {
                            bot_steps = 0.004;
                        }
                        else if (key == '5')
                        {
                            bot_steps = 0.005;
                        }
                        else if (key == '6')
                        {
                            bot_steps = 0.006;
                        }
                        else if (key == '7')
                        {
                            bot_steps = 0.007;
                        }
                        else if (key == '8')
                        {
                            bot_steps = 0.008;
                        }
                        else if (key == '9')
                        {
                            bot_steps = 0.009;
                        }
                        else if (key == '0')
                        {
                            bot_steps = 0.010;
                        }
                        else if (key == 'w')
                        {
                            temp.y += bot_steps;
                        }
                        else if (key == 's')
                        {
                            temp.y -= bot_steps;
                        }
                        else if (key == 'a')
                        {
                            temp.x += bot_steps;
                        }
                        else if (key == 'd')
                        {
                            temp.x -= bot_steps;
                        }
                        else if (key == 'z')
                        {
                            temp.z += bot_steps;
                        }
                        else if (key == 'c')
                        {
                            temp.z -= bot_steps;
                        }
                        else if (key == 'q')
                        {
                            break;
                        }
                        else
                        {
                            continue;
                        }
                        ret1 = robot1_go(temp, robot1_current_pose.orientation, robot1_move_group);
                        if (!ret1)
                        {
                            std::cout << "wrong cartesian" << std::endl;
                            return 0;
                        }
                    }
                }

                robot1_current_pose = robot1_move_group.getCurrentPose().pose;

                hole_loc_record[current_hole] = robot1_current_pose.position;
                hole_quat_record[current_hole] = robot1_current_pose.orientation;

                auto detect_xy = hole_detect::find_hole(cam_z);
                while (1)
                {
                    std::cout << "press space to next step, or others to detect again" << std::endl;
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                    else
                    {
                        detect_xy = hole_detect::find_hole(cam_z);
                    }
                }

                if (detect_xy.size() > 0)
                {
                    ROS_INFO(("hole_detect_record_base " + std::to_string(current_hole) +
                              "\nx: " + std::to_string(detect_xy[0]) +
                              "\ny: " + std::to_string(detect_xy[1]))
                                 .c_str());
                    hole_detect_record_base[current_hole] = detect_xy;
                }
                else
                {
                    std::cout << "nothing detect, wrong init of hole_detect_record_base" << std::endl;
                    return 0;
                }
            }
            if (linear_cali)
            {
                int current_hole = 0;
                std::cout << "linear calibrating..." << std::endl;
                double bound = 0.005;
                auto loc_temp = hole_loc_record[current_hole];

                cv::Mat cam_mat = cv::Mat(2, 2, CV_64FC1);
                cv::Mat phy_mat = cv::Mat(2, 2, CV_64FC1);

                // go 0.005, 0
                loc_temp.x += bound;
                bool ret1 = robot1_go(loc_temp, hole_quat_record[current_hole], robot1_move_group);
                if (!ret1)
                {
                    std::cout << "wrong cartesian" << std::endl;
                    return 0;
                }
                auto cam_delta_xy1 = get_xy(cam_z, robot2_io_states_client);
                cv::waitKey(2000);

                cam_delta_xy1[0] -= hole_detect_record_base[current_hole][0];
                cam_delta_xy1[1] -= hole_detect_record_base[current_hole][1];

                cam_mat.at<double>(0, 0) = cam_delta_xy1[0];
                cam_mat.at<double>(1, 0) = cam_delta_xy1[1];

                geometry_msgs::Pose robot1_current_pose;
                robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                phy_mat.at<double>(0, 0) = robot1_current_pose.x - hole_loc_record[current_hole].x;
                phy_mat.at<double>(1, 0) = robot1_current_pose.y - hole_loc_record[current_hole].y;

                // go 0, 0.005
                loc_temp.x = hole_loc_record[current_hole].x;
                loc_temp.y += bound;
                ret1 = robot1_go(loc_temp, hole_quat_record[current_hole], robot1_move_group);
                if (!ret1)
                {
                    std::cout << "wrong cartesian" << std::endl;
                    return 0;
                }
                auto cam_delta_xy2 = get_xy(cam_z, robot2_io_states_client);
                cv::waitKey(2000);

                cam_delta_xy2[0] -= hole_detect_record_base[current_hole][0];
                cam_delta_xy2[1] -= hole_detect_record_base[current_hole][1];

                cam_mat.at<double>(0, 1) = cam_delta_xy2[0];
                cam_mat.at<double>(1, 1) = cam_delta_xy2[1];

                robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                phy_mat.at<double>(0, 1) = robot1_current_pose.x - hole_loc_record[current_hole].x;
                phy_mat.at<double>(1, 1) = robot1_current_pose.y - hole_loc_record[current_hole].y;

                // relative, bot plus means hole minus
                auto linear_cv = (cam_mat.inv()) * (-phy_mat);

                linear_cali_mat[0] = cam_mat.at<double>(0, 0);
                linear_cali_mat[1] = cam_mat.at<double>(0, 1);
                linear_cali_mat[2] = cam_mat.at<double>(1, 0);
                linear_cali_mat[3] = cam_mat.at<double>(1, 1);

                std::cout << "liear_cali_mat: \n";
                std::cout << linear_cali_mat[0] << '\t' << linear_cali_mat[1] << '\n';
                std::cout << linear_cali_mat[2] << '\t' << linear_cali_mat[3] << '\n'
                          << std::endl;
            }
            {
                ROS_INFO("calibrating cam_robot wait pos");
                geometry_msgs::Pose robot1_current_pose;
                bool ret1 = true;

                if (init_success)
                {
                    ret1 = robot1_go(robot1_wait_point, robot1_wait_quat, robot1_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "please move to wanted wait pos, press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                robot1_current_pose = robot1_move_group.getCurrentPose().pose;
                robot1_wait_point = robot1_current_pose.position;
                robot1_wait_quat = robot1_current_pose.orientation;
            }
        }

        bool screw_bot_cali = false;
        if (screw_bot_cali)
        {
            for (int current_hole = 0; current_hole < total_holes; current_hole++)
            {
                ROS_INFO(("calibrating hole " + std::to_string(current_hole) + ", screw_robot go").c_str());
                geometry_msgs::Pose robot2_current_pose;
                bool ret1 = true;

                if (init_success)
                {
                    ret1 = robot2_go2(hole_loc_record2[current_hole], hole_quat_record2[current_hole], robot2_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                robot2_current_pose = robot2_move_group.getCurrentPose().pose;
                hole_loc_record2[current_hole] = robot2_current_pose.position;
                hole_quat_record2[current_hole] = robot2_current_pose.orientation;
            }

            {
                ROS_INFO("calibrating screw_robot wait pos");
                geometry_msgs::Pose robot2_current_pose;
                bool ret1 = true;

                if (init_success)
                {
                    ret1 = robot2_go(robot2_wait_point, robot2_wait_quat, robot2_move_group);
                    if (!ret1)
                    {
                        std::cout << "wrong cartesian" << std::endl;
                        return 0;
                    }
                }
                else
                {
                    ROS_INFO("no init pos provided, move robot by hand");
                }

                std::cout << "please move to wanted wait pos, press space to next step" << std::endl;
                while (1)
                {
                    char key = cv::waitKey(0);
                    if (key == ' ')
                    {
                        break;
                    }
                }
                robot2_current_pose = robot2_move_group.getCurrentPose().pose;
                robot2_wait_point = robot2_current_pose.position;
                robot2_wait_quat = robot2_current_pose.orientation;
            }
        }

        save_data();

        robot2_io_states_srv.request.state = 0.0;
        robot2_io_states_client.call(robot2_io_states_srv);

        ROS_INFO("init data cali done");
    }

    // while (1)
    {
        // ROS_INFO("start running");
        // //move above
        int current_hole = 0;
        bool ret1 = true;

        auto temp1 = hole_loc_record[current_hole];
        temp1.x += 0.006;
        temp1.y += 0.004;

        ret1 = robot1_go(temp1, hole_quat_record[current_hole], robot1_move_group);
        if (!ret1)
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }

        if (use_cam)
            cam_xy[current_hole] = get_xy(cam_z, robot2_io_states_client);

        //MOVE TO NEXT HOLE
        current_hole++;

        temp1 = hole_loc_record[current_hole];
        temp1.x += 0.006;
        temp1.y += 0.004;

        ret1 = robot1_go(temp1, hole_quat_record[current_hole], robot1_move_group);
        if (!ret1)
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }

        if (use_cam)
            cam_xy[current_hole] = get_xy(cam_z, robot2_io_states_client);

        //robot1 move away
        ret1 = robot1_go(robot1_wait_point, robot1_wait_quat, robot1_move_group);
        if (!ret1)
        {
            std::cout << "wrong cartesian" << std::endl;
            return 0;
        }

        // //robot2 move in
        current_hole = 0;
        if (cam_xy[current_hole].size() > 0 && use_cam)
        {

            double cam_delta_x = cam_xy[current_hole][0] - hole_detect_record_base[current_hole][0];
            double cam_delta_y = cam_xy[current_hole][1] - hole_detect_record_base[current_hole][1];

            double delta_x, delta_y;
            delta_x = linear_cali_mat[0] * cam_delta_x + linear_cali_mat[1] * cam_delta_y;
            delta_y = linear_cali_mat[2] * cam_delta_x + linear_cali_mat[3] * cam_delta_y;

            geometry_msgs::Pose robot1_current_pose;
            robot1_current_pose = robot1_move_group.getCurrentPose().pose;
            double bot1_dx = robot1_current_pose.x - hole_loc_record[current_hole].x;
            double bot1_dy = robot1_current_pose.y - hole_loc_record[current_hole].y;

            delta_x += bot1_dx;
            delta_y += bot1_dy;

            std::cout << "\n####################" << std::endl;
            std::cout << "delta x: " << delta_x << std::endl;
            std::cout << "delta y: " << delta_y << std::endl;
            std::cout << "####################\n" << std::endl;

            auto point_ = hole_loc_record2[current_hole];
            point_.x += delta_x;
            point_.y += delta_y;
            // robot2_go(point_, hole_quat_record2[current_hole], robot2_move_group);
        }

        robot2_go(hole_loc_record2[current_hole], hole_quat_record2[current_hole], robot2_move_group);
        // screw_hole(robot2_io_states_client);

        // //robot2 begin to screw
        // //1.分料 DO2

        // return 0;

        //next hole
        current_hole++;
        ROS_INFO("Move to next hole!");

        if (cam_xy[current_hole].size() > 0 && use_cam)
        {
            double cam_delta_x = cam_xy[current_hole][0] - hole_detect_record_base[current_hole][0];
            double cam_delta_y = cam_xy[current_hole][1] - hole_detect_record_base[current_hole][1];

            double delta_x, delta_y;
            delta_x = linear_cali_mat[0] * cam_delta_x + linear_cali_mat[1] * cam_delta_y;
            delta_y = linear_cali_mat[2] * cam_delta_x + linear_cali_mat[3] * cam_delta_y;

            geometry_msgs::Pose robot1_current_pose;
            robot1_current_pose = robot1_move_group.getCurrentPose().pose;
            double bot1_dx = robot1_current_pose.x - hole_loc_record[current_hole].x;
            double bot1_dy = robot1_current_pose.y - hole_loc_record[current_hole].y;

            delta_x += bot1_dx;
            delta_y += bot1_dy;

            std::cout << "\n####################" << std::endl;
            std::cout << "delta x: " << delta_x << std::endl;
            std::cout << "delta y: " << delta_y << std::endl;
            std::cout << "####################\n" << std::endl;

            auto point_ = hole_loc_record2[current_hole];
            point_.x += delta_x;
            point_.y += delta_y;
            // robot2_go(point_, hole_quat_record2[current_hole], robot2_move_group);
        }
        robot2_go(hole_loc_record2[current_hole], hole_quat_record2[current_hole], robot2_move_group);
        // screw_hole(robot2_io_states_client);
        //robot2 move away
        ROS_INFO("robot2 move away");
        robot2_go2(robot2_wait_point, robot2_wait_quat, robot2_move_group);

        ros::Duration(2.0).sleep();

        //robot1 gripper
    }
}
