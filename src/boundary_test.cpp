//
// Created by observer0724 on 2/24/17.
//

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <cwru_davinci_traj_streamer/trajAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <time.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include <iostream>
#include <string>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <stdlib.h>
#include <stdio.h>
#include "opencv/cv.hpp"
#include <cwru_davinci_kinematics/davinci_kinematics.h>
#include <fstream>
#include <sstream>
using namespace cv;
bool freshImage;
bool freshCameraInfo;

using namespace std;
double weight_data = 0.0;
bool g_server_goal_completed= false;




void doneCb(const actionlib::SimpleClientGoalState& state,
            const cwru_davinci_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
    g_server_goal_completed = true;
}

void scalerCallback(const std_msgs::Float64& weight)
{
    ROS_INFO("received value is: %f",weight.data);

    weight_data = weight.data;
}

void newImageCallback(const sensor_msgs::ImageConstPtr& msg, cv::Mat* outputImage)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
        outputImage[0] = cv_ptr->image;
        freshImage = true;

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg ->encoding.c_str());
    }

}






int main(int argc, char **argv) {
    //Set up our node.
    ros::init(argc, argv, "take_picture");
    ros::NodeHandle nh;



    Mat rawImage_left = cv::Mat::zeros(640, 920, CV_8UC3);
    Mat rawImage_right = cv::Mat::zeros(640, 920, CV_8UC3);
    Mat save_image = cv::Mat::zeros(640, 920, CV_8UC3);


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub_l = it.subscribe("/davinci_endo/left/image_raw", 1,
    boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));
    image_transport::Subscriber img_sub_r = it.subscribe("/davinci_endo/right/image_raw", 1,
    boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

    Mat seg_left;
    Mat seg_right;
    ROS_INFO("1");
    vector<Eigen::Vector3d> x_vecs1, y_vecs1, z_vecs1, tip_origins1;
    vector<Eigen::Vector3d> x_vecs2, y_vecs2,z_vecs2, tip_origins2;
    Eigen::Vector3d x_vec1, y_vec1,z_vec1, tip_origin1;
    Eigen::Vector3d x_vec2, y_vec2,z_vec2, tip_origin2;
    vector<double> gripper_angs1, gripper_angs2;
    double gripper_ang1, gripper_ang2;
    vector<double> arrival_times;
    double arrival_time;
    ROS_INFO("2");
    x_vec1<<1,0,0;
    y_vec1<<0,0,-1;
    z_vec1<<0,1,0;
    y_vec1 = z_vec1.cross(x_vec1);
    tip_origin1<<0,0,0;
    ROS_INFO("3");
    x_vecs1.push_back(x_vec1);
    y_vecs1.push_back(y_vec1);
    z_vecs1.push_back(z_vec1);
    tip_origins1.push_back(tip_origin1);
    gripper_ang1 = 0.0;
    gripper_ang2 = 0.0;
		gripper_angs1.push_back(gripper_ang1);
    gripper_angs2.push_back(gripper_ang2);
    x_vecs1.push_back(x_vec1);
    y_vecs1.push_back(y_vec1);
    z_vecs1.push_back(z_vec1);
    tip_origins1.push_back(tip_origin1);
    gripper_ang1 = 0.0;
    gripper_ang2 = 0.0;
		gripper_angs1.push_back(gripper_ang1);
    gripper_angs2.push_back(gripper_ang2);
    x_vecs1.push_back(x_vec1);
    y_vecs1.push_back(y_vec1);
    z_vecs1.push_back(z_vec1);
    tip_origins1.push_back(tip_origin1);
    gripper_ang1 = 0.0;
    gripper_ang2 = 0.0;
		gripper_angs1.push_back(gripper_ang1);
    gripper_angs2.push_back(gripper_ang2);
    x_vecs1.push_back(x_vec1);
    y_vecs1.push_back(y_vec1);
    z_vecs1.push_back(z_vec1);
    tip_origins1.push_back(tip_origin1);
    gripper_ang1 = 0.0;
    gripper_ang2 = 0.0;
		gripper_angs1.push_back(gripper_ang1);
    gripper_angs2.push_back(gripper_ang2);

    ROS_INFO("4");
    x_vec2<<0.60615, 0.0384314,  0.794421;
    z_vec2<<0.791841, -0.122883, -0.598237;
    tip_origin2<<0.154536 ,-0.0435045  ,-0.121711;
    x_vecs2.push_back(x_vec2);
    z_vecs2.push_back(z_vec2);
    y_vec2 = z_vec2.cross(x_vec2);
    y_vecs2.push_back(y_vec2);
    tip_origins2.push_back(tip_origin2);
    ROS_INFO("5");
    x_vec2<<0.450551,  0.870663, -0.197359;
    z_vec2<<0.402415 ,  -0.3954 ,-0.825664;
    tip_origin2<<0.0852329,-0.0816097,-0.140307;

    x_vecs2.push_back(x_vec2);
    z_vecs2.push_back(z_vec2);
    y_vec2 = z_vec2.cross(x_vec2);
    y_vecs2.push_back(y_vec2);
    tip_origins2.push_back(tip_origin2);


    x_vec2<<0.894881 , 0.0381243  , 0.444673;
    z_vec2<<0.446283 ,-0.0863354  ,-0.890718;
    tip_origin2<<0.121268, 0.0163573, -0.134332;

    x_vecs2.push_back(x_vec2);
    z_vecs2.push_back(z_vec2);
    y_vec2 = z_vec2.cross(x_vec2);
    y_vecs2.push_back(y_vec2);
    tip_origins2.push_back(tip_origin2);


    x_vec2<<0.72405 , 0.031784 , 0.689014;
    z_vec2<<0.673484,  0.183066 ,-0.716175;
    tip_origin2<<0.0663062, -0.0279604,  -0.139117;


    x_vecs2.push_back(x_vec2);
    z_vecs2.push_back(z_vec2);
    y_vec2 = z_vec2.cross(x_vec2);
    y_vecs2.push_back(y_vec2);
    tip_origins2.push_back(tip_origin2);

    arrival_time = 3;
    ROS_INFO("6");
    Eigen::Matrix3d R;
    Eigen::Affine3d des_gripper_affine1,des_gripper_affine2;
    vector<Eigen::Affine3d> gripper1_affines,gripper2_affines;
    for (int i=0;i<4;i++) {
        R.col(0) = x_vecs1[i];
        R.col(1) = y_vecs1[i];
        R.col(2) = z_vecs1[i];
        des_gripper_affine1.linear() = R;
        des_gripper_affine1.translation() = tip_origins1[i];
        gripper1_affines.push_back(des_gripper_affine1);

        R.col(0) = x_vecs2[i];
        R.col(1) = y_vecs2[i];
        R.col(2) = z_vecs2[i];
        des_gripper_affine2.linear() = R;
        des_gripper_affine2.translation() = tip_origins2[i];
        gripper2_affines.push_back(des_gripper_affine2);
    }
    ROS_INFO("7");
    Vectorq7x1 q_vec1,q_vec2;
    q_vec1.resize(7);
    q_vec2.resize(7);

    Davinci_IK_solver ik_solver1;
    Davinci_IK_solver ik_solver2;

    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.resize(14);

    trajectory_msgs::JointTrajectory des_trajectory;



    ROS_INFO("8");


    for (int ipose=0;ipose<4;ipose++) {
        des_gripper_affine1 = gripper1_affines[ipose];
        cout<<des_gripper_affine1.linear().transpose()<<endl;
        cout<<des_gripper_affine1.translation().transpose()<<endl;
        ik_solver1.ik_solve(des_gripper_affine1); //convert desired pose into equiv joint displacements
        q_vec1 = ik_solver1.get_soln();
        q_vec1(6) = gripper_angs1[ipose];
        cout<<"qvec1: "<<q_vec1.transpose()<<endl;
        des_gripper_affine2 = gripper2_affines[ipose];
        ik_solver2.ik_solve(des_gripper_affine2); //convert desired pose into equiv joint displacements
        q_vec2 = ik_solver2.get_soln();
        q_vec2(6) = gripper_angs2[ipose];
        cout<<"qvec2: "<<q_vec2.transpose()<<endl;
        //repackage q's into a trajectory;

        for (int i=0;i<7;i++) {
            trajectory_point.positions[i] = q_vec1(i);
            trajectory_point.positions[i+7] = q_vec2(i);
            //should fix up jaw-opening values...do this later
        }

        trajectory_point.time_from_start = ros::Duration(arrival_time);
        des_trajectory.points.push_back(trajectory_point);
    }


    ROS_INFO("9");
    trajectory_msgs::JointTrajectory TopLeft;
    trajectory_msgs::JointTrajectory TopRight;
    trajectory_msgs::JointTrajectory BottomRight;
    trajectory_msgs::JointTrajectory BottomLeft;

    TopLeft.points.push_back(des_trajectory.points[0]);
    TopRight.points.push_back(des_trajectory.points[1]);
    BottomLeft.points.push_back(des_trajectory.points[2]);
    BottomRight.points.push_back(des_trajectory.points[3]);




    TopLeft.header.stamp = ros::Time::now();

    cwru_davinci_traj_streamer::trajGoal tgoal;
    tgoal.trajectory = TopLeft;

    srand(time(NULL));
    tgoal.traj_id = rand();

    //Locate and lock the action server
    actionlib::SimpleActionClient<
    cwru_davinci_traj_streamer::trajAction
    > action_client("trajActionServer", true);
    bool server_exists = action_client.waitForServer(ros::Duration(5.0));
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    ROS_INFO("Waiting for server: ");
    while (!server_exists && ros::ok()) {
        server_exists = action_client.waitForServer(ros::Duration(5.0));
        ROS_WARN("Could not connect to server; retrying...");
    }
    ROS_INFO("SERVER LINK LATCHED");

    //Send our message:
    ROS_INFO("Sending trajectory with ID %u", tgoal.traj_id);

    //ROS_INFO("%f",weight.data);
    int pic_num = 0;
    char temp[16];
    string folder = "/home/dvrk/Desktop/yida_pictures/";
    string name;

    Mat raw_copy_left;
    Mat raw_copy_right;

    g_server_goal_completed= false;
    action_client.sendGoal(tgoal,&doneCb);
    while(!g_server_goal_completed){
        doneCb;
        ros::Duration(2).sleep();
        ROS_INFO("STILL MOVING");
    }
    ROS_INFO("Taking topleft picture");
    ros::Duration(2).sleep();
    if (freshImage){
        sprintf(temp,"%d",pic_num);
        string file(temp);
        raw_copy_right = rawImage_right.clone();
        for (int k = 0; k < raw_copy_right.rows; ++k)
        {
            for (int j = 0; j < raw_copy_right.cols; ++j)
            {
                raw_copy_right.at<cv::Vec3b>(k,j)[0] = rawImage_right.at<cv::Vec3b>(k,j)[2];
                raw_copy_right.at<cv::Vec3b>(k,j)[2] = rawImage_right.at<cv::Vec3b>(k,j)[0];
            }
        }
        imwrite(name,raw_copy_right);
        freshImage = false;
        pic_num ++;
    }
    ROS_INFO("Picture TopLeft taken");


    tgoal.traj_id = rand();
    tgoal.trajectory = TopRight;
    g_server_goal_completed= false;
    action_client.sendGoal(tgoal,&doneCb);
    while(!g_server_goal_completed){
        doneCb;
        ros::Duration(2).sleep();
        ROS_INFO("STILL MOVING");
    }
    ROS_INFO("Taking topright picture");
    ros::Duration(2).sleep();
    if (freshImage){
        sprintf(temp,"%d",pic_num);
        string file(temp);
        raw_copy_right = rawImage_right.clone();
        for (int k = 0; k < raw_copy_right.rows; ++k)
        {
            for (int j = 0; j < raw_copy_right.cols; ++j)
            {
                raw_copy_right.at<cv::Vec3b>(k,j)[0] = rawImage_right.at<cv::Vec3b>(k,j)[2];
                raw_copy_right.at<cv::Vec3b>(k,j)[2] = rawImage_right.at<cv::Vec3b>(k,j)[0];
            }
        }
        imwrite(name,raw_copy_right);
        freshImage = false;
        pic_num ++;
    }
    ROS_INFO("Picture Topright taken");


    tgoal.traj_id = rand();
    tgoal.trajectory = BottomRight;
    g_server_goal_completed= false;
    action_client.sendGoal(tgoal,&doneCb);
    while(!g_server_goal_completed){
        doneCb;
        ros::Duration(2).sleep();
        ROS_INFO("STILL MOVING");
    }
    ROS_INFO("Taking topright picture");
    ros::Duration(2).sleep();
    if (freshImage){
        sprintf(temp,"%d",pic_num);
        string file(temp);
        raw_copy_right = rawImage_right.clone();
        for (int k = 0; k < raw_copy_right.rows; ++k)
        {
            for (int j = 0; j < raw_copy_right.cols; ++j)
            {
                raw_copy_right.at<cv::Vec3b>(k,j)[0] = rawImage_right.at<cv::Vec3b>(k,j)[2];
                raw_copy_right.at<cv::Vec3b>(k,j)[2] = rawImage_right.at<cv::Vec3b>(k,j)[0];
            }
        }
        imwrite(name,raw_copy_right);
        freshImage = false;
        pic_num ++;
    }
    ROS_INFO("Picture Bottomright taken");


    tgoal.traj_id = rand();
    tgoal.trajectory = BottomLeft;
    g_server_goal_completed= false;
    action_client.sendGoal(tgoal,&doneCb);
    while(!g_server_goal_completed){
        doneCb;
        ros::Duration(2).sleep();
        ROS_INFO("STILL MOVING");
    }
    ROS_INFO("Taking topright picture");
    ros::Duration(2).sleep();
    if (freshImage){
        sprintf(temp,"%d",pic_num);
        string file(temp);
        raw_copy_right = rawImage_right.clone();
        for (int k = 0; k < raw_copy_right.rows; ++k)
        {
            for (int j = 0; j < raw_copy_right.cols; ++j)
            {
                raw_copy_right.at<cv::Vec3b>(k,j)[0] = rawImage_right.at<cv::Vec3b>(k,j)[2];
                raw_copy_right.at<cv::Vec3b>(k,j)[2] = rawImage_right.at<cv::Vec3b>(k,j)[0];
            }
        }
        imwrite(name,raw_copy_right);
        freshImage = false;
        pic_num ++;
    }
    ROS_INFO("Picture BottomLeft taken");








    //Wait for it to finish.
    while(!action_client.waitForResult(ros::Duration(arrival_time + 4.0)) && ros::ok()){
        ROS_WARN("CLIENT TIMED OUT- LET'S TRY AGAIN...");
        //Could add logic here to resend the request or take other actions if we conclude that
        //the original is NOT going to get served.
    }
    //Report back what happened.
    ROS_INFO(
            "Server state is %s, goal state for trajectory %u is %i",
            action_client.getState().toString().c_str(),
            action_client.getResult()->traj_id,
            action_client.getResult()->return_val
    );

    //This has to do with the intermittent "Total Recall" bug that breaks the trajectory interpolator
    //If you see this appear in your execution, you are running the program too soon after starting Gazebo.
    if(action_client.getState() ==  actionlib::SimpleClientGoalState::RECALLED){
        ROS_WARN("Server glitch. You may panic now.");
    }

    return 0;
}
