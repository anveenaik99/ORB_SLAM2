/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<thread>
#include<fstream>
#include<chrono>
#include<geometry_msgs/PoseStamped.h>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;
static cv::Mat offset = cv::Mat::zeros(4,4,CV_32F);
geometry_msgs::PoseStamped odom;

bool first = false;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle *nh):mpSLAM(pSLAM){
      odom_p = nh->advertise<geometry_msgs::PoseStamped>("mavros/fake_gps/mocap/pose", 50,this);
      std::thread Thread(&ImageGrabber::vision_thread,this);
      Thread.detach();
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher odom_p;
    tf::TransformBroadcaster br;

    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    void vision_thread();
    bool flag;
};

void ImageGrabber::vision_thread()
{
  ros::Rate r(50);
  while(ros::ok())
  {
      if(!first)
        continue;
      odom.header.stamp = ros::Time::now();
      odom_p.publish(odom);
      r.sleep();
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh;
    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM,&nh);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }



    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "stereo/left/image", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "stereo/right/image", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    ros::Time current_time = ros::Time::now();
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
      cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat pos,dst;

    static cv::Mat prev_pose;
    const cv::Mat check = cv::Mat::eye(4,4, CV_32F);
    dst = cv::Mat::zeros(4,4, CV_32F);
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        pos = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
        if(!pos.empty())
          cv::bitwise_xor(pos, check, dst);
        if(!pos.empty()){//&&cv::countNonZero(dst) > 0){
          prev_pose = pos.clone();
          cout<<endl<<"Previous"<<prev_pose<<endl;
        }
    }
    else
    {
        pos = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
        if(!pos.empty()){
          prev_pose = pos.clone();
          cout<<endl<<"Previous"<<prev_pose<<endl;
        }
    }


    if (pos.empty() ){

      try{

        if(!flag){
          cv::add(offset,prev_pose.clone(),offset);
          flag = true;
        }
          cout<<endl<<"offset"<<endl<<offset;
        mpSLAM->Reset();
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception1: %s", e.what());
        return;
      }
      return;
    }
    else{
      flag = false;
    }



    /* global left handed coordinate system */
    static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    // static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
    //                                                            -1, 1,-1, 1,
    //                                                            -1,-1, 1, 1,
    //                                                             1, 1, 1, 1);

    //prev_pose * T = pose
    // cv::Mat translation =  (pos * pose_prev.inv()).mul(flipSign);
	//cv::Mat translation =  (pos).mul(flipSign);
    // world_lh = world_lh * translation;
    world_lh = pos;
    // pose_prev = pos.clone();
  // const tf::Matrix3x3 rotation45degX(	    1, 0, 0,
  //                                           0, 0.707, 0.707,
  //                                           0, -0.707, 0.707
		// 			    	);
	tf::Matrix3x3 world_lh3(   world_lh.at<float>(0,0),   world_lh.at<float>(0,1),   world_lh.at<float>(0,2),
                                   world_lh.at<float>(1,0),   world_lh.at<float>(1,1),   world_lh.at<float>(1,2),
                                    world_lh.at<float>(2,0),  world_lh.at<float>(2,1), world_lh.at<float>(2,2));
	// world_lh3 = world_lh3*rotation45degX;

    /* transform into global right handed coordinate system, publish in ROS*/
    // tf::Matrix3x3 cameraRotation_rh(  - world_lh3[0][0],   world_lh3[0][1],   world_lh3[0][2],
    //                               - world_lh3[1][0],   world_lh3[1][1],   world_lh3[1][2],
    //                                 world_lh3[2][0], -world_lh3[2][1], - world_lh3[2][0]);

  //  tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - orld_lh.at<float>(2,3) );
tf::Vector3 cameraTranslation_rh( -world_lh.at<float>(2,3)- offset.at<float>(2,3),world_lh.at<float>(0,3)+offset.at<float>(0,3),world_lh.at<float>(1,3) );

    //rotate 270deg about x and 270deg about z to get ENU: x forward, y left, z up
 //    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
 //                                            0, 0, 1,
 //                                            1, 0, 0);
 // const tf::Matrix3x3 rotation90degX(   1, 0, 0,
 //                                          0, 0, 1,
 //                                           0, -1, 0);
    const tf::Matrix3x3 rotation270degY(-0.0000000,  0.0000000, -1.0000000,
   0.0000000,  1.0000000,  0.0000000,
   1.0000000,  0.0000000, -0.0000000 );
    const tf::Matrix3x3 rotation270degZ(-0.0000000,  1.0000000,  0.0000000
  -1.0000000, -0.0000000,  0.0000000
   0.0000000,  0.0000000,  1.0000000);
    tf::Matrix3x3 rotation270degYZ = rotation270degY * rotation270degZ; 

 	tf::Matrix3x3 globalRotation_rh = cameraRotation_rh*rotation270degYZ;
        tf::Vector3 globalTranslation_rh = cameraTranslation_rh;// change translation matrix
    tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_pose"));

    //publish odometry
    //nav_msgs::Odometry odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = "map";

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

    //set the position
    odom.pose.position.x = globalTranslation_rh[0] ;
    odom.pose.position.y = globalTranslation_rh[1] ;
    odom.pose.position.z = globalTranslation_rh[2];

    odom.pose.orientation = odom_quat;

    //set the velocity
    // odom.child_frame_id = "base_link";
    // odom.twist.twist.linear.x = 0;
    // odom.twist.twist.linear.y = 0;
    // odom.twist.twist.angular.z = 0;

    //publish the message
    first =  true;

}
