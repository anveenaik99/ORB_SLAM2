/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"

geometry_msgs::PoseStamped pose;
float inp_z = 2.0;
// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;
// }
bool doPub = true;
void feedbackfn(const nav_msgs::Odometry::ConstPtr& odom_data)
{
    // MAVpose= *odom_data;
    pose.pose.position.x = odom_data->pose.pose.position.x;
    pose.pose.position.y = odom_data->pose.pose.position.y;
    pose.pose.position.z = inp_z;
}

void lost_track(const std_msgs::Empty::ConstPtr& lTrack){
    doPub = false;
}

int main(int argc, char **argv)
{
	if(argc >1)
	{
    		inp_z = atof(argv[1]);
 	}		
	else 
	{
		inp_z = 0.0;
	
	}	
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber imu_yaw = nh.subscribe("/odom", 10, feedbackfn);	
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    //         ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");
   // ros::Subscriber lost = nh.subscribe("lost_track", 10, lost_track);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);
    doPub = true;
    // wait for FCU connection
    // while(ros::ok() && current_state.connected){
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // for(int i = 0; i<=10; i++)
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;
	int i=0;
    while(ros::ok() && doPub){
       /* if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }*/

        local_pos_pub.publish(pose);
	++i;
	if(i<=100)
	{
        ros::spinOnce();
        
	}
    rate.sleep();
    }

    return 0;
}

