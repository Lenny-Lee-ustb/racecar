//
// Created by Steven Zhang on 18-12-14.
// art racecar
//
//电机速度和舵机速度转换
#include "../include/art_racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#define SPEED_MAX 39000
#define SPEED_MIN 37000
#define SPEED_MID 38000
void TwistCallback(const geometry_msgs::Twist& twist)
{
    double angle;
    double speed;
    double angleinital=37300;//舵机中值
    double speedinital=SPEED_MID;//速度最大值
    double anglechange=1500;//角度可变范围
    double speedchange=180;//速度可变范围

    //ROS_INFO("x= %f", twist.linear.x);
    //ROS_INFO("z= %f", twist.angular.z);
    angle = angleinital + twist.angular.z;
    speed = speedinital + twist.linear.x;
    //ROS_INFO("angle= %d",uint16_t(angle));
	if(angle>angleinital+anglechange)
	{
		angle = angleinital+anglechange;
	}
	else if (angle<angleinital-anglechange)
		angle=angleinital-anglechange;

	if(speed>SPEED_MAX)
	{
		speed = SPEED_MAX;
	}
	else if (speed<SPEED_MIN)
		speed = SPEED_MIN;


    send_cmd(uint16_t(speed),uint16_t(angle));
}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(38400,data);
    ros::init(argc, argv, "art_driver");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/car/cmd_vel",1,TwistCallback);



    ros::spin();

}
