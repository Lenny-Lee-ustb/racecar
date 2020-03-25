/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
//#include "PID.h"
//#include "art_car_controller.hpp"

#define PI 3.14159265358979
int start_loop_flag = 0;
int start_speed = 1560;
// extern  PID  pid_speed;

double turn_erro = 0;
double last_turn_erro = 0;

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller {
public:
  L1Controller();
  void initMarker();
  bool isForwardWayPt(const geometry_msgs::Point &wayPt,
                      const geometry_msgs::Pose &carPose);
  bool isWayPtAwayFromLfwDist(const geometry_msgs::Point &wayPt,
                              const geometry_msgs::Point &car_pos);
  bool isWayinPtAwayFromLfwDist(const geometry_msgs::Point &wayPt,
                                const geometry_msgs::Point &car_pos);
  bool isWayoutPtAwayFromLfwDist(const geometry_msgs::Point &wayPt,
                                 const geometry_msgs::Point &car_pos);
  double getYawFromPose(const geometry_msgs::Pose &carPose);
  double getEta(const geometry_msgs::Pose &carPose);
  double getCar2GoalDist();
  double getL1Distance(const double &_Vcmd);
  double getSteeringAngle(double eta);
  double getGasInput(const float &current_v);
  geometry_msgs::Point
  get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose);
  void PID_init();

private:
  struct PID *speed_pid;
  ros::NodeHandle n_;
  ros::Subscriber odom_sub, path_sub, goal_sub;
  ros::Publisher pub_, marker_pub;
  ros::Timer timer1, timer2;
  tf::TransformListener tf_listener;

  visualization_msgs::Marker points, line_strip, goal_circle;
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Point odom_goal_pos;
  nav_msgs::Odometry odom;
  nav_msgs::Path map_path, odom_path;

  double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
  double P_speed, I_speed, D_speed;
  double p_turn, d_turn;

  double Gas_gain, baseAngle, Angle_gain, goalRadius;
  int controller_freq, baseSpeed;
  bool foundForwardPt, goal_received, goal_reached;
  int car_stop;

  void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
  void goalReachingCB(const ros::TimerEvent &);
  void controlLoopCB(const ros::TimerEvent &);

}; // end of class

double L1Controller::getEta(const geometry_msgs::Pose &carPose) {
  geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

  double eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
  return eta; //车子的当前角度
}

double L1Controller::getSteeringAngle(double eta) {
  // double steeringAnge = -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI);
  double steeringAnge = eta * (180.0 / PI);
  // ROS_INFO("Steering Angle = %.2f", steeringAnge);
  return steeringAnge;
}

double L1Controller::getGasInput(const float &current_v) {
  double u = (Vcmd - current_v) * Gas_gain;
  // ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
  return u;
}

double L1Controller::getL1Distance(const double &_Vcmd) {
  double L1 = 0;
  if (_Vcmd < 1.34)
    L1 = 3 * 1.5 / 3.0; // 3 / 3.0;
  else if (_Vcmd > 1.34 && _Vcmd < 5.36)
    L1 = _Vcmd * 2.24 * 1.5 / 3.0;
  else
    L1 = 12 * 1.5 / 3.0;
  return L1;
}

double L1Controller::getCar2GoalDist() {
  geometry_msgs::Point car_pose = odom.pose.pose.position;
  double car2goal_x = odom_goal_pos.x - car_pose.x;
  double car2goal_y = odom_goal_pos.y - car_pose.y;

  double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);

  return dist2goal;
}

//从车身姿态中获得角度
double L1Controller::getYawFromPose(const geometry_msgs::Pose &carPose) {
  float x = carPose.orientation.x; //轴向量的x
  float y = carPose.orientation.y; //轴向量的y
  float z = carPose.orientation.z; //轴向量的z
  float w = carPose.orientation.w; //极坐标系下的角度

  double tmp, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q); //解算四元数
  quaternion.getRPY(tmp, tmp,
                    yaw); //分别存放R/P/Y  R:横滚角 P:俯仰角  Y: 航向角

  return yaw; //返回航向角
}

//判断是否是前进点
bool L1Controller::isForwardWayPt(const geometry_msgs::Point &wayPt,
                                  const geometry_msgs::Pose &carPose) {
  float car2wayPt_x = wayPt.x - carPose.position.x;
  float car2wayPt_y = wayPt.y - carPose.position.y;
  double car_theta = getYawFromPose(carPose) + PI;
  if (car_theta > 2 * PI) {
    car_theta = car_theta - 2 * PI;
  }
  float car_car2wayPt_x =
      cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
  float car_car2wayPt_y =
      -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;

  if (car_car2wayPt_x > 0) /*is Forward WayPt*/
    return true;
  else
    return false;
}

//寻找前瞻点
bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point &wayPt,
                                          const geometry_msgs::Point &car_pos) {
  double dx = wayPt.x - car_pos.x;
  double dy = wayPt.y - car_pos.y;
  double dist = sqrt(dx * dx + dy * dy);

  if (dist < Lfw)
    return false;
  else if (dist >= Lfw)
    return true;
}

//寻找前瞻范围内正中间的点
bool L1Controller::isWayinPtAwayFromLfwDist(
    const geometry_msgs::Point &wayPt, const geometry_msgs::Point &car_pos) {
  double dx = wayPt.x - car_pos.x;
  double dy = wayPt.y - car_pos.y;
  double dist = sqrt(dx * dx + dy * dy);

  if (dist < Lfw / 2)
    return false;
  else if (dist >= Lfw / 2)
    return true;
}

//寻找前瞻范围外的点
bool L1Controller::isWayoutPtAwayFromLfwDist(
    const geometry_msgs::Point &wayPt, const geometry_msgs::Point &car_pos) {
  double dx = wayPt.x - car_pos.x;
  double dy = wayPt.y - car_pos.y;
  double dist = sqrt(dx * dx + dy * dy);

  if (dist < Lfw * 3 / 2)
    return false;
  else if (dist >= Lfw * 3 / 2)
    return true;
}

//计算前瞻向量
geometry_msgs::Point
L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose) {
  geometry_msgs::Point carPose_pos = carPose.position;
  double carPose_yaw = getYawFromPose(carPose); //获得车当前航向角
  geometry_msgs::Point forwardPt;
  geometry_msgs::Point forwardPt1;
  geometry_msgs::Point forwardPt2;
  geometry_msgs::Point forwardPt3;
  geometry_msgs::Point odom_car2WayPtVec;
  foundForwardPt = false;
  double forwardPtcontrollinex = 0;
  double forwardPtcontrolliney = 0;
  double forwardPtcontrolwholex = 0;
  double forwardPtcontrolwholey = 0;
  double forwardPtcontrolx = 0, forwardPtcontroly = 0;
  double eps = 1e-10;
  int C = 100;
  int bot_pt;
  int pt_lim;
  double tempangle, tempdis, tempr, area, tempThor;
  if (!goal_reached) //未到达目标点
  {
    for (int i = 0; i < map_path.poses.size(); i++) {
      geometry_msgs::PoseStamped map_path_pose = map_path.poses[i]; //获取路径点
      geometry_msgs::PoseStamped odom_path_pose;
      double S[i];
      try //可能抛出异isWayPtAwayFromLfwDist常的语句
      {
        tf_listener.transformPose("odom", ros::Time(0), map_path_pose, "map",
                                  odom_path_pose);
        geometry_msgs::Point odom_path_wayPt =
            odom_path_pose.pose.position; //获得路径的位置点
        bool _isForwardWayPt =
            isForwardWayPt(odom_path_wayPt, carPose); //判断是否为前进点

        if (1 || _isForwardWayPt) //是前进点
        {
          bool _isWayPtAwayFromLfwDist =
              isWayPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);
          bool _isWayinPtAwayFromLfwDist =
              isWayinPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);
          bool _isWayoutPtAwayFromLfwDist =
              isWayoutPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);
          if (_isWayinPtAwayFromLfwDist) {
            forwardPt1 = odom_path_wayPt;
            bot_pt = i;
          }
          if (_isWayPtAwayFromLfwDist) {
            forwardPt2 = odom_path_wayPt;
          }
          if (_isWayoutPtAwayFromLfwDist) {
            forwardPt3 = odom_path_wayPt;
            pt_lim = i;
          }
          for (i = pt_lim - 1; i >= bot_pt; i--) {
            tempangle =
                atan2(forwardPt3.x - forwardPt1.x, forwardPt3.y - forwardPt1.y);
            tempdis = forwardPt2.y - (forwardPt1.y +
                                      (forwardPt2.x - forwardPt1.x) *
                                          (forwardPt3.y - forwardPt1.y) /
                                          (forwardPt3.x - forwardPt1.x));
            tempr = tempdis * sin(tempangle);
            area += (tempr - C * 10000) / 10000;
            S[i] = area;
          }
          if (i && S[i] < S[i - 1]) {
            S[i] = S[i - 1];
          }
          tempThor = (i + 1) * (i + 1) * C / (C + S[i]);
          if (_isWayPtAwayFromLfwDist) //该点超过前瞻范围
          {
            forwardPtcontrolx =
                forwardPtcontrolwholex / (forwardPtcontrollinex + eps);
            forwardPtcontroly =
                forwardPtcontrolwholey / (forwardPtcontrolliney + eps);
            // forwardPt = odom_path_wayPt;    //最远前瞻点
            foundForwardPt = true;
            break; //结束循环
          } else {
            forwardPtcontrollinex = tempThor;
            forwardPtcontrolliney = tempThor;
            forwardPtcontrolwholex +=
                (odom_path_wayPt.x - carPose_pos.x) * tempThor;
            forwardPtcontrolwholey +=
                (odom_path_wayPt.y - carPose_pos.y) * tempThor;
          }
        }
      } catch (tf::TransformException &ex) //异常获取
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }

  } else if (goal_reached) //到达目标点
  {
    forwardPt = odom_goal_pos; //最远前瞻点为目标点
    foundForwardPt = false;    //
                               // ROS_INFO("goal REACHED!");
  }

  /*Visualized Target Point on RVIZ*/
  /*Clear former target point Marker*/
  points.points.clear();
  line_strip.points.clear();

  if (foundForwardPt && !goal_reached) {
    points.points.push_back(carPose_pos);
    points.points.push_back(forwardPt);
    line_strip.points.push_back(carPose_pos);
    line_strip.points.push_back(forwardPt);
  }

  marker_pub.publish(points);
  marker_pub.publish(line_strip);
  /*=============  RVIZ  ================*/
  /*odom_car2WayPtVec.x = -cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) -
  sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
  odom_car2WayPtVec.y = sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) -
  cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);*/
  odom_car2WayPtVec.x = cos(carPose_yaw) *
                        (forwardPtcontrolx)+sin(carPose_yaw) *
                        (forwardPtcontroly);
  odom_car2WayPtVec.y = -sin(carPose_yaw) *
                        (forwardPtcontrolx)+cos(carPose_yaw) *
                        (forwardPtcontroly);
  return odom_car2WayPtVec;
}

//================================  订阅信息的回调函数
//==================================//
/*
 *用来改变目标点是否收到和到达的回调函数
*/
void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg) {
  try {
    geometry_msgs::PoseStamped odom_goal;
    tf_listener.transformPose("odom", ros::Time(0), *goalMsg, "map", odom_goal);
    odom_goal_pos = odom_goal.pose.position;
    goal_received = true;
    goal_reached = false;

    /*Draw Goal on RVIZ*/
    goal_circle.pose = odom_goal.pose;
    marker_pub.publish(goal_circle);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}
/*
 *里程计信息订阅的回调函数
*/
void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  odom = *odomMsg;
}

/*
 *路径信息订阅的回调函数
*/
void L1Controller::pathCB(const nav_msgs::Path::ConstPtr &pathMsg) {
  map_path = *pathMsg;
}

//=========================== 订阅信息的回调函数 end
//======================================//

/********************/
/*        PID       */
/********************/

typedef struct PID { //结构体定义
  double SetPoint;   //设定值
  double Proportion; // Proportion 比例系数
  double Integral;   // Integral   积分系数
  double Derivative; // Derivative  微分系数
  double LastError;  // Error[-1]  前一拍误差
  double PreError;   // Error[-2]  前两拍误差

} PID;

void PIDInit(struct PID *pp) // PID参数初始化，都置0
{
  memset(pp, 0, sizeof(PID));
}

double PIDCal(struct PID *pp, double ThisError) {
  //增量式PID算法（需要控制的不是控制量的绝对值，而是控制量的增量）
  double pError, dError, iError;
  double templ;
  pError = ThisError - pp->LastError;                      //比例
  iError = ThisError;                                      //积分
  dError = ThisError - 2 * (pp->LastError) + pp->PreError; //微分
  //增量计算
  templ = pp->Proportion * pError + pp->Integral * iError +
          pp->Derivative * dError; //增量

  //存储误差用于下次运算
  pp->PreError = pp->LastError;
  pp->LastError = ThisError;

  return templ;
}

struct PID pid_speed;
struct PID pid_angle;

void L1Controller::PID_init() {
  PIDInit(&(pid_speed));
  pid_speed.SetPoint = Vcmd;
  pid_speed.Proportion = P_speed;
  pid_speed.Integral = I_speed;
  pid_speed.Derivative = D_speed;
}

// rviz 显示初始化
void L1Controller::initMarker() {
  points.header.frame_id = line_strip.header.frame_id =
      goal_circle.header.frame_id = "odom";
  points.ns = line_strip.ns = goal_circle.ns = "Markers";
  points.action = line_strip.action = goal_circle.action =
      visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w =
      goal_circle.pose.orientation.w = 1.0;
  points.id = 0;
  line_strip.id = 1;
  goal_circle.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  goal_circle.type = visualization_msgs::Marker::CYLINDER;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;

  goal_circle.scale.x = goalRadius;
  goal_circle.scale.y = goalRadius;
  goal_circle.scale.z = 0.1;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // goal_circle is yellow
  goal_circle.color.r = 1.0;
  goal_circle.color.g = 1.0;
  goal_circle.color.b = 0.0;
  goal_circle.color.a = 0.5;
}

L1Controller::L1Controller() {
  // Private parameters handler
  ros::NodeHandle pn("~");

  // Car parameter
  pn.param("L", L, 0.26);
  pn.param("Lrv", Lrv, 10.0);
  pn.param("Vcmd", Vcmd, 1.0);
  pn.param("lfw", lfw, 0.13);
  pn.param("lrv", lrv, 10.0);

  // Controller parameter
  pn.param("controller_freq", controller_freq, 20);
  pn.param("AngleGain", Angle_gain, -1.0);
  pn.param("GasGain", Gas_gain, 1.0);
  pn.param("baseSpeed", baseSpeed, 1575);
  pn.param("baseAngle", baseAngle, 90.0);
  pn.param("p_turn", p_turn, 20.0);
  pn.param("d_turn", d_turn, 5.0);

  // Publishers and Subscribers
  odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);
  path_sub = n_.subscribe("/move_base_node/NavfnROS/plan", 1,
                          &L1Controller::pathCB, this);
  goal_sub =
      n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
  marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
  pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);

  // Timer
  timer1 = n_.createTimer(ros::Duration((1.0) / controller_freq),
                          &L1Controller::controlLoopCB,
                          this); // Duration(0.05) -> 20Hz
  timer2 = n_.createTimer(ros::Duration((0.5) / controller_freq),
                          &L1Controller::goalReachingCB,
                          this); // Duration(0.05) -> 20Hz

  // Init variables
  Lfw = goalRadius = getL1Distance(Vcmd);
  foundForwardPt = false;
  goal_received = false;
  goal_reached = false;
  cmd_vel.linear.x = 0; // 1500 for stop
  cmd_vel.angular.z = baseAngle;

  // Show info
  ROS_INFO("[param] baseSpeed: %d", baseSpeed);
  ROS_INFO("[param] baseAngle: %f", baseAngle);
  ROS_INFO("[param] AngleGain: %f", Angle_gain);
  ROS_INFO("[param] Vcmd: %f", Vcmd);
  ROS_INFO("[param] Lfw: %f", Lfw);

  // Visualization Marker Settings
  initMarker();
  car_stop = 0;
}

void L1Controller::goalReachingCB(const ros::TimerEvent &) {

  if (goal_received) {
    double car2goal_dist = getCar2GoalDist();
    if (car2goal_dist < goalRadius) {
      goal_reached = true;
      goal_received = false;
      // ROS_INFO("Goal Reached !");
      car_stop = 100;
    }
  }
}

void L1Controller::controlLoopCB(const ros::TimerEvent &) {
  int count = 100;
  geometry_msgs::Pose carPose = odom.pose.pose;
  geometry_msgs::Twist carVel = odom.twist.twist;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = baseAngle;

  if (goal_received) {
    /*Estimate Steering Angle*/
    double eta = getEta(carPose);
    turn_erro = eta / (2 * PI) * 360; //把eta角当作当前的偏差
    ROS_INFO("eta = %.2f", eta / (2 * PI) * 3600);
    last_turn_erro = turn_erro; //存储上一次的偏差
    ROS_INFO("foundForwardPt = %s ", foundForwardPt ? "True" : "False");
    if (foundForwardPt) {

      //自己的pid控制，偏差是eta，目标是0,采用位置式pid控制
      cmd_vel.angular.z = baseAngle + p_turn * turn_erro +
                          d_turn * (turn_erro - last_turn_erro);
      // cmd_vel.angular.z = baseAngle + getSteeringAngle(eta)*Angle_gain;
      // cmd_vel.angular.z = baseAngle + getSteeringAngle(eta)*Angle_gain;
      /*Estimate Gas Input*/
      ROS_INFO("cmd_vel.angular.z = %.2f\n", cmd_vel.angular.z);
      if (!goal_reached) {
        if (start_loop_flag++ <= 10) {

          double u = getGasInput(carVel.linear.x);

          cmd_vel.linear.x = start_speed * 5 + u;
          // cmd_vel.linear.x = start_speed + PIDCal(&pid_speed,u);

          start_speed += 4;
          if (cmd_vel.linear.x > baseSpeed)
            cmd_vel.linear.x = baseSpeed;
          ROS_INFO("baseSpeed = %.2f\t Steering angle = %.2f", cmd_vel.linear.x,
                   cmd_vel.angular.z);
        } else {
          // ROS_INFO("!goal_reached");

          double u = getGasInput(carVel.linear.x);
          cmd_vel.linear.x = baseSpeed + u;
          // cmd_vel.linear.x = start_speed + PIDCal(&pid_speed,u);

          ROS_INFO("Gas = %.2f\t Steering angle = %.2f", cmd_vel.linear.x,
                   cmd_vel.angular.z);
        }
      }
    }
  }
  if (car_stop > 0) {
    car_stop--;
    if (car_stop <= 1) {
      car_stop = 1;
    }
    start_loop_flag = 0;
    if (carVel.linear.x > 0) {

      cmd_vel.linear.x = 0;
      pub_.publish(cmd_vel);
      // for(int i=0;i<20;i++)
      // {
      //     pub_.publish(cmd_vel);
      //     sleep(0.1);
      //     ROS_INFO("cat stop cmd_vel= %f",cmd_vel.linear.x);
      // }

    } else {
      car_stop = 0;
      cmd_vel.linear.x = 0;
      pub_.publish(cmd_vel);

      // ROS_INFO("cmd_vel= %f",cmd_vel.linear.x);
    }
  } else {
    pub_.publish(cmd_vel);
    car_stop = 0;
    // ROS_INFO("car run cmd_vel= %f",cmd_vel.linear.x);
  }
  ROS_INFO("cmd_vel= %f", cmd_vel.linear.x);
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv) {
  // Initiate ROS
  ros::init(argc, argv, "art_car_controller");
  L1Controller controller;
  controller.PID_init();
  ros::spin();
  return 0;
}
