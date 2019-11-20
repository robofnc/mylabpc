/* Author: Koshi Makihara,Takumi Hachimine */
/*
Warning 
ROSkineticとindigo でヘッダ名とかが微妙に違うので注意
あと　C++でプログラム作ると、CMakeList とか,いちいち変更するのめんどいし,いきなりROSを誰にも
教えられずにやるのは、死ぬよ。。。てかさ，ｃ++で書くと変更したら，その都度catkin_makeしなきゃいけないよね
python保存するだけで反映されるのに．あと，依存関係もCMake.textに書かなならん．
初学者殺しやな．
てか，cとpythonどちらかで記述できればいいんだけど，参考サイトごとに，言語が違うから，結局両言語ともやらなならん．
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include  <math.h>

#include "std_msgs/String.h"//read stringmsgs
//#include "second_ros/adder.h"
#include <geometry_msgs/Point.h>

// Kinematic model

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Point.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#define pai 3.14159265

 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_shot_frame");//いつものROSの宣言
  ros::NodeHandle node_handle;  //今回は別にいらない、通信するときとか
  ros::AsyncSpinner spinner(1); //スピナーの数？を決める（多いほどよい？）
  spinner.start();//スピナーをはじめる

  //ros::Duration sleep_time(10.0);
  ros::Rate rate(100.0);


  //ここから
  double roll = 85*(M_PI/180.0);//green
  double pitch = 120*(M_PI/180.0);//blue
  double yaw = 90*(M_PI/180.0);//red

  geometry_msgs::Quaternion s;
  geometry_msgs::Quaternion &q = s;
  
  tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
  quaternionTFToMsg(quat,q);

  std::cout << "x = " << q.x << std::endl;
  std::cout << "y = " << q.y << std::endl;
  std::cout << "z = " << q.z << std::endl;
  std::cout << "w = " << q.w << std::endl;


  roll = 88*(M_PI/180.0);//red
  pitch = 120*(M_PI/180.0);//green
  yaw = 90*(M_PI/180.0);//ble

  geometry_msgs::Quaternion s1;
  geometry_msgs::Quaternion &q1 = s1;

  tf::Quaternion quat1=tf::createQuaternionFromRPY(roll,pitch,yaw);
  quaternionTFToMsg(quat1,q1);

  std::cout << "x = " << q1.x << std::endl;
  std::cout << "y = " << q1.y << std::endl;
  std::cout << "z = " << q1.z << std::endl;
  std::cout << "w = " << q1.w << std::endl;

  //ここまでは、roll,pitch,yawをquaternionに変換している

while (node_handle.ok()){


            static tf::TransformBroadcaster br00;
            tf::Transform transform00;
            transform00.setOrigin(tf::Vector3(0.026,0.35,0.3));
            tf::Quaternion q0;
            q0.setX(q.x);
            q0.setY(q.y);
            q0.setZ(q.z);
            q0.setW(q.w);

            transform00.setRotation(q0);
            br00.sendTransform(tf::StampedTransform(transform00, ros::Time::now(), "base_link", "s_frame_1"));


            static tf::TransformBroadcaster br01;
            tf::Transform transform01;
            transform01.setOrigin(tf::Vector3(-0.15,0.35,0.2));
            tf::Quaternion q01;
            q01.setX(q1.x);
            q01.setY(q1.y);
            q01.setZ(q1.z);
            q01.setW(q1.w);
            transform01.setRotation(q01);
            br01.sendTransform(tf::StampedTransform(transform01, ros::Time::now(), "base_link", "s_frame_2"));


            rate.sleep();
            }

}