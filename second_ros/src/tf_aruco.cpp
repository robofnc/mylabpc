#include <ros/ros.h>
#include <tf/transform_broadcaster.h>//座標系をパブリッシュするのに必要なヘッダ 
#include <tf/transform_listener.h>//座標系を購読するのに，必要なヘッダ

#include  <math.h>

#include "std_msgs/String.h"//read stringmsgs
//#include "second_ros/adder.h"//独自のメッセージをインクルードしている．
//catkim_make時にこのヘッダがないと言われたら，一回このヘッダをコメントアウトして，コンパイル．再度コメント外して，コンパイルすればおｋ
//一回，コンパイルすることでヘッダを自動的に生成している．
#include <geometry_msgs/Point.h>

// Kinematic model

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#define pai 3.14159265



int flag=0;
double x=0,y=0,z=0;
double vlo,aves=0,ref=0,vaves=0,vvlo=0,fref=0;


void chatterCallback(const geometry_msgs::PoseStamped arucos)//力覚センサを読み取る．独自のメッセージの型を作成し，使用．
{
  x=arucos.pose.position.x;
  y=arucos.pose.position.y;
  z=arucos.pose.position.z;

double qx=arucos.pose.orientation.x;
double qy=arucos.pose.orientation.y;
double qz=arucos.pose.orientation.z;
double qw=arucos.pose.orientation.w;



    static tf::TransformBroadcaster br3;
            tf::Transform transform3;
            transform3.setOrigin(tf::Vector3(z,y,x));
            transform3.setRotation(tf::Quaternion(qx,qy,qz,qw));
            br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "camera_link", "srcoo"));  
  printf("X_position%f\n",x);

  return;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_movements");//いつものROSの宣言
  ros::NodeHandle node_handle;  //今回は別にいらない、通信するときとか
  ros::AsyncSpinner spinner(50); //スピナーの数？を決める（多いほどよい？）
  spinner.start();//スピナーをはじめる



  //ここから
  double roll = 0*(M_PI/180.0);//green
  double pitch = 0*(M_PI/180.0);//blue
  double yaw = 0*(M_PI/180.0);//red

  geometry_msgs::Quaternion s;
  geometry_msgs::Quaternion &q = s;
  
  tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
  quaternionTFToMsg(quat,q);

  std::cout << "x = " << q.x << std::endl;
  std::cout << "y = " << q.y << std::endl;
  std::cout << "z = " << q.z << std::endl;
  std::cout << "w = " << q.w << std::endl;
  //ここまでは、roll,pitch,yawをquaternionに変換している

  ros::Rate rate(1000.0);
  ros::Time now = ros::Time(0);
  sleep(1.0);

  // SubscriberとしてchatterというトピックがSubscribeし、トピックが更新されたときは
  // chatterCallbackという名前のコールバック関数を実行する
  ros::Subscriber sub = node_handle.subscribe("aruco_single/pose", 1000, chatterCallback);
  sleep(1.0);
  

  while (node_handle.ok()){

 rate.sleep();
            }

}