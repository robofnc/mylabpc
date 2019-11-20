/* Author: Koshi Makihara,Takumi Hachimine */
/*Warning ROSkineticとindigo でヘッダ名とかが微妙に違うので注意
あと　C++でプログラム作ると、CMakeList とか,いちいち変更するのめんどいし,いきなりROS誰にも
教えられずにやるのは、死ぬよ。。。*/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Kinematic model
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Point.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_movements");//いつものROSの宣言
  ros::NodeHandle node_handle;  //今回は別にいらない、通信するときとか
  ros::AsyncSpinner spinner(1); //スピナーの数？を決める（多いほどよい？）
  spinner.start();//スピナーをはじめる

  moveit::planning_interface::MoveGroupInterface group("manipulator");
  group.setPlannerId("RRTConnectkConfigDefault");

  std::map<std::string, double> joints;
  joints["shoulder_pan_joint"] = -1.57;
  joints["shoulder_lift_joint"] = -1.3;
  joints["elbow_joint"] = -1.3;
  joints["wrist_1_joint"] = -1.3;
  joints["wrist_2_joint"] = 1.57;
  joints["wrist_3_joint"] = 0.0;  
  group.setJointValueTarget(joints);
  group.move();

   //ここから
  double rolls = 90*(M_PI/180.0);
  double pitchs= 90*(M_PI/180.0);
  double yaws = 90*(M_PI/180.0);

  geometry_msgs::Quaternion ss;
  geometry_msgs::Quaternion &qq = ss;
  
  tf::Quaternion quatt=tf::createQuaternionFromRPY(rolls,pitchs,yaws);
  quaternionTFToMsg(quatt,qq);

  std::cout << "x = " << qq.x << std::endl;
  std::cout << "y = " << qq.y << std::endl;
  std::cout << "z = " << qq.z << std::endl;
  std::cout << "w = " << qq.w << std::endl;

//ここまでは、roll,pitch,yawをquaternionに変換している

  geometry_msgs::Pose goal;
  goal.position.x = 0.255;
  goal.position.y = 0.546;
  goal.position.z = 0.456;
  goal.orientation.w = qq.x;
  goal.orientation.x = qq.y;
  goal.orientation.y = qq.z;
  goal.orientation.z = qq.w;
  group.setPoseTarget(goal);
  group.move();



}