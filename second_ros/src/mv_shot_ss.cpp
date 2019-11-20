#include <ros/ros.h>
#include <tf/transform_broadcaster.h>//座標系をパブリッシュするのに必要なヘッダ 
#include <tf/transform_listener.h>//座標系を購読するのに，必要なヘッダ

#include  <math.h>

#include "std_msgs/String.h"//read stringmsgs
//#include "second_ros/adder.h"//独自のメッセージをインクルードしている．
//catkim_make時にこのヘッダがないと言われたら，一回このヘッダをコメントアウトして，コンパイル．再度コメント外して，コンパイルすればおｋ
//一回，コンパイルすることでヘッダを自動的に生成している．
#include <geometry_msgs/Point.h>
#include <cmath>
// Kinematic model

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Point.h>
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
double x=0;
double vlo,aves=0,ref=0,vaves=0,vvlo=0,fref=0;
double bk=0.002;


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.42;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.5;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = -0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.0;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "world";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.5;
  collision_objects[2].primitives[0].dimensions[1] = 0.5;
  collision_objects[2].primitives[0].dimensions[2] = 0.5;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.0;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = -0.25+0.04;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  collision_objects[3].header.frame_id = "world";
  collision_objects[3].id = "objt";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.2;
  collision_objects[3].primitives[0].dimensions[1] = 0.4;
  collision_objects[3].primitives[0].dimensions[2] = 0.42;

  /* Define the pose of the object. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = -0.4;
  collision_objects[3].primitive_poses[0].position.y = 0;
  collision_objects[3].primitive_poses[0].position.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[3].operation = collision_objects[3].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_movements");//いつものROSの宣言
  ros::NodeHandle node_handle;  //今回は別にいらない、通信するときとか
  ros::AsyncSpinner spinner(1); //スピナーの数？を決める（多いほどよい？）
  spinner.start();//スピナーをはじめる

  moveit::planning_interface::MoveGroupInterface group("manipulator");//動かしたいアーム（planning group）の設定--movegroupのどこがgroupなのか分からない時は、.srdf開けば行けるかも．
  group.setPlannerId("RRTConnectkConfigDefault");//逆運動学plannnerの設定．
  group.setPlanningTime(20.0);//計算時間（たぶん長いほど良い）

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());//動かすアームの最上位リンク情報をROSINFOで表示
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());//動かすアームの最下位エンドエフェクタのリンク情報をROSINFOで表示
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //Rvizにいろいろビジュアランジングするときに必要

  addCollisionObjects(planning_scene_interface);
    
  //ここから
  double roll = 90*(M_PI/180.0);//green
  double pitch = 90*(M_PI/180.0);//blue
  double yaw = 90*(M_PI/180.0);//red

  geometry_msgs::Quaternion s;
  geometry_msgs::Quaternion &q = s;
  
  tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
  quaternionTFToMsg(quat,q);

  std::cout << "x = " << q.x << std::endl;
  std::cout << "y = " << q.y << std::endl;
  std::cout << "z = " << q.z << std::endl;
  std::cout << "w = " << q.w << std::endl;
  //ここまでは、roll,pitch,yawをquaternionに変換している




  ros::Rate rate(100.0);
  ros::Time now = ros::Time(0);


  
tf::TransformListener listener1;
  tf::StampedTransform transform1;
  listener1.waitForTransform("/base_link", "/s_frame_1",now, ros::Duration(3.0));
  listener1.lookupTransform("/base_link","/s_frame_1",now,transform1);
  double f1_pos[]={transform1.getOrigin().x(),transform1.getOrigin().y(),transform1.getOrigin().z()};
  double f1_ori[]={transform1.getRotation().x(),transform1.getOrigin().y(),transform1.getOrigin().z()};


  tf::TransformListener listener2;
  tf::StampedTransform transform2;
  listener2.waitForTransform("/base_link", "/s_frame_2",now, ros::Duration(3.0));
  listener2.lookupTransform("/base_link","/s_frame_2",now,transform2);
  double f2_pos[]={transform2.getOrigin().x(),transform2.getOrigin().y(),transform2.getOrigin().z()};
  double f2_ori[]={transform2.getRotation().x(),transform2.getRotation().y(),transform2.getRotation().z()};
  /*
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;//二個目のタスクの宣言（タスクごとに毎回宣言）


  moveit_msgs::OrientationConstraint ocm;  //パスの制約（動かすときに姿勢を変えたくない時）
  ocm.link_name = "ee_link";  //姿勢を変えたくないリンク
  ocm.header.frame_id = "/base_link"; //基準座標系
  ocm.orientation.x = q.x;
  ocm.orientation.y = q.y;
  ocm.orientation.z = q.z;
  ocm.orientation.w = q.w;
  ocm.absolute_x_axis_tolerance = 0.05; //x許容交差（低いほど正確）
  ocm.absolute_y_axis_tolerance = 0.05;//y許容交差
  ocm.absolute_z_axis_tolerance = 0.05;//ｚ許容交差
  ocm.weight = 2.0;
  
  
  moveit_msgs::Constraints test_constraints; //制約の宣言
  test_constraints.orientation_constraints.push_back(ocm); //制約を入れ込む  
  group.setPathConstraints(test_constraints); //入れこまれた制約をplanning groupに適用

  geometry_msgs::PoseStamped target_pose2;
  target_pose2.header.stamp = ros::Time::now();
  target_pose2.header.frame_id = "/base_link";
  //target_pose2.header.frame_id = "/ppp";
  target_pose2.pose.orientation.x = q.x;
  target_pose2.pose.orientation.y = q.y;
  target_pose2.pose.orientation.z = q.z;
  target_pose2.pose.orientation.w = q.w;
  //target_pose2.pose.orientation.w = 1.0;
  target_pose2.pose.position.x = transform.getOrigin().x();
  target_pose2.pose.position.y = transform.getOrigin().y();
  target_pose2.pose.position.z = transform.getOrigin().z();
  group.setPoseTarget(target_pose2);
  group.plan(my_plan2);
  group.execute(my_plan2);

  ROS_INFO("Planfinished!!!!!!!!!!");


  */

    
  moveit::planning_interface::MoveGroupInterface::Plan my_planc;//実態化

  geometry_msgs::PoseStamped target_posec;//目標座標、姿勢の型
  target_posec.header.stamp = ros::Time(0); //TFの関係によりたぶん現在時刻を設定する？
  target_posec.header.frame_id = "/base_link";//基準となる座標系
  
  target_posec.pose.orientation.x =transform1.getRotation().getX();//q.x; //quaternion,x 基準の座標系からみた姿勢
  target_posec.pose.orientation.y =transform1.getRotation().getY();//q.y; //quaternion,y
  target_posec.pose.orientation.z =transform1.getRotation().getZ();//q.z; //quaternion,z
  target_posec.pose.orientation.w =transform1.getRotation().getW();//q.w; //quaternion,w
  
  target_posec.pose.position.x =transform1.getOrigin().x();
  target_posec.pose.position.y =transform1.getOrigin().y();
  target_posec.pose.position.z =transform1.getOrigin().z();
  
  group.setPoseTarget(target_posec);
  group.plan(my_planc);
  group.execute(my_planc);
  ROS_INFO("finish_frame_1");
  
  sleep(30);

  geometry_msgs::PoseStamped target_pose1;//目標座標、姿勢の型
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;//実態化
 
  target_pose1.header.stamp = ros::Time(0); //TFの関係によりたぶん現在時刻を設定する？
  target_pose1.header.frame_id = "/base_link";//基準となる座標系
  
  target_pose1.pose.orientation.x =transform2.getRotation().getX();//q.x; //quaternion,x 基準の座標系からみた姿勢
  target_pose1.pose.orientation.y =transform2.getRotation().getY();//q.y; //quaternion,y
  target_pose1.pose.orientation.z =transform2.getRotation().getZ();//q.z; //quaternion,z
  target_pose1.pose.orientation.w =transform2.getRotation().getW();//q.w; //quaternion,w
  
  target_pose1.pose.position.x =transform2.getOrigin().x();//transform.getOrigin().x();
  target_pose1.pose.position.y =transform2.getOrigin().y();// transform.getOrigin().y();
  target_pose1.pose.position.z =transform2.getOrigin().z();//transform.getOrigin().z();
  
  group.setPoseTarget(target_pose1);
  group.plan(my_plan1);
  group.execute(my_plan1);
  ROS_INFO("finish_frame_2");

  
  while (ros::ok()){
  /*
  static tf::TransformBroadcaster br1;
            tf::Transform transform1;
            transform1.setOrigin(tf::Vector3(p1[0],p1[1],p1[2]));
            tf::Quaternion q1;
            transform2.setRotation(tf::Quaternion(q.x,q.y,q.z,q.w));
            br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "base_link", "rp1"));
  */

    static tf::TransformBroadcaster br4;
            tf::Transform transform4;
            transform4.setOrigin(tf::Vector3(0,0,5));
            transform4.setRotation(tf::Quaternion(0,0,0,1));
            br4.sendTransform(tf::StampedTransform(transform4, ros::Time::now(), "base_link", "debug_frag"));        
  

  
  rate.sleep();
            }


}