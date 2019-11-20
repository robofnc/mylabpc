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
  ros::init(argc, argv, "baxter_movessssments");//いつものROSの宣言
  ros::NodeHandle node_handle;  //今回は別にいらない、通信するときとか
  ros::AsyncSpinner spinner(1); //スピナーの数？を決める（多いほどよい？）
  spinner.start();//スピナーをはじめる

  //ros::Duration sleep_time(10.0);
  ros::Rate rate(100.0);

//   moveit::planning_interface::MoveGroupInterface group("manipulator");//動かしたいアーム（planning group）の設定--movegroupのどこがgroupなのか分からない時は、.srdf開けば行けるかも．
//   group.setPlannerId("RRTConnectkConfigDefault");//逆運動学plannnerの設定．
//   group.setPlanningTime(20.0);//計算時間（たぶん長いほど良い）

//   ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());//動かすアームの最上位リンク情報をROSINFOで表示
//   ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());//動かすアームの最下位エンドエフェクタのリンク情報をROSINFOで表示
  
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //Rvizにいろいろビジュアランジングするときに必要


//   //ここから
//   double roll = 90*(M_PI/180.0);//green
//   double pitch = 105*(M_PI/180.0);//blue
//   double yaw = 90*(M_PI/180.0);//red

//   geometry_msgs::Quaternion s;
//   geometry_msgs::Quaternion &q = s;
  
//   tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
//   quaternionTFToMsg(quat,q);

//   std::cout << "x = " << q.x << std::endl;
//   std::cout << "y = " << q.y << std::endl;
//   std::cout << "z = " << q.z << std::endl;
//   std::cout << "w = " << q.w << std::endl;

//   //ここまでは、roll,pitch,yawをquaternionに変換している

//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;//実態化

//   geometry_msgs::PoseStamped target_pose1;//目標座標、姿勢の型
//   target_pose1.header.stamp = ros::Time(0); //TFの関係によりたぶん現在時刻を設定する？
//   target_pose1.header.frame_id = "/base_link";//基準となる座標系
  
//   target_pose1.pose.orientation.x =q.x; //transform.getRotation().getX();//q.x; //quaternion,x 基準の座標系からみた姿勢
//   target_pose1.pose.orientation.y =q.y; //transform.getRotation().getY();//q.y; //quaternion,y
//   target_pose1.pose.orientation.z =q.z; //transform.getRotation().getZ();//q.z; //quaternion,z
//   target_pose1.pose.orientation.w =q.w; //transform.getRotation().getW();//q.w; //quaternion,w
  
//   target_pose1.pose.position.x =0.1;//transform.getOrigin().x();
//   target_pose1.pose.position.y =0.35;// transform.getOrigin().y();
//   target_pose1.pose.position.z =0.3;//transform.getOrigin().z();
  
//   group.setPoseTarget(target_pose1);
//   group.plan(my_plan);
//   group.execute(my_plan);
//   ROS_INFO("atempted p1");

//  std::vector<geometry_msgs::Pose> waypoints;//経由地点を設定


//   geometry_msgs::Pose target_pose3 =group.getCurrentPose().pose;
//   target_pose3.position.x += 0.05;
//   target_pose3.position.z += 0.09;
//   waypoints.push_back(target_pose3);  // up and out
//   group.setPoseTarget(target_pose3);
//   group.move();
//   ROS_INFO("target pose 01");
//   target_pose3.position.y += 0.02;
//   waypoints.push_back(target_pose3);  // left
//   group.setPoseTarget(target_pose3);
//   group.move();
//   ROS_INFO("target pose 02");
//   //target_pose3.position.z -= 0.02;
//   target_pose3.position.y += 0.02;
//   //target_pose3.position.x -= 0.02;
//   waypoints.push_back(target_pose3);  // down and right (back to start)
//   group.setPoseTarget(target_pose3);
//   group.move();
//   ROS_INFO("target pose 03");

//   ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

//   while(planning_scene_diff_publisher.getNumSubscribers() < 1)
//   {
//     ros::WallDuration sleep_t(0.5);
//     sleep_t.sleep();
//   }
//    moveit_msgs::AttachedCollisionObject attached_object;
//   attached_object.link_name = "world";
//   /* The header must contain a valid TF frame*/
//   attached_object.object.header.frame_id = "world";
//   /* The id of the object */
//   attached_object.object.id = "box";

//   /* A default pose */
//   geometry_msgs::Pose pose;
//   pose.orientation.w = 1.0;

//   /* Define a box to be attached */
//   shape_msgs::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[0] = 1;
//   primitive.dimensions[1] = 1;
//   primitive.dimensions[2] = -0.1;

//   attached_object.object.primitives.push_back(primitive);
//   attached_object.object.primitive_poses.push_back(pose);

// // Note that attaching an object to the robot requires 
// // the corresponding operation to be specified as an ADD operation
//   attached_object.object.operation = attached_object.object.ADD;
//   ROS_INFO("Adding the object into the world at the location of the right wrist.");
//   moveit_msgs::PlanningScene planning_scene;
//   planning_scene.world.collision_objects.push_back(attached_object.object);
//   planning_scene.is_diff = true;
//   planning_scene_diff_publisher.publish(planning_scene);
//   //sleep_time.sleep();
//   ROS_INFO("Plan 1 finished!!!!!!!!!!");

// }
while (node_handle.ok()){


            static tf::TransformBroadcaster br00;
            tf::Transform transform00;
            transform00.setOrigin(tf::Vector3(0.026,0.612,0.141));
            tf::Quaternion q0;
            q0.setX(-0.164);
            q0.setY(0.707);
            q0.setZ(0.021);
            q0.setW(0.688);

            //q0.setRPY(105.179,0,90.046);
            transform00.setRotation(q0);
            br00.sendTransform(tf::StampedTransform(transform00, ros::Time::now(), "base_link", "real_degss"));
            rate.sleep();
            }

}