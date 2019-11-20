#include <ros/ros.h>//ROSのヘッダー
#include <tf/transform_broadcaster.h>//座標系をパブリッシュするのに必要なヘッダ 
#include <tf/transform_listener.h>//座標系を購読するのに，必要なヘッダ

#include  <math.h>

#include "std_msgs/String.h"//read stringmsgs
//#include "second_ros/adder.h"//独自のメッセージをインクルードしている．
//catkim_make時にこのヘッダがないと言われたら，一回このヘッダをコメントアウトして，コンパイル．再度コメント外して，コンパイルすればおｋ
//一回，コンパイルすることでヘッダを自動的に生成している．
#include <geometry_msgs/Point.h>//メッセージの型をインクルード
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
#include<ros_start/f3darray.h>//独自の型をインクルード
#define pai 3.14159265



int flag=0;
double vlo,aves=0,ref=0,vaves=0,vvlo=0,fref=0;
double bk=0.002;
int i;
double x[100]={};
double y[100]={};
double t[100]={};


void chatterCallback(const ros_start::f3darray& msg)//経路計画で生成した際のx座標を読み取る．独自のメッセージの型を作成し，使用．
{
 
  for(i=0;msg.x[i]!=100;i++){
    //番兵を100として読みとった配列の長さを求める
  }

  for(int j=0;j<=i;j++){
    x[j]=msg.x[j];
  }
    return;
}

void chatterCallback1(const ros_start::f3darray& msg)//read y axis f3darray messeage about generated path planning
{
 
  for(i=0;msg.y[i]!=100;i++){

  }

  for(int j=0;j<=i;j++){
    y[j]=msg.y[j];
 
  }
    return;


}

void chatterCallback2(const ros_start::f3darray& msg)//上と同じ
{
 
  for(i=0;msg.x[i]!=100;i++){
  }
  //printf("ookisa%d\n",i);
  for(int j=0;j<=i;j++){
    t[j]=msg.t[j];
  }
    return;
}
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)//set collision_objects into the planning_scene
{
  // Planning Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "obj1";//UR3の左右どちらかの障害物
  collision_objects[0].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.5;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.4;
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
  ros::NodeHandle node_handle;  
  ros::NodeHandle node_handle1;
  ros::NodeHandle node_handle2;
  ros::AsyncSpinner spinner(1); //スピナーの数？を決める（多いほどよい？）
  spinner.start();//スピナーをはじめる

 

     
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

 
  moveit::planning_interface::MoveGroupInterface group("manipulator");//動かしたいアーム（planning group）の設定--movegroupのどこがgroupなのか分からない時は、.srdf開けば行けるかも．
  group.setPlannerId("RRTConnectkConfigDefault");//逆運動学plannnerの設定．
  group.setPlanningTime(10.0);//計算時間（たぶん長いほど良い）

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());//動かすアームの最上位リンク情報をROSINFOで表示
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());//動かすアームの最下位エンドエフェクタのリンク情報をROSINFOで表示
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //Rvizにいろいろビジュアランジングするときに必要
  moveit::planning_interface::MoveGroupInterface::Plan my_planc;//実態化
  
  addCollisionObjects(planning_scene_interface);//insert object. go to addCollisionObjects function


  ros::Subscriber  sub = node_handle.subscribe("a_point", 1000, chatterCallback);//subscribe Astar planned data

  sleep(1.0);
 for(int i=0;x[i]!=100;i++){

  }
  for (int j=0;j<i;j++){  
    printf("x=%f\n",x[j]);
  }  
  


  ros::Subscriber  sub1 = node_handle1.subscribe("a_point", 1000, chatterCallback1);
  sleep(1.0);
   for(int i=0;y[i]!=100;i++){

  }

  for (int j=0;j<i;j++){
  printf("y=%f\n",y[j]);
  }  



  ros::Subscriber  sub2 = node_handle2.subscribe("a_point", 1000, chatterCallback2);
 
  sleep(1.0);
   for(int i=0;t[i]!=100;i++){

  }

  for (int j=0;j<i;j++){
  printf("t=%f\n",t[j]);
  }  


  tf::TransformListener listener1;
  tf::StampedTransform transform1;
  listener1.waitForTransform("/base_link", "/real_degss",now, ros::Duration(3.0));
  listener1.lookupTransform("/base_link","/real_degss",now,transform1);
 
  double deg[]={transform1.getRotation().x(),transform1.getRotation().y(),transform1.getRotation().z(),transform1.getRotation().w()};
  printf("%f\n%f\n%f\n%f\n",deg[0],deg[1],deg[2],deg[3]);
  q.x=deg[0];
  q.y=deg[1];
  q.z=deg[2];
  q.w=deg[3];
  tf::Quaternion q1(q.x, q.y, q.z, q.w);
  tf::Matrix3x3 m(q1);
  m.getRPY(roll, pitch, yaw);
  std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl; 

  geometry_msgs::PoseStamped target_posec;//目標座標、姿勢の型
  target_posec.header.stamp = ros::Time(0); //TFの関係によりたぶん現在時刻を設定する？
  target_posec.header.frame_id = "/base_link";//基準となる座標系
  
  target_posec.pose.orientation.x =deg[0]; //transform.getRotation().getX();//q.x; //quaternion,x 基準の座標系からみた姿勢
  target_posec.pose.orientation.y =deg[1]; //transform.getRotation().getY();//q.y; //quaternion,y
  target_posec.pose.orientation.z =deg[2]; //transform.getRotation().getZ();//q.z; //quaternion,z
  target_posec.pose.orientation.w =deg[3]; //transform.getRotation().getW();//q.w; //quaternion,w
  
  target_posec.pose.position.x =0;//transform.getOrigin().x();
  target_posec.pose.position.y =0.5;// transform.getOrigin().y();
  target_posec.pose.position.z =0.3;//transform.getOrigin().z();
  
  group.setPoseTarget(target_posec);
  group.plan(my_planc);
  group.execute(my_planc);
  ROS_INFO("way_point1");


  double ox=0.6;
  double oy=0.1;

  for(int k=0;k<i;k++){

  ROS_INFO("approachig_p1");

  std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl; 

  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;//実態化

  geometry_msgs::Pose target_pose1=group.getCurrentPose().pose;//group.getCurrentPose().poseで現在のエンドエフェクタの手先位置，姿勢を取得．
  geometry_msgs::PoseStamped target_pose3;//目標座標、姿勢の型

  target_pose3.header.stamp = ros::Time(0); //TFの関係によりたぶん現在時刻を設定する？

  target_pose3.header.frame_id = "/base_link";//基準となる座標系

  double  rolls;
  double  pitchs;
  double  yaws;

  rolls =roll;
  pitchs=pitch+(t[k]*(pai/180));
  yaws  =yaw;

  std::cout << "rolls: " << rolls << ", pitchs: " << pitchs << ", yaws: " << yaws << std::endl; 
  
  geometry_msgs::Quaternion s;
  geometry_msgs::Quaternion &q = s;
  
  tf::Quaternion quat=tf::createQuaternionFromRPY(rolls,pitchs,yaws);
  quaternionTFToMsg(quat,q);


  target_pose3.pose.orientation.x =q.x; //transform.getRotation().getX();//q.x; //quaternion,x 基準の座標系からみた姿勢
  target_pose3.pose.orientation.y =q.y; //transform.getRotation().getY();//q.y; //quaternion,y
  target_pose3.pose.orientation.z =q.z; //transform.getRotation().getZ();//q.z; //quaternion,z
  target_pose3.pose.orientation.w =q.w; //transform.getRotation().getW();//q.w; //quaternion,w
  std::cout << "x = " << q.x << std::endl;
  std::cout << "y = " << q.y << std::endl;
  std::cout << "z = " << q.z << std::endl;
  std::cout << "w = " << q.w << std::endl;
  target_pose3.pose.position.x =0;//transform.getOrigin().x();
  target_pose3.pose.position.y =ox+(x[k]*0.001);// transform.getOrigin().y();
  target_pose3.pose.position.z =oy+(y[k]*0.001);//transform.getOrigin().z();
  
  group.setPoseTarget(target_pose3);
  group.plan(my_plan3);
  group.execute(my_plan3);
  sleep(1.0);
  }

  ROS_INFO("FINISH!!");
  /*
  std::map<std::string, double> joints;
  joints["shoulder_pan_joint"] = -1.26;
  joints["shoulder_lift_joint"] = -0.64;
  joints["elbow_joint"] = -2.44;
  joints["wrist_1_joint"] = -0.66;
  joints["wrist_2_joint"] = 1.56;
  joints["wrist_3_joint"] = 0.007;  
  group.setJointValueTarget(joints);
  group.move();
  
  */
  
  while (ros::ok()){
  /*
  static tf::TransformBroadcaster br1;
            tf::Transform transform1;
            transform1.setOrigin(tf::Vector3(p1[0],p1[1],p1[2]));
            tf::Quaternion q1;
            transform2.setRotation(tf::Quaternion(q.x,q.y,q.z,q.w));
            br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "base_link", "rp1"));
  */
  /*
  static tf::TransformBroadcaster br2;
            tf::Transform transform2;
            transform2.setOrigin(tf::Vector3(0,0,0));
            tf::Quaternion q1;
            transform2.setRotation(tf::Quaternion(q.x,q.y,q.z,q.w));
            br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "pr1"));          
  */


  
  rate.sleep();
            }



}