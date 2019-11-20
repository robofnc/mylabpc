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


void chatterCallback(const sensor_msgs::JointState msg)//力覚センサを読み取る．独自のメッセージの型を作成し，使用．
{
  /**/
  
  double p[]={msg.effort[0],msg.effort[1],msg.effort[2],msg.effort[3],msg.effort[4],msg.effort[5]};
  double q[]={msg.velocity[0],msg.velocity[1],msg.velocity[2],msg.velocity[3],msg.velocity[4],msg.velocity[5]};
  printf("listen wrench%f\n",p[2]);

    aves=fabs(p[0])+fabs(p[1])+fabs(p[2])+fabs(p[3])+fabs(p[4])+fabs(p[5]);
    vaves=fabs(q[0])+fabs(q[1])+fabs(q[2])+fabs(q[3])+fabs(q[4])+fabs(q[5]);
  /*for (int i = 0 ; i < 10 ; i++)
  {
    aves += msg.effort[2];
  }
  vlo=vlo*0.9+(aves/10)*0.1;
    */

    vlo=vlo*0.5+aves*0.5;
    ref=ref*0.99+aves*0.01;
    vvlo=vvlo*0.5+vaves*0.5;
    fref=fref*0.99+vaves*0.01;
  printf("vlo%f\n",vlo);
  printf("ref%f\n",ref);
  printf("vvlo%f\n",vvlo);
  printf("fref%f\n",fref);

  printf("effort_abs%f\n",fabs(ref-vlo));
  double diff=fabs(ref-vlo);
  double fdif=fabs(fref-vvlo);
  printf("velo_abs%f\n",fabs(vvlo-fref));

  if(diff>0.300||fdif>0.2){
      sleep(0.005);
      if (diff>0.300||fdif>0.2)
      {
          printf("flag1\n");
     flag= 1;
      }
  }
    else
    {
        printf("flag0\n");
        flag=0;
    }
    
}
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
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

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
  collision_objects[3].primitives[0].dimensions[2] = 0.4;

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
  group.setPlanningTime(45.0);//計算時間（たぶん長いほど良い）

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());//動かすアームの最上位リンク情報をROSINFOで表示
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());//動かすアームの最下位エンドエフェクタのリンク情報をROSINFOで表示
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //Rvizにいろいろビジュアランジングするときに必要

  addCollisionObjects(planning_scene_interface);
    
  //ここから
  double roll = 90*(M_PI/180.0);//green
  double pitch = 130*(M_PI/180.0);//blue
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




  ros::Rate rate(10.0);
  ros::Time now = ros::Time(0);


  
/*
  // SubscriberとしてchatterというトピックがSubscribeし、トピックが更新されたときは
  // chatterCallbackという名前のコールバック関数を実行する
  ros::Subscriber sub = node_handle.subscribe("joint_states", 1000, chatterCallback);
*/
tf::TransformListener listener1;
  tf::StampedTransform transform1;
  listener1.waitForTransform("/base_link", "/newmerical_p1",now, ros::Duration(3.0));
  listener1.lookupTransform("/base_link","/newmerical_p1",now,transform1);
  double np1[]={transform1.getOrigin().x(),transform1.getOrigin().y(),transform1.getOrigin().z()};

  tf::TransformListener listener2;
  tf::StampedTransform transform2;
  listener2.waitForTransform("/base_link", "/newmerical_2",now, ros::Duration(3.0));
  listener2.lookupTransform("/base_link","/newmerical_2",now,transform2);
  double np2[]={transform2.getOrigin().x(),transform2.getOrigin().y(),transform2.getOrigin().z()};

  double np[]={np2[0]-np1[0],np2[1]-np1[1],np2[2]-np1[2]};
  double  length=pow((np[0]*np[0])+(np[1]*np[1])+(np[2]*np[2]),0.5);
  np[0]=np[0]/length;
  np[1]=np[1]/length;
  np[2]=np[2]/length;
  //printf("%f\n%f\n%f\n",np[0],np[1],np[2]);
  
  
  double  pss1[]={0.25*np[0],
            0.25*np[1],
            0.25*np[2]};

  double  pss2[]={0.5*np[0],
          0.5*np[1],
          0.5*np[2]};

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

  tf::TransformListener listener12;
  tf::StampedTransform transform12;
  listener12.waitForTransform("/base_link", "/p2_offset",now, ros::Duration(3.0));
  listener12.lookupTransform("/base_link","/p2_offset",now,transform12);
  double np12[]={transform12.getOrigin().x(),transform12.getOrigin().y(),transform12.getOrigin().z()};
  printf("p2x=%f\np2y=%f\np2z=%f\n",np12[0],np12[1],np12[2]);

  tf::TransformListener listener3;
  tf::StampedTransform transform3;
  listener3.waitForTransform("/base_link", "/p3_offset",now, ros::Duration(3.0));
  listener3.lookupTransform("/base_link","/p3_offset",now,transform3);
  double np13[]={transform3.getOrigin().x(),transform3.getOrigin().y(),transform3.getOrigin().z()};
  printf("p3x=%f\np3y=%f\np3z=%f\n",np13[0],np13[1],np13[2]);
  
  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform("/base_link", "/p1_offset",now, ros::Duration(3.0));
  listener.lookupTransform("/base_link","/p1_offset",now,transform);

  tf::TransformListener listenerc;
  tf::StampedTransform transformc;
  listenerc.waitForTransform("/base_link","/cent",now, ros::Duration(3.0));
  listenerc.lookupTransform("/base_link","/cent",now,transformc);
  double cen[]={transformc.getOrigin().x(),transformc.getOrigin().y(),transformc.getOrigin().z()};
  /*
  while (node_handle.ok()){
  static tf::TransformBroadcaster bcs;
            tf::Transform transformnbcs;
            transformnbcs.setOrigin(tf::Vector3( cen[0], cen[1], cen[2]));
            bcs.sendTransform(tf::StampedTransform(transformnbcs, ros::Time::now(), "base_link", "deaadfgasg"));       
  
  rate.sleep();
            }
  */
  double  f=transform.getOrigin().x();
  double  g=transform.getOrigin().y();
  double  h=transform.getOrigin().z();
  printf("p1x=%f\np1y=%f\np1z=%f\n",f,g,h);

  
    
  moveit::planning_interface::MoveGroupInterface::Plan my_planc;//実態化

  geometry_msgs::PoseStamped target_posec;//目標座標、姿勢の型
  target_posec.header.stamp = ros::Time(0); //TFの関係によりたぶん現在時刻を設定する？
  target_posec.header.frame_id = "/base_link";//基準となる座標系
  
  target_posec.pose.orientation.x =q.x; //transform.getRotation().getX();//q.x; //quaternion,x 基準の座標系からみた姿勢
  target_posec.pose.orientation.y =q.y; //transform.getRotation().getY();//q.y; //quaternion,y
  target_posec.pose.orientation.z =q.z; //transform.getRotation().getZ();//q.z; //quaternion,z
  target_posec.pose.orientation.w =q.w; //transform.getRotation().getW();//q.w; //quaternion,w
  
  target_posec.pose.position.x =cen[0];//transform.getOrigin().x();
  target_posec.pose.position.y =cen[1];// transform.getOrigin().y();
  target_posec.pose.position.z =cen[2];//transform.getOrigin().z();
  
  group.setPoseTarget(target_posec);
  group.plan(my_planc);
  group.execute(my_planc);
  ROS_INFO("way_point1");
  

  geometry_msgs::PoseStamped target_pose1;//目標座標、姿勢の型
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;//実態化
 
  target_pose1.header.stamp = ros::Time(0); //TFの関係によりたぶん現在時刻を設定する？
  target_pose1.header.frame_id = "/base_link";//基準となる座標系
  
  target_pose1.pose.orientation.x =q.x; //transform.getRotation().getX();//q.x; //quaternion,x 基準の座標系からみた姿勢
  target_pose1.pose.orientation.y =q.y; //transform.getRotation().getY();//q.y; //quaternion,y
  target_pose1.pose.orientation.z =q.z; //transform.getRotation().getZ();//q.z; //quaternion,z
  target_pose1.pose.orientation.w =q.w; //transform.getRotation().getW();//q.w; //quaternion,w
  
  target_pose1.pose.position.x =transform.getOrigin().x();//transform.getOrigin().x();
  target_pose1.pose.position.y =transform.getOrigin().y();// transform.getOrigin().y();
  target_pose1.pose.position.z =transform.getOrigin().z();//transform.getOrigin().z();
  
  group.setPoseTarget(target_pose1);
  group.plan(my_plan1);
  group.execute(my_plan1);
  ROS_INFO("atempted p1offset");

    // SubscriberとしてchatterというトピックがSubscribeし、トピックが更新されたときは
  // chatterCallbackという名前のコールバック関数を実行する
  ros::Subscriber sub = node_handle.subscribe("joint_states", 1000, chatterCallback);
  sleep(3.0);

  geometry_msgs::PoseStamped target_pose2;

  flag=0;
  double p1[]={0,0,0};
  std::vector<geometry_msgs::Pose> waypoints;// 経由地点の作成，接触点までアプローチする．
  for(int i=0;flag!=1;i++){
  ROS_INFO("approachig_p1");
  geometry_msgs::Pose target_pose1=group.getCurrentPose().pose;//group.getCurrentPose().poseで現在のエンドエフェクタの手先位置，姿勢を取得．
  //target_pose2.orientation.x=q.x;
  //target_pose2.orientation.y=q.y;
  //target_pose2.orientation.z=q.z;
  //target_pose2.orientation.w=q.w;
  target_pose1.position.x +=0.005*np[0];
  target_pose1.position.y +=0.005*np[1];
  target_pose1.position.z +=0.005*np[2];
  p1[0]=target_pose1.position.x;
  p1[1]=target_pose1.position.y;
  p1[2]=target_pose1.position.z;
  waypoints.push_back(target_pose1);
  group.setPoseTarget(target_pose1);
  group.move();
  sleep(1.0);
  }

  ROS_INFO("1st step finished");
  printf("t_p1x=%f\nt_p1y=%f\nt_p1z=%f\n",p1[0],p1[1],p1[2]);



 

  moveit::planning_interface::MoveGroupInterface::Plan my_plan12;//実態化

  geometry_msgs::PoseStamped target_pose12;//目標座標、姿勢の型
  target_pose12.header.stamp = ros::Time(0); //TFの関係によりたぶん現在時刻を設定する？
  target_pose12.header.frame_id = "/base_link";//基準となる座標系
  target_pose12.pose.orientation.x =q.x; //transform.getRotation().getX();//q.x; //quaternion,x 基準の座標系からみた姿勢
  target_pose12.pose.orientation.y =q.y; //transform.getRotation().getY();//q.y; //quaternion,y
  target_pose12.pose.orientation.z =q.z; //transform.getRotation().getZ();//q.z; //quaternion,z
  target_pose12.pose.orientation.w =q.w; //transform.getRotation().getW();//q.w; //quaternion,w

  target_pose12.pose.position.x =np12[0];//transform.getOrigin().x();
  target_pose12.pose.position.y =np12[1];// transform.getOrigin().y();
  target_pose12.pose.position.z =np12[2];//transform.getOrigin().z();
  
  group.setPoseTarget(target_pose12);
  group.plan(my_plan12);
  group.execute(my_plan12);
  ROS_INFO("atempted p2ofsset");
  double t_p2[]={target_pose12.pose.position.x ,target_pose12.pose.position.y,target_pose12.pose.position.z};
  printf("t_p2x=%f\nt_p2y=%f\nt_p2z=%f\n",t_p2[0],t_p2[1],t_p2[2]);


sleep(3.0);

flag=0;
sub = node_handle.subscribe("joint_states", 1000, chatterCallback);
geometry_msgs::PoseStamped target_pose22;


  double p2[]={0,0,0};
  std::vector<geometry_msgs::Pose> waypoints2;// 経由地点の作成，接触点までアプローチする．
  for(int i=0;flag!=1;i++){
  ROS_INFO("approachig_p2");
  geometry_msgs::Pose target_pose2=group.getCurrentPose().pose;//group.getCurrentPose().poseで現在のエンドエフェクタの手先位置，姿勢を取得．
  //target_pose2.orientation.x=q.x;
  //target_pose2.orientation.y=q.y;
  //target_pose2.orientation.z=q.z;
  //target_pose2.orientation.w=q.w;
  target_pose2.position.x +=0.005*np[0];
  target_pose2.position.y +=0.005*np[1];
  target_pose2.position.z +=0.005*np[2];
  p2[0]=target_pose2.position.x;
  p2[1]=target_pose2.position.y;
  p2[2]=target_pose2.position.z;
  waypoints2.push_back(target_pose2);
  group.setPoseTarget(target_pose2);
  group.move();
  sleep(1.0);
  }

  
  ROS_INFO("2st step finished");
  printf("t_p2x=%f\nt_p2y=%f\nt_p2z=%f\n",p2[0],p2[1],p2[2]);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;//実態化

  geometry_msgs::PoseStamped target_pose3;//目標座標、姿勢の型
  target_pose3.header.stamp = ros::Time(0); //TFの関係によりたぶん現在時刻を設定する？
  target_pose3.header.frame_id = "/base_link";//基準となる座標系
  target_pose3.pose.orientation.x =q.x; //transform.getRotation().getX();//q.x; //quaternion,x 基準の座標系からみた姿勢
  target_pose3.pose.orientation.y =q.y; //transform.getRotation().getY();//q.y; //quaternion,y
  target_pose3.pose.orientation.z =q.z; //transform.getRotation().getZ();//q.z; //quaternion,z
  target_pose3.pose.orientation.w =q.w; //transform.getRotation().getW();//q.w; //quaternion,w

  target_pose3.pose.position.x =np13[0];//transform.getOrigin().x();
  target_pose3.pose.position.y =np13[1];// transform.getOrigin().y();
  target_pose3.pose.position.z =np13[2];//transform.getOrigin().z();
  
  group.setPoseTarget(target_pose3);
  group.plan(my_plan3);
  group.execute(my_plan3);
  ROS_INFO("atempted p3ofsset");
  double t_p3[]={target_pose3.pose.position.x ,target_pose3.pose.position.y,target_pose3.pose.position.z};
  printf("t_p3x=%f\nt_p3y=%f\nt_p3z=%f\n",t_p3[0],t_p3[1],t_p3[2]);


sleep(3.0);
flag=0;
sub = node_handle.subscribe("joint_states", 1000, chatterCallback);

  double p3[]={0,0,0};
  std::vector<geometry_msgs::Pose> waypoints3;// 経由地点の作成，接触点までアプローチする．
  for(int i=0;flag!=1;i++){
  ROS_INFO("approachig_p3");
  geometry_msgs::Pose target_pose3=group.getCurrentPose().pose;//group.getCurrentPose().poseで現在のエンドエフェクタの手先位置，姿勢を取得．
  //target_pose2.orientation.x=q.x;
  //target_pose2.orientation.y=q.y;
  //target_pose2.orientation.z=q.z;
  //target_pose2.orientation.w=q.w;
  target_pose3.position.x +=0.005*np[0];
  target_pose3.position.y +=0.005*np[1];
  target_pose3.position.z +=0.005*np[2];
  p3[0]=target_pose3.position.x;
  p3[1]=target_pose3.position.y;
  p3[2]=target_pose3.position.z;
  waypoints3.push_back(target_pose3);
  group.setPoseTarget(target_pose3);
  group.move();
  sleep(1.0);
  }

  ROS_INFO("3st step finished");
  printf("t_p3x=%f\nt_p3y=%f\nt_p3z=%f\n",p3[0],p3[1],p3[2]);


  std::map<std::string, double> joints;
  joints["shoulder_pan_joint"] = -1.26;
  joints["shoulder_lift_joint"] = -0.64;
  joints["elbow_joint"] = -2.44;
  joints["wrist_1_joint"] = -0.66;
  joints["wrist_2_joint"] = 1.56;
  joints["wrist_3_joint"] = 0.007;  
  group.setJointValueTarget(joints);
  group.move();

  //ここから

  double a[]={p1[0],p1[1],p1[2]};
  double b[]={p2[0],p2[1],p2[2]};
  double c[]={p3[0],p3[1],p3[2]};

  double AB[]={b[0]-a[0],b[1]-a[1],b[2]-a[2]};
  double AC[]={c[0]-c[0],c[1]-a[1],c[2]-a[2]};

  double A=(b[1]-a[1])*(c[2]-a[2])-(c[1]-a[1])*(b[2]-a[2]);
  double B=(b[2]-a[2])*(c[0]-a[0])-(c[2]-a[2])*(b[0]-a[0]);
  double C=(b[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(b[1]-a[1]);
  double D=-(A*a[0]+B*a[1]+C*a[2]);

  printf("%fx+%fy+%fz+%f=0\n",A,B,C,D);//平面の方程式Ax+By+Cz+D=0で表示．
  //ここまでは，三点の座標から，平面の方程式を算出している．

  double tx=atan(B/C);//x軸周りの傾き（回転）を計算以下y軸，ｚ軸周り，単位はradian.
  double ty=atan(A/C);
  double tz=atan(A/B);

  double degx=tx/(pai/180);//transform dradian->degree
  double degy=ty/(pai/180);
  double degz=tz/(pai/180);



  

  while (node_handle.ok()){
  /*
  static tf::TransformBroadcaster br1;
            tf::Transform transform1;
            transform1.setOrigin(tf::Vector3(target_pose3.pose.position.x,target_pose3.pose.position.y,target_pose3.pose.position.z));
            //tf::Quaternion q1;
            //q1.setRPY(-1.8,1.3,-1.82);
            transform1.setRotation(tf::Quaternion(transform.getRotation().getX(), 
                                                    transform.getRotation().getY(), 
                                                    transform.getRotation().getZ(),
                                                    transform.getRotation().getW()));
            br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "base_link", "p1ss"));
  */

  static tf::TransformBroadcaster br2;
            tf::Transform transform2;
            transform2.setOrigin(tf::Vector3(t_p2[0],t_p2[1],t_p2[2]));
            //tf::Quaternion q1;
            //q1.setRPY(-1.8,1.3,-1.82);
            transform2.setRotation(tf::Quaternion(transform.getRotation().getX(), 
                                                    transform.getRotation().getY(), 
                                                    transform.getRotation().getZ(),
                                                    transform.getRotation().getW()));
            br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "n111"));          
  /*
  static tf::TransformBroadcaster br3;
            tf::Transform transform3;
            transform3.setOrigin(tf::Vector3(np12[0],np12[1],np12[2]));
            //tf::Quaternion q1;
            //q1.setRPY(-1.8,1.3,-1.82);
            transform3.setRotation(tf::Quaternion(q.x,q.y,q.z,q.w));
            br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "base_link", "n22"));  
    */
  
  rate.sleep();
            }
  /*
  double A,B,C,D;
  double a[]={1,2,-2};
  double b[]={3,-2,1};
  double  c[]={5,1,-4};


  A,B,C,D=plane_dect(a,b,c);
  */


}