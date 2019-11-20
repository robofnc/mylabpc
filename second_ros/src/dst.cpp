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
double vlo,aves=0,ref=0;


void chatterCallback(const sensor_msgs::JointState msg)//力覚センサを読み取る．独自のメッセージの型を作成し，使用．
{
  /**/
  
  double p[]={msg.effort[0],msg.effort[1],msg.effort[2],msg.effort[3],msg.effort[4],msg.effort[5]};
  printf("listen wrench%f\n",p[2]);

    aves=p[0]+p[1]+p[2]+p[3]+p[4]+p[5];
  /*for (int i = 0 ; i < 10 ; i++)
  {
    aves += msg.effort[2];
  }
  vlo=vlo*0.9+(aves/10)*0.1;
    */

    vlo=vlo*0.1+aves*0.9;
    ref=ref*0.99+aves*0.01;

  printf("vlo%f\n",vlo);
  printf("ref%f\n",ref);
  printf("fabs%f\n",fabs(ref-vlo));
  double diff=fabs(ref-vlo);
  printf("abs%f\n",fabs(ref-vlo));

  if(diff>0.20000){
      printf("flag1\n");
     flag= 1;
  }
    else
    {
        printf("flag0\n");
        flag=0;
    }
    
 /*
    if(0==0){
      printf("分岐%f\n",msg.effort[1]);
          flag=1;
    }
*/

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
  collision_objects[0].primitives[0].dimensions[2] = 1;

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
  collision_objects[1].primitives[0].dimensions[2] = 1;

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
  collision_objects[2].primitives[0].dimensions[0] = 0.4;
  collision_objects[2].primitives[0].dimensions[1] = 0.4;
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
  collision_objects[3].primitives[0].dimensions[2] = 1;

  /* Define the pose of the object. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = -0.5;
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
  double pitch = 100*(M_PI/180.0);//blue
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


  

  // SubscriberとしてchatterというトピックがSubscribeし、トピックが更新されたときは
  // chatterCallbackという名前のコールバック関数を実行する
  ros::Subscriber sub = node_handle.subscribe("joint_states", 1000, chatterCallback);
  
  
  std::map<std::string, double> joints;
  joints["shoulder_pan_joint"] = -1.26;
  joints["shoulder_lift_joint"] = -0.64;
  joints["elbow_joint"] = -2.44;
  joints["wrist_1_joint"] = -0.66;
  joints["wrist_2_joint"] = 1.56;
  joints["wrist_3_joint"] = 0.007;  
  group.setJointValueTarget(joints);
  group.move();




  while (node_handle.ok()){

  rate.sleep();
            }



}

  