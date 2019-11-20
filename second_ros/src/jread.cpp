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
  printf("abs%d\n",abs(ref-vlo));

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



  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }
   moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "world";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "world";
  /* The id of the object */
  attached_object.object.id = "box";
  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.8;
  primitive.dimensions[1] = 1.0;
  primitive.dimensions[2] = -0.1;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

// Note that attaching an object to the robot requires 
// the corresponding operation to be specified as an ADD operation
  attached_object.object.operation = attached_object.object.ADD;
  ROS_INFO("Adding the object into the world at the location of the right wrist.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
 

  
  moveit_msgs::AttachedCollisionObject attached_object1;
  attached_object1.link_name = "thin_link";
 
  attached_object1.object.header.frame_id = "thin_link";

  attached_object1.object.id = "box";

  geometry_msgs::Pose pose1;
  pose1.orientation.w = 1.0;

 
  shape_msgs::SolidPrimitive primitive1;
  primitive1.type = primitive1.BOX;
  primitive1.dimensions.resize(3);
  primitive1.dimensions[0] = 0.8;
  primitive1.dimensions[1] = 1.0;
  primitive1.dimensions[2] = -0.2;

  attached_object1.object.primitives.push_back(primitive1);
  attached_object1.object.primitive_poses.push_back(pose1);

  attached_object1.object.operation = attached_object1.object.ADD;
  ROS_INFO("Adding the object into the world at the location of the right wrist.");
  moveit_msgs::PlanningScene planning_scene1;
  planning_scene.world.collision_objects.push_back(attached_object1.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);


  ros::Rate rate(10.0);
  ros::Time now = ros::Time(0);

  
  /*
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

  double  f=transform.getOrigin().x();
  double  g=transform.getOrigin().y();
  double  h=transform.getOrigin().z();
  printf("p1x=%f\np1y=%f\np1z=%f\n",f,g,h);
  */

  

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
  
  rate.sleep();
            }



}

  



  
    

/*
  for(int i=0;flag!=1;i++){


  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;//二個目のタスクの宣言（タスクごとに毎回宣言）
  //robot_state::RobotState start_state(*group.getCurrentState());
  //geometry_msgs::Pose start_pose2;
  //start_pose2.orientation.w = 1.0;
  //start_pose2.position.x = 0.55;
  //start_pose2.position.y = -0.05;
  //start_pose2.position.z = 0.8;
  //const robot_state::JointModelGroup *joint_model_group =
  //                start_state.getJointModelGroup(group.getName());
  //start_state.setFromIK(joint_model_group, target_pose1);
  //group.setStartState(start_state);


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
  target_pose2.pose.position.x = 00.1;
  //target_pose2.pose.position.x = 0.0;
  target_pose2.pose.position.y = x=0.55+(i*0.01);
  target_pose2.pose.position.z = 0.3;
  group.setPoseTarget(target_pose2);
  group.plan(my_plan2);
  group.execute(my_plan2);

  ROS_INFO("Planfinished!!!!!!!!!!");
  }

  printf("q.x%f\n",q.x);
  printf("x%f\n",x);
*/
