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

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#define pai 3.14159265

// %Tag(CALLBACK)%
/*
void chatterCallback(const std_msgs::String::ConstPtr& msg)
  {
  
  printf("ommc[%s]\n",msg->data.c_str());
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  }
*/


int flag=0;
double x=0;

/*
void chatterCallback(const geometry_msgs::Point msg)
{
    //int result;
    //result = msg.a + msg.b;
    //printf("a:%d + b:%d = %d\n",msg.a , msg.b, result );
    //printf("%f\n",msg.x);
    if(0==msg.x){
      flag=1;
    }

}
*/



int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_movements");//いつものROSの宣言
  ros::NodeHandle node_handle;  //今回は別にいらない、通信するときとか
  ros::AsyncSpinner spinner(1); //スピナーの数？を決める（多いほどよい？）
  spinner.start();//スピナーをはじめる

  moveit::planning_interface::MoveGroupInterface group("manipulator");//動かしたいアーム（planning group）の設定--movegroupのどこがgroupなのか分からない時は、.srdf開けば行けるかも．
   

  group.setPlannerId("RRTConnectkConfigDefault");//逆運動学plannnerの設定．
  group.setPlanningTime(15.0);//計算時間（たぶん長いほど良い）


  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());//動かすアームの最上位リンク情報をROSINFOで表示
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());//動かすアームの最下位エンドエフェクタのリンク情報をROSINFOで表示
  
  
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //Rvizにいろいろビジュアランジングするときに必要
  
  /*マニピュレータの初期姿勢（jointごとの角度（ラジアン）を指定）を決定する*/
  
  std::map<std::string, double> joints;
  joints["shoulder_pan_joint"] = -1.26;
  joints["shoulder_lift_joint"] = -0.68;
  joints["elbow_joint"] = -2.44;
  joints["wrist_1_joint"] = -0.66;
  joints["wrist_2_joint"] = 1.56;
  joints["wrist_3_joint"] = 0.007;  
  group.setJointValueTarget(joints);
  group.move();
  

/*
  geometry_msgs::Pose goal;
  goal.position.x = 0.255;
  goal.position.y = 0.246;
  goal.position.z = 0.256;
  goal.orientation.w = 0.991428;
  goal.orientation.x = 0.0085588;
  goal.orientation.y = -0.087665;
  goal.orientation.z = -0.0964952;
  group.setPoseTarget(goal);
  group.move();
*/

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
  std::cout << "wscatkisddd = " << q.w << std::endl;

//ここまでは、roll,pitch,yawをquaternionに変換している
  /*
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;//実態化

  geometry_msgs::PoseStamped target_pose1;//目標座標、姿勢の型
  target_pose1.header.stamp = ros::Time::now(); //TFの関係によりたぶん現在時刻を設定する？
  target_pose1.header.frame_id = "/base_link";//基準となる座標系
  target_pose1.pose.orientation.x = -0.636; //quaternion,x 基準の座標系からみた姿勢
  target_pose1.pose.orientation.y = -0.307; //quaternion,y
  target_pose1.pose.orientation.z = 0.699; //quaternion,z
  target_pose1.pose.orientation.w = 0.113; //quaternion,w
  target_pose1.pose.position.x = -0.039;
  target_pose1.pose.position.y = -0.263;
  target_pose1.pose.position.z = 0.258;

  group.setPoseTarget(target_pose1); //planning groupに座標、姿勢を設定

  //std::vector<geometry_msgs::Pose> waypoints;//経由地点を設定（未実装）

  //geometry_msgs::Pose target_pose3 = target_pose1;
  //target_pose3.position.x += 0.5;
  //target_pose3.position.z -= 0.9;
  //waypoints.push_back(target_pose3);  // up and out

  //target_pose3.position.y -= 0.2;
  //waypoints.push_back(target_pose3);  // left

  //target_pose3.position.z -= 0.2;
  //target_pose3.position.y += 0.2;
  //target_pose3.position.x -= 0.2;
  //waypoints.push_back(target_pose3);  // down and right (back to start)

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively
  // disabling it.
  //moveit_msgs::RobotTrajectory trajectory;
  //double fraction = group.computeCartesianPath(waypoints,
  //                                             0.01,  // eef_step
  //                                            0.0,   // jump_threshold
  //                                             trajectory);
  //std::vector<double> group_variable_values;
  //group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  
  //group_variable_values[0] = -1.0;  
  //group_variable_values[1] = -1.0;  
  //group.setJointValueTarget(group_variable_values);


  group.plan(my_plan);//必要（1タスクごと）
  group.execute(my_plan);//上の行で計画したタスクを実機で実行（目標座標姿勢にエンドエフェクタを動かす）
  */
  ROS_INFO("Plan 1 finished!!!!!!!!!!");

};

  



  
    

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
