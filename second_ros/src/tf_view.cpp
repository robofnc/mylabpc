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


int flag=0;
double x=0;

void chatterCallback(const geometry_msgs::Point msg)//力覚せセンサを読み取る．独自のメッセージの型を作成し，使用．
{
  /**/
    if(0==msg.x){
          flag=1;
    }

}

double  plane_dect(double a[3], double b[3], double c[3])//接触した3点分の座標を元に，平面を算出し，各軸に対する傾きを計算し，メインに返信する関数．
{
  //ここから
  double AB[]={b[0]-a[0],b[1]-a[1],b[2]-a[2]};
  double AC[]={c[0]-c[0],c[1]-a[1],c[2]-a[2]};

  double A=(b[1]-a[1])*(c[2]-a[2])-(c[1]-a[1])*(b[2]-a[2]);
  double B=(b[2]-a[2])*(c[0]-a[0])-(c[2]-a[2])*(b[0]-a[0]);
  double C=(b[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(b[1]-a[1]);
  double D=-(A*a[0]+B*a[1]+C*a[2]);

  //printf("%fx+%fy+%fz+%f=0\n",A,B,C,D);//平面の方程式Ax+By+Cz+D=0で表示．
//ここまでは，三点の座標から，平面の方程式を算出している．

  double tx=atan(B/C);//x軸周りの傾き（回転）を計算以下y軸，ｚ軸周り，単位はradian.
  double ty=atan(A/C);
  double tz=atan(A/B);

  double degx=tx/(pai/180);//transform dradian->degree
  double degy=ty/(pai/180);
  double degz=tz/(pai/180);

  //printf("%f,%f\n%f,%f\n%f,%f\n",tx,degx,ty,degy,tz,degz);
  
  return(tx,ty,tz);
}



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
  
  
  //ここから
  double roll = 90*(M_PI/180.0);//green
  double pitch = 105*(M_PI/180.0);//blue
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


  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform("/base_link", "/p1_offset",now, ros::Duration(3.0));
  listener.lookupTransform("/base_link","/p1_offset",now,transform);
  
  double  f=transform.getOrigin().x();
  double  g=transform.getOrigin().y();
  double  h=transform.getOrigin().z();
  printf("X=%f\nY=%f\nZ=%f\n",f,g,h);
}