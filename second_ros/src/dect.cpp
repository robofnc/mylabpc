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




int main(int argc, char **argv)
{
  ros::init(argc, argv, "bdect");//いつものROSの宣言
  ros::NodeHandle node_handle;  //今回は別にいらない、通信するときとか
  ros::AsyncSpinner spinner(1); //スピナーの数？を決める（多いほどよい？）
  spinner.start();//スピナーをはじめる

  ros::Rate rate(100.0);
  ros::Time now = ros::Time(0);




tf::TransformListener listener1;
  tf::StampedTransform transform1;
  listener1.waitForTransform("/base_link", "/housenn",now, ros::Duration(3.0));
  listener1.lookupTransform("/base_link","/housenn",now,transform1);
  double np1[]={transform1.getOrigin().x(),transform1.getOrigin().y(),transform1.getOrigin().z()};
  printf("x=%f\ny=%f\nz=%f\n",np1[0],np1[1],np1[2]);

tf::TransformListener listener2;
  tf::StampedTransform transform2;
  listener2.waitForTransform("/base_link", "/pr2",now, ros::Duration(3.0));
  listener2.lookupTransform("/base_link","/pr2",now,transform2);
  double p1[]={transform2.getOrigin().x(),transform2.getOrigin().y(),transform2.getOrigin().z()};
  printf("x=%f\ny=%f\nz=%f\n",p1[0],p1[1],p1[2]);


tf::TransformListener listener3;
  tf::StampedTransform transform3;
  listener3.waitForTransform("/base_link", "/housenn1",now, ros::Duration(3.0));
  listener3.lookupTransform("/base_link","/housenn1",now,transform3);
  double p2[]={transform3.getOrigin().x(),transform3.getOrigin().y(),transform3.getOrigin().z()};
  printf("x=%f\ny=%f\nz=%f\n",p2[0],p2[1],p2[2]);


tf::TransformListener listener4;
  tf::StampedTransform transform4;
  listener4.waitForTransform("/base_link", "/cent",now, ros::Duration(3.0));
  listener4.lookupTransform("/base_link","/cent",now,transform4);
  double cent[]={transform4.getOrigin().x(),transform4.getOrigin().y(),transform4.getOrigin().z()};
  printf("x=%f\ny=%f\nz=%f\n",cent[0],cent[1],cent[2]);  

  double  sg1=pow((np1[0]-cent[0])*(np1[0]-cent[0])+(np1[1]-cent[1])*(np1[1]-cent[1])+(np1[2]-cent[2])*(np1[2]-cent[2]),0.5);
  double  sg2=pow((p2[0]-cent[0])*(p2[0]-cent[0])+(p2[1]-cent[1])*(p2[1]-cent[1])+(p2[2]-cent[2])*(p2[2]-cent[2]),0.5);
  printf("sg1=%f\nsg2=%f\n",sg1,sg2);

  if(sg1>sg2){
    np1[0]=np1[0];
    np1[1]=np1[1];
    np1[2]=np1[2];
  }
  else
  {
    np1[0]=p2[0];
    np1[1]=p2[1];
    np1[2]=p2[2];
  }

  printf("define_hose-n\nx=%f\ny=%f\nz=%f\n",np1[0],np1[1],np1[2]);
  

  double a[]={0,np1[1],np1[2]};
  double yz[]={0,0,1};

  double  la=pow((a[0]*a[0])+(a[1]*a[1])+(a[2]*a[2]),0.5);
  double  lyz=pow((yz[0]*yz[0])+(yz[1]*a[1])+(yz[2]*yz[2]),0.5);

  double ayz=a[2];

  double radx=acos(ayz/(la*lyz));
  double degx=radx/(pai/180);//transform dradian->degree

printf("rotate.x%f\n",degx);

double b[]={np1[0],np1[1],0};
double xy[]={0,1,0};
double  lb=pow((b[0]*b[0])+(b[1]*b[1])+(b[2]*b[2]),0.5);
double  lxy=pow((xy[0]*xy[0])+(xy[1]*xy[1])+(xy[2]*xy[2]),0.5);
double bxy=b[1];

double radz=acos(bxy/(lb*lxy));
double degz=radz/(pai/180);
printf("rotate.z%f\n",degz);
double roll=0;
double pitch=0;//y ao

if(radx>3.14/2){
  roll=(((3.14/2)-radx)+(3.14/2));
}
else
{
  roll =(((3.14/2)-radx)+(3.14/2));//x aka
}



if(0>np1[0]){
  pitch=(-(3.14-radz))+(3.14/2);
}
else
{
  pitch = ((3.14-radz))+(3.14/2);//x aka
}





double yaw = 3.14/2;//z

geometry_msgs::Quaternion s;
geometry_msgs::Quaternion &q = s;
  

tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
quaternionTFToMsg(quat,q);

std::cout << "x = " << q.x << std::endl;
std::cout << "y = " << q.y << std::endl;
std::cout << "z = " << q.z << std::endl;
std::cout << "w = " << q.w << std::endl;
//ここまでは、roll,pitch,yawをquaternionに変換している
  while (ros::ok()){


  static tf::TransformBroadcaster br2;
            tf::Transform transform2;
            transform2.setOrigin(tf::Vector3(p1[0],p1[1],p1[2]));
            transform2.setRotation(tf::Quaternion(q.x,q.y,q.z,q.w));
            br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "real_deg"));          

  rate.sleep();
            }


}