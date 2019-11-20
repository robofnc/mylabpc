/* Author: Koshi Makihara,Takumi Hachimine */
/*
ROSkineticとindigo でヘッダ名とかが微妙に違うので注意
あと　C++でプログラム作ると、CMakeList とか,いちいち変更するのめんどいし,いきなりROSを誰にも
教えられずにやるのは、死ぬ...てかさ，ｃ++で書くと変更したら，その都度catkin_makeしなきゃいけないよね
python保存するだけで反映されるのに．
あと，依存関係もCMake.textに書かなならん．初学者殺しやな．
cとpythonどちらかで記述できればいいんじゃないかって思うんだけど
参考サイトごとに，言語が違うから，結局両言語ともやらなならん．
わかる奴に聞いたほうが早いし，聞いた人間によるけど，ちゃんと教えてくれる人に当たれば，
ちゃんと使えるようになります．
*/

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

void chatterCallback(const geometry_msgs::Point msg)//力覚センサを読み取る．独自のメッセージの型を作成し，使用．
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
  primitive.dimensions[0] = 1;
  primitive.dimensions[1] = 1;
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
  //sleep_time.sleep();
  ROS_INFO("Plan 1 finished!!!!!!!!!!");
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



  /*
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;//実態化

  geometry_msgs::PoseStamped target_pose1;//目標座標、姿勢の型
  target_pose1.header.stamp = ros::Time::now(); //TFの関係によりたぶん現在時刻を設定する？
  target_pose1.header.frame_id = "/base_link";//基準となる座標系
  target_pose1.pose.orientation.x = q.x; //quaternion,x 基準の座標系からみた姿勢
  target_pose1.pose.orientation.y = q.y; //quaternion,y
  target_pose1.pose.orientation.z = q.z; //quaternion,z
  target_pose1.pose.orientation.w = q.w; //quaternion,w
  target_pose1.pose.position.x = 0.1;
  target_pose1.pose.position.y = 0.5;
  target_pose1.pose.position.z = 0.4;

  group.setPoseTarget(target_pose1); //planning groupに座標、姿勢を設定
  group.plan(my_plan);//必要（1タスクごと）
  group.execute(my_plan);//上の行で計画したタスクを実機で実行（目標座標姿勢にエンドエフェクタを動かす）
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
  
  ROS_INFO("Plan 1 finished!!!!!!!!!!");

  */
  

  



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
  ROS_INFO("FUCKIN");

  geometry_msgs::PoseStamped target_pose1;//目標座標、姿勢の型
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;//実態化
  /*
  
  moveit_msgs::OrientationConstraint ocm1;  //パスの制約（動かすときに姿勢を変えたくない時）
  ocm1.link_name = "ee_link";  //姿勢を変えたくないリンク
  ocm1.header.frame_id = "/base_link"; //基準座標系
  
  ocm1.orientation.x = q.x;
  ocm1.orientation.y = q.y;
  ocm1.orientation.z = q.z;
  ocm1.orientation.w = q.w;
  
  ocm1.absolute_x_axis_tolerance = 0.05; //x許容交差（低いほど正確）
  ocm1.absolute_y_axis_tolerance = 0.05;//y許容交差
  ocm1.absolute_z_axis_tolerance = 0.05;//ｚ許容交差
  ocm1.weight = 2.0;
  
  moveit_msgs::Constraints test_constraints1; //制約の宣言
  test_constraints1.orientation_constraints.push_back(ocm1); //制約を入れ込む  
  group.setPathConstraints(test_constraints1); //入れこまれた制約をplanning groupに適用
  */

  //geometry_msgs::PoseStamped target_pose1;//目標座標、姿勢の型
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
  ros::Subscriber sub = node_handle.subscribe("chatter", 1000, chatterCallback);
  
  
  geometry_msgs::PoseStamped target_pose2;
  /*
  for(int i=0;flag!=1;i++){

  ROS_INFO("approach p1");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
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

  
  target_pose2.header.stamp = ros::Time(0);
  target_pose2.header.frame_id = "/base_link";
  target_pose2.pose.orientation.x = q.x;
  target_pose2.pose.orientation.y = q.y;
  target_pose2.pose.orientation.z = q.z;
  target_pose2.pose.orientation.w = q.w;
 
  target_pose2.pose.position.x = transform.getOrigin().x()+i*0.01*np[0];
  target_pose2.pose.position.y = transform.getOrigin().y()+i*0.01*np[1];
  target_pose2.pose.position.z = transform.getOrigin().z()+i*0.01*np[2];
  group.setPoseTarget(target_pose2);
  group.plan(my_plan2);
  group.execute(my_plan2);
  sleep(3.0);
  }
  */
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

  /*
    std::map<std::string, double> joints1;
  joints1["shoulder_pan_joint"] = -1.26;
  joints1["shoulder_lift_joint"] = -0.64;
  joints1["elbow_joint"] = -2.44;
  joints1["wrist_1_joint"] = -0.66;
  joints1["wrist_2_joint"] = 1.56;
  joints1["wrist_3_joint"] = 0.007;  
  group.setJointValueTarget(joints1);
  group.move();
  */
  

 

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


sleep(2.0);
flag=0;
sub = node_handle.subscribe("chatter", 1000, chatterCallback);
geometry_msgs::PoseStamped target_pose22;

/*
for(int i=0;flag!=1;i++){

  ROS_INFO("approach p2");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan22;
  moveit_msgs::OrientationConstraint ocm2;  //パスの制約（動かすときに姿勢を変えたくない時）
  ocm2.link_name = "ee_link";  //姿勢を変えたくないリンク
  ocm2.header.frame_id = "/base_link"; //基準座標系
  ocm2.orientation.x = q.x;
  ocm2.orientation.y = q.y;
  ocm2.orientation.z = q.z;
  ocm2.orientation.w = q.w;
  ocm2.absolute_x_axis_tolerance = 0.05; //x許容交差（低いほど正確）
  ocm2.absolute_y_axis_tolerance = 0.05;//y許容交差
  ocm2.absolute_z_axis_tolerance = 0.05;//ｚ許容交差
  ocm2.weight = 2.0;
  
  
  moveit_msgs::Constraints test_constraints2; //制約の宣言
  test_constraints2.orientation_constraints.push_back(ocm2); //制約を入れ込む  
  group.setPathConstraints(test_constraints2); //入れこまれた制約をplanning groupに適用

  
  target_pose22.header.stamp = ros::Time(0);
  target_pose22.header.frame_id = "/base_link";
  target_pose22.pose.orientation.x = q.x;
  target_pose22.pose.orientation.y = q.y;
  target_pose22.pose.orientation.z = q.z;
  target_pose22.pose.orientation.w = q.w;
 
  target_pose22.pose.position.x = np12[0]+i*0.01*np[0];
  target_pose22.pose.position.y = np12[1]+i*0.01*np[1];
  target_pose22.pose.position.z = np12[2]+i*0.01*np[2];
  group.setPoseTarget(target_pose22);
  group.plan(my_plan22);
  group.execute(my_plan22);
  sleep(3.0);
  
  }
  */

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


sleep(2.0);
flag=0;
sub = node_handle.subscribe("chatter", 1000, chatterCallback);
/*
geometry_msgs::PoseStamped target_pose33;
for(int i=0;flag!=1;i++){

  ROS_INFO("approach p3");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan33;
  moveit_msgs::OrientationConstraint ocm3;  //パスの制約（動かすときに姿勢を変えたくない時）
  ocm3.link_name = "ee_link";  //姿勢を変えたくないリンク
  ocm3.header.frame_id = "/base_link"; //基準座標系
  ocm3.orientation.x = q.x;
  ocm3.orientation.y = q.y;
  ocm3.orientation.z = q.z;
  ocm3.orientation.w = q.w;
  ocm3.absolute_x_axis_tolerance = 0.05; //x許容交差（低いほど正確）
  ocm3.absolute_y_axis_tolerance = 0.05;//y許容交差
  ocm3.absolute_z_axis_tolerance = 0.05;//ｚ許容交差
  ocm3.weight = 2.0;
  
  
  moveit_msgs::Constraints test_constraints3; //制約の宣言
  test_constraints3.orientation_constraints.push_back(ocm3); //制約を入れ込む  
  group.setPathConstraints(test_constraints3); //入れこまれた制約をplanning groupに適用

  
  target_pose33.header.stamp = ros::Time(0);
  target_pose33.header.frame_id = "/base_link";
  target_pose33.pose.orientation.x = q.x;
  target_pose33.pose.orientation.y = q.y;
  target_pose33.pose.orientation.z = q.z;
  target_pose33.pose.orientation.w = q.w;
 
  target_pose33.pose.position.x = np13[0]+i*0.01*np[0];
  target_pose33.pose.position.y = np13[1]+i*0.01*np[1];
  target_pose33.pose.position.z = np13[2]+i*0.01*np[2];
  group.setPoseTarget(target_pose33);
  group.plan(my_plan33);
  group.execute(my_plan33);

  sleep(3.0);
  
  }
  */
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
