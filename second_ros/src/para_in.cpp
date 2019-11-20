// ros/ros.h　ROSに関する基本的なAPIのためのヘッダ
#include "ros/ros.h"
// comp_tutrial/adder.h　adder.msgから生成されたメッセージを定義しているヘッダ
#include "second_ros/adder.h"

int main(int argc, char **argv)
{
  // 初期化のためのAPI
  // このノードは"para_in"という名前であるという意味
  ros::init(argc, argv, "para_in");

  // ノードハンドラの宣言
  ros::NodeHandle n;

  //Publisherとしての定義
  // n.advertise<comp_tutorial::adder>("para_input", 1000);
  // comp_tutorial::adder型のメッセージをpara_inputというトピックへ配信する
  //"1000"はトピックキューの最大値
  ros::Publisher para_pub = n.advertise<second_ros::adder>("para_input", 1000);

  //1秒間に1つのメッセージをPublishする
  ros::Rate loop_rate(1);

  //comp_tutrial::adder型のオブジェクトを定義
  //adder.msgで定義したa,bはメンバ変数としてアクセスできる
  second_ros::adder msg;

  int count = 0;
  while (ros::ok())//ノードが実行中は基本的にros::ok()=1
  {
    msg.a = count;
    msg.b = count;
    para_pub.publish(msg);//PublishのAPI
    printf("a = %d b = %d \n",msg.a , msg.b );
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
  return 0;
}