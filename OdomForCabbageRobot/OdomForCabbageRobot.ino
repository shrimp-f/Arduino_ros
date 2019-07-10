/* 
 * rosserial Planar Odometry Example
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <MsTimer2.h>


ros::NodeHandle  nh;
geometry_msgs::Twist Twist_msg;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;


double one_count_distance = 0.01; //エンコーダー1カウントで進む距離
double left_distance = 0.0; //左車輪の累積移動距離→こいつらをTFに使う
double right_distance = 0.0; //右車輪の累積移動距離


double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom[] = "/odom";

void get_message(const geometry_msgs::Twist& cmd_msg){
  digitalWrite(13,HIGH);
  
  if(cmd_msg.linear.x >= 0){
    digitalWrite(13,HIGH);
    analogWrite(7,cmd_msg.linear.x*250);
    analogWrite(8,0);
    delay(1);
  }else if(cmd_msg.linear.x < 0){
    digitalWrite(13,LOW);
    analogWrite(7,0);
    analogWrite(8,abs(cmd_msg.linear.x*250));
    delay(1);
  }

  if(cmd_msg.angular.z >= 0){
    analogWrite(44,cmd_msg.angular.z*250);
    analogWrite(45,0);
    delay(1);
  }else if(cmd_msg.angular.z < 0){
    analogWrite(44,0);
    analogWrite(45,abs(cmd_msg.angular.z*250));
    delay(1);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("/cabbage1/cmd_vel", get_message);


void setup()
{
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(44,OUTPUT);
  pinMode(45,OUTPUT);
  pinMode(13,OUTPUT);


  
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(sub);
}

void loop()
{  
  // drive in a circle
  double dx = 0.2;
  double dtheta = 0.18;
  x += cos(theta)*dx*0.1;
  y += sin(theta)*dx*0.1;
  theta += dtheta*0.1;
  if(theta > 3.14)
    theta=-3.14;
    
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  nh.spinOnce();
  
  delay(10);
}
