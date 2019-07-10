#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;//ros node
geometry_msgs::Twist Twist_msg;

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

void setup(){
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(44,OUTPUT);
  pinMode(45,OUTPUT);
  pinMode(13,OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}


void loop(){

  nh.spinOnce();
  delay(1);
}
