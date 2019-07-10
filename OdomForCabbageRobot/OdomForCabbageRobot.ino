/* 
 * rosserial Planar Odometry Example
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <MsTimer2.h>

#define Kp 0.01
#define Ki 0
#define Kd 0.0005
#define PID_SCALE 40

#define SERIAL
#define LOOP_DELAY 200

//モーター制御まわり
volatile int motor_Lvel = 0;//motor rotation
volatile int pre_motor_Lvel = 0;//
volatile float wheel_Lvel = 0;
volatile int motor_Rvel = 0;//motor rotation
volatile int pre_motor_Rvel = 0;//
volatile float wheel_Rvel = 0;

// PID
float duty_R = 100.;
float duty_L = 100.;
float dt, preTime;
float P_R, I_R, D_R, preP_R;
float P_L, I_L, D_L, preP_L;

const float reduction_ratio = 280;//減速比
const float wheel_circumference = 810;//mm 円周
const int reset_msec = 100;//msec,timer割り込みの周期
float vel_mag = 1000./float(reset_msec);//magnification//タイヤの速度計算のための係数


//ROSまわり
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

//AttachInterruptのcallback
void Rcount(){
  motor_Rvel++;
}
void Lcount(){
  motor_Lvel++;
}

//timerのcallback
void reset() {
  wheel_Rvel = vel_mag * float(motor_Rvel) / reduction_ratio *wheel_circumference;
  wheel_Lvel = vel_mag * float(motor_Lvel) / reduction_ratio *wheel_circumference;
  pre_motor_Rvel = motor_Rvel;
  pre_motor_Lvel = motor_Lvel;
  motor_Rvel = 0;
  motor_Lvel = 0;
}

//subscriberのcallback
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

  MsTimer2::set(reset_msec, reset); // 500msごとにオンオフ
  MsTimer2::start();

//  attachInterrupt(2, Rcount, FALLING);//21pin
  attachInterrupt(3, Rcount, CHANGE);//22pin
//  attachInterrupt(4, Lcount, FALLING);//19pin
  attachInterrupt(5, Lcount, CHANGE);//18pin

  Serial.begin(9600);

  preTime = micros();
  duty_R = 100.0;
  duty_L = 100.0;

}

void loop()
{  
  float target_wheel_Lvel = 1500.;
  float target_wheel_Rvel = 1500.;

  float target_motor_Rvel = 60.;
  float target_motor_Lvel = 60.;
  ////// PID //////
  dt = (micros() - preTime) / 1000000;
  preTime = micros();
  P_R  = target_motor_Rvel - pre_motor_Rvel;
  P_L  = target_motor_Lvel - pre_motor_Lvel;
//  Serial.print("P ");
//  Serial.println(P);
  
//  I += P * dt;
  D_R  = (P_R - preP_R) / dt;
  D_L  = (P_L - preP_L) / dt;
  preP_R = P_R;
  preP_L = P_L;

//  duty += Kp * P + Ki * I + Kd * D;

  duty_R += PID_SCALE* (Kp * P_R + Kd * D_R);
//  duty_R += PID_SCALE* (Kp * P_R);
  duty_R = constrain(duty_R,0,240);
  duty_L += PID_SCALE* (Kp * P_L + Kd * D_L);
//  duty_L += PID_SCALE* (Kp * P_L);
  duty_L = constrain(duty_L,0,240);
  
  /////////////////


//  analogWrite(44, 100);
  analogWrite(44, duty_R);
  analogWrite(45,0);
  analogWrite(8, duty_L);
  analogWrite(7,0);


  #ifdef SERIAL
    Serial.print("target_motor_vel:");
    Serial.print(target_motor_Lvel);
    Serial.print(",");
    Serial.print(target_motor_Rvel);
    Serial.print(" duty:");
    Serial.print(duty_L);
    Serial.print(",");
    Serial.print(duty_R);
    Serial.print(" motor_vel:");
//    Serial.print(pre_motor_Lvel);
//    Serial.print(" ");
    Serial.println(pre_motor_Lvel);
    Serial.print(",");
    Serial.println(pre_motor_Rvel);
  #endif

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
