/* 
 * rosserial Planar Odometry Example
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <MsTimer2.h>
#include <math.h>  /* M_PI */

#define Kp 0.01
#define Ki 0
#define Kd 0.0005
#define PID_SCALE 40

#define _SERIAL
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
const float wheel_circumference = 0.810;//m 円周
const double tread = 0.610;//m トレッド幅
const int reset_msec = 100;//msec,timer割り込みの周期
//float vel_mag = 1000./float(reset_msec);//magnification//タイヤの速度計算のための係数
const double encoder_reso = 21.5;//エンコーダー1週あたりのカウントすう 減速比280で1周で約6000回てんだったので。

long long all_count_L = 0;//とりあえず置く。カウント積算用。オーバーフローするので対策する。
long long all_count_R = 0;//とりあえず置く。カウント積算用。オーバーフローするので対策する。

//ROSまわり
ros::NodeHandle  nh;
geometry_msgs::Twist Twist_msg;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double one_count_distance = 0.01; //エンコーダー1カウントで進む距離
double left_distance = 0.0; //左車輪の累積移動距離→こいつらをTFに使う
double right_distance = 0.0; //右車輪の累積移動距離

// TF用
double x = 0.0;
double y = 0.0;
double theta = 1.57;
//global wheel odometry
double x_wo = 0;
double y_wo = 0;
double theta_wo = 0;
//変位　loop内で計算して、timer割り込み時に足す
double x_wo_dis = 0;
double y_wo_dis = 0;
double theta_wo_dis = 0;


char base_link[] = "/base_link";
char odom[] = "/odom";

//AttachInterruptのcallback
void Rcount(){
  motor_Rvel++;
  all_count_R++;
}
void Lcount(){
  motor_Lvel++;
  all_count_L++;
}

//timerのcallback
void reset() {
//  wheel_Rvel = vel_mag * float(motor_Rvel) / reduction_ratio *wheel_circumference;
//  wheel_Lvel = vel_mag * float(motor_Lvel) / reduction_ratio *wheel_circumference;
  pre_motor_Rvel = motor_Rvel;
  pre_motor_Lvel = motor_Lvel;
  motor_Rvel = 0;
  motor_Lvel = 0;

  x_wo += x_wo_dis;
  y_wo += y_wo_dis;
  theta_wo += theta_wo_dis;

  x = x_wo;
  y = y_wo;
  theta = theta_wo;

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

void rosinfo(int val){
  char info[7];
  info[0] = '0' + val /100000;
  info[1] = '0' + val % 100000 /10000;
  info[2] = '0' + val % 10000 /1000;
  info[3] = '0' + val % 1000 /100;
  info[4] = '0' + val % 100 /10;
  info[5] = '0' + val % 10;
  info[6] = '\0';
  nh.loginfo(info);
  
}
void rosinfo(double val){
  char info[7];
  info[0] = '0' + val /100000;
  info[1] = '0' + (int)val % 100000 /10000;
  info[2] = '0' + (int)val % 10000 /1000;
  info[3] = '0' + (int)val % 1000 /100;
  info[4] = '0' + (int)val % 100 /10;
  info[5] = '0' + (int)val % 10;
  info[6] = '\0';
  nh.loginfo(info);
  
}

void rosprint(int num){
  char str1[30]={0}; //0で埋める
  sprintf(str1,"%d",num);  
  nh.loginfo(str1);
}
void rosprint(double num){
  char str1[30]={0}; //0で埋める
  dtostrf(num,10,6,str1);
  nh.loginfo(str1);
}
void rosprint(char *num){
  char str1[30]={0}; //0で埋める
  sprintf(str1,"aaaa%s",*num);  
  nh.loginfo(str1);
}


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
  attachInterrupt(3, Rcount, CHANGE);//20pin
//  attachInterrupt(4, Lcount, FALLING);//19pin
  attachInterrupt(5, Lcount, CHANGE);//18pin

  #ifdef SERIAL
//    Serial.begin(9600);
  #endif

  preTime = micros();
  duty_R = 0.0;
  duty_L = 0.0;
}




void loop()
{  
//  float target_wheel_Lvel = 1500.;
//  float target_wheel_Rvel = 1500.;
  static double target_motor_Rvel = 0.;//60. default
  static double target_motor_Lvel = 0.;    

  if(all_count_L < 6000000 ){
    target_motor_Rvel = 90.;//60. default
    target_motor_Lvel = 90.;    
  }else{
    target_motor_Rvel = 0.;//60. default
    target_motor_Lvel = 0.;    
    char flag[6];
    flag[0] = 'f';
    flag[1] = 'l';
    flag[2] = 'a';
    flag[3] = 'g';
    flag[4] = 'z';
    flag[5] = '\0';
//    nh.loginfo(flag);
  }
//  rosinfo(all_count_L);
//  rosinfo(duty_R);
//  rosinfo(motor_Rvel);

  
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

/*
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
*/

  // drive in a circle
  /*
  double dx = 0.2;
  double dtheta = 0.18;
  x += cos(theta)*dx*0.1;
  y += sin(theta)*dx*0.1;
  theta += dtheta*0.1;
  if(theta > 3.14)
    theta=-3.14;
  */

  /**************
   * ホイールオドメトリによる自己位置推定
   * d_ は⊿の意味
   * 足しあわせているのはtimer割り込みのcallbackない
  ***************/
  //sで計算
//  double wr = pre_motor_Rvel / (reset_msec/1000.) / encoder_reso *2*M_PI;
//  double wl = pre_motor_Lvel / (reset_msec/1000.) / encoder_reso *2*M_PI;
  const double te = reset_msec/1000.; //s
  // m/0.1s
  double d_vr = wheel_circumference * pre_motor_Rvel / encoder_reso / reduction_ratio;
  double d_vl = wheel_circumference * pre_motor_Lvel / encoder_reso / reduction_ratio;
  char words3[20] = "pre_motor_Rvel";
  nh.loginfo(words3);
  rosprint(pre_motor_Rvel);

  //m
  double d_Lr = d_vr;
  double d_Ll = d_vl;

  char words[6] = "d_vr";
  nh.loginfo(words);
  rosprint(d_vr);
  char words2[6] = "d_vl";
  nh.loginfo(words2);
  rosprint(d_vl);

  double d_L = (d_Lr + d_Ll) * 0.5;
  double d_theta = (d_Lr - d_Ll) / tread;
  double rho = 0; //旋回半径

  theta_wo_dis = d_theta;

  if(d_theta > 0.01){
    rho = d_L / d_theta;
    double d_L_dash = 2. * rho * sin(d_theta /2.);
    x_wo_dis = d_L_dash * cos(theta_wo - 0.5*d_theta);
    y_wo_dis = d_L_dash * sin(theta_wo - 0.5*d_theta);
  }else{
    x_wo_dis = d_L * cos(theta_wo - 0.5*d_theta);
    y_wo_dis = d_L * sin(theta_wo - 0.5*d_theta);
  }
/*
  char flag[6] = "d_Lr";
  nh.loginfo(flag);
  rosprint(d_Lr);
  char flag2[6] = "d_Ll";
  nh.loginfo(flag2);
  rosprint(d_Ll);
  char flag3[6] = "d_L";
  nh.loginfo(flag3);
  rosprint(d_L);
*/
  /****************/

  
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
