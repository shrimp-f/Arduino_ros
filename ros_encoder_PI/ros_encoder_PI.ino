#include <MsTimer2.h>

#define Kp 0.01
#define Ki 0
#define Kd 0.0005
#define PID_SCALE 40

#define SERIAL
#define LOOP_DELAY 200

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

void Rcount(){
  motor_Rvel++;
}
void Lcount(){
  motor_Lvel++;
}

void reset() {
  wheel_Rvel = vel_mag * float(motor_Rvel) / reduction_ratio *wheel_circumference;
  wheel_Lvel = vel_mag * float(motor_Lvel) / reduction_ratio *wheel_circumference;
  pre_motor_Rvel = motor_Rvel;
  pre_motor_Lvel = motor_Lvel;
  motor_Rvel = 0;
  motor_Lvel = 0;
}


void setup() {
  pinMode(13, OUTPUT);

  pinMode(44, OUTPUT);//R motor 
  pinMode(45, OUTPUT);//R
  pinMode(7, OUTPUT);//L
  pinMode(8, OUTPUT);//L


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

void loop() {
  float target_wheel_Lvel = 1500.;
  float target_wheel_Rvel = 1500.;

//  float target_motor_Lvel = target_wheel_Lvel / vel_mag / wheel_circumference *reduction_ratio;
//  float target_motor_Rvel = target_wheel_Rvel / vel_mag / wheel_circumference *reduction_ratio;

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

//    SerialMonitor();
  #endif
  
  delay(LOOP_DELAY);
}


inline void SerialMonitor(){
  
/*  
  Serial.print("wheel_vel: ");
  Serial.print(wheel_Lvel);
  Serial.print(" ");
  Serial.println(wheel_Rvel);
*/
}
