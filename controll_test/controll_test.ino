#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor[3] = {AFMS.getMotor(1),AFMS.getMotor(2),AFMS.getMotor(3)};
#define pwm_upper 255
#define spd_upper 10
#define thre_wait 10
#define integ_upper 200
#define zero_width 5
//#define PI 3.141592653589793   

const int pinA1 = 2;//A相 割り込み0
const int pinB1 = 3;//B相 割り込み1
const int pinA2 = 4;//A相 割り込み2
const int pinB2 = 5;//B相 割り込み3
const int pinA3 = 6;//A相 割り込み4
const int pinB3 = 7;//B相 割り込み5
const double L = 0.22; //車体半径[m]
const double R = 0.05; //車輪半径[m]
const double alp = PI/6;
const double trans_mat[3][3] = {{-sin(alp), cos(alp), L},
                                {-sin(alp+PI*2/3), cos(alp+PI*2/3),L},
                                {-sin(alp-PI*2/3), cos(alp-PI*2/3),L}};

volatile int enc_count[3] = {0,0,0};
const int ts=10; //millisec
const int gear_ratio = 30;
const int cpr = 64; //count per revolution
const int ratio = gear_ratio * cpr;
int count_old[3] = {0,0,0};
//int t_old = 0;
int t,tc,t3,t_delay = 0;
double omega[3] = {0,0,0};
double integ[3] = {0,0,0};
double h_integ = 0;
int wait = 0;
void setup()
{
  Serial.begin(115200);
  pinMode(pinA1,INPUT);
  pinMode(pinB1,INPUT);
  pinMode(pinA2,INPUT);
  pinMode(pinB2,INPUT);
  pinMode(pinA3,INPUT);
  pinMode(pinB3,INPUT);
  attachInterrupt(pinA1, enc_changedPinA1, CHANGE); //pinAの信号変化に合わせて割り込み処理
  attachInterrupt(pinB1, enc_changedPinB1, CHANGE); //pinBの信号変化に合わせて割り込み処理
  attachInterrupt(pinA2, enc_changedPinA2, CHANGE); //pinAの信号変化に合わせて割り込み処理
  attachInterrupt(pinB2, enc_changedPinB2, CHANGE); //pinBの信号変化に合わせて割り込み処理
  attachInterrupt(pinA3, enc_changedPinA3, CHANGE); //pinAの信号変化に合わせて割り込み処理
  attachInterrupt(pinB3, enc_changedPinB3, CHANGE); //pinBの信号変化に合わせて割り込み処理
  AFMS.begin();  // create with the default frequency 1.6KHz

    while (!Serial) {
    ; //シリアル通信ポートが正常に接続されるまで抜け出さない
  }
}



double diff[3][5]={{0,0,0,0,0},
                   {0,0,0,0,0},
                   {0,0,0,0,0}};
double pid(double sensor,double target,int pin_n){
  double kp = 10;
  double kv = 0;
  double ki = 30;
  double pu;
  double du;
  double iu;
  int u_pid;
  double error;
  diff[pin_n][4] = diff[pin_n][3];
  diff[pin_n][3] = diff[pin_n][2];
  diff[pin_n][2] = diff[pin_n][1];
  diff[pin_n][1] = diff[pin_n][0];
  diff[pin_n][0] = sensor;
  error = (target - diff[pin_n][0]);
  pu = error * kp;
  du = (diff[pin_n][4] - diff[pin_n][0]) * kv;
  integ[pin_n] += error * ts * 0.001;
  if(integ[pin_n] > integ_upper){
    integ[pin_n] = integ_upper;
  }else if(integ[pin_n] < -integ_upper){
    integ[pin_n] = -integ_upper;
  }
  iu = integ[pin_n] * ki;
  u_pid = pu+iu+du;
  if(u_pid>pwm_upper){
    u_pid = pwm_upper;
  }else if(u_pid < - pwm_upper){
    u_pid = -pwm_upper;
  }
  return u_pid;
}

double mv[3];
//int center = 128;
int center = 25;
double tgt_spd = 0;
int posi = 0 ;
const int val_size = 2;
int values[val_size] = {0, 0};
bool isValids[val_size] = {false, false};
int t_old = millis();
double xdot_vec[3] = {0,0,0};
double v_vec[3] = {0,0,0};
double omega_d[3] = {0,0,0};

void culc_v(){
  for(int i=0;i<3;i++){
    omega_d[i] = ( trans_mat[i][0]*xdot_vec[0]
                  + trans_mat[i][1]*xdot_vec[1]
                  + trans_mat[i][2]*xdot_vec[2] ) / R;
  }
}

double xdot = 0.1;
double thetadot = PI/6;
const double coeff = 2*PI*1000/(ts*ratio);
void loop()
{ 
  tc = millis();
  t3 = tc - t_old - t_delay;
  //Serial.println(t_old);
  t_old = tc;
  t_delay = ts - t3;
  
  //Serial.println(t3);
  /*
  for(int i = 0; i < 3; i++){
    omega[i] = (double)(enc_count[i] - count_old[i])/ts/ratio * 1000;
    count_old[i] = enc_count[i];
  }
  */
  int tc1 = millis();
  
  int inputchar; //入力状態の読み取りに使う
  if (Serial.available() > 0) {
             
    inputchar = Serial.read(); //シリアル通信で送信された値を読み取る
    //Serial.write(inputchar);
    
  //if(inputchar!=-1){
    switch(inputchar){
      //前進
      case 'w':
      xdot_vec[1] += xdot;
      Serial.print("w");
      break;
      
      //後進
      case 's':
      xdot_vec[1] -= xdot;
      Serial.print("s");
      break;
      
      //左折
      case 'a':
      xdot_vec[0] -= xdot;
      Serial.print("a");
      break;
      
      //右折
      case 'd':
      xdot_vec[0] += xdot;
      Serial.print("d");
      break;
      //全停止
      case 'x':
      for(int j=0;j<3;j++){
        xdot_vec[j] = 0;
        integ[j] = 0;
      }
      Serial.print("x");
      break;
      
      //並進停止
      case '6':
      xdot_vec[0] = 0; 
      xdot_vec[1] = 0;
      Serial.print("e");
      break;
      //右旋回
      case 'e':
      xdot_vec[2] += thetadot; // [rad/sec]
      Serial.print("w");
      break;
      
      //左旋回
      case 'q':
      xdot_vec[2] -= thetadot; // [rad/sec]
      Serial.print("s");
      break;
      //回転停止
      case '9':
      xdot_vec[2] = 0; // [rad/sec]
      Serial.print("s");
      break;
      }
  }
  culc_v();
  
  //Serial.println(posi);
  for(int i = 0; i < 3; i++){
    omega[i] = (enc_count[i] - count_old[i]) * coeff;
    count_old[i] = enc_count[i];
    mv[i] = pid(omega[i],omega_d[i],i);
    if(mv[i]>0){
      myMotor[i]->run(BACKWARD);
      myMotor[i]->setSpeed(mv[i]);
    }else{
      myMotor[i]->run(FORWARD);
      myMotor[i]->setSpeed(-mv[i]);
    }
  }
  
  Serial.println(tgt_spd);
  for(int i = 1; i < 2; i++){
  Serial.print(i);
  Serial.print(": ");
  Serial.print(" ");
  Serial.print(omega_d[i]);
  Serial.print(" ");
  Serial.print(omega[i]);
  Serial.print(" ");
  Serial.println(mv[i]);
  }
  Serial.println(t_delay);
  Serial.println();
  
  int tc2 = millis();
  
  //Serial.println(mv[2]);
  //t_old = millis();
  if(t_delay < 0){t_delay = 0;}
  delay(t_delay);
  //Serial.println(t_delay);
  t += ts;
}
void count_inc(int pin_n)
{
  ++enc_count[pin_n];
}
void count_dec(int pin_n)
{
  --enc_count[pin_n];
}
//pinAの割り込み処理
void enc_changedPinA1()
{ 
  if(digitalRead(pinA1))
  {
    if(digitalRead(pinB1)) count_dec(0);
    else count_inc(0);
  } else {
    if(digitalRead(pinB1)) count_inc(0);
    else count_dec(0);
  }
}
//pinBの割り込み処理
void enc_changedPinB1()
{ 
  if(digitalRead(pinB1))
  {
    if(digitalRead(pinA1)) count_inc(0);
    else count_dec(0);
  } else {
    if(digitalRead(pinA1)) count_dec(0);
    else count_inc(0);
  }
}
//pinAの割り込み処理
void enc_changedPinA2()
{ 
  if(digitalRead(pinA2))
  {
    if(digitalRead(pinB2)) count_dec(1);
    else count_inc(1);
  } else {
    if(digitalRead(pinB2)) count_inc(1);
    else count_dec(1);
  }
}
//pinBの割り込み処理
void enc_changedPinB2()
{ 
  if(digitalRead(pinB2))
  {
    if(digitalRead(pinA2)) count_inc(1);
    else count_dec(1);
  } else {
    if(digitalRead(pinA2)) count_dec(1);
    else count_inc(1);
  }
}
//pinAの割り込み処理
void enc_changedPinA3()
{
  if(digitalRead(pinA3))
  {
    if(digitalRead(pinB3)) count_dec(2);
    else count_inc(2);
  } else {
    if(digitalRead(pinB3)) count_inc(2);
    else count_dec(2);
  }
}
//pinBの割り込み処理
void enc_changedPinB3()
{   
  if(digitalRead(pinB3))
  { 
    if(digitalRead(pinA3)) count_inc(2);
    else count_dec(2);
  } else {
    if(digitalRead(pinA3)) count_dec(2);
    else count_inc(2);
  }
}
