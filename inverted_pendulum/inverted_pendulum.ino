#include <TimerOne.h>

int i;
volatile int pot_target=0;
volatile int pot_current=0;
volatile double epsilon_sum=0; //偏差積分値
volatile double old_epsilon=0; //前回の偏差
volatile double epsilon; //偏差
volatile int output,pid_mv;
volatile double P,I,D;
//////////// パラメータ ////////////////
const double Kp=17.0;  //比例ゲイン
const double Ki=0.01;  //積分ゲイン
const double Kd=13.5;  //微分ゲイン
//////////////////////////////////////

void setup() {
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  delay(1000);
  for(i=1;i<100;i++) pot_target +=analogRead(A5); //0~1023
  pot_target=pot_target/i;
  Timer1.initialize();
  Timer1.attachInterrupt(pid_ctl,1000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
 * pin7 | pin8 | OUTPUT
 * ----------------------
 * LOW  | HIGH | 右回転
 * ----------------------
 * HIGH | LOW  | 左回転
 */
  if(output>=0){
    analogWrite(6,output);
    digitalWrite(7,HIGH);
    digitalWrite(8,LOW);
    
  }
  else{
    analogWrite(6,-output);
    digitalWrite(7,LOW); 
    digitalWrite(8,HIGH); 
  }
}

//PID制御
void pid_ctl(){
  pot_current=analogRead(A5);
  epsilon=(double)pot_target-pot_current; //偏差を計算
  epsilon_sum +=epsilon;  //偏差を積分
  
  /*P,I,D項の算出*/
  P=Kp*epsilon;
  I=Ki*epsilon_sum;
  D=Kd*(epsilon-old_epsilon);
  pid_mv=P+I+D;
  
  output=constrain(pid_mv,-255,255);  //-255~255の範囲に限定
  old_epsilon=epsilon;
}
