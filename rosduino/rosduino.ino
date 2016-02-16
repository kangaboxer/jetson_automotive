/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ArduinoHardware.h>
#include <Servo.h> 
#include <LiquidCrystal.h>

#define FWD_L 30  // 左モータのFWDピン
#define PWM_R_L 8  // 左モータのREVピン
#define PWM_F_L 11  // 左モータのPWMピン
#define FWD_R 31  // 右モータのFWDピン
#define PWM_R_R 12  // 右モータのFEVピン
#define PWM_F_R 10  // 右モータのPWMピン
#define DEFAULT_ANGLE 3
#define SERVO 9  // Servo pin
#define SPEAKER 50  // speaker pin
#define US_TRIG 33 // Ultrasonic trigger
#define US_ECHO 35  // Ultrasonic echo

int speed_F = 0; // 左モータの回転速度（0～255）
int angle_S = 90;
float Distance;
int Duration;
Servo servo;        //Servoオブジェクトを作成
ros::NodeHandle  nh;
LiquidCrystal lcd(45,43,53,51,49,47);

void messageCb( const geometry_msgs::Twist& msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  angle_S = -1.0 * (msg.angular.z + 1.73) * 180.0 / 3.14 + DEFAULT_ANGLE;
  speed_F = (msg.linear.x * 255);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("vel:");
  lcd.print(speed_F);
  lcd.print(",rad:");
  lcd.print(angle_S-90);
  
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(1);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(11);
  digitalWrite(US_TRIG, LOW);
  Duration = pulseIn(US_ECHO, HIGH);
  if (Duration > 0) {
    Distance = Duration/2;
    Distance = Distance*340*100/1000000;
    lcd.setCursor(0,1);
    lcd.print("dist:");
    lcd.print(Distance);
  }
  if ( Distance < 15 && speed_F < 0 ) {
    speed_F = 0;
  }
  
  // モータの回転制御
  if ( speed_F == 0 ) { 
    analogWrite( PWM_F_L, 0 );
    analogWrite( PWM_R_L, 0 );
    analogWrite( PWM_F_R, 0 );
    analogWrite( PWM_R_R, 0 );
  }
  else if ( speed_F > 0 ) { 
    // 回転速度の設定
    analogWrite( PWM_F_L, abs(speed_F) );
    analogWrite( PWM_R_L, 0 );
    analogWrite( PWM_F_R, abs(speed_F) );
    analogWrite( PWM_R_R, 0 );
  }
  else if ( speed_F < 0 ) { 
    // 回転速度の設定
    analogWrite( PWM_F_L, 0 );
    analogWrite( PWM_R_L, abs(speed_F) );
    analogWrite( PWM_F_R, 0 );
    analogWrite( PWM_R_R, abs(speed_F) );
  }
  servo.write(angle_S);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void setup()
{ 
  // Set baund rate
  nh.getHardware()->setBaud(115200);
  // Set LED pin
  pinMode(13, OUTPUT);
  //Set LCD 
  analogWrite(5, 70);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Setup:waiting...");
  //Set Ultrasonic
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  // ROS init
  nh.initNode();
  nh.subscribe(sub);
  // Init DC motors
  pinMode( FWD_L, OUTPUT );
  pinMode( PWM_R_L, OUTPUT );
  pinMode( PWM_F_L, OUTPUT );
  pinMode( FWD_R, OUTPUT );
  pinMode( PWM_R_R, OUTPUT );
  pinMode( PWM_F_R, OUTPUT );                       
  digitalWrite( FWD_L, HIGH );
  digitalWrite( FWD_R, HIGH );
  //Init servo
  servo.attach(SERVO);  //SERVOピンをサーボの信号線として設定
  servo.write((angle_S - DEFAULT_ANGLE));
  
  // Check Ultrasonic
  while(true) {
    digitalWrite(US_TRIG, LOW);
    delayMicroseconds(1);
    digitalWrite(US_TRIG, HIGH);
    delayMicroseconds(11);
    digitalWrite(US_TRIG, LOW);
    Duration = pulseIn(US_ECHO, HIGH);
    if (Duration > 0) {
      break;
    }
    delay(100);
  }
  
  // Finish setup
  tone(SPEAKER,2000,300);  
  delay(350);  
  tone(SPEAKER,2000,300);
  lcd.clear();
  lcd.setCursor(0,0);  
  lcd.print("Setup: Ok"); 
  lcd.setCursor(0,1);  
  lcd.print("Starting...");
}

void loop()
{  
  
  nh.spinOnce();
  delay(1);

}

