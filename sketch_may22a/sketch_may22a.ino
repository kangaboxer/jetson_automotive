#include <Servo.h> 
Servo servo;        //Servoオブジェクトを作成

static int FWD_L = 2;  // 左モータのFWDピン
static int REV_L = 4;  // 左モータのREVピン
static int PWM_L = 3;  // 左モータのPWMピン

static int FWD_R = 5;  // 右モータのFWDピン
static int REV_R = 7;  // 右モータのFEVピン
static int PWM_R = 6;  // 右モータのPWMピン

int speed_L = 0; // 左モータの回転速度（0～255）
int speed_R = 0; // 左モータの回転速度（0～255）
int angle_S = 90;

void setup() {
  // 通信速度の設定
  Serial.begin(9600);

  // モータの制御ピンを出力に設定
  pinMode( FWD_L, OUTPUT );
  pinMode( REV_L, OUTPUT );
  pinMode( PWM_L, OUTPUT );
  pinMode( FWD_R, OUTPUT );
  pinMode( REV_R, OUTPUT );
  pinMode( PWM_R, OUTPUT );                       
  servo.attach(9);  //D9ピンをサーボの信号線として設定
}

void loop() {
  if ( Serial.available() >= 7 ) {
    int key = Serial.read();
    Serial.write(key);
    // LまたはRの後に続く数値を取得 
    if (key=='L') { 
      speed_L = readIntValue();
      Serial.print(speed_L); 
      while(Serial.available() > 0){
        int check = Serial.read();
        Serial.write(check);
        if(check == 'E') {
          break;
        }
      }
      Serial.write("\n");

    } 
    else if (key=='R') { 
      speed_R = readIntValue(); 
      Serial.print(speed_R); 
      while(Serial.available() > 0){
        int check = Serial.read();
        Serial.write(check);
        if(check == 'E') {
          break;
        }
      }
      Serial.write("\n");

    } 
    else if (key=='S') { 
      angle_S = readIntValue(); 
      Serial.print(angle_S);
      while(Serial.available() > 0){
        int check = Serial.read();
        Serial.write(check);
        if(check == 'E') {
          break;
        }
      }
      Serial.write("\n");

    }

    servo.write(angle_S);

    // 回転速度の設定
    analogWrite( PWM_L, abs(speed_L) );
    analogWrite( PWM_R, abs(speed_R) );

    // モータの回転制御
    if ( speed_L == 0 ) { 
      motor_stop( FWD_L, REV_L ); 
    }
    else if ( speed_L > 0 ) { 
      motor_forward( FWD_L, REV_L ); 
    }
    else if ( speed_L < 0 ) { 
      motor_reverse( FWD_L, REV_L ); 
    }

    if ( speed_R == 0 ) { 
      motor_stop( FWD_R, REV_R ); 
    }
    else if ( speed_R > 0 ) { 
      motor_forward( FWD_R, REV_R ); 
    }
    else if ( speed_R < 0 ) { 
      motor_reverse( FWD_R, REV_R ); 
    }
  }
}


// モータの正転
void motor_forward( int pin_fwd, int pin_rev ) {
  digitalWrite( pin_fwd, HIGH );
  digitalWrite( pin_rev, LOW );  
}

// モータの逆転
void motor_reverse( int pin_fwd, int pin_rev ) {
  digitalWrite( pin_fwd, LOW );
  digitalWrite( pin_rev, HIGH );  
}

// モータの停止
void motor_stop( int pin_fwd, int pin_rev ) {
  digitalWrite( pin_fwd, HIGH );
  digitalWrite( pin_rev, HIGH );  
}

// 数値を読み込む関数
int readIntValue() {
  int i = 0;
  char string[10];

  while ( i < sizeof(string) ) {
    if ( Serial.available() ) {
      char c = Serial.read();
      if ( (c>='0' && c<='9') || c=='-' ) {
        string[i] = c;
        i++;
      } 
      else {
        string[i] = '\0';       
        break;
      }
    }
  }
  Serial.write(string);
  Serial.write(",");
  return atoi(string);
}





