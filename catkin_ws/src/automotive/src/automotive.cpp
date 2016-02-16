#if 0
#include <stdio.h>  
#include <stdlib.h>  
#include <unistd.h>  
#include <fcntl.h>  
#include <string.h>  
#include <termios.h>  
#include <time.h>  
#include <sys/wait.h>  
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

      
/* <asm/termbits.h> で定義されているボーレートの設定．これは 
   <termios.h>からインクルードされる． */  
#define BAUDRATE B9600  
//Arduinoのソフトウェアシリアルでも使えるように9600bpsにする  
      
/* 適切なシリアルポートを指すように，この定義を変更 
 * 我が家の環境ではArduinoは/dev/ttyACM0になってました*/  
#define MODEMDEVICE "/dev/arduino"  
      
#define BUFFSIZE 256  
#define COULDNOTFORK -1  
      
#define FALSE 0  
#define TRUE 1  
      
volatile int STOP = FALSE;  
static int fd = -1;  
      
/* Functions */  
void serial_init(int fd);  
void child_process();  
void parent_process(pid_t result_pid);  

      
// シリアルポートの初期化  
void serial_init(int fd) {  
  struct termios tio;  
  memset(&tio, 0, sizeof(tio));  
  tio.c_cflag = CS8 | CLOCAL | CREAD;  
  /* 
     BAUDRATE: ボーレートの設定．cfsetispeed と cfsetospeed も使用できる． 
     CS8     : 8n1 (8 ビット，ノンパリティ，ストップビット 1) 
     CLOCAL  : ローカル接続，モデム制御なし 
     CREAD   : 受信文字(receiving characters)を有効にする． 
  */  
      
  tio.c_cc[VTIME] = 0; /* キャラクタ間タイマは未使用 */  
      
  /* 
     ICANON  : カノニカル入力(行単位入力）を有効にする 
  */  
  tio.c_lflag = ICANON;  
      
  /* 
     IGNPAR  : パリティエラーのデータは無視する 
     ICRNL   : CR を NL に対応させる(これを行わないと，他のコンピュータで 
     CR を入力しても，入力が終りにならない) 
     それ以外の設定では，デバイスは raw モードである(他の入力処理は行わない) 
  */  
  tio.c_iflag = IGNPAR | ICRNL;  
      
  // ボーレートの設定  
  cfsetispeed(&tio, BAUDRATE);  
  cfsetospeed(&tio, BAUDRATE);  
  // デバイスに設定を行う  
  tcsetattr(fd, TCSANOW, &tio);  
}  
      

void parent_process(const geometry_msgs::Twist::ConstPtr& twist_msg) {  
      
  char input[BUFFSIZE];  
  int writecount = 0;  
  int i = 0;  
  int inputcount = 0;  

  sprintf(input, "F%04d.ES%04d.E", 
	  (int)(twist_msg->linear.x * 255.0), 
	  (int)((twist_msg->angular.z + 1.73) * 180.0 / 3.14));
  
  

  //改行コード埋め込み  
  for (i = 0; i < BUFFSIZE; i++) {  
    if (input[i] == '\0') {  
      inputcount = i;  
      break;  
    }  
  }  
  
  writecount = write(fd, &input, inputcount);  
  if (writecount < 0) {  
    fprintf(stdout, "Could not write to serial port %d\n", writecount);  
  }  
  usleep(150000);
}  
      
int main(int argc, char **argv) {  
      
  struct termios oldtio, newtio;  
  char buf[255];  
  /* 
     読み書きのためにモデムデバイスをオープンする．ノイズによって CTRL-C 
     がたまたま発生しても接続が切れないように，tty 制御はしない． 
  */  
      
  fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);  
  if (fd < 0) {  
    perror(MODEMDEVICE);  
    return (-1);  
  }  
      
  tcgetattr(fd, &oldtio); /* 現在のシリアルポートの設定を待避させる*/  
  memset(&newtio, 0, sizeof(newtio));/* 新しいポートの設定の構造体をクリアする */  
      
  //シリアルポート準備  
  serial_init(fd);  
  ros::init(argc, argv, "automotive");

  ros::NodeHandle n;

  ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1, parent_process);
  ros::spin();
      
  return 0;  
}  
#else

#define BAUDRATE B9600
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int serial_ardinowrite(char *devicename,char *input)
{
  int fd;
  struct termios oldtio,newtio;
 
  fd = open(devicename,O_RDWR|O_NONBLOCK); //デバイスのオープン
  if(fd<0) //デバイスのオープンに失敗した場合
    {
      printf("ERROR on device open.\n");
      exit(1);
    } 
 
  ioctl(fd,TCGETS,&oldtio);//現状のシリアルポート設定を退避
  newtio = oldtio;
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  ioctl(fd,TCSETS,&newtio);
  write(fd,input,21);
  ioctl(fd,TCSETS,&oldtio);
 
  close(fd);
 
  return 0; 
}

void cmd_callback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
  char buf[32];
  sprintf(buf, "L%04d.ER%04d.ES%04d.E", 
	  (int)(twist_msg->linear.x * 255.0), 
	  (int)(twist_msg->linear.x * 255.0),
	  (int)((twist_msg->angular.z + 1.73) * 180.0 / 3.14));
  serial_ardinowrite("/dev/ttyACM0", buf);
  printf("%s\n", buf);
  usleep(100000);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "automotive");

  ros::NodeHandle n;
  ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1, cmd_callback);
  ros::spin();

  return 0;
}

#endif
