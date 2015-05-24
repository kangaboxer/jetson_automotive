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
