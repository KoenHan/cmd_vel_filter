#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class CmdVelEcho {
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  ros::Rate loop_rate;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber sub = nh.subscribe(
      "cmd_vel_filter", 1,
      &CmdVelEcho::cmdVelSubCallback, this);
    
  geometry_msgs::Twist cmd_vel_base;
  bool flag = false;

public:
  CmdVelEcho(
    const ros::NodeHandle& nh_,
    const ros::NodeHandle& nh_private_,
    const ros::Rate& loop_rate_) : nh{nh_}, nh_private{nh_private_}, loop_rate{loop_rate_} {}
  
  void cmdVelSubCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_base = *msg;
    flag = true;
  }

  void start() {
    while(ros::ok()) {
        ros::spinOnce();
        if(flag) {
          geometry_msgs::Twist msg = cmd_vel_base;
          pub.publish(msg);
        }

        loop_rate.sleep();
    }
  };
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_vel_echo");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Rate loop_rate(50); // Hz

    CmdVelEcho echo(nh, nh_private, loop_rate);
    echo.start();

    ros::shutdown();

    return 0;
}