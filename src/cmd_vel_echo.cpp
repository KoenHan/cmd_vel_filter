#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Int32.h>

class CmdVelEcho {
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  ros::Rate loop_rate;

  // 他パッケージの都合上(cmd_vel -> cmd_vel_filter)の順番にしました.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_filter", 10);
  ros::Subscriber sub = nh.subscribe(
      "cmd_vel", 1,
      &CmdVelEcho::cmdVelSubCallback, this);
  ros::Subscriber area_type_pub = nh.subscribe("area_type", 1, &CmdVelEcho::areaTypeSubCallback, this);
  geometry_msgs::Twist cmd_vel_base;
  bool flag = false;
  int area_type = 0;
  int slowdown_area = 3;
  double slowdown_vel = 0.2;


public:
  CmdVelEcho(
    const ros::NodeHandle& nh_,
    const ros::NodeHandle& nh_private_,
    const ros::Rate& loop_rate_) : nh{nh_}, nh_private{nh_private_}, loop_rate{loop_rate_} {}
  
  void cmdVelSubCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_base = *msg;
    flag = true;
  }

  void areaTypeSubCallback(const std_msgs::Int32::ConstPtr& msg) {
    area_type = msg->data;
  }

  bool isSlowDownArea() {
    if (area_type == slowdown_area){
      return true;
    } else {
      return false;
    }
  }

  void start(double slowdown_vel) {
    while(ros::ok()) {
      ros::spinOnce();
      if(flag) {
        geometry_msgs::Twist msg = cmd_vel_base;
        if (isSlowDownArea()){
          msg.linear.x = std::min(msg.linear.x, slowdown_vel);
        }
        std::cout << "now linear.x... " << msg.linear.x << std::endl;
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

    double slowdown_vel;
    nh_private.param("slowdown_vel", slowdown_vel, 0.2);

    CmdVelEcho echo(nh, nh_private, loop_rate);
    echo.start(slowdown_vel);

    ros::shutdown();

    return 0;
}