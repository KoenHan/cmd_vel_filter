#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cstddef>
#include <float.h> // DBL_EPSILON
#include <math.h>  // fabs, fmax


// cmd_vel を見て変になってたら、ロボットを止める
class CmdVelSupervisor{
public:

    CmdVelSupervisor(){
        ros::NodeHandle nh;
        ros::NodeHandle nh_("~");
        twist_pub_ = 
            nh.advertise<geometry_msgs::Twist>("cmd_vel/filtered", 10);  
        twist_sub_ =
            nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &CmdVelSupervisor::cmdVelCallback, this);
        timer_ = nh.createTimer(ros::Duration(0.05), &CmdVelSupervisor::timerCallback, this);

        nh_.param("wait_sec_tolerance", wait_sec_tolerance_, 0.5);

        reseted_time_ = ros::Time::now();
        zero_twist_msg.linear.x = 0.0;
        zero_twist_msg.linear.y = 0.0;
        zero_twist_msg.linear.z = 0.0;
        zero_twist_msg.angular.x = 0.0;
        zero_twist_msg.angular.y = 0.0;
        zero_twist_msg.angular.z = 0.0;
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
        twist_ = *msg;
        if (!isSameTwist(last_twist_, twist_) || twist_.linear.x <= 0) { // 直前のtwistと違う、または速度が負のとき
            reseted_time_ = ros::Time::now();
            last_twist_ = twist_;
            twist_pub_.publish(twist_);
            // ROS_INFO("Published REAL!!");
        }
    }

    void timerCallback(const ros::TimerEvent&){
        cmdVelCheck();
    }

    void cmdVelCheck(){
        ros::Duration duration = (ros::Time::now() - reseted_time_);
        double elapsed_time_sec = duration.sec + duration.nsec / 1000000000.0;
        if ( elapsed_time_sec > wait_sec_tolerance_) { // 0.5秒間cmd_velが変化しない、または、0.5秒間cmd_velが送られていないとき
            // std::cout << elapsed_time_sec << std::endl;
            if(!isSameTwist(last_twist_, zero_twist_msg)) publishZeroCmdVel();
            reseted_time_ = ros::Time::now();
        }
    }

    bool isSameFloat(float a, float b){
        if (fabs(a - b) <= DBL_EPSILON * fmax(1, fmax(fabs(a), fabs(b)))) {
            return true;
        }
        return false;
    }

    bool isSameTwist(geometry_msgs::Twist last_twist, geometry_msgs::Twist current_twist){
        if (isSameFloat(last_twist.linear.x, current_twist.linear.x)
            && isSameFloat(last_twist.angular.z, current_twist.angular.z)){
                return true;
            }
        return false;
    }

    void publishZeroCmdVel(){
        // 停止司令を流す
        geometry_msgs::Twist msg;
        msg.linear.x = 0.01;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;
        twist_pub_.publish(msg);
        // ROS_INFO("Published FAKE!!");
    }

private:
    ros::Publisher twist_pub_;
    ros::Subscriber twist_sub_;
    ros::Timer timer_;
    geometry_msgs::Twist last_twist_{};
    geometry_msgs::Twist twist_{};
    ros::Time reseted_time_;
    double wait_sec_tolerance_; //cmd_velの変化をどれだけ待つか
    geometry_msgs::Twist zero_twist_msg;

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_vel_supervisor");

    CmdVelSupervisor a;

    ros::spin();

    return 0;
}