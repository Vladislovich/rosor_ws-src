#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>

ros::Publisher cmd_vel_pub;
ros::Time last_cmd_time, last_around_time;

double go_around[2] = {0, 0};
int go_forward = 0;
double time_to_around = 3.5;

double linear_speed = 0.08; 
double angular_speed = 0.14; 

double timelightstop = 0.5;
double timeout = 3.0;  // Таймаут в секундах
double timestop = 5.0;

void cmdVelCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
    last_cmd_time = ros::Time::now();  // Обновляем время последнего полученного сообщения
    go_around[0] = (double)msg->data[14];
    go_around[1] = (double)msg->data[0]/10;

    go_forward = msg->data[1];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "emergency_return");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/diff_drive_controller/cmd_vel", 10);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/status_buttons", 10, cmdVelCallback);

    last_cmd_time = ros::Time::now();  // Инициализация времени последнего сообщения

    ros::Rate rate(10);  // Частота проверки (10 Гц)
    double around_time_dif = 0;

    while (ros::ok())
    {
        ros::spinOnce();
        double return_time_dif = (ros::Time::now() - last_cmd_time).toSec();

        if (return_time_dif > timelightstop){
            geometry_msgs::Twist cmd_vel;
            if (return_time_dif < timeout || return_time_dif > timestop){
                ROS_WARN("Publishing emergency stop");
                cmd_vel.linear.x = 0; 
            }
            if (return_time_dif > timeout && return_time_dif < timestop){
                ROS_WARN("Publishing emergency return");
                cmd_vel.linear.x = -linear_speed;
            }  
            cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel);
        }

        if (go_around[0] == 1 && around_time_dif == 0) last_around_time = ros::Time::now();
        if (go_around[0] == 0) around_time_dif = 0;
        if (go_around[0] == 1) around_time_dif = (ros::Time::now() - last_around_time).toSec();
        if (around_time_dif > 0)
        {
            geometry_msgs::Twist cmd_vel;
            if (around_time_dif > go_around[1] && around_time_dif < go_around[1] + time_to_around){
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = angular_speed;
                ROS_INFO("around: ROTATE");
            }
            if ((around_time_dif < go_around[1] || around_time_dif > go_around[1] + time_to_around) && around_time_dif < 2 * go_around[1] + time_to_around){
                cmd_vel.linear.x = linear_speed;
                cmd_vel.angular.z = 0;
                ROS_INFO("around: FORWARD");
            }
            if (around_time_dif > 2 * go_around[1] + time_to_around){
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                ROS_INFO("around: STOP");
            }
            cmd_vel_pub.publish(cmd_vel);
        }

        if (go_forward == 1)
        {
            ROS_INFO("FORWARD");
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = linear_speed;
            cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel);
        }

        rate.sleep();
    }

    return 0;
}
