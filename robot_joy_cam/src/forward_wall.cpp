#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <vector>

ros::Publisher cmd_vel_pub;
ros::Publisher distance_pub;
double min_distance = 1.0;

void ScanCallback(const sensor_msgs::LaserScan& scan)
{    
    double min_angle = 100;
    std::vector<std::vector<double>> points;
    min_distance = 1.0;
    for (int i = 0; i < scan.ranges.size(); ++i)
        points.push_back({scan.angle_min + i*scan.angle_increment, scan.ranges[i]});

    for (int i = 0; i < points.size(); ++i)
        if (abs(points[i][0]) < 0.2 && points[i][1] < min_distance)
            min_distance = points[i][1]; 

    //ROS_INFO("min_angle: %f", min_angle);
    //ROS_INFO("min_distance: %f", min_distance);

    std_msgs::Float32 dist_msg;
    dist_msg.data = min_distance;
    distance_pub.publish(dist_msg);
}

void CmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{    
    geometry_msgs::Twist cmd_vel_res;
    cmd_vel_res = cmd_vel;

    if (min_distance < 0.3 && cmd_vel_res.linear.x > 0)
        cmd_vel_res.linear.x = 0;

    cmd_vel_pub.publish(cmd_vel_res);
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "forward_wall");
    ros::NodeHandle nh;
    
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/diff_drive_controller/cmd_vel", 10);
    distance_pub = nh.advertise<std_msgs::Float32>("/front_distance", 10);

    ros::Subscriber sub_vel = nh.subscribe("/cmd_vel", 10, CmdVelCallback);
    ros::Subscriber sub_scan = nh.subscribe("/scan", 10, ScanCallback);

    ros::Rate loop_rate(50);  // Частота публикации status_pub (10 Гц)

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}