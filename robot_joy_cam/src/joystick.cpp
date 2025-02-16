#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>

double linear_speed = 0.08;  
double angular_speed = 0.14; 

ros::Publisher cmd_vel_pub;
ros::Publisher status_pub;
std_msgs::Int32MultiArray status_msg, prev_status_msg;
bool joy_received = false;

std::vector<int> counters {0};
std::vector<int> tumblers {14, 1, 3, 5}; //rightstik(forward+around), butUP, circle(video rec), square(lowvideo)

// Функция для обновления данных кнопок (запускается при новом сообщении от джойстика)
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{    
    joy_received = true;
    geometry_msgs::Twist cmd_vel;
    if (abs(joy->axes[4]) > 0.1 || abs(joy->axes[3]) > 0.1){
        cmd_vel.linear.x = joy->axes[4] * linear_speed;  
        cmd_vel.angular.z = joy->axes[3] * angular_speed; 
        cmd_vel_pub.publish(cmd_vel);    
    }

    // Обновление глобальной переменной status_msg
    status_msg.data.clear();
    status_msg.data.push_back(joy->axes[6]);
    status_msg.data.push_back(joy->axes[7]);
    for (size_t i = 0; i < joy->buttons.size(); ++i)
        status_msg.data.push_back(joy->buttons[i]);

    for (auto& count : counters)
        status_msg.data[count] = prev_status_msg.data[count] - status_msg.data[count];
    for (auto& tumbler : tumblers)
        status_msg.data[tumbler] = (status_msg.data[tumbler] ^ prev_status_msg.data[tumbler]);

    prev_status_msg = status_msg;


    ROS_INFO("Linear X: %f, Angular Z: %f", cmd_vel.linear.x, cmd_vel.angular.z);
    std::string info{" "};
    for (size_t i = 0; i < status_msg.data.size(); ++i)
        info = info + std::to_string(status_msg.data[i]) + ", ";
        ROS_INFO("Button:%s", info.c_str());
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "joystick_listener");
    ros::NodeHandle nh;

    for (int i = 0; i < 15; i++)
        prev_status_msg.data.push_back(0);
    
    nh.param("linear_speed", linear_speed, 0.5);  
    nh.param("angular_speed", angular_speed, 1.0); 

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    status_pub = nh.advertise<std_msgs::Int32MultiArray>("/status_buttons", 10);    
    ros::Subscriber sub = nh.subscribe("joy", 10, joyCallback);

    ros::Rate loop_rate(50);  // Частота публикации status_pub (10 Гц)

    while (ros::ok())
    {
        ros::spinOnce();
        if (!joy_received)
            status_msg = prev_status_msg;
        status_pub.publish(status_msg);  // Публикация состояния кнопок
        joy_received = false;
        loop_rate.sleep();
    }

    return 0;
}
