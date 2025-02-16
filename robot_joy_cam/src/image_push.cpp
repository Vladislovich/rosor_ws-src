#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Int32MultiArray.h>

bool low_quality_video = false;

void ButtonCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
    low_quality_video = (bool)msg->data[5];
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "compressed_image_publisher");
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub = nh.subscribe("/status_buttons", 10, ButtonCallback);
    ros::Publisher pub = nh.advertise<sensor_msgs::CompressedImage>("/camera/image/compressed", 5);

    cv::VideoCapture cap("/dev/video0");
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    if (!cap.isOpened()) {
        ROS_ERROR("Ошибка: Не удалось открыть камеру!");
        return -1;
    }

    ros::Rate loop_rate(30);

    while (nh.ok()) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            ROS_WARN("Пустой кадр!");
            continue;
        }

        if (low_quality_video) cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        
        // Сжатие изображения в JPEG
        std::vector<uchar> buffer;
        cv::imencode(".jpg", frame, buffer, {cv::IMWRITE_JPEG_QUALITY, 50});

        // Создание ROS сообщения
        sensor_msgs::CompressedImage msg;
        msg.format = "jpeg";
        msg.data.assign(buffer.begin(), buffer.end());

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
