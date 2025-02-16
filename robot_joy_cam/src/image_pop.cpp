#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>

bool record_video = false;
cv::VideoWriter video_writer;
double front_dist = 1.0;
cv::Size base_size;

void correctRedTint(cv::Mat& image) 
{
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    
    // Усредним яркость каналов и определим корректирующий коэффициент
    double meanRed = cv::mean(channels[2])[0];
    double meanGreen = cv::mean(channels[1])[0];
    double meanBlue = cv::mean(channels[0])[0];
    
    double correctionFactor = (meanGreen + meanBlue) / (2.0 * meanRed);
    channels[2] *= correctionFactor; // Коррекция красного канала
    meanRed = cv::mean(channels[2])[0];
    
    correctionFactor = (meanGreen + meanRed) / (2.0 * meanBlue);
    channels[0] *= correctionFactor; // Коррекция синего канала
    meanBlue = cv::mean(channels[0])[0];

    correctionFactor = (meanBlue + meanRed) / (2.0 * meanGreen);
    channels[1] *= correctionFactor; // Коррекция зеленого канала
    
    cv::merge(channels, image);
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    try {
        // Декодируем сжатый массив байтов обратно в изображение OpenCV
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        
        if (image.empty()) {
            ROS_WARN("Ошибка: получено пустое изображение!");
            return;
        }

        //cv::flip(image, image, 0);
        //cv::flip(image, image, 1);

        //cv::blur(image, image, cv::Size(3, 3));
        correctRedTint(image);

        cv::putText(image, std::to_string((int)(front_dist*100)), cv::Point(10, 20),
                    cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0),  1);

        cv::imshow("Полученное изображение", image);

        base_size = image.size();

        if (record_video && video_writer.isOpened())
            video_writer.write(image);

        cv::waitKey(1);
    } catch (const cv::Exception& e) {
        ROS_ERROR("Ошибка OpenCV: %s", e.what());
    }
}

std::string getCurrentTime() 
{
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

void ButtonCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
    bool video_switch = (bool)msg->data[3];
    
    if (record_video == 0 && video_switch == 1){
		video_writer.open("/home/ublaptop/rosor_ws/src/robot_joy_cam/videos" + getCurrentTime() + ".avi", 
                  cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, 
                  base_size, true);

		if (video_writer.isOpened()) {
			record_video = true;
			ROS_INFO("RECORDING START-------------------");
		}
	} 

	if (record_video == 1 && video_switch == 0) {
		record_video = false;
		video_writer.release();
		ROS_INFO("-------------------RECORDING ENDED");
	}
}


void FrontDistanceCallback(const std_msgs::Float32& msg)
{
    front_dist = (double)msg.data;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "compressed_image_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber camera_sub = nh.subscribe("/camera/image/compressed", 5, imageCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/status_buttons", 10, ButtonCallback);
    ros::Subscriber distance_sub = nh.subscribe("/front_distance", 10, FrontDistanceCallback);
    ros::spin();
}
