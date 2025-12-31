#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <std_msgs/String.h>
#include <iostream>

class QrReader {
public:
    QrReader() : nh_("~") {
        std::string image_topic;
        nh_.param<std::string>("image_topic", image_topic, "/camera/rgb/image_raw");
        
        sub_ = nh_.subscribe(image_topic, 1, &QrReader::imageCallback, this);
        pub_ = nh_.advertise<std_msgs::String>("/mission/qr_code", 10);
        
        // OpenCV QRCodeDetector
        qrDecoder_ = cv::QRCodeDetector();
        ROS_INFO("QR Reader Node Initialized listening on %s", image_topic.c_str());
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            std::string data, bbox, rectifiedImage;
            // Detect and Decode
            std::string decoded_info = qrDecoder_.detectAndDecode(cv_ptr->image);
            
            if (!decoded_info.empty()) {
                std_msgs::String output_msg;
                output_msg.data = decoded_info;
                pub_.publish(output_msg);
                ROS_INFO_THROTTLE(1.0, "QR Code Detected: %s", decoded_info.c_str());
            }

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        } catch (cv::Exception& e) {
            ROS_ERROR("OpenCV exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    cv::QRCodeDetector qrDecoder_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "qr_reader_node");
    QrReader reader;
    ros::spin();
    return 0;
}
