#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class ImageNode : public rclcpp::Node {
public:
    ImageNode() : Node("image_publisher"), img_count_(0) {
        RCLCPP_INFO(this->get_logger(), "Publishing Images...");
        
        std::string video_path = "/home/shivani/ros2_ws/src/object_tracking_viz/media/test1.mp4";
        camera_.open(0, cv::CAP_V4L2);
        // camera_.open(video_path);
        if (!camera_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            rclcpp::shutdown();
            return;
        }

        camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(40), 
                                        std::bind(&ImageNode::timer_callback, this));
    }

private:
    void timer_callback() {
        cv::Mat frame;
        if (camera_.read(frame)) {
            cv::resize(frame, frame, cv::Size(820, 620), 0, 0, cv::INTER_CUBIC);
            
            auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            img_pub_->publish(*img_msg);
            RCLCPP_INFO(this->get_logger(), "Published image %d", img_count_++);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture camera_;
    int img_count_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}