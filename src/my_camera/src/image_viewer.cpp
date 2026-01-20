#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class ImageViewer : public rclcpp::Node
{
public:
    ImageViewer() : Node("image_viewer")
    {
        // Parameter so as to switch topics without recompiling
        topic_ = this->declare_parameter<std::string>(
            "image_topic",
            "/world/world_demo/model/tugbot/link/camera_front/sensor/color/image");

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_, rclcpp::SensorDataQoS(),
            std::bind(&ImageViewer::cb, this, std::placeholders::_1));

        cv::namedWindow("camera", cv::WINDOW_NORMAL);
        RCLCPP_INFO(get_logger(), "Subscribing to: %s", topic_.c_str());
    }

    ~ImageViewer() override
    {
        cv::destroyAllWindows();
    }

private:
    void cb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try
        {
            // Most Gazebo camera bridges use "rgb8" or "bgr8"
            auto cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);

            cv::Mat frame = cv_ptr->image;
            if (msg->encoding == "rgb8")
            {
                cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
            }

            cv::imshow("camera", frame);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "exception: %s", e.what());
        }
    }

    std::string topic_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageViewer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
