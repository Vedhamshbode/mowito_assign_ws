#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

class ImageConverterNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer;
    cv::Mat color_image_;
    cv::Mat grayscale_image_;
    bool colored_image;
    std::string input_topic;
    std::string output_topic;

    void image_subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            color_image_ = cv_ptr->image;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void imageconvertercallback(const example_interfaces::srv::SetBool_Request::SharedPtr request, 
                                const example_interfaces::srv::SetBool_Response::SharedPtr response ){

        if (color_image_.empty())
        {
            response->success = false;
            response->message = "No image available.";
            return;
        }
        colored_image = request->data;
        response->success = true;
        if (colored_image){
            response->message ="Switched to colored video.";
        }else{
            response->message ="Switched to grayscale video.";
        }
    }
    void imagepublishercallback(){
        if (color_image_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No image available.");
            return;
        }
        if(colored_image){
            sensor_msgs::msg::Image::SharedPtr ros_color_image = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",color_image_).toImageMsg();
            image_publisher_->publish(*ros_color_image);
        }
        else{
            cv::cvtColor(color_image_, grayscale_image_, cv::COLOR_BGR2GRAY);
            sensor_msgs::msg::Image::SharedPtr ros_grayscale_image = cv_bridge::CvImage(std_msgs::msg::Header(),"mono8",grayscale_image_).toImageMsg();
            image_publisher_->publish(*ros_grayscale_image);
        }
    }

public:
    ImageConverterNode() : Node("image_converter_node"){

        this->declare_parameter<std::string>("input_topic", "/image_raw");
        this->declare_parameter<std::string>("output_topic", "/converted_image");

        input_topic = this->get_parameter("input_topic").as_string();
        output_topic = this->get_parameter("output_topic").as_string();

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);
        image_subscriber_ = this-> create_subscription<sensor_msgs::msg::Image>(input_topic, 10, bind(&ImageConverterNode::image_subscriber_callback, this, placeholders::_1));
        service_ = this->create_service<example_interfaces::srv::SetBool>("Color_BnW_Switch", 
                                                                            bind(&ImageConverterNode::imageconvertercallback, 
                                                                            this, 
                                                                            placeholders::_1, 
                                                                            placeholders::_2));
        timer = this->create_wall_timer(chrono::milliseconds(33), bind(&ImageConverterNode::imagepublishercallback, this));
        RCLCPP_INFO(this->get_logger(), "ImageConvertorNode has been started --- :)");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = make_shared<ImageConverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}