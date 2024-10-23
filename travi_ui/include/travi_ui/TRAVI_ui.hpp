#ifndef TRAVI_UI_HPP
#define TRAVI_UI_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class TraviUINode : public rclcpp::Node {
public:
    TraviUINode();

private:
    void polygonCallback(const std_msgs::msg::String::SharedPtr msg);
    void roadTypeCallback(const std_msgs::msg::String::SharedPtr msg);
    void trafficStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void stopStatusCallback(const std_msgs::msg::String::SharedPtr msg);

    void processVideo();
    void drawTransparentPolygon(cv::Mat &img, const std::vector<cv::Point> &pts, const cv::Scalar &color, double alpha);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr polygon_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr road_type_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_status_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stop_status_subscription_;
};

#endif // TRAVI_UI_HPP

