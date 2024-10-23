#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>

class KeyboardPublisher : public rclcpp::Node {
public:
    KeyboardPublisher() : Node("keyboard_publisher") {
        // 퍼블리셔 생성
        yolo_car_publisher_ = this->create_publisher<std_msgs::msg::String>("/yolo_car", 10);
        
        // 입력 처리를 위한 별도의 스레드 생성
        input_thread_ = std::thread(&KeyboardPublisher::processKeyboardInput, this);
    }

    ~KeyboardPublisher() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    // keyboard_publisher.cpp에서 TC1 수정
    void executeTestCase1() {
        auto msg = std_msgs::msg::String();
        std::stringstream ss;
        float depth_values[] = {2.0, 2.0, 2.0, 4.0, 6.0, 8.0}; // 2초 동안 유지 후 증가
        int id = 1;  // 같은 차이므로 ID는 동일
        float x1 = 0.4, y1 = 0.4, x2 = 0.6, y2 = 0.6;  // 같은 차선 범위

        // 5초에 걸쳐 depth 변화
        for (int i = 0; i < 6; ++i) {
            ss.str("");  // 스트림 초기화
            ss << std::fixed << std::setprecision(2) << x1 << " " << y1 << " " << x2 << " " << y2 << " " << depth_values[i] << " " << id;
            msg.data = ss.str();
            yolo_car_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "TC1 - YOLO car data: %s", msg.data.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }


    // YOLO 차량 데이터를 퍼블리시하는 함수
    void publishCarData(float x1, float y1, float x2, float y2, float depth, int id) {
        auto msg = std_msgs::msg::String();
        std::stringstream ss;

        // YOLO 차량 데이터 문자열 생성
        ss << std::fixed << std::setprecision(2) << x1 << " " << y1 << " " << x2 << " " << y2 << " " << depth << " " << id;
        msg.data = ss.str();
        
        yolo_car_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "YOLO car data published: %s", msg.data.c_str());
    }

    // 키보드 입력 처리 함수
    void processKeyboardInput() {
        std::string input;

        while (rclcpp::ok()) {
            std::cout << "키 입력 (1: TC1 실행): ";
            std::cin >> input;

            if (input == "1") {
                executeTestCase1();  // TC1 실행
            } else {
                std::cout << "알 수 없는 입력입니다. 다시 시도하세요." << std::endl;
            }
        }
    }

    // 퍼블리셔
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr yolo_car_publisher_;

    // 입력 처리용 스레드
    std::thread input_thread_;
};

// 메인 함수
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardPublisher>());
    rclcpp::shutdown();
    return 0;
}
