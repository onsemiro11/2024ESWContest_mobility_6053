#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <random> 

class KeyboardPublisher : public rclcpp::Node {
public:
    KeyboardPublisher() : Node("keyboard_publisher") {
        // 퍼블리셔 생성
        yolo_car_publisher_ = this->create_publisher<std_msgs::msg::String>("/yolo_car", 10);
        yolo_traffic_light_publisher_ = this->create_publisher<std_msgs::msg::String>("/yolo_traffic_light", 10);
        road_type_publisher_ = this->create_publisher<std_msgs::msg::String>("/road_type", 10);

        // 입력 처리를 위한 별도의 스레드 생성
        input_thread_ = std::thread(&KeyboardPublisher::processKeyboardInput, this);
    }

    ~KeyboardPublisher() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    // TC0: 차량 및 신호등 BBox를 자동으로 계속 생성하는 시나리오
    void executeTestCase0() {
        auto road_msg = std_msgs::msg::String();
        road_msg.data = "RoadMarkArrow_Straight";
        road_type_publisher_->publish(road_msg);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> car_x_dist(0.0, 1.0);
        std::uniform_real_distribution<> car_y_dist(0.5, 0.8);
        std::uniform_real_distribution<> car_depth_dist(1.0, 10.0);
        std::uniform_real_distribution<> light_x_dist(0.0, 1.0);
        std::uniform_real_distribution<> light_y_dist(0.0, 0.5);
        std::uniform_int_distribution<> id_dist(1, 100);
        std::uniform_int_distribution<> count_dist(0, 5);  // 0~5개의 객체

        std::vector<std::string> traffic_light_classes = {"veh_go", "veh_goLeft", "veh_stop", "veh_stopLeft", "veh_warning"};
        std::uniform_int_distribution<> class_dist(0, traffic_light_classes.size() - 1);

        while (rclcpp::ok()) {
            int car_count = count_dist(gen);
            int light_count = count_dist(gen);

            // 여러 개의 랜덤 차량 BBox 생성 및 퍼블리시
            for (int i = 0; i < car_count; ++i) {
                publishRandomCarBBox(car_x_dist(gen), car_y_dist(gen), car_depth_dist(gen), id_dist(gen));
            }

            // 여러 개의 랜덤 신호등 BBox 생성 및 퍼블리시
            for (int i = 0; i < light_count; ++i) {
                publishRandomTrafficLightBBox(light_x_dist(gen), light_y_dist(gen), id_dist(gen), traffic_light_classes[class_dist(gen)]);
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));  // 1초 간격으로 생성
        }
    }


    // 랜덤한 차량 BBox 퍼블리시 함수
    void publishRandomCarBBox(float x1, float y1, float depth, int id) {
        auto msg = std_msgs::msg::String();
        std::stringstream ss;
        float x2 = x1 + 0.2;  // BBox 너비를 랜덤하게 설정
        float y2 = y1 + 0.2;  // BBox 높이를 랜덤하게 설정

        ss << std::fixed << std::setprecision(2) << x1 << " " << y1 << " " << x2 << " " << y2 << " " << depth << " " << id;
        msg.data = ss.str();
        yolo_car_publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "TC0 - YOLO car data: %s", msg.data.c_str());
    }

    // 랜덤한 신호등 BBox 퍼블리시 함수
    void publishRandomTrafficLightBBox(float x1, float y1, int id, const std::string &class_label) {
        auto msg = std_msgs::msg::String();
        std::stringstream ss;
        float x2 = x1 + 0.1;  // BBox 너비
        float y2 = y1 + 0.15;  // BBox 높이

        ss << std::fixed << std::setprecision(2) << x1 << " " << y1 << " " << x2 << " " << y2 << " " << class_label << " " << id;
        msg.data = ss.str();
        yolo_traffic_light_publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "TC0 - YOLO traffic light data: %s", msg.data.c_str());
    }



    // TC1: 차량 및 신호등의 BBox 크기 및 위치가 변경되는 시나리오
    // TC1에서 YOLO 좌표 및 ID를 제대로 퍼블리시하고 있는지 확인
    void executeTestCase1() {
        auto msg = std_msgs::msg::String();
        std::stringstream ss;
        
        // depth 값이 2초 후에 서서히 증가
        float depth_values[] = {2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 7.0};  // 10초 동안의 depth 값
        float light_x1 = 0.8, light_y1 = 0.1, light_x2_values[] = {0.82, 0.82, 0.82, 0.82, 0.83, 0.83, 0.84, 0.84, 0.85, 0.86};  // 신호등 x2 좌표 변화
        float light_y2_values[] = {0.2, 0.2, 0.2, 0.2, 0.25, 0.25, 0.3, 0.3, 0.35, 0.35};  // 신호등 y2 좌표 변화
        int car_id = 1;  // 같은 차이므로 ID는 동일
        float car_x1 = 0.4, car_y1 = 0.4, car_x2 = 0.6, car_y2 = 0.6;  // 차량의 Bounding Box 범위
        int light_id = 100;  // 신호등 ID

        // Road Type 설정
        auto road_msg = std_msgs::msg::String();
        road_msg.data = "RoadMarkArrow_Straight";
        road_type_publisher_->publish(road_msg);

        // 10초 동안 depth 및 신호등 크기 변화
        for (int i = 0; i < 10; ++i) {
            // 차량 정보 퍼블리시
            ss.str("");  // 스트림 초기화
            ss << std::fixed << std::setprecision(2) << car_x1 << " " << car_y1 << " " << car_x2 << " " << car_y2 << " " << depth_values[i] << " " << car_id;
            msg.data = ss.str();
            yolo_car_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "TC1 - YOLO car data: %s", msg.data.c_str());

            // 2초 후부터 신호등 크기 변화 시작
            std::string traffic_light_status = (i < 4) ? "veh_stop" : "veh_go";  // 2초 후 신호등 변화
            publishTrafficLightData(light_x1, light_y1, light_x2_values[i], light_y2_values[i], traffic_light_status, light_id);
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
   


    // YOLO 신호등 데이터를 퍼블리시하는 함수
    void publishTrafficLightData(float x1, float y1, float x2, float y2, const std::string &class_label, int id) {
        auto msg = std_msgs::msg::String();
        std::stringstream ss;

        // YOLO 신호등 데이터 문자열 생성
        ss << std::fixed << std::setprecision(2) << x1 << " " << y1 << " " << x2 << " " << y2 << " " << class_label << " " << id;
        msg.data = ss.str();
        
        yolo_traffic_light_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "YOLO traffic light data published: %s", msg.data.c_str());
    }

    // 키보드 입력 처리 함수
    void processKeyboardInput() {
        std::string input;

        while (rclcpp::ok()) {
            std::cout << "키 입력 (0: TC0 실행(랜덤), 1: TC1 실행(앞차출발), 2: TC2 실행(직진1), 3: TC3 실행(직진2), 4: TC4 실행(좌1), 5: TC5 실행(좌2)): ";
            std::cin >> input;

            if (input == "0") {
                executeTestCase0();  // TC0 실행
            } else if (input == "1") {
                executeTestCase1();  // TC1 실행
            } else if (input == "2") {
                executeTestCase2();  // TC2 실행
            } else if (input == "3") {
                executeTestCase3();  // TC3 실행
            } else if (input == "4") {
                executeTestCase4();  // TC4 실행
            } else if (input == "5") {
                executeTestCase5();  // TC5 실행
            } else {
                std::cout << "알 수 없는 입력입니다. 다시 시도하세요." << std::endl;
            }
        }
    }

    // TC2: 앞에 차량이 없음, 신호등이 출발 신호로 변경
    void executeTestCase2() {
        auto msg = std_msgs::msg::String();
        std::stringstream ss;

        // Road Type 설정
        auto road_msg = std_msgs::msg::String();
        road_msg.data = "RoadMarkArrow_Straight";
        road_type_publisher_->publish(road_msg);

        // 10초 동안의 시나리오
        for (int i = 0; i < 10; ++i) {
            // 차량 정보 퍼블리시
            ss.str("");  // 스트림 초기화
            ss << std::fixed << std::setprecision(2) << 0.4 << " " << 0.4 << " " << 0.6 << " " << 0.6 << " " << ((i < 2 || i >= 6) ? 5.0 : -1.0) << " " << 1;
            msg.data = ss.str();
            yolo_car_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "TC2 - YOLO car data: %s", msg.data.c_str());

            // 신호등 정보 퍼블리시
            std::string traffic_light_status = (i < 4) ? "veh_stopLeft" : "veh_goLeft";
            publishTrafficLightData(0.8, 0.1, 0.82, 0.2, traffic_light_status, 100);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // TC3: 앞에 차량이 없음, 신호등이 출발 신호로 변경
    void executeTestCase3() {
        auto msg = std_msgs::msg::String();
        std::stringstream ss;

        // Road Type 설정
        auto road_msg = std_msgs::msg::String();
        road_msg.data = "RoadMarkArrow_StraightLeft";
        road_type_publisher_->publish(road_msg);

        // 10초 동안의 시나리오
        for (int i = 0; i < 10; ++i) {
            // 차량 정보 퍼블리시
            ss.str("");  // 스트림 초기화
            ss << std::fixed << std::setprecision(2) << 0.4 << " " << 0.4 << " " << 0.6 << " " << 0.6 << " " << ((i < 2 || i >= 6) ? 5.0 : -1.0) << " " << 1;
            msg.data = ss.str();
            yolo_car_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "TC3 - YOLO car data: %s", msg.data.c_str());

            // 신호등 정보 퍼블리시
            std::string traffic_light_status = (i < 4) ? "veh_stop" : "veh_go";
            publishTrafficLightData(0.8, 0.1, 0.82, 0.2, traffic_light_status, 100);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // TC4: 앞에 차량이 없음, 신호등이 출발 신호로 변경
    void executeTestCase4() {
        auto msg = std_msgs::msg::String();
        std::stringstream ss;

        // Road Type 설정
        auto road_msg = std_msgs::msg::String();
        road_msg.data = "RoadMarkArrow_Left";
        road_type_publisher_->publish(road_msg);

        // 10초 동안의 시나리오
        for (int i = 0; i < 10; ++i) {
            // 차량 정보 퍼블리시
            ss.str("");  // 스트림 초기화
            ss << std::fixed << std::setprecision(2) << 0.4 << " " << 0.4 << " " << 0.6 << " " << 0.6 << " " << ((i < 2 || i >= 6) ? 5.0 : -1.0) << " " << 1;
            msg.data = ss.str();
            yolo_car_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "TC4 - YOLO car data: %s", msg.data.c_str());

            // 신호등 정보 퍼블리시
            std::string traffic_light_status = (i < 4) ? "veh_go" : "veh_goLeft";
            publishTrafficLightData(0.8, 0.1, 0.82, 0.2, traffic_light_status, 100);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // TC5: 앞에 차량이 없음, 신호등이 출발 신호로 변경
    void executeTestCase5() {
        auto msg = std_msgs::msg::String();
        std::stringstream ss;

        // Road Type 설정
        auto road_msg = std_msgs::msg::String();
        road_msg.data = "Unprotected_LeftTurn";
        road_type_publisher_->publish(road_msg);

        // 10초 동안의 시나리오
        for (int i = 0; i < 10; ++i) {
            // 차량 정보 퍼블리시
            ss.str("");  // 스트림 초기화
            ss << std::fixed << std::setprecision(2) << 0.4 << " " << 0.4 << " " << 0.6 << " " << 0.6 << " " << ((i < 2 || i >= 6) ? 5.0 : -1.0) << " " << 1;
            msg.data = ss.str();
            yolo_car_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "TC5 - YOLO car data: %s", msg.data.c_str());

            // 신호등 정보 퍼블리시
            std::string traffic_light_status = (i < 4) ? "veh_stopWarning" : "veh_stopLeft";
            publishTrafficLightData(0.8, 0.1, 0.82, 0.2, traffic_light_status, 100);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // 퍼블리셔
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr yolo_car_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr yolo_traffic_light_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr road_type_publisher_;

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
