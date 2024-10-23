#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <chrono>
#include <cmath>  // sqrt 등 수학 함수 사용

class DataDecisionNode : public rclcpp::Node {
public:
    DataDecisionNode() : Node("data_decision_node") {
        // 객체 정보 구독 (객체 ID, BBOX 등)
        object_subscription_ = this->create_subscription<sensor_msgs::msg::RegionOfInterest>(
            "/object_detection", 10, std::bind(&DataDecisionNode::objectCallback, this, std::placeholders::_1));

        // 차량 앞 검출 정보 구독 (Depth 포함)
        depth_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth_info", 10, std::bind(&DataDecisionNode::depthCallback, this, std::placeholders::_1));

        // stop_status 퍼블리셔
        stop_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/stop_status", 10);

        // front_car 퍼블리셔
        front_car_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/front_car", 10);

        // 타이머 설정 (1초마다 판단 실행)
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DataDecisionNode::checkStatus, this));
    }

private:
    // 객체 정보 콜백 함수: 객체의 ID 및 BBOX(사각형 범위) 정보를 처리
    void objectCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg) {
        int detected_id = msg->x_offset;  // 객체 ID를 예시로 x_offset로 사용

        // 트래킹 상태가 존재하지 않으면 새로운 트래킹 정보 생성
        if (tracked_objects_.find(detected_id) == tracked_objects_.end()) {
            tracked_objects_[detected_id] = ObjectTrackingInfo{
                this->now(), 0.0, std::make_pair(0.0, 0.0), false
            };
        }

        // BBOX 크기 계산: 너비 * 높이
        double current_bbox_size = static_cast<double>(msg->width) * static_cast<double>(msg->height);

        // BBOX 중심점 계산
        double current_center_x = msg->x_offset + (msg->width / 2.0);
        double current_center_y = msg->y_offset + (msg->height / 2.0);
        auto current_bbox_center = std::make_pair(current_center_x, current_center_y);

        // 이전 BBOX 크기 및 중심점과 비교
        double size_change = std::fabs(current_bbox_size - tracked_objects_[detected_id].last_bbox_size);
        double center_distance = std::sqrt(
            std::pow(current_bbox_center.first - tracked_objects_[detected_id].last_bbox_center.first, 2) +
            std::pow(current_bbox_center.second - tracked_objects_[detected_id].last_bbox_center.second, 2)
        );

        // 크기와 중심의 변화량이 작으면 객체가 정지 상태라고 판단
        if (size_change < 10.0 && center_distance < 10.0) {
            if (!tracked_objects_[detected_id].tracking_started) {
                tracked_objects_[detected_id].tracking_start_time = this->now();
                tracked_objects_[detected_id].tracking_started = true;
            }
        } else {
            // 변화가 있으면 트래킹 초기화
            tracked_objects_[detected_id].tracking_started = false;
        }

        // 마지막 BBOX 크기 및 중심 업데이트
        tracked_objects_[detected_id].last_bbox_size = current_bbox_size;
        tracked_objects_[detected_id].last_bbox_center = current_bbox_center;
    }

    // 깊이 정보 콜백 함수: 차량 앞에 있는 객체의 깊이 정보를 처리
    void depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        bool car_in_front = false;

        // 포인트 클라우드 데이터에서 차량이 있는지 판단
        for (size_t i = 0; i < msg->height * msg->width; ++i) {
            float depth = msg->data[i];  // 예시로 단순히 처리
            if (depth > 0 && depth <= 8.0) {
                car_in_front = true;
                break;
            }
        }

        // 차량이 앞에 있는지 여부 퍼블리시
        std_msgs::msg::Bool front_car_msg;
        front_car_msg.data = car_in_front;
        front_car_publisher_->publish(front_car_msg);
    }

    // 정지 상태를 체크하는 함수
    void checkStatus() {
        bool any_object_stopped = false;

        // 각 객체에 대해 정지 상태를 체크
        for (auto& [id, tracking_info] : tracked_objects_) {
            if (tracking_info.tracking_started) {
                auto elapsed_time = (this->now() - tracking_info.tracking_start_time).seconds();
                if (elapsed_time >= 5.0) {
                    any_object_stopped = true;
                    break;
                }
            }
        }

        // 정지 상태 퍼블리시
        std_msgs::msg::Bool stop_status_msg;
        stop_status_msg.data = any_object_stopped;
        stop_status_publisher_->publish(stop_status_msg);
    }

    // 객체 트래킹 정보를 저장하는 구조체
    struct ObjectTrackingInfo {
        rclcpp::Time tracking_start_time;
        double last_bbox_size;
        std::pair<double, double> last_bbox_center;
        bool tracking_started;
    };

    // 구독자, 퍼블리셔, 타이머
    rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr object_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_car_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 객체별로 트래킹 상태를 관리하는 맵 (객체 ID를 키로 사용)
    std::unordered_map<int, ObjectTrackingInfo> tracked_objects_;
};

// 메인 함수
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataDecisionNode>());
    rclcpp::shutdown();
    return 0;
}
