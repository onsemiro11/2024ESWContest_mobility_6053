#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <chrono>
#include <string>
#include <sstream>
#include <future>
#include <thread>

// Bounding Box 구조체 정의 (타임스탬프 추가)
struct BBox {
    float x1, y1, x2, y2;
    float depth;
    int id;
    std::string class_label;
    rclcpp::Time timestamp;  // 박스의 마지막 업데이트 시간
};

class TraviUINode : public rclcpp::Node {
    // 상태 정보
    bool stop_status_;
    bool front_car_;
    std::string road_type_;
    std::string traffic_status_;
    std::string previous_traffic_status_;
    std::vector<cv::Point> lanePts_;
    bool departure_alarm_;  // 출발 알림 상태 추가
    bool alarm_playing_;  // 알람이 울리고 있는지 여부 추가

    // 트래킹 정보 저장
    std::unordered_map<int, float> previous_depths_;  // 이전 깊이값 저장
    std::unordered_map<int, BBox> tracked_lights_;  // BBox 타입으로 변경
    std::vector<BBox> bbox_list_;  // Bounding Box 리스트

    // ROS2 구독자
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_light_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr car_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr road_type_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr polygon_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_subscription_;

    // 비디오 처리 스레드
    std::thread video_thread_;
    cv::Mat frame;
    int frame_width = 640;
    int frame_height = 480;
    std::mutex frame_mutex_;

    cv::Mat straight_arrow_;
    cv::Mat left_arrow_;

    cv::Mat veh_go_;
    cv::Mat veh_goLeft_;
    cv::Mat veh_stop_;
    cv::Mat veh_stopLeft_;
    cv::Mat veh_nosign_;


    // 박스 표시 유지 시간 (초)
    const double BOX_DISPLAY_DURATION = 1.0;

    std::thread alarm_thread_;  // 알람 소리 재생 스레드
    std::string current_traffic_light_label_;  // 현재 신호등 상태를 저장할 멤버 변수 추가

public:
    TraviUINode() : Node("travi_ui_node") {
        // 신호등 정보 구독 (YOLO 형식: x1, y1, x2, y2, class, id)
        traffic_light_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/traffic_results", 10, std::bind(&TraviUINode::yoloTrafficLightCallback, this, std::placeholders::_1));

        // 차량 정보 구독 (YOLO 형식: x1, y1, x2, y2, depth)
        car_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/vehicle_results", 10, std::bind(&TraviUINode::yoloCarCallback, this, std::placeholders::_1));

        // 차선 정보 구독 (도로 타입)
        road_type_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/road_results", 10, std::bind(&TraviUINode::roadTypeCallback, this, std::placeholders::_1));

        // 폴리곤 좌표 구독 (차선 정보 시각화를 위해 사용)
        polygon_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/polygon_data", 10, std::bind(&TraviUINode::polygonCallback, this, std::placeholders::_1));

        
        frame_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10, std::bind(&TraviUINode::image_callback, this, std::placeholders::_1));

        // 초기값 설정
        road_type_ = "RoadMarkArrow_Straight";
        stop_status_ = false;
        front_car_ = false;
        traffic_status_ = "veh_stop";
        previous_traffic_status_ = "veh_stop";

        // 비디오 처리 스레드 실행
        video_thread_ = std::thread(std::bind(&TraviUINode::processVideo, this));
        // 출발 알람 체크 타이머 (1초마다 알람 체크)
        alarm_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TraviUINode::checkAndPlayDepartureAlarm, this));

        
        // 화살표 이미지 로드
        straight_arrow_ = loadAndResizeImage("straight_arrow.png", 200, 200);
        left_arrow_ = loadAndResizeImage("left_arrow.png", 200, 200);

        // 신호등 이미지 로드
        veh_go_ = loadAndResizeImage("straight.png", 50, 50);
        veh_goLeft_ = loadAndResizeImage("straightleft.png", 50, 50);
        veh_stop_ = loadAndResizeImage("stop.png", 50, 50);
        veh_stopLeft_ = loadAndResizeImage("Left.png", 50, 50);
        veh_nosign_ = loadAndResizeImage("nosign.png", 50, 50);
        
    }
    ~TraviUINode() {
        if (video_thread_.joinable()) {
            video_thread_.join();
        }
        if (alarm_thread_.joinable()) {
            alarm_thread_.join();  // 소멸자에서 스레드가 완료될 때까지 기다림
        }
    }

private:
    rclcpp::TimerBase::SharedPtr alarm_timer_;

    cv::Mat loadAndResizeImage(const std::string& filename, int width, int height) {
        std::string path = "src/travi_ui/media/img/" + filename;
        cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", filename.c_str());
            return cv::Mat();
        }

        // cv::resize(img, img, cv::Size(width, height));
        // RCLCPP_INFO(this->get_logger(), "Loaded image: %s", filename.c_str());
        return img;
    }

    void overlayImage(cv::Mat &background, const cv::Mat &foreground, const cv::Point &location) {
        if (foreground.channels() == 4) {
            std::vector<cv::Mat> channels(4);
            cv::split(foreground, channels);

            cv::Mat bgr_foreground;
            cv::merge(std::vector<cv::Mat>{channels[0], channels[1], channels[2]}, bgr_foreground);  // Merge only BGR channels

            cv::Mat alpha_channel = channels[3];  // Alpha channel

            // Overlay based on the alpha channel
            for (int y = 0; y < background.rows; ++y) {
                for (int x = 0; x < background.cols; ++x) {
                    if (x >= location.x && x < location.x + foreground.cols && y >= location.y && y < location.y + foreground.rows) {
                        int fg_x = x - location.x;
                        int fg_y = y - location.y;

                        uchar alpha = alpha_channel.at<uchar>(fg_y, fg_x);
                        if (alpha > 0) {
                            for (int c = 0; c < 3; ++c) {
                                background.at<cv::Vec3b>(y, x)[c] = 
                                    (1.0 - alpha / 255.0) * background.at<cv::Vec3b>(y, x)[c] + 
                                    (alpha / 255.0) * bgr_foreground.at<cv::Vec3b>(fg_y, fg_x)[c];
                            }
                        }
                    }
                }
            }
        } else {
            foreground.copyTo(background(cv::Rect(location.x, location.y, foreground.cols, foreground.rows)));
        }
    }



    // 신호등 상태에 따라 출발 알림을 줄지 결정하는 함수
    bool shouldTriggerDepartureAlarm(const std::string &class_label) {
        // 도로 타입에 따른 straight_road 설정
        bool straight_road = (road_type_ == "RoadMarkArrow_Straight" ||
                            road_type_ == "RoadMarkArrow_StraightLeft" ||
                            road_type_ == "RoadMarkArrow_StraightRight");

        // 앞차가 없고, 내가 정지 상태일 때만 출발 알림을 주는 로직
        if (!front_car_ && stop_status_) {
            if (class_label == "veh_goLeft") {
                // class_label이 "veh_goLeft"일 때는 무조건 출발
                return true;
            } else if (straight_road && class_label == "veh_go") {
                // straight_road가 true이고, class_label이 "veh_go"이면 출발
                return true;
            } else if (!straight_road && class_label == "veh_stopLeft") {
                // straight_road가 false이고, class_label이 "veh_stopLeft"이면 출발
                return true;
            } else {
                // 나머지는 출발 알림 끔
                departure_alarm_ = false;
            }
        }
        return false;
    }

    void checkAndPlayDepartureAlarm() {
        if (!stop_status_) {
            // 차량이 움직이고  있을 때는 알람을 꺼야 함            
            RCLCPP_INFO(this->get_logger(), "움직이고있으니까 알람 꺼.");

            resetAlarmIfMoving();  
        } else if (departure_alarm_ && !alarm_playing_ && stop_status_) {
            // 차량이 멈춰있고, 특수한 상황일 때만 알람을 켬
            RCLCPP_INFO(this->get_logger(), "출발알람 켜.");
            playBeepSound();
        }
    }
    
    void playBeepSound() {
        if (stop_status_){
            if (departure_alarm_ && !alarm_playing_) {
                alarm_playing_ = true;

                // 알람 소리 재생을 비동기로 처리
                alarm_thread_ = std::thread([this]() {
                    system("mpg123 -q ./src/travi_ui/src/beep.mp3");

                    // 1초 후에 알람 종료
                    std::this_thread::sleep_for(std::chrono::seconds(2));

                    // 알람 종료 후 상태 업데이트
                    alarm_playing_ = false;
                    departure_alarm_ = false;
                    RCLCPP_INFO(this->get_logger(), "알람 종료");
                });

                // detach()로 스레드를 백그라운드에서 실행
                alarm_thread_.detach();
            }
        }
    }

    void resetAlarmIfMoving() {
        if (!stop_status_ && departure_alarm_) {

            RCLCPP_INFO(this->get_logger(), "알람 끔.");

            // 알람을 강제로 중단하고 상태를 재설정
            system("pkill -f 'mpg123 -q ./src/travi_ui/src/beep.mp3'");

            departure_alarm_ = false;
            alarm_playing_ = false;
        }
    }

    // 박스의 색상을 결정하는 함수 (신호등 상태에 따라)
    cv::Scalar getBoxColor(const std::string &class_label) {
        if (class_label == "veh_go") {
            return cv::Scalar(180, 255, 180);  // 연한 초록색
        } else if (class_label == "veh_goLeft") {
            return cv::Scalar(180, 255, 180);  // 연한 초록색
        } else if (class_label == "veh_noSign") {
            return cv::Scalar(200, 200, 200);  // 회색
        } else if (class_label == "veh_stop") {
            return cv::Scalar(255, 180, 180);  // 연한 빨간색
        } else if (class_label == "veh_stopLeft") {
            return cv::Scalar(255, 180, 180);  // 연한 빨간색
        } else if (class_label == "veh_stopWarning") {
            return cv::Scalar(255, 210, 150);  // 연한 주황색
        } else if (class_label == "veh_warning") {
            return cv::Scalar(255, 255, 180);  // 연한 노란색
        } else if (class_label == "car") {
            return cv::Scalar(180, 200, 255);  // 연한 파란색 (차량)
        }
        return cv::Scalar(255, 255, 255);  // 기본 흰색
    }

    void drawBBox(cv::Mat &frame, const BBox &bbox) {
        // 박스 색상 결정
        cv::Scalar box_color = getBoxColor(bbox.class_label);

        // 박스 그리기
        cv::rectangle(frame, cv::Point(bbox.x1, bbox.y1), cv::Point(bbox.x2, bbox.y2), box_color, 2);

        // 클래스 및 ID를 상단에 표시 (박스와 동일한 배경색, 글씨는 흰색)
        std::string label = bbox.class_label + " - id: " + std::to_string(bbox.id);
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline); // 폰트 크기와 두께 조정
        cv::rectangle(frame, cv::Point(bbox.x1, bbox.y1 - textSize.height - 10), cv::Point(bbox.x1 + textSize.width, bbox.y1), box_color, cv::FILLED);
        cv::putText(frame, label, cv::Point(bbox.x1, bbox.y1 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2); // 두께 2로 설정

        // 차량일 경우 오른쪽 하단에 depth 값 표시
        if (bbox.class_label == "car") {
            std::stringstream depth_label_ss;
            depth_label_ss << std::fixed << std::setprecision(1) << "depth: " << bbox.depth << "m";  // 소수점 첫째 자리만 표시
            std::string depth_label = depth_label_ss.str();
            cv::putText(frame, depth_label, cv::Point(bbox.x2 - 120, bbox.y2 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 1);
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (msg->data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received an empty frame.");
            return;
        }
        cv::Mat frame(msg->height, msg->width, CV_8UC3, (void*)msg->data.data());
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            this->frame = frame.clone();  // Store a copy of the frame for processing
        }
    }

    // 도로 타입 콜백 함수
    void roadTypeCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "[]") {
            road_type_ = " ";
            return;
        }

        std::istringstream iss(msg->data);
        std::vector<std::string> tokens;
        std::string token;

        while (iss >> token) {
            tokens.push_back(token);
        }

        road_type_ = tokens[tokens.size() - 2];
    }



    // 폴리곤 좌표 콜백 함수
    void polygonCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::stringstream ss(msg->data);
        std::vector<cv::Point> new_polygon;
        int x, y;
        while (ss >> x >> y) {
            new_polygon.push_back(cv::Point(x, y));
        }
        if (!new_polygon.empty()) {
            lanePts_ = new_polygon;
        }
    }

    // YOLO Traffic Light 데이터 콜백 함수
    void yoloTrafficLightCallback(const std_msgs::msg::String::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "YOLO Traffic Light data received: %s", msg->data.c_str());

        // 데이터 파싱
        std::stringstream ss(msg->data);
        float x1, y1, x2, y2;
        std::string class_label;
        int id;
        ss >> x1 >> y1 >> x2 >> y2 >> class_label >> id;

        // YOLO 좌표 변환 (0~1 값을 프레임 크기에 맞게 변환)
        x1 *= frame_width;
        y1 *= frame_height;
        x2 *= frame_width;
        y2 *= frame_height;

        // RCLCPP_INFO(this->get_logger(), "Converted Traffic Light coordinates: [%f, %f, %f, %f]", x1, y1, x2, y2);

        // 신호등 BBox를 맵에 추가 (처음 받은 신호등 ID일 경우)
        if (tracked_lights_.find(id) == tracked_lights_.end()) {
            // RCLCPP_INFO(this->get_logger(), "New traffic light detected, adding to tracked_lights_: id = %d", id);
            tracked_lights_[id] = {x1, y1, x2, y2, 0, id, class_label, this->now()};
            bbox_list_.push_back(tracked_lights_[id]);
        } else {
            BBox prev_bbox = tracked_lights_[id];

            // 중심점 변화량 및 bbox 크기 변화량 계산
            float prev_center_x = (prev_bbox.x1 + prev_bbox.x2) / 2;
            float prev_center_y = (prev_bbox.y1 + prev_bbox.y2) / 2;
            float curr_center_x = (x1 + x2) / 2;
            float curr_center_y = (y1 + y2) / 2;
            float center_diff = std::sqrt(std::pow(curr_center_x - prev_center_x, 2) + std::pow(curr_center_y - prev_center_y, 2));

            float prev_size = (prev_bbox.x2 - prev_bbox.x1) * (prev_bbox.y2 - prev_bbox.y1);
            float curr_size = (x2 - x1) * (y2 - y1);
            float size_diff = std::abs(curr_size - prev_size);

            if (center_diff > 5.0 || size_diff > 0.05 * prev_size) {
                stop_status_ = false;
                // RCLCPP_INFO(this->get_logger(), "Vehicle is moving, stop_status = false");
                resetAlarmIfMoving();
            } else {
                stop_status_ = true;
                // RCLCPP_INFO(this->get_logger(), "Vehicle is stopped, stop_status = true");
            }

            tracked_lights_[id] = {x1, y1, x2, y2, 0, id, class_label, this->now()};
            auto it = std::find_if(bbox_list_.begin(), bbox_list_.end(), [id](const BBox& bbox) {
                return bbox.id == id;
            });
            if (it != bbox_list_.end()) {
                *it = tracked_lights_[id];
            } else {
                bbox_list_.push_back(tracked_lights_[id]);
            }
        }

        // 신호등 상태에 따른 출발 알림 판단
        if (shouldTriggerDepartureAlarm(class_label)) {
            // RCLCPP_INFO(this->get_logger(), "Traffic signal allows departure, setting departure alarm.");
            departure_alarm_ = true;
        }

        // 현재 신호등 상태를 멤버 변수에 저장
        current_traffic_light_label_ = class_label;
    }

    // YOLO Car 데이터 콜백 함수
    void yoloCarCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "[]") {
            msg->data = " ";
        }
        // RCLCPP_INFO(this->get_logger(), "YOLO Car data received: %s", msg->data.c_str());


        std::stringstream ss(msg->data);
        std::string box_data;

        while (std::getline(ss, box_data, ';')) {
            std::stringstream box_ss(box_data);
            float x1, y1, x2, y2, depth;
            int id;
            box_ss >> x1 >> y1 >> x2 >> y2 >> depth >> id;

            x1 *= frame_width;
            y1 *= frame_height;
            x2 *= frame_width;
            y2 *= frame_height;

            // RCLCPP_INFO(this->get_logger(), "Converted coordinates: [%f, %f, %f, %f], Depth: %f, ID: %d", x1, y1, x2, y2, depth, id);

            BBox bbox = {x1, y1, x2, y2, depth, id, "car", this->now()};
            auto it = std::find_if(bbox_list_.begin(), bbox_list_.end(), [id](const BBox& bbox) {
                return bbox.id == id;
            });
            if (it != bbox_list_.end()) {
                *it = bbox;
            } else {
                bbox_list_.push_back(bbox);
            }

            if (previous_depths_.find(id) != previous_depths_.end()) {
                float previous_depth = previous_depths_[id];
                if (depth > previous_depth && previous_depth <= 8.0) {
                    // RCLCPP_INFO(this->get_logger(), "Front car is moving, prepare to start!");
                    departure_alarm_ = true;
                }
            }
            previous_depths_[id] = depth;
        }
    }

    // 비디오 처리 함수
    void processVideo() {
        cv::VideoWriter video("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(frame_width, frame_height));

        while (rclcpp::ok()) {
            cv::Mat frame_copy;
            {   
                std::lock_guard<std::mutex> lock(frame_mutex_);
                if (this->frame.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "Frame is empty, skipping drawing.");
                    continue;
                }
                frame_copy = this->frame;  // Use the current frame
            }
            cv::resize(frame_copy, frame_copy, cv::Size(frame_width, frame_height));

            // 차선 폴리곤 그리기
            drawTransparentPolygon(frame_copy, lanePts_, cv::Scalar(255, 200, 200, 128), 0.5);

            // 도로 타입에 따른 화살표 그리기
            cv::Mat *arrow = nullptr;
            cv::Point arrowCenter(frame_width / 2, frame_height - 10);

            if (road_type_ == "RoadMarkArrow_Straight" || road_type_ == "RoadMarkArrow_StraightLeft" || road_type_ == "RoadMarkArrow_StraightRight") {
                arrow = &straight_arrow_;
            } else {
                arrow = &left_arrow_;
            }

            if (arrow != nullptr) {
                // RCLCPP_INFO(this->get_logger(), "Drawing straight arrow");
                cv::Rect roi(arrowCenter.x - straight_arrow_.cols / 2, arrowCenter.y - straight_arrow_.rows, straight_arrow_.cols, straight_arrow_.rows);
                if (roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= frame_copy.cols && roi.y + roi.height <= frame_copy.rows) {
                    overlayImage(frame_copy, straight_arrow_, cv::Point(arrowCenter.x - straight_arrow_.cols / 2, arrowCenter.y - straight_arrow_.rows));
                }
            }


            // 신호등 UI 그리기
            // RCLCPP_INFO(this->get_logger(), "Drawing traffic light label: %s", current_traffic_light_label_.c_str());
            cv::Mat *traffic_light_image = nullptr;

            if (current_traffic_light_label_ == "veh_go") {
                traffic_light_image = &veh_go_;
            } else if (current_traffic_light_label_ == "veh_goLeft") {
                traffic_light_image = &veh_goLeft_;
            } else if (current_traffic_light_label_ == "veh_stop") {
                traffic_light_image = &veh_stop_;
            } else if (current_traffic_light_label_ == "veh_stopLeft") {
                traffic_light_image = &veh_stopLeft_;
            } else {
                traffic_light_image = &veh_nosign_;
            }
            

            if (traffic_light_image != nullptr) {
                cv::Point trafficLightPosition(frame_width / 2 - 100 , 10); 
                cv::Rect roi(trafficLightPosition.x, trafficLightPosition.y, traffic_light_image->cols, traffic_light_image->rows);
                if (roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= frame_copy.cols && roi.y + roi.height <= frame_copy.rows) {
                    overlayImage(frame_copy, *traffic_light_image, trafficLightPosition);
                }
            }

            // 현재 시간 가져오기
            rclcpp::Time current_time = this->now();

            // Bounding Box 그리기 및 일정 시간이 지나면 제거
            for (auto it = bbox_list_.begin(); it != bbox_list_.end();) {
                double elapsed_time = (current_time - it->timestamp).seconds();
                if (elapsed_time < BOX_DISPLAY_DURATION) {
                    drawBBox(frame_copy, *it);
                    ++it;
                } else {
                    it = bbox_list_.erase(it);  // 시간이 지난 BBox는 제거
                }
            }

            video.write(frame_copy);
            cv::imshow("Video", frame_copy);
            if (cv::waitKey(10) == 27) {
                break;
            }
        }

        video.release();
        cv::destroyAllWindows();
    }

    void drawTransparentPolygon(cv::Mat &img, const std::vector<cv::Point> &pts, const cv::Scalar &color, double alpha) {
        if (pts.empty()) {
            return;
        }

        cv::Mat overlay;
        img.copyTo(overlay);
        cv::fillConvexPoly(overlay, pts, color);
        cv::addWeighted(overlay, alpha, img, 1 - alpha, 0, img);
    }

    void drawCustomArrow(cv::Mat &img, cv::Point center, const std::string &direction, cv::Scalar color, double alpha = 0.7, int arrow_width = 30, int arrow_height = 50, int arrow_head = 40) {
        cv::Mat overlay;
        img.copyTo(overlay);
        std::vector<cv::Point> arrow_body, arrow_head_points;
        if (direction == "RoadMarkArrow_Straight" || direction == "RoadMarkArrow_StraightLeft" || direction == "RoadMarkArrow_StraightRight") {
            // PRINT DIRECTION RCLPP INFO
            // RCLCPP_INFO(this->get_logger(), "Direction: %s", direction.c_str());
            

            arrow_body = {
                cv::Point(center.x - arrow_width / 2, center.y),
                cv::Point(center.x + arrow_width / 2, center.y),
                cv::Point(center.x + arrow_width / 2, center.y - arrow_height),
                cv::Point(center.x - arrow_width / 2, center.y - arrow_height)
            };
            arrow_head_points = {
                cv::Point(center.x - arrow_width, center.y - arrow_height),
                cv::Point(center.x + arrow_width, center.y - arrow_height),
                cv::Point(center.x, center.y - arrow_height - arrow_head)
            };
        } else {
            // RCLCPP_INFO(this->get_logger(), "Direction: %s", direction.c_str());
            arrow_body = {
                cv::Point(center.x+20, center.y - arrow_width / 2),
                cv::Point(center.x+20, center.y + arrow_width / 2),
                cv::Point(center.x+20 - arrow_height, center.y + arrow_width / 2),
                cv::Point(center.x+20 - arrow_height, center.y - arrow_width / 2)
            };
            arrow_head_points = {
                cv::Point(center.x+20 - arrow_height, center.y - arrow_width),
                cv::Point(center.x+20 - arrow_height, center.y + arrow_width),
                cv::Point(center.x+20 - arrow_height - arrow_head, center.y)
            };
        }
        cv::fillConvexPoly(overlay, arrow_body, color);
        cv::fillConvexPoly(overlay, arrow_head_points, color);
        cv::addWeighted(overlay, alpha, img, 1 - alpha, 0, img);
    }

    void drawTrafficLightUI(cv::Mat &img, bool isActive, const std::string &lightState) {
        int width = 170;
        int height = 60;
        cv::Point top_left(img.cols / 2 - width / 2, 50);
        cv::Rect trafficLightRect(top_left, cv::Size(width, height));

        // 배경 그리기
        cv::rectangle(img, trafficLightRect, cv::Scalar(0, 0, 0), -1);

        // 기본 색상 설정
        cv::Scalar redCircle = cv::Scalar(50, 50, 50);
        cv::Scalar greenCircle = cv::Scalar(50, 50, 50);
        cv::Scalar greenLeftArrow = cv::Scalar(50, 50, 50);

        // 신호 상태에 따른 색상 설정
        if (lightState == "veh_go") {
            greenCircle = cv::Scalar(0, 255, 0);
        } else if (lightState == "veh_goLeft") {
            greenCircle = cv::Scalar(0, 255, 0);
            greenLeftArrow = cv::Scalar(0, 255, 0);
        } else if (lightState == "veh_stop") {
            redCircle = cv::Scalar(0, 0, 255);
        } else if (lightState == "veh_stopLeft") {
            greenLeftArrow = cv::Scalar(0, 255, 0);
        }

        // 신호등 동그라미 및 화살표 그리기
        cv::circle(img, cv::Point(top_left.x + 30, top_left.y + height / 2), 20, redCircle, -1);
        cv::circle(img, cv::Point(top_left.x + 85, top_left.y + height / 2), 20, greenCircle, -1);
        cv::circle(img, cv::Point(top_left.x + 140, top_left.y + height / 2), 20, cv::Scalar(50, 50, 50), -1);

        if (greenLeftArrow == cv::Scalar(0, 255, 0)) {
            drawCustomArrow(img, cv::Point(top_left.x + 135, top_left.y + height / 2), "left", greenLeftArrow, 0.7, 10, 20, 10);
        }
    }
};

// 메인 함수
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraviUINode>());
    rclcpp::shutdown();
    return 0;
}
