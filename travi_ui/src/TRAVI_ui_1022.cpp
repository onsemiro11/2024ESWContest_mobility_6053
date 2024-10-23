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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_subscription_;

    // 비디오 처리 스레드
    std::thread video_thread_;
    cv::Mat frame;
    int frame_width = 640;
    int frame_height = 480;
    std::mutex frame_mutex_;
    std::mutex bbox_mutex_;

    cv::Mat straight_arrow_;
    cv::Mat left_arrow_;
    cv::Mat right_arrow_;

    cv::Mat veh_go_;
    cv::Mat veh_goLeft_;
    cv::Mat veh_stop_;
    cv::Mat veh_stopLeft_;
    cv::Mat veh_nosign_;

    cv::Mat departure_image_car_;
    cv::Mat departure_image_traffic_;

    // 박스 표시 유지 시간 (초)
    const double BOX_DISPLAY_DURATION = 0.1;

    std::thread alarm_thread_;  // 알람 소리 재생 스레드
    std::string current_traffic_light_label_;  // 현재 신호등 상태를 저장할 멤버 변수 추가

    std::unordered_map<int, rclcpp::Time> traffic_light_times_; // Traffic light detection times

    rclcpp::Time stop_start_time_;  // 정지 시작 시간

    int frame_count = 0;


public:
    TraviUINode() : Node("travi_ui_node") {


        RCLCPP_INFO(this->get_logger(), "WELCOME to TRAVI UI !");
        RCLCPP_INFO(this->get_logger(), "Team: TRAVI");
        RCLCPP_INFO(this->get_logger(), "Team members: 이현도, 전하연");
        RCLCPP_INFO(this->get_logger(), "Made for the 2024 Embedded Software Contest");
        RCLCPP_INFO(this->get_logger(), "LICENSE: CC BY-NC 4.0");
        RCLCPP_INFO(this->get_logger(), "For more details, visit: https://creativecommons.org/licenses/by-nc/4.0/");



        // 신호등 정보 구독 (YOLO 형식: x1, y1, x2, y2, class, id)
        traffic_light_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/traffic_results", 10, std::bind(&TraviUINode::yoloTrafficLightCallback, this, std::placeholders::_1));

        // 차량 정보 구독 (YOLO 형식: x1, y1, x2, y2, depth)
        car_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/vehicle_results", 10, std::bind(&TraviUINode::yoloCarCallback, this, std::placeholders::_1));

        // 차선 정보 구독 (도로 타입)
        road_type_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/road_results", 10, std::bind(&TraviUINode::roadTypeCallback, this, std::placeholders::_1));
        
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
        straight_arrow_ = loadAndResizeImage("straight_n.png", 200, 150);
        left_arrow_ = loadAndResizeImage("left_n.png", 200, 180);
        right_arrow_ = loadAndResizeImage("right_n.png", 200, 180);

        // 신호등 이미지 로드
        veh_go_ = loadAndResizeImage("straight.png", 200, 65);
        veh_goLeft_ = loadAndResizeImage("straightleft.png", 200, 65);
        veh_stop_ = loadAndResizeImage("stop.png", 200, 65);
        veh_stopLeft_ = loadAndResizeImage("left.png", 200, 65);
        veh_nosign_ = loadAndResizeImage("nosign.png", 200, 65);

        // 출발 알림 이미지 로드
        departure_image_car_ = loadAndResizeImage("move_car.png", 200, 180);
        departure_image_traffic_ = loadAndResizeImage("move_frontcar.png", 200, 180);
        
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

        cv::resize(img, img, cv::Size(width, height));
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
                    system("mpg123 -q /home/main/check_ws/src/travi_ui/src/beep.mp3");

                    // 1초 후에 알람 종료
                    std::this_thread::sleep_for(std::chrono::seconds(4));

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
            system("pkill -f 'mpg123 -q /home/main/check_ws/src/travi_ui/src/beep.mp3'");

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
        // RCLCPP_INFO(this->get_logger(), "Class: %s", bbox.class_label.c_str());

        // 박스 그리기
        cv::rectangle(frame, cv::Point(bbox.x1, bbox.y1), cv::Point(bbox.x2, bbox.y2), box_color, 2);

        // 클래스 및 ID를 상단에 표시 (박스와 동일한 배경색, 글씨는 흰색)
        std::string label = bbox.class_label + " - id: " + std::to_string(bbox.id);
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline); // 폰트 크기와 두께 조정
        cv::rectangle(frame, cv::Point(bbox.x1, bbox.y1 - textSize.height - 10), cv::Point(bbox.x1 + textSize.width, bbox.y1), box_color, cv::FILLED);
        cv::putText(frame, label, cv::Point(bbox.x1, bbox.y1 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2); // 두께 2로 설정

        // 차량일 경우 오른쪽 하단에 depth 값 표시
        // if (bbox.class_label == "car") {
        //    std::stringstream depth_label_ss;
        //    depth_label_ss << std::fixed << std::setprecision(1) << "depth: " << bbox.depth << "m";  // 소수점 첫째 자리만 표시
        //    std::string depth_label = depth_label_ss.str();
         //   cv::putText(frame, depth_label, cv::Point(bbox.x2 - 120, bbox.y2 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 1);
        //}
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
        if (msg->data.empty() || msg->data == "[]") {
            RCLCPP_WARN(this->get_logger(), "Received an empty road data.");
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

    void yoloTrafficLightCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data.empty() || msg->data == "[]") {
            RCLCPP_WARN(this->get_logger(), "Received an empty traffic light data.");
            return;
        }
        std::stringstream ss(msg->data);
        float x1, y1, x2, y2;
        std::string class_label;
        int id;
        ss >> x1 >> y1 >> x2 >> y2 >> class_label >> id;

        // Transform YOLO coordinates (0-1 values to frame dimensions)
        x1 *= frame_width;
        y1 *= frame_height;
        x2 *= frame_width;
        y2 *= frame_height;

        // Get current time
        auto current_time = this->now();

        // RCLCPP_INFO(this->get_logger(), "Converted coordinates: [%f, %f, %f, %f], ID: %d, Class: %s", x1, y1, x2, y2, id, class_label.c_str());
        // Update the tracked traffic light bounding box
        tracked_lights_[id] = BBox{x1, y1, x2, y2, 0, id, class_label, current_time};

        BBox bbox = {x1, y1, x2, y2, 0, id, class_label, current_time};
        auto it = std::find_if(bbox_list_.begin(), bbox_list_.end(), [id](const BBox& bbox) {
            return bbox.id == id;
        });
        if (it != bbox_list_.end()) {
            *it = bbox;
        } else {
            bbox_list_.push_back(bbox);
        }
        
        // Check if traffic light is newly detected
        if (traffic_light_times_.find(id) == traffic_light_times_.end()) {
            traffic_light_times_[id] = current_time;
            stop_status_ = false;  // Initially set stop status to false for new traffic light
        } else {
            // Calculate elapsed time
            auto elapsed_time = current_time - traffic_light_times_[id];
            if (elapsed_time.seconds() >= 1) {
                current_traffic_light_label_ = tracked_lights_[id].class_label;
                stop_status_ = true;
                stop_start_time_ = current_time;  // Store the time when stop_status_ was set to true
                // RCLCPP_INFO(this->get_logger(), "Stop status = True (Traffic light ID %d detected for 5 seconds)", id);
                current_traffic_light_label_ = class_label;
            }
        }


        // RCLCPP_INFO(this->get_logger(), "Traffic light ID: %d, Class: %s", tracked_lights_[id].id, tracked_lights_[id].class_label.c_str());

        // Check if traffic light with that ID has disappeared
        for (auto it = traffic_light_times_.begin(); it != traffic_light_times_.end(); ) {
            if (tracked_lights_.find(it->first) == tracked_lights_.end()) {  // Traffic light ID is no longer detected
                auto time_since_last_seen = current_time - it->second;
                if (time_since_last_seen.seconds() >= 5) {  // If 5 seconds have passed since stop_status_ was set to true
                    stop_status_ = false;
                    // RCLCPP_INFO(this->get_logger(), "Stop status = False (Traffic light ID %d no longer detected for 5 seconds)", it->first);
                    traffic_light_times_.erase(it++);  // Erase the entry for this traffic light
                } else {
                    ++it;
                }
            } else {
                ++it;
            }
        }

        // Trigger departure alarm if needed
        if (shouldTriggerDepartureAlarm(class_label)) {
            departure_alarm_ = true;
            // RCLCPP_INFO(this->get_logger(), "Departure alarm triggered.");
        }
    }



    // YOLO Car 데이터 콜백 함수
    void yoloCarCallback(const std_msgs::msg::String::SharedPtr msg) {
        // if message length is more than 0
        if (msg->data.empty() || msg->data == "[]") {
            RCLCPP_WARN(this->get_logger(), "Received an empty car data.");
            return;
        }
        
        // if (msg->data == "[]") {
            // msg->data = " ";
        // }
        RCLCPP_INFO(this->get_logger(), "YOLO Car data received: %s", msg->data.c_str());


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

            // RCLCPP_INFO(this->get_logger(), "Converted coordinates: [%f, %f, %f, %f], Depth: %f, ID: %d, Class: %s", x1, y1, x2, y2, depth, id, "car");

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
        cv::namedWindow("Travi UI", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("Travi UI", 1920, 1080);
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
            video.write(frame_copy);

            // Arrow type handling
            cv::Mat* arrow = nullptr;
            cv::Point arrowCenter(frame_width / 2, frame_height - 10);

            if (straight_arrow_.empty() || left_arrow_.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Arrow images are not initialized.");
                continue;
            }

            ///////////////////////////////////// TODO Debugging
            // departure_alarm_ = true;
            // front_car_ = true;
            /////////////////////////////////////

            if (departure_alarm_ && alarm_playing_) {
                if (front_car_) {
                    arrow = &departure_image_traffic_;
                } else {
                    arrow = &departure_image_car_;
                }
            } else {
                if (road_type_ == "RoadMarkArrow_Straight" || road_type_ == "RoadMarkArrow_StraightLeft" || road_type_ == "RoadMarkArrow_StraightRight") {
                    arrow = &straight_arrow_;
                } else {
                    arrow = &left_arrow_;
                }
            }

            if (arrow != nullptr) {
                // RCLCPP_INFO(this->get_logger(), "Drawing straight arrow %s", road_type_.c_str());
                cv::Rect roi(arrowCenter.x - arrow->cols / 2, arrowCenter.y - arrow->rows, arrow->cols, arrow->rows);
                if (roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= frame_copy.cols && roi.y + roi.height <= frame_copy.rows) {
                    overlayImage(frame_copy, *arrow, cv::Point(arrowCenter.x - arrow->cols / 2, arrowCenter.y - arrow->rows));
                } else {
                    RCLCPP_WARN(this->get_logger(), "ROI out of bounds.");
                }
            }

            // Traffic light UI handling
            cv::Mat* traffic_light_image = nullptr;

            if (stop_status_) {
                if (current_traffic_light_label_ == "veh_go") {
                    traffic_light_image = &veh_go_;
                } else if (current_traffic_light_label_ == "veh_goLeft") {
                    traffic_light_image = &veh_goLeft_;
                } else if (current_traffic_light_label_ == "veh_stop") {
                    traffic_light_image = &veh_stop_;
                } else if (current_traffic_light_label_ == "veh_stopLeft") {
                    traffic_light_image = &veh_stopLeft_;
                } else if (current_traffic_light_label_ == "veh_warn") {
                    traffic_light_image = &veh_nosign_;
                }

                if (traffic_light_image != nullptr) {
                    // RCLCPP_INFO(this->get_logger(), "Drawing traffic light image: %s", current_traffic_light_label_.c_str());
                    cv::Point trafficLightPosition(frame_width / 2 - 100, 10); 
                    cv::Rect roi(trafficLightPosition.x, trafficLightPosition.y, traffic_light_image->cols, traffic_light_image->rows);
                    if (roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= frame_copy.cols && roi.y + roi.height <= frame_copy.rows) {
                        overlayImage(frame_copy, *traffic_light_image, trafficLightPosition);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Traffic light ROI out of bounds.");
                    }
                }
            }

            // Current time retrieval
            rclcpp::Time current_time = this->now();

            // Draw Bounding Boxes and remove after certain time
            {
                std::lock_guard<std::mutex> lock(bbox_mutex_); // Ensure this mutex exists and is used properly
                for (auto it = bbox_list_.begin(); it != bbox_list_.end();) {
                    double elapsed_time = (current_time - it->timestamp).seconds();
                    if (elapsed_time < BOX_DISPLAY_DURATION) {
                        drawBBox(frame_copy, *it);
                        ++it;
                    } else {
                        it = bbox_list_.erase(it);  // Remove expired BBoxes
                    }
                }
            }

            
            cv::imshow("Travi UI", frame_copy);
            if (cv::waitKey(10) == 27) {
                break;
            }
        }

        video.release();
        cv::destroyWindow("Travi UI"); // Clean up
    }

};



// 메인 함수
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraviUINode>());
    rclcpp::shutdown();
    return 0;
}
