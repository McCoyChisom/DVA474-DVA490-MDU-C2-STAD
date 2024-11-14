#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#define UDP_PORT 12345
#define BUFFER_SIZE 65536

class UdpArucoReceiver : public rclcpp::Node {
public:
    UdpArucoReceiver() : Node("udp_aruco_receiver") {
    // Initialize publisher for processed images
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco_marker_image", 10);

    // Logging after initialization to confirm everything is working.
    RCLCPP_INFO(this->get_logger(), "UDP ArUco Receiver node has started.");
}


    // Function to receive image data over UDP
    void receiveImage(cv::Mat &frame) {
        int sockfd;
        struct sockaddr_in server_addr, client_addr;
        socklen_t addrlen = sizeof(client_addr);

        // Create socket
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
            return;
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(UDP_PORT);

        // Bind socket to the address and port
        if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed!");
            close(sockfd);
            return;
        }

        // Receive image data in chunks and reassemble
        char buffer[BUFFER_SIZE];
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &addrlen);
        if (n > 0) {
            std::vector<uchar> img_data(buffer, buffer + n);
            frame = cv::imdecode(img_data, cv::IMREAD_COLOR);  // Decode to OpenCV image
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive image data!");
        }
        close(sockfd);
    }

    // Function to detect ArUco markers and extract coordinates
    void detectArucoMarkers(cv::Mat &frame) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        // Load ArUco dictionary
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

        // Detect markers
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);

        // Draw detected markers
        cv::aruco::drawDetectedMarkers(frame, corners, ids);

        // Extract coordinates of markers (example with 3D pose estimation)
        if (!ids.empty()) {
            for (size_t i = 0; i < ids.size(); i++) {
                // Assume that the camera matrix and distortion coefficients are known
                cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
                cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
                cv::Vec3d rvec, tvec;

                // Estimate pose (assuming a known marker size)
                float marker_length = 0.1;  // 10cm marker size (example)
                cv::aruco::estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs, rvec, tvec);

                // Draw pose on the image
                cv::aruco::drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1);

                // Extract 3D coordinates (x, y, z)
                double x = tvec[0];
                double y = tvec[1];
                double z = tvec[2];
                RCLCPP_INFO(this->get_logger(), "Marker ID: %d - Coordinates: (%.2f, %.2f, %.2f)", ids[i], x, y, z);
            }
        }
    }

    // Main processing function
    void process() {
        cv::Mat frame;

        // Step 1: Receive an image via UDP
        receiveImage(frame);

        if (!frame.empty()) {
            // Step 2: Detect ArUco markers and extract coordinates
            detectArucoMarkers(frame);

            // Step 3: Publish the processed image to a ROS topic
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            image_pub_->publish(*msg);

            // Display the processed image
            cv::imshow("ArUco Markers", frame);
            cv::waitKey(1);  // Adjust this delay as needed
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    //auto node = std::make_shared<rclcpp::Node>("udp_aruco_receiver"); 
    auto node = std::make_shared<UdpArucoReceiver>();

    // Main loop
    rclcpp::Rate rate(10);  // Adjust loop rate as necessary
    while (rclcpp::ok()) {
        node->process();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}


