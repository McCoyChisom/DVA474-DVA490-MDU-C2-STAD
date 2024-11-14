#include "rclcpp/rclcpp.hpp"                          // ROS 2 client library for C++
#include "sensor_msgs/msg/image.hpp"                  // ROS 2 message type for camera images
#include "sensor_msgs/msg/camera_info.hpp"            // ROS 2 message type for camera calibration info
#include "geometry_msgs/msg/pose_stamped.hpp"         // ROS 2 message type for 3D poses with timestamps
#include <cv_bridge/cv_bridge.h>                      // Utility to convert between ROS and OpenCV image formats
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>                          // OpenCV ArUco library for marker detection                          
#include <opencv2/imgproc.hpp>                        // OpenCV image processing functions
#include <opencv2/highgui.hpp>                        // OpenCV high-level GUI functions
#include <Eigen/Dense>                                // Eigen library for matrix and vector operations
#include <opencv2/core/eigen.hpp>                     // OpenCV-Eigen interoperability

// #include "opencv2/aruco.hpp"

// Define the main class ArucoTrackerNode, derived from rclcpp::Node for ROS 2
class ArucoTrackerNode : public rclcpp::Node
{
public:
    // Constructor for ArucoTrackerNode class
    ArucoTrackerNode() : Node("aruco_tracker_node")  // Name the node "aruco_tracker_node"
    {
       // sensor_msgs::Image
       
       auto image = sensor_msgs::msg::Image();
        // Declare and retrieve ArUco marker parameters
        this->declare_parameter<int>("aruco_id", 0);  // ID of the marker to detect
        this->declare_parameter<double>("marker_size", 0.4);  // Size of the marker in meters
        this->declare_parameter<std::string>("marker_image_path", "/home/chisom/STAD_mccoychisom/src/aruco_tracker/resources/aruco_marker_0.png");
        
        
        this->get_parameter("aruco_id", aruco_id_);
        this->get_parameter("marker_size", marker_size_);
        this->get_parameter("marker_image_path", marker_image_path_);
        
        // Load the ArUco marker image from the specified path
        marker_image_ = cv::imread(marker_image_path_, cv::IMREAD_GRAYSCALE); // Load as grayscale
        if (marker_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load marker image from %s", marker_image_path_.c_str());
        }
        

         // Set up an image subscriber
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", rclcpp::QoS(1),                     // Image Source: subscribe to a ROS topic  (/camera/image_raw)                    
            std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

        // Set up a camera info subscriber
        camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", rclcpp::QoS(1),
            std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));
            
            
         // Set up a publisher for the marker pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_pose", rclcpp::QoS(1));

        // Initialize ArUco marker detection parameters and dictionary
        
       // aruco_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
      //  detector_params_ = cv::aruco::DetectorParameters::create();
      // **Correction 1:** Assigning to cv::Ptr<cv::aruco::Dictionary> directly
        //aruco_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        
        aruco_dictionary_ = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250));
        //detector_params_ = cv::aruco::DetectorParameters::create();
        //detector_params_ = cv::aruco::DetectorParameters::createDefault();
       //detector_params_ = cv::aruco::DetectorParameters();
       detector_params_ = cv::makePtr<cv::aruco::DetectorParameters>();
       }

private:
    // Callback function for image messages
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
        
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            process_image(cv_ptr->image, msg->header.stamp);  // Process the image
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());  // Log errors
        }
    }

    // Process the OpenCV image to detect and estimate pose of the ArUco marker
    void process_image(cv::Mat &image, const rclcpp::Time &timestamp)
    {
        std::vector<int> ids;  // Vector to hold detected marker IDs
        std::vector<std::vector<cv::Point2f>> corners;  // Vector to hold marker corners

        // Detect ArUco markers in the image
        cv::aruco::detectMarkers(image, aruco_dictionary_, corners, ids, detector_params_);

        if (ids.size() > 0) {  // If markers are detected
            for (size_t i = 0; i < ids.size(); i++) {
                if (ids[i] == aruco_id_) {  // Check if the detected marker matches the target ID
                
                    // Define the 3D points of the marker corners in the marker's coordinate frame
                    //cv::Point3f marker_points[4] = {
                      std::vector<cv::Point3f> marker_points = {
                        cv::Point3f(-marker_size_ / 2, -marker_size_ / 2, 0),
                        cv::Point3f(marker_size_ / 2, -marker_size_ / 2, 0),
                        cv::Point3f(marker_size_ / 2, marker_size_ / 2, 0),
                        cv::Point3f(-marker_size_ / 2, marker_size_ / 2, 0)
                    };

                    cv::Mat rvec, tvec;  // Rotation and translation vectors
                    // Estimate the pose of the marker relative to the camera
                    cv::solvePnP(marker_points, corners[i], camera_matrix_, dist_coeffs_, rvec, tvec);

                    cv::Mat rot_mat;  // Rotation matrix
                    cv::Rodrigues(rvec, rot_mat);  // Convert rotation vector to rotation matrix

                    // Convert rotation matrix from OpenCV to Eigen format
                    Eigen::Matrix3d rot_eigen;
                    cv::cv2eigen(rot_mat, rot_eigen);
                    
                    // Convert the rotation matrix to a quaternion
                    Eigen::Quaterniond quaternion(rot_eigen);

                    // Create a PoseStamped message to store the marker's pose
                    geometry_msgs::msg::PoseStamped pose_msg;
                    pose_msg.header.stamp = timestamp;
                    pose_msg.header.frame_id = "camera_frame";
                    pose_msg.pose.position.x = tvec.at<double>(0);
                    pose_msg.pose.position.y = tvec.at<double>(1);
                    pose_msg.pose.position.z = tvec.at<double>(2);

                    // Set orientation using quaternion
                    pose_msg.pose.orientation.x = quaternion.x();
                    pose_msg.pose.orientation.y = quaternion.y();
                    pose_msg.pose.orientation.z = quaternion.z();
                    pose_msg.pose.orientation.w = quaternion.w();

                    // Publish the pose message
                    pose_publisher_->publish(pose_msg);
                }
            }
        }
    }

    // Callback for camera calibration information
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
         // Store the camera intrinsic matrix
        camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        dist_coeffs_ = cv::Mat(1, 5, CV_64F);  // Store distortion coefficients
        for (int i = 0; i < 5; i++)
        {
            dist_coeffs_.at<double>(i) = msg->d[i];
        }
    }

    // ROS 2 subscriptions and publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    // Camera parameters and marker detection configuration
    cv::Mat dist_coeffs_;  // Camera distortion coefficients
    cv::Mat camera_matrix_;  // Camera intrinsic matrix
    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;  // ArUco marker dictionary
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;  // ArUco marker detector parameters
    cv::Mat marker_image_;  // Store the loaded marker image

    int aruco_id_;  // Target marker ID to detect
    double marker_size_;  // Size of the marker in meters
    std::string marker_image_path_;  // Path to the marker image file
};

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS 2
    rclcpp::spin(std::make_shared<ArucoTrackerNode>());  // Run the ArucoTrackerNode
    rclcpp::shutdown();  // Shutdown ROS 2
    return 0;
}
