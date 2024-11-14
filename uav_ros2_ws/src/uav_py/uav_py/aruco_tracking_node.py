import rclpy
from rclpy.node import Node
from uav_py.video_stream import VideoStream
from uav_py.aruco_single_tracker import ArucoSingleTracker
from std_msgs.msg import String  # Import message type; replace with a custom message if needed
# For custom messages, use: from uav_msgs.msg import Vision

class ArucoTrackingNode(Node):
    def __init__(self):
        super().__init__('aruco_tracking_node')
        
        # Initialize the Video Stream and ArUco Tracker
        self.source = VideoStream()
        self.tracker = ArucoSingleTracker(
            id_to_find=0,
            marker_size=50,
            source=self.source
        )
        
        # Publisher for tracking data
        self.tracking_publisher = self.create_publisher(String, 'aruco_tracking', 10)
        # Adjust above message type (String) to a custom message if needed

        # Timer to periodically call the tracking function
        self.timer = self.create_timer(0.1, self.track_and_publish)  # Calls 10 times per second

    def track_and_publish(self):
        # Get tracking data
        tracking_data = self.tracker.track_live(verbose=False, show_video=False)
        
        if tracking_data:  # Only publish if marker is detected
            marker_id, position, rotation = tracking_data  # Assuming these are returned by `track_live`

            # Publish marker info; convert data as needed to match the ROS message format
            msg = String()
            msg.data = f"ID: {marker_id}, Position: {position}, Rotation: {rotation}"
            self.tracking_publisher.publish(msg)
            
            self.get_logger().info(f"Published marker tracking data: {msg.data}")

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the ArUco Tracking Node
    aruco_tracking_node = ArucoTrackingNode()
    
    # Keep the node running
    rclpy.spin(aruco_tracking_node)
    
    # Cleanup
    aruco_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

