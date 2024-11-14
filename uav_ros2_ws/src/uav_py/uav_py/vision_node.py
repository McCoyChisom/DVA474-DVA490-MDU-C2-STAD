#!/usr/bin/python3

from rclpy.node import Node
import math
import rclpy
import signal
import sys
from roboflowoak import RoboflowOak
from uav_msgs.msg import Vision
import time


class vision_node(Node):
    def __init__(self):
        print("Starting vision_node...")
        super().__init__("vision_node")
        print("Creating vision_topic publisher...")
        self.publisher = self.create_publisher(Vision, "vision_topic", 10)
        self.create_timer(0.5, self.publish_msg)

    def publish_msg(self):
        msg = self.result  # Get the current prediction result
        self.publisher.publish(msg)
        print("Published: ", str(msg))


# Exit on: CTRL + C
def sigint_callback(sig, frame):
    print("\n")
    print("Shutting down vision_node...")
    rclpy.shutdown()
    sys.exit(0)


class Predictor:
    def __init__(self):
        print("Starting model...")
        try:
            self.model = RoboflowOak(
                model="landing-pad-test-2",
                confidence=0.8,
                overlap=0.5,
                version="5",
                api_key="3dyKguLbaut4uUdSJ8px",
                rgb=True,
                depth=False,
                device=None,
                blocking=True
            )
            # conditional check around the RoboflowOak initialization to allow for a fallback or 
            # mock functionality when the OAK-D camera isnot connected. Added on Saturday 12th Oct. 2024.

            print("Model initialized successfully.")
        except Exception as e:
            print(f"Warning: {e}")
            print("Running in offline mode or with mock data.")
            self.model = None  # Set to None or a mock model if not connected

        self.red = []
        self.blue = []
        self.result = Vision()

    def predict(self) -> bool:
        if self.model is None:   #Checking 12/10/2024
            print("Camera model, not connected. Cannot perform prediction.")
            return False
            
        # Attempt to perform detection
        try:
            result, _, _, _ = self.model.detect()
            predictions = result["predictions"]

            return_value = False

            if predictions:
                blue_found = False
                red_found = False

                for p in predictions:
                    if p.class_name == 'blue':
                        blue_found = True
                        self.blue = p
                    elif p.class_name == 'red':
                        red_found = True
                        self.red = p

                if blue_found and red_found:
                    # Create prediction result
                    self.result.red_x = int(self.red.x)
                    self.result.red_y = int(self.red.y)
                    self.result.red_width = int(self.red.width)
                    self.result.red_height = int(self.red.height)
                    self.result.blue_x = int(self.blue.x)
                    self.result.blue_y = int(self.blue.y)
                    self.result.blue_width = int(self.blue.width)
                    self.result.blue_height = int(self.blue.height)

                    return_value = True

            return return_value
        except Exception as e:                #Checking 12/10/2024
            print(f"Prediction error: {e}")
            return False

    def get_prediction(self) -> Vision:
        return self.result


def main(args=None):
    signal.signal(signal.SIGINT, sigint_callback)
    rclpy.init(args=args)

    node = vision_node()
    predictor = Predictor()

    t0 = time.time()

    while rclpy.ok():
        prediction_ok = predictor.predict()
        if prediction_ok:
            msg = predictor.get_prediction()
            msg.time_seconds = time.time() - t0
            node.publish_msg()  # No need to pass msg here now
            t0 = time.time()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

