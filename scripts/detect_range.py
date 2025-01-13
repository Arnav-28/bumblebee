#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import argparse

class CompressedImageProcessor(Node):
    def __init__(self, range_filter='HSV'):
        super().__init__('compressed_image_processor')
        
        # ROS2 subscription
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed',  
            self.listener_callback, 
            10
        )
        
        # Color filtering setup
        self.range_filter = list(range_filter)
        self.setup_trackbars()
        
        # Tennis ball HSV ranges (initial suggested values)
        self.min_vals = np.array([25, 100, 100], dtype=np.uint8)
        self.max_vals = np.array([75, 255, 255], dtype=np.uint8)
        
        # Hough Circle parameters
        self.hough_params = {
            'method': cv2.HOUGH_GRADIENT,
            'dp': 1,
            'minDist': 80,
            'param1': 50,
            'param2': 30,
            'minRadius': 20,
            'maxRadius': 100
        }

    def setup_trackbars(self):
        """Create trackbars for adjusting color range."""
        cv2.namedWindow("Trackbars", 0)
        for i in ["MIN", "MAX"]:
            v = 0 if i == "MIN" else 255
            for j in ['H', 'S', 'V']:
                cv2.createTrackbar(f"{j}_{i}", "Trackbars", v, 255, lambda x: None)

    def get_trackbar_values(self):
        """Retrieve current trackbar values."""
        values = []
        for i in ["MIN", "MAX"]:
            for j in ['H', 'S', 'V']:
                v = cv2.getTrackbarPos(f"{j}_{i}", "Trackbars")
                values.append(v)
        return values
    
    

    def process_image(self, image):
        """Process image with color filtering and circle detection."""
        # Rotate and flip image
        image = cv2.flip(cv2.rotate(image, cv2.ROTATE_180), 1)
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Get color range from trackbars or use predefined values
        channel_values = self.get_trackbar_values()
        min_vals = channel_values[:3]
        max_vals = channel_values[3:]
        
        # Create threshold mask
        mask = cv2.inRange(hsv, tuple(min_vals), tuple(max_vals))
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Apply mask
        result = cv2.bitwise_and(image, image, mask=mask)
        
        # Detect circles
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        return image, mask, result, contours

    def listener_callback(self, msg):
        try:
            # Process image
            cv_image, thresh, result, contours = self.process_image(
                cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            )
            
            # Prepare output image
            frame_with_circles = cv_image.copy()
            
            # Draw circles
            for contour in contours:
                # Calculate contour area
                area = cv2.contourArea(contour)

                # Filter contours by area to reduce noise
                if area > 500:  # Adjust this value based on your specific use case
                    # Calculate minimum enclosing circle
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)

                    # Draw circle and center
                    cv2.circle(frame_with_circles, center, radius, (0, 255, 0), 2)
                    cv2.circle(frame_with_circles, center, 2, (0, 0, 255), 3)

                    # Optional: print circle details
                    print(f"Circle detected: Center = {center}, Radius = {radius}")
    
            # Display results
            cv2.imshow('Detected Circles', frame_with_circles)
            cv2.imshow("Threshold", thresh)
            cv2.imshow("Filtered", result)
            
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='ROS2 Compressed Image Processor')
    parser.add_argument('-f', '--filter', 
                        choices=['BGR', 'HSV'], 
                        default='HSV', 
                        help='Color space for filtering')
    parsed_args = parser.parse_args()
    
    image_processor = CompressedImageProcessor(parsed_args.filter)
    
    try:    
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        pass
    finally:
        image_processor.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()