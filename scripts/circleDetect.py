#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import argparse


class CompressedImageProcessor(Node):
    def __init__(self, filter_type='HSV', alpha=0.8):
        super().__init__('compressed_image_processor')
        
        # Subscription to the compressed image topic
        self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            10
        )
        
        # Configuration for image processing
        self.color_filter = filter_type
        self.alpha = alpha  # Weight for the complementary filter
        self.previous_circle = None  # For the complementary filter
        self.min_hsv = np.array([60, 143, 87], dtype=np.uint8)
        self.max_hsv = np.array([81, 255, 255], dtype=np.uint8)
        
        # Hough Circle Detection Parameters
        self.hough_params = {
            'dp': 1,
            'minDist': 80,
            'param1': 50,
            'param2': 30,
            'minRadius': 20,
            'maxRadius': 100,
        }
        self.get_logger().info("Node initialized successfully.")

    def complementary_filter(self, current_circle):
        """Smooth the detected circle using a complementary filter."""
        if self.previous_circle is None:
            self.previous_circle = current_circle
        else:
            self.previous_circle = self.alpha * self.previous_circle + (1 - self.alpha) * current_circle
        return self.previous_circle

    def process_image(self, image):
        """Process the image to detect and filter circles."""
        # Preprocess image (flip and HSV conversion if needed)
        image = cv2.flip(cv2.rotate(image, cv2.ROTATE_180), 1)
        processed = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) if self.color_filter == 'HSV' else image
        
        # Apply color filtering
        mask = cv2.inRange(processed, self.min_hsv, self.max_hsv)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Apply mask
        result = cv2.bitwise_and(image, image, mask=mask)
        
        # Detect circles
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        return image, mask, result, contours


    def image_callback(self, msg):
        """Callback for receiving and processing images."""
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
            self.get_logger().error(f"Failed to process image: {e}")

    def destroy_node(self):
        """Destroy the node and close OpenCV windows."""
        super().destroy_node()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    # Parse arguments
    parser = argparse.ArgumentParser(description='ROS2 Compressed Image Processor')
    parser.add_argument('--filter', type=str, choices=['BGR', 'HSV'], default='HSV', help='Color space for filtering.')
    parser.add_argument('--alpha', type=float, default=0.8, help='Complementary filter weight (0 < alpha <= 1).')
    args = parser.parse_args()
    
    # Create and run the image processor node
    image_processor = CompressedImageProcessor(filter_type=args.filter, alpha=args.alpha)
    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        image_processor.get_logger().info('Shutting down...')
    finally:
        image_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
