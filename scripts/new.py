import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import argparse

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        """
        Initialize PID Controller
        
        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param setpoint: Desired setpoint value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = -1.5
        self.output_max = 1.5
        self.setpoint = setpoint
        
        self.previous_error = 0
        self.integral = 0
        
    def compute(self, current_value, dt=1.0):
        """
        Compute PID control output
        
        :param current_value: Current measured value
        :param dt: Time delta (default 1.0)
        :return: Control output
        """
        # Calculate error
        error = self.setpoint - current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.previous_error) / dt
        
        # Calculate total output
        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))
        # Update for next iteration
        self.previous_error = error
        
        return output

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
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # PID Controllers
        self.angular_z_pid = PIDController(kp=0.0025, ki=0.0, kd=0.0, setpoint=320)  # Assuming 640x480 image
        self.linear_x_pid = PIDController(kp=0.02, ki=0.0, kd=0.0, setpoint=90)  # Desired ball radius
        
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
        
        # Detect contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        return image, contours

    def send_cmd_vel(self, angular_z, linear_x):
        """Send velocity commands to the robot."""
        twist = Twist()
        twist.angular.z = angular_z
        twist.linear.x = linear_x
        self.cmd_vel_pub.publish(twist)

    def image_callback(self, msg):
        """Callback for receiving and processing images."""
        try:
            # Process image
            cv_image, contours = self.process_image(
                cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            )
            
            # Prepare output image
            frame_with_circles = cv_image.copy()
            
            # Default velocity commands
            center = ( 320, 240)
            radius = 90
            angular_z = 0.0
            linear_x = 0.0
            
            # Find largest contour (ball)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea, default=None)
                # Calculate contour area
                area = cv2.contourArea(largest_contour)

                # Filter contours by area to reduce noise
                if area > 500:  # Adjust this value based on your specific use case
                    # Calculate minimum enclosing circle
                    (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                    center = (int(x), int(y))
                    radius = int(radius)

                    # Draw circle and center
                    cv2.circle(frame_with_circles, center, radius, (0, 255, 0), 2)
                    cv2.circle(frame_with_circles, center, 2, (0, 0, 255), 3)

                    # PID Control for horizontal positioning (angular z)
                    angular_z = -self.angular_z_pid.compute(center[0])
                    
                    # PID Control for ball size (linear x)
                    linear_x = self.linear_x_pid.compute(radius)
                    
            # Optional: print control details
            self.get_logger().info(
                f"Ball: Center = {center}, Radius = {radius}, "
                f"Angular Z = {angular_z:.4f}, Linear X = {linear_x:.4f}"
            )
                    
            # Send velocity commands
            self.send_cmd_vel(0.0, linear_x)
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
    parser = argparse.ArgumentParser(description='ROS2 Compressed Image Processor with PID Control')
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