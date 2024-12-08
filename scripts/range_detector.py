import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Subscribe to the image topic
        # Replace '/camera/image_raw' with the actual topic name from your ros2 run command
        self.subscription = self.create_subscription(
            Image, 
            '/image_raw',  # Default topic for v4l2_camera node
            self.listener_callback, 
            10  # QoS profile depth
        )
        
        # CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display the image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # Wait for 1 ms and process OpenCV events
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create and spin the node
    image_subscriber = ImageSubscriber()
    
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
