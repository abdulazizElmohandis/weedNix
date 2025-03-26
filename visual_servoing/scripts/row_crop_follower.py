import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# this is change

class RowCropFollower:
    def __init__(self):
        rospy.init_node('row_crop_follower', anonymous=True)
        self.bridge = CvBridge()
        # /camera_sim/color/image_raw for simulation
        # /camera/color/image_raw for RealSense
        self.image_sub = rospy.Subscriber("/camera_sim/color/image_raw", Image, self.image_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz loop rate

        # Parameters
        self.h_min, self.h_max = 35, 85  # Green color range
        self.s_min, self.s_max = 50, 255
        self.v_min, self.v_max = 50, 255
        self.point_spacing = 30  # Density of points
        self.display_extracted = True  # Toggle extracted points display
        self.kp = 0.005  # Proportional gain for steering
        self.line_detected = False  # Track if a line has been detected

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            desired_width = 640  # Adjust to match launch file settings
            desired_height = 480
            cv_image = cv2.resize(cv_image, (desired_width, desired_height), interpolation=cv2.INTER_LINEAR)
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, (self.h_min, self.s_min, self.v_min), (self.h_max, self.s_max, self.v_max))

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            filtered_contours = [cnt for cnt in contours if 500 < cv2.contourArea(cnt) < 10000]

            rospy.loginfo(f"Contours found: {len(contours)}")
            rospy.loginfo(f"Filtered contours (valid size): {len(filtered_contours)}")

            points = self.extract_and_draw_points(cv_image, filtered_contours)
            line_params = self.fit_and_draw_line(cv_image, points)

            rospy.loginfo(f"Extracted points: {len(points)}")


            if line_params:
                self.line_detected = True  # Set flag when a line is successfully detected
            
            self.follow_line(line_params)

            cv2.imshow("Original Image", cv_image)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)

            self.rate.sleep()  # Maintain loop rate

        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def extract_and_draw_points(self, image, contours):
        points = []
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(image, (cx, cy), 3, (255, 0, 0), -1)
                points.append((cx, cy))

                # Generate dense points within the contour
                mask = np.zeros(image.shape[:2], dtype=np.uint8)
                cv2.drawContours(mask, [cnt], -1, 255, thickness=cv2.FILLED)
                y_indices, x_indices = np.where(mask == 255)
                sampled_indices = np.arange(0, len(x_indices), self.point_spacing)

                for i in sampled_indices:
                    cv2.circle(image, (x_indices[i], y_indices[i]), 2, (255, 0, 0), -1)
                    points.append((x_indices[i], y_indices[i]))
        return points

    def fit_and_draw_line(self, image, points):
        if len(points) > 10:
            if len(points) < 20:  # If too few points are detected
                rospy.sleep(0.2)  # Slow down to reduce CPU load
            points = np.array(points, dtype=np.float32)
            [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

            # Ensure we don't divide by zero
            if abs(vx) < 1e-6:
                return None

            left_y = int((-x * vy / vx) + y)
            right_y = int(((image.shape[1] - x) * vy / vx) + y)
            cv2.line(image, (0, left_y), (image.shape[1], right_y), (0, 0, 255), 2)
            return (vx, vy, x, y)

        return None

    def follow_line(self, line_params):
        twist = Twist()

        if self.line_detected and line_params:
            vx, vy, x, y = line_params
            image_center = 320  # Assuming 640px width, center is 320
            error = image_center - x

            twist.linear.x = 0.2  # Forward speed
            twist.angular.z = -self.kp * error  # Steering correction
        else:
            # Stop movement if no valid line is detected
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    try:
        follower = RowCropFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
