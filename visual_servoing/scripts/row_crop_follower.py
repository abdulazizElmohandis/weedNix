import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class RowCropFollower:
    def __init__(self):
        rospy.init_node('row_crop_follower', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera_sim/color/image_raw", Image, self.image_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz loop rate

        # Parameters
        self.h_min, self.h_max = 35, 85  # Green color range
        self.s_min, self.s_max = 50, 255
        self.v_min, self.v_max = 50, 255
        self.point_spacing = 70  # Density of points
        self.kp = 0.005  # Proportional gain for steering
        self.line_detected = False
        self.show_images = False  # Flag to enable/disable image display


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            desired_width, desired_height = 640, 480  # Resize if needed

            # Define ROI 
            roi_y_start = int(0.1 * cv_image.shape[0])
            roi = cv_image[roi_y_start:, :]  # Crop only once

            roi = cv2.resize(roi, (desired_width, desired_height), interpolation=cv2.INTER_LINEAR)


            # Convert to HSV and apply masking only in ROI
            hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, (self.h_min, self.s_min, self.v_min), 
                                         (self.h_max, self.s_max, self.v_max))

                
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)



            # Find contours in ROI
            
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            filtered_contours = [cnt for cnt in contours if 150 < cv2.contourArea(cnt) < 100000]
            rospy.loginfo(f"Total filtered_contours : {len(filtered_contours)}")


            # Draw ROI rectangle on the full image
            #cv2.rectangle(cv_image, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (0, 255, 255), 2)

            points = self.extract_and_draw_points(roi, filtered_contours)  # Process only the ROI
            line_params = self.fit_and_draw_line(roi, points)

            if line_params:
                self.line_detected = True  # Set flag when a line is successfully detected

            self.follow_line(line_params)

            #cv2.drawContours(roi, filtered_contours, -1, (0, 0, 255), 2)  # Red color for contours

            # Show images
            #cv2.imshow("Full Image with ROI", cv_image)  # Full image with ROI marked
            if self.show_images:
                cv2.imshow("ROI Processing", roi)  # Show only the cropped ROI
                cv2.imshow("Mask (ROI)", mask)  # Mask for ROI
                cv2.waitKey(1)

            self.rate.sleep()

        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def extract_and_draw_points(self, image, contours, spacing=30):
        points = []
        rospy.loginfo(f"Total contours found: {len(contours)}")

        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, contours, -1, 255, thickness=cv2.FILLED)

        y_coords, x_coords = np.where(mask == 255)  # Faster than iterating
        sampled_indices = np.arange(0, len(x_coords), spacing)
        sampled_indices = np.arange(0, len(x_coords), spacing)

        
        for i in sampled_indices:
            x, y = x_coords[i], y_coords[i]
            cv2.circle(image, (x, y), 2, (255, 0, 0), -1)
            points.append((x, y))

        rospy.loginfo(f"Total extracted points: {len(points)}")
        return points






    def fit_and_draw_line(self, image, points):
        if len(points) >= 2:
            points = np.array(points, dtype=np.float32)
            [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

            if abs(vx) < 1e-6:
                rospy.logwarn("Detected line is nearly vertical! Skipping.")
                return None

            left_y = int((-x * vy / vx) + y)
            right_y = int(((image.shape[1] - x) * vy / vx) + y)
            cv2.line(image, (0, left_y), (image.shape[1], right_y), (0, 0, 255), 2)
            return (vx, vy, x, y)

        rospy.logwarn("Not enough points to fit a line!")
        return None

    def follow_line(self, line_params):
        twist = Twist()

        if self.line_detected and line_params:
            vx, vy, x, y = line_params
            image_center = 320  # Assuming 640px width
            error = (image_center - x).item()  # Convert NumPy array to scalar

            twist.linear.x = 0.2
            twist.angular.z = -self.kp * error
            rospy.loginfo(f"Following line: Error={error:.2f}, Angular.z={twist.angular.z:.2f}")
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            rospy.logwarn("Stopping: No line detected!")

        self.cmd_pub.publish(twist)


if __name__ == '__main__':
    try:
        follower = RowCropFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
