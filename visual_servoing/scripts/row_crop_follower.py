import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class NeighbourhoodTracker:
    def __init__(self, image_width, image_height, initial_Xc, initial_Yc, width_L, height_H):
        self.image_width = image_width
        self.image_height = image_height
        self.initial_Xc = initial_Xc
        self.initial_Yc = initial_Yc
        self.width_L = width_L
        self.height_H = height_H

        # Current state of the window center
        self.current_Xc = float(self.initial_Xc)
        self.current_Yc = float(self.initial_Yc)

        # Results for the current frame (initialize empty)
        self.points_in_neighbourhood = np.empty((0, 2), dtype=np.float32)
        self.extreme_points = [None, None] # [top_point, bottom_point]

        print(f"NeighbourhoodTracker initialized.")
        print(f"  Initial Center: ({self.current_Xc}, {self.current_Yc})")
        print(f"  Size (LxH): ({self.width_L} x {self.height_H})")

    def reset(self):
        """Resets the window to its initial position."""
        self.current_Xc = float(self.initial_Xc)
        self.current_Yc = float(self.initial_Yc)
        self.points_in_neighbourhood = np.empty((0, 2), dtype=np.float32)
        self.extreme_points = [None, None]
        print("*********** Neighbourhood Reset ***********")
        print(f"  Center set to: ({self.current_Xc}, {self.current_Yc})")


    def filter_points(self, all_detected_points):
        """
        Filters the input points to keep only those inside the current neighbourhood window.

        Args:
            all_detected_points (np.ndarray): An (N, 2) NumPy array of detected points [x, y].
                                             Assumed dtype is float32 or float64.
        """
        if not isinstance(all_detected_points, np.ndarray) or all_detected_points.ndim != 2 or all_detected_points.shape[1] != 2:
             # Handle empty or invalid input gracefully
             self.points_in_neighbourhood = np.empty((0, 2), dtype=np.float32)
             # print("Warning: Invalid input to filter_points.")
             return

        half_L = self.width_L / 2.0
        half_H = self.height_H / 2.0
        min_X = self.current_Xc - half_L
        max_X = self.current_Xc + half_L
        min_Y = self.current_Yc - half_H
        max_Y = self.current_Yc + half_H

        if all_detected_points.shape[0] > 0:
            x_coords = all_detected_points[:, 0]
            y_coords = all_detected_points[:, 1]

            mask_x = (x_coords > min_X) & (x_coords < max_X)
            mask_y = (y_coords > min_Y) & (y_coords < max_Y)

            combined_mask = mask_x & mask_y

            self.points_in_neighbourhood = all_detected_points[combined_mask]
        else:
            self.points_in_neighbourhood = np.empty((0, 2), dtype=all_detected_points.dtype)

        # print(f"Points in neighbourhood: {self.points_in_neighbourhood.shape[0]}")

    def find_best_window_position(self, all_detected_points, step_size=20):
        """
        Finds the best window position (X, Y) that maximizes the density of
        detected points within the window.

        Args:
            all_detected_points (np.ndarray): (N, 2) NumPy array of detected points.
            step_size (int): The step size for sliding the window (in pixels).

        Returns:
            tuple: (best_x, best_y, max_density) - The X and Y coordinates of the
                   best window position and the maximum density found.
        """
        half_L = self.width_L / 2.0
        half_H = self.height_H / 2.0

        # Define the search space
        x_start = int(half_L)
        x_end = int(self.image_width - half_L)
        y_start = int(half_H)
        y_end = int(self.image_height - half_H)

        max_density = 0
        best_x = self.current_Xc  # Initialize with current position
        best_y = self.current_Yc

        # Iterate over all possible window positions
        for x in range(x_start, x_end, step_size):
            for y in range(y_start, y_end, step_size):
                # Calculate the window boundaries
                min_x = x - half_L
                max_x = x + half_L
                min_y = y - half_H
                max_y = y + half_H

                # Count the number of points within the window
                x_coords = all_detected_points[:, 0]
                y_coords = all_detected_points[:, 1]

                mask_x = (x_coords >= min_x) & (x_coords <= max_x)
                mask_y = (y_coords >= min_y) & (y_coords <= max_y)

                combined_mask = mask_x & mask_y
                num_points_in_window = np.sum(combined_mask)

                # Calculate the density of points within the window
                density = num_points_in_window / (self.width_L * self.height_H)

                # Update the best window position if the current density is higher
                if density > max_density:
                    max_density = density
                    best_x = x
                    best_y = y

        return best_x, best_y, max_density


    def update_position_for_next_frame(self, all_detected_points):
        """
        Updates the window's horizontal (current_Xc) and vertical (current_Yc) center
        for the next frame, positioning the window over the area with the highest
        density of detected points.

        Args:
            all_detected_points (np.ndarray): (N, 2) NumPy array from the current frame.
        """
        if not isinstance(all_detected_points, np.ndarray) or all_detected_points.ndim != 2 or all_detected_points.shape[1] != 2:
            # print("Warning: Invalid input to update_position_for_next_frame.")
            return  # Do not update if input is invalid

        if all_detected_points.shape[0] > 0:
            # Calculate the window boundaries
            half_L = self.width_L / 2.0
            half_H = self.height_H / 2.0

            # Calculate the density of points within the window at different positions
            best_x, best_y, max_density = self.find_best_window_position(all_detected_points)

            # Update the window position to the position with the highest density
            self.current_Xc = float(best_x)
            self.current_Yc = float(best_y)

        else:
            # No points detected: Move towards the center of the image
            image_center_y = self.image_height / 2.0
            # Adjust the Y position *gradually* towards the center
            self.current_Yc += 0.1 * (image_center_y - self.current_Yc)  # Adjust 0.1 for speed

        # Optional: Clamp the center X and Y to keep the window within bounds
        min_center_x = self.width_L / 2.0
        max_center_x = self.image_width - (self.width_L / 2.0)
        self.current_Xc = np.clip(self.current_Xc, min_center_x, max_center_x)

        min_center_y = self.height_H / 2.0
        max_center_y = self.image_height - (self.height_H / 2.0)
        self.current_Yc = np.clip(self.current_Yc, min_center_y, max_center_y)


    def draw(self, image, color=(255, 204, 102), thickness=2):
        """
        Draws the current neighbourhood window onto the provided image.

        Args:
            image (np.ndarray): The image (NumPy array) to draw on.
            color (tuple): BGR color tuple.
            thickness (int): Line thickness.
        """
        if image is None:
            return

        half_L = self.width_L / 2.0
        half_H = self.height_H / 2.0

        # Calculate top-left corner (ensure integer coordinates for drawing)
        top_left_x = int(self.current_Xc - half_L)
        top_left_y = int(self.current_Yc - half_H)

        # Calculate bottom-right corner
        bottom_right_x = int(top_left_x + self.width_L)
        bottom_right_y = int(top_left_y + self.height_H)

        # Define points for cv2.rectangle
        pt1 = (top_left_x, top_left_y)
        pt2 = (bottom_right_x, bottom_right_y)

        # Draw the rectangle
        cv2.rectangle(image, pt1, pt2, color, thickness)

        # Optional: Draw the center point
        # center_pt = (int(self.current_Xc), int(self.current_Yc))
        # cv2.circle(image, center_pt, 3, (0, 0, 255), -1) # Red dot

    def compute_extreme_points(self):
        """
        Finds the points with min and max Y coordinates within the neighbourhood.
        Stores results in self.extreme_points: [top_point, bottom_point].
        Points are (x, y) NumPy arrays or None if no points are inside.
        """
        self.extreme_points = [None, None] # Reset
        points_inside = self.points_in_neighbourhood

        if points_inside.shape[0] > 0:
            y_coords_inside = points_inside[:, 1]
            min_y_idx = np.argmin(y_coords_inside)
            max_y_idx = np.argmax(y_coords_inside)

            # Store the actual points (shape (2,))
            self.extreme_points[0] = points_inside[min_y_idx] # Top-most point
            self.extreme_points[1] = points_inside[max_y_idx] # Bottom-most point
        # else: self.extreme_points remains [None, None]


# this is change

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
        self.show_images = True  # Flag to enable/disable image display

        # Neighbourhood Tracker Parameters (adjust these!)
        self.neighbourhood_width = 300  # Adjust as needed
        self.neighbourhood_height = 200  # Adjust as needed
        self.initial_x = 320  # Center of the image
        self.initial_y = 240  # Center of the image (480 / 2)
        self.tracker = NeighbourhoodTracker(640, 480, self.initial_x, self.initial_y, self.neighbourhood_width, self.neighbourhood_height) # Image width and height


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            desired_width, desired_height = 640, 480  # Resize if needed
            cv_image = cv2.resize(cv_image, (desired_width, desired_height), interpolation=cv2.INTER_LINEAR)

            # 1. Filter points using the tracker's current window position
            all_points = self.detect_potential_points(cv_image) # Detect points in the whole image
            self.tracker.filter_points(all_points)
            points_in_neighbourhood = self.tracker.points_in_neighbourhood

            # 2. Fit line to points in the neighbourhood
            line_params = self.fit_and_draw_line(cv_image, points_in_neighbourhood)  # Use the filtered points

            if line_params:
                self.line_detected = True
            else:
                self.line_detected = False

            self.follow_line(line_params)

            # 3. Update the tracker's window position for the next frame
            self.tracker.update_position_for_next_frame(all_points)  # Use all points from this frame

            # 4. Draw the neighbourhood on the image (for visualization)
            display_frame = cv_image.copy()
            self.tracker.draw(display_frame)
            for pt in points_in_neighbourhood:
                cv2.circle(display_frame, (int(pt[0]), int(pt[1])), 3, (0, 255, 255), -1)  # Yellow dots

            if self.show_images:
                cv2.imshow("Tracking Window", display_frame)
                cv2.waitKey(1)

            self.rate.sleep()

        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def detect_potential_points(self, image):
        """
        Detects potential crop row points in the entire image using HSV masking and contour extraction.

        Args:
            image (np.ndarray): The input image.

        Returns:
            np.ndarray: An (N, 2) NumPy array of detected points (x, y).
        """
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, (self.h_min, self.s_min, self.v_min),
                                     (self.h_max, self.s_max, self.v_max))

        # Check if the mask is empty (all black pixels)
        if cv2.countNonZero(mask) == 0:
            # If no green pixels are found, return an empty array immediately
            return np.empty((0, 2), dtype=np.float32)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        filtered_contours = [cnt for cnt in contours if 150 < cv2.contourArea(cnt) < 100000]

        points = []
        for contour in filtered_contours:
            for point in contour:
                x, y = point[0]
                points.append([float(x), float(y)])  # Convert to float

        return np.array(points, dtype=np.float32) if points else np.empty((0, 2), dtype=np.float32)

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
        elif len(points) > 0:
            rospy.logwarn("Not enough points to fit a line! Need at least 2. Points found: " + str(len(points)))
            return None
        else:
            rospy.logwarn("No points to fit a line!")
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