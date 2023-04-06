#!/usr/bin/env python3
# coding: utf-8

import time
from math import sin, cos
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from hector_uav_msgs.srv import EnableMotors

import cv2
from cv_bridge import CvBridge, CvBridgeError

from nav_msgs.msg import Odometry
import tf2_ros
import tf.transformations as ttt

PLANING_HORIZON = 50

TIME_LIFTOFF = 3

V_MAX = 2.05
W_MAX = 0.35

u_x = 1

Kp_z = 0.5

Kp_y = 0.015
Kd_y = 0.0005
Ki_y = 0.0000825

Kp_w = 0.01555
Kd_w = 0.000095
Ki_w = 0.000165

RING_AVOIDANCE_TIME = 5  # [seconds]
DEFAULT_ALTITUDE = 3    # [meters]


class SimpleMover():

    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.cv_bridge = CvBridge()

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("cam_1/camera/image", Image,
                         self.line_camera_callback)
        rospy.Subscriber("cam_2/camera/image", Image,
                         self.ring_camera_callback)

        rospy.Subscriber('/ground_truth/state', Odometry, self.obom_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.drone_state = [0] * 6  # position vector
        self.y_error = 0
        self.omega_error = 0
        self.rate = rospy.Rate(30)
        self.z_des = DEFAULT_ALTITUDE

        self.line_image = None
        self.ring_image = None
        self.state = "free_flight"
        self.red_ring_detected = False
        self.blue_ring_detected = False
        self.time_start_up = 0
        self.avoidance_time = 0
        self.e_x_blue, self.e_y_blue = 0, 0

    def fsm_update(self):

        if self.red_ring_detected:
            self.state = "drone_up"

        elif RING_AVOIDANCE_TIME < self.avoidance_time < RING_AVOIDANCE_TIME + 4:
            self.state = "drone_down"
            self.time_start_up = 0

        elif self.blue_ring_detected:
            self.state = "drone_blue_ring"

        else:
            self.state = "free_flight"

    def obom_callback(self, msg):
        """ Pose of a robot extraction"""
        try:
            transform = self.tfBuffer.lookup_transform(
                'world', 'base_stabilized', rospy.Time()).transform
            x, y, z = transform.translation.x, transform.translation.y, transform.translation.z
            quat = transform.rotation
            r, p, y = ttt.euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w])

            self.drone_state = [x, y, z, r, p, y]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Can't get transform from world to base_link")

    def ring_camera_callback(self, msg):

        # """ Computer vision stuff for Rings"""

        try:
            self.ring_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        cv_image = self.ring_image
        # red

        lower = np.uint8([0, 0, 90])
        upper = np.uint8([30, 30, 120])
        cv_image, red_pose, red_radius = self.ring_detector(
            cv_image, lower, upper, (0, 0, 255))

        # blue

        lower = np.uint8([40, 20, 20])
        upper = np.uint8([80, 50, 50])
        cv_image, blue_pose, blue_radius = self.ring_detector(
            cv_image, lower, upper, (255, 0, 0))

        # print(red_radius, blue_radius)

        if 50 < red_radius < 70 or 50 < blue_radius < 80:
            if red_radius > blue_radius:
                self.blue_ring_detected = False
                self.red_ring_detected = True
            else:
                self.red_ring_detected = False
                self.blue_ring_detected = True
                # offset in ring xy-plane to fly through center of a ring
                # error = <center of image> - <center of ring>
                self.e_x_blue = 160 - blue_pose[0]
                self.e_y_blue = 120 - blue_pose[1]

        else:
            self.blue_ring_detected = False
            self.red_ring_detected = False

        # save results
        self.image_2 = cv_image

    def line_camera_callback(self, msg):
        """ Computer vision stuff"""
        try:
            self.line_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        if self.line_image is None:
            return
        cv_camera = self.line_image
        grey_image = cv2.cvtColor(cv_camera, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(grey_image, 8, 255, cv2.THRESH_BINARY_INV)
        cv_camera = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cv2.line(cv_camera, (160, 0), (160, 240), (0, 123, 0), 1)
        cv2.line(cv_camera, (0, 120), (320, 120), (0, 123, 0), 1)

        # "steering" conrol
        top_points = np.where(mask[10] >= 10)
        mid_points = np.where(mask[int(msg.height / 2.0)] >= 10)
        if (not np.isnan(np.average(top_points)) and not np.isnan(np.average(mid_points))):
            top_line_point = int(np.average(top_points))
            mid_line_point = int(np.average(mid_points))
            self.omega_error = top_line_point - mid_line_point

            cv2.circle(cv_camera, (top_line_point, 10), 5, (0, 0, 255), 1)
            cv2.circle(cv_camera, (mid_line_point, int(
                msg.height/2)), 5, (0, 0, 255), 1)
            cv2.line(cv_camera, (mid_line_point, int(msg.height/2)),
                     (top_line_point, 10), (0, 0, 255), 3)

        # y-offset control
        __, cy_list = np.where(mask >= 10)
        if not np.isnan(np.average(cy_list)):
            cy = int(np.average(cy_list))
            self.y_error = msg.width / 2 - cy

            cv2.circle(cv_camera, (cy, int(msg.height/2)),
                       7, (0, 255, 0), 1)
            cv2.line(cv_camera, (160, 120),
                     (cy, int(msg.height/2)), (0, 255, 0), 3)

        self.show_image(cv_camera, title='Line follower')

    def ring_detector(self, image, lower, upper, color):

        color_mask = cv2.inRange(image, lower, upper)

        color_contours, _ = cv2.findContours(
            color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if color_contours:

            max_len_c = 0

            c = color_contours[0]

            for i in range(0, len(color_contours)):

                if len(color_contours[i]) > max_len_c:

                    c = color_contours[i]

                    max_len_c = len(color_contours[i])

            self.color_distance = max_len_c

            M = cv2.moments(c)

            if M['m00'] != 0:

                cx = int(M['m10']/M['m00'])

                cy = int(M['m01']/M['m00'])

            else:

                cx = 0

                cy = 0

            (x1, y1), color_r = cv2.minEnclosingCircle(c)

            if color_r > 10:

                image = cv2.circle(image, (cx, cy), radius=5,
                                   color=color, thickness=-1)

                cv2.drawContours(color_r, c, -1, (0, 255, 0), 1)

                color_r = cv2.circle(color_r, (int(x1), int(y1)), radius=int(
                    color_r), color=color, thickness=4)

                return image, (x1, y1), color_r[0]

        return image, (0, 0), 0

    def show_image(self, img, title='Camera 1'):
        cv2.imshow(title, img)
        cv2.waitKey(3)

    def enable_motors(self):
        try:
            rospy.wait_for_service('enable_motors', 2)
            call_service = rospy.ServiceProxy('enable_motors', EnableMotors)
            response = call_service(True)
        except Exception as e:
            print("Error while try to enable motors: ", e)

    def spin(self):
        self.enable_motors()
        print()
        if (self.ring_image is not None and self.line_image is not None):

            self.show_image(self.line_image, title='Line')

            self.show_image(self.ring_image, title='Rings')

        # Initialisations
        altitude_prev = 0
        y_error_prev = 0
        omega_error_prev = 0

        alpha = self.drone_state[5]

        time_start = rospy.get_time()
        time_prev = time_start
        while not rospy.is_shutdown():
            try:
                self.fsm_update()

                if self.state == "drone_up":
                    self.z_des = 5

                    if self.time_start_up == 0:
                        self.time_start_up = rospy.get_time()

                elif self.state == "drone_down":
                    self.z_des = DEFAULT_ALTITUDE

                elif self.state == "drone_blue_ring":
                    self.z_des += 0.001 * self.e_y_blue
                    if self.z_des < 1.5:
                        self.z_des = 1.5
                    pass

                elif self.state == "free_flight":
                    pass

                else:
                    rospy.logerr("Error: state name error!")

                self.avoidance_time = rospy.get_time() - self.time_start_up
                print(self.state, self.z_des)

                # Time stuff
                t = rospy.get_time() - time_start
                dt = t - time_prev
                time_prev = t
                if dt == 0:
                    dt = 1 / 30.

                # TODO: Write here altitude PD controller
                # Here!
                # u_z = K(z_des - z(t)) - Bdz(t)
                K = Kp_z
                B = 0.0
                z_des = self.z_des
                dt = 1.0 / 30.0
                u_z = K*(z_des - self.drone_state[2]) - \
                    B*(self.drone_state[2] - altitude_prev) / dt
                altitude_prev = self.drone_state[2]

                # TODO: Steering controls
                # u_z = K_s*omega_error(k) - B_s * (omega_error(k) - omega_error(k-1))
                K_s = Kp_w
                B_s = Kd_w
                u_omega_z = -K_s*self.omega_error + \
                    B_s*(self.omega_error - omega_error_prev) / dt

                omega_error_prev = self.omega_error

                # TODO: Offset control
                # u_y = K_o*y_error(k) - B_o * (y_error(k) - y_error(k-1))
                K_o = Kp_y
                B_o = Kd_y
                u_y = K_o*self.y_error + \
                    B_o*(self.y_error - y_error_prev)/dt

                y_error_prev = self.y_error

                twist_msg = Twist()
                twist_msg.linear.x = u_x
                twist_msg.linear.y = u_y
                twist_msg.linear.z = u_z
                twist_msg.angular.z = u_omega_z
                self.cmd_vel_pub.publish(twist_msg)

            except KeyboardInterrupt:
                break

            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    simple_mover = SimpleMover()
    simple_mover.spin()
