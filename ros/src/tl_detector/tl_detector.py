#!/usr/bin/env python
import sys
from collections import namedtuple
from math import pow, sqrt

import yaml

import cv2
import rospy
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from light_classification.tl_classifier import TLClassifier
from sensor_msgs.msg import Image
from styx_msgs.msg import (Lane, TrafficLight, TrafficLightArray,
                           TrafficWaypoint)

STATE_COUNT_THRESHOLD = 3

Point = namedtuple('Point', ['x', 'y'])

LightData = namedtuple('LightData', ['index', 'state'])


def distance(point1, point2):
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2))


def get_light_point(light):
    position = light.pose.pose.position
    return Point(int(position.x), int(position.y))


class Detector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.car_index = 0
        self.waypoints = None
        self.stop_lines = []

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        self.pub = rospy.Publisher('/traffic_waypoint', TrafficWaypoint,
                                   queue_size=1)

        self.state = TrafficLight.UNKNOWN
        self.state_count = 0

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            stop_line = self.get_stop_line()
            if stop_line is None:
                continue

            state = self.get_traffic_light_state()
            if state == TrafficLight.UNKNOWN:
                continue

            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count == STATE_COUNT_THRESHOLD:
                tw = TrafficWaypoint(stop_line, state)
                self.pub.publish(tw)

            self.state_count += 1

    def pose_cb(self, msg):
        if self.waypoints is None:
            return

        current_dist = distance(msg.pose.position,
                                self.waypoints[self.car_index])

        for i in range(self.car_index + 1, len(self.waypoints)):
            dist = distance(msg.pose.position, self.waypoints[i])
            if dist > current_dist:
                break
            current_dist = dist
            self.car_index = i

    def waypoints_cb(self, msg):
        self.waypoints = []
        for waypoint in msg.waypoints:
            position = waypoint.pose.pose.position
            self.waypoints.append(Point(position.x, position.y))

        config_string = rospy.get_param('/traffic_light_config')

        self.stop_lines = []
        for position in yaml.load(config_string)['stop_line_positions']:
            index = self.get_closest_waypoint(Point(position[0], position[1]))
            self.stop_lines.append(index)
        self.stop_lines.sort()

    def get_closest_waypoint(self, point):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            point: point to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if self.waypoints is None or len(self.waypoints) == 0:
            rospy.logerr('No waypoints in traffic light detector')
            return -1

        closest_dist = sys.maxint

        for i in range(len(self.waypoints)):
            dist = distance(self.waypoints[i], point)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i

        return closest_index

    def get_stop_line(self):
        if self.stop_lines is None:
            return None

        for index in self.stop_lines:
            if index > self.car_index:
                return index

        return None


class ImageDetector(Detector):
    def __init__(self):
        super(ImageDetector, self).__init__()

        self.camera_image = None

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        rospy.Subscriber('/image_color', Image, self.image_cb)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, 'bgr8')

        # Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        # TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN


class DummyDetector(Detector):
    def __init__(self):
        super(DummyDetector, self).__init__()

        self.traffic_lights = None
        self.tl_map = {}

        rospy.Subscriber('/vehicle/traffic_lights',
                         TrafficLightArray, self.traffic_cb)

    def traffic_cb(self, msg):
        if self.waypoints is None:
            return

        if self.traffic_lights is None:
            self.traffic_lights = {}
            for light in msg.lights:
                point = get_light_point(light)
                index = self.get_closest_waypoint(light.pose.pose.position)
                self.traffic_lights[point] = LightData(index, light.state)
                self.tl_map[index] = point
        else:
            for light in msg.lights:
                point = get_light_point(light)
                index = self.traffic_lights[point].index
                self.traffic_lights[point] = LightData(index, light.state)

    def get_traffic_light_state(self):
        if self.traffic_lights is None:
            return TrafficLight.UNKNOWN

        tl_index = None

        for index in sorted(self.tl_map.keys()):
            if index > self.car_index:
                tl_index = index
                break

        if tl_index == None:
            return TrafficLight.UNKNOWN

        dist = distance(self.waypoints[tl_index],
                        self.waypoints[self.car_index])
        if dist > 75:
            return TrafficLight.UNKNOWN

        return self.traffic_lights[self.tl_map[tl_index]].state


if __name__ == '__main__':
    try:
        detector = DummyDetector()
        detector.loop()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
