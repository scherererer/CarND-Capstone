#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        sub1 = rospy.Subscriber("/traffic_waypoint", Int32, self.traffic_cb)
        # Cruft?
        #sub1 = rospy.Subscriber("/obstacle_waypoint", message_type, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = [];

        rospy.spin()

    def pose_cb(self, msg):
        '''
        :param msg: Current position of the vehicle of type PoseStamped
        '''
        # TODO: Implement

        lane = Lane();

        # Pass through the header info
        lane.header = msg.header

        wpidx = self.find_closest_waypoint(self.waypoints, msg.pose);

        for i in range (LOOKAHEAD_WPS):
            lane.waypoints.append (self.waypoints[(wpidx + i) % len(self.waypoints)]);

        self.final_waypoints_pub.publish(lane);

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints;

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        '''
        Computes the distance between two waypoints in a list along the piecewise linear arc
        connecting all waypoints between the two.

        :param waypoints: a list of waypoints
        :param wp1: Index of the first waypoint in the list
        :param wp2: Index of the second waypoint in the list
        '''
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def find_closest_waypoint(self, waypoints, pose):
        '''
        Find the closet waypoint to the given position
        :return: Index of the closest waypoint
        '''

        # TODO: This needs to only get waypoints _ahead_ of the car
        closestIndex = 0;
        closestDistSquared = 99999999;

        dsquared = lambda a, b: (a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2;
        for i, wp in enumerate (waypoints):
            testDistSquared = dsquared (wp.pose.pose.position, pose.position);
            if (testDistSquared < closestDistSquared):
                closestIndex = i;

        return closestIndex;


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
