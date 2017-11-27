#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, TrafficWaypoint, Waypoint

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


def angleDiff(a, b):
    '''
    Get the difference between two angles
    '''
    diff = math.fmod(b - a + math.pi, math.pi * 2.0);
    if (diff < 0):
        diff += math.pi * 2.0;
    return diff - math.pi;


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.twist_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        sub1 = rospy.Subscriber("/traffic_waypoint", TrafficWaypoint, self.traffic_cb)
        # Cruft?
        #sub1 = rospy.Subscriber("/obstacle_waypoint", message_type, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Base list of waypoints
        self.base_waypoints = [];
        # If the index is -1 then no detection has been made
        self.lightindex = -1;
        self.lightstate = TrafficWaypoint.UNKNOWN;

        self.lastvelocity = 0;

        rospy.spin()

    def pose_cb(self, msg):
        '''
        :param msg: Current position of the vehicle of type PoseStamped
        '''
        lane = Lane();

        # Pass through the header info
        lane.header = msg.header

        startwpindex = self.find_closest_waypoint(self.base_waypoints, msg.pose);

        for i in range (LOOKAHEAD_WPS):
            lane.waypoints.append (self.base_waypoints[(startwpindex + i) % len(self.base_waypoints)]);

        SPEED_LIMIT = rospy.get_param("/waypoint_loader/velocity");

        distanceToDest = 0;
        targetVelocity = 0;
        numwpts = self.lightindex - startwpindex if startwpindex <= self.lightindex else self.lightindex + len(self.base_waypoints) - startwpindex;

        if (self.lightindex < 0 or self.lightstate == TrafficWaypoint.GREEN):
            # Top Speed!
            distanceToDest = self.distance(self.base_waypoints, startwpindex,
                                           (startwpindex + 100) % len(self.base_waypoints));
            targetVelocity = SPEED_LIMIT;
        elif (numwpts < LOOKAHEAD_WPS and self.lightstate == TrafficWaypoint.RED):
            # Account for slowdown
            distanceToDest = self.distance(self.base_waypoints, startwpindex, self.lightindex);
            targetVelocity = 0.0;
        else:
            # TODO: Allow running through a yellow light as long as it doesn't cause an abrupt stop
            distanceToDest = self.distance(self.base_waypoints, startwpindex, self.lightindex);
            targetVelocity = 0.0;

        idealAccel = (targetVelocity**2 - self.lastvelocity**2) / (2 * distanceToDest);

        v = self.lastvelocity;
        dt = ((targetVelocity - self.lastvelocity) / idealAccel) / numwpts;

        for i in range(len(lane.waypoints)):
            v += idealAccel * dt;
            self.set_waypoint_velocity(lane.waypoints, i, v);

        rospy.logwarn('pose; v0 {}, vt {}, v_0 {}'.format(self.lastvelocity, targetVelocity, self.get_waypoint_velocity(lane.waypoints[0])))

        self.final_waypoints_pub.publish(lane);

    def twist_cb(self, msg):
        self.lastvelocity = msg.twist.linear.x;

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints;

    def traffic_cb(self, msg):
        # This is the index of the nearest upcoming red light
        # Used in pose_cb
        self.lightindex = msg.index;
        self.lightstate = msg.state;

        #if msg.state == TrafficWaypoint.RED:
        #    state = "red"
        #elif msg.state == TrafficWaypoint.YELLOW:
        #    state = "yellow"
        #elif msg.state == TrafficWaypoint.GREEN:
        #    state = "green"
        #else:
        #    state = "unknown"
        #rospy.logwarn('traffic light; waypoint: {}; state: {}'.format(msg.index, state))

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

        closestIndex = -1;
        closestDistSquared = 99999999;

        dsquared = lambda a, b: (a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2;
        calcAzimuth = lambda a, b: math.atan2(b.y-a.y, b.x-a.x);
        for i, wp in enumerate (waypoints):
            # Is this waypoint the closest candidate?
            testDistSquared = dsquared(wp.pose.pose.position, pose.position);
            if (testDistSquared >= closestDistSquared):
                continue;

            # Is this waypoint in front of us?
            vehicleToPointAzimuth = calcAzimuth(pose.position, wp.pose.pose.position);
            quaternion = (pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w);
            heading = tf.transformations.euler_from_quaternion(quaternion)[2]

            if (math.fabs(angleDiff(vehicleToPointAzimuth, heading)) >= math.pi/2.0):
                continue;

            closestDistSquared = testDistSquared;
            closestIndex = i;

        assert(closestIndex >= 0);

        return closestIndex;


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
