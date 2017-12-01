from yaw_controller import *
from pid import *
from acc_pid import *
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, kp, ki, kd, max_speed, vehicle_mass, accel_limit, decel_limit):
        self.yawc = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.pidc = AccumulatingPID(kp, ki, kd, -1, 1)

    def control(self, command_twist, current_twist, dt):
        cmd_vel = command_twist.twist.linear.x
        cmd_steer = command_twist.twist.angular.z
        current_vel = current_twist.twist.linear.x
        current_steer = current_twist.twist.angular.z
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        vel_err = (cmd_vel - current_vel)

        if current_vel > 80:
            vel_err *= 2

        accel = self.pidc.step(vel_err, dt)
        steer = self.yawc.get_steering(current_vel, cmd_steer, current_vel)

        # steer = self.low_pass.filt(steer)

        #rospy.logwarn('curr_steer={:6.3f} cmd_steer={:6.3f} steer={:6.3f}'.format(
        #    current_steer, cmd_steer, steer))

        #rospy.logwarn('curr_vel={:+6.3f} cmd_vel={:+6.3f} err={:+6.3f} accel={:+6.3f}'.format(
        #    current_vel, cmd_vel, vel_err, accel))

        #if cmd_vel < .1:
        #    return 0, 100, steer

        #if (vel_err < 0):
        if (accel <= 0):
            brake_amount = -accel * 200
            return 0, brake_amount, steer

        return accel, 0, steer

    def reset(self):
        self.pidc.reset();
