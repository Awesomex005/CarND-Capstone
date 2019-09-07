
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# with this controller, it was driving along down the road, but it's still wandered a little bit within lane.
# and that's actually an artifact of the Autoware code that we use.
# The Autoware code dosen't recompute your trajectory until you've passed a certain distance away from the waypoints
# or certain angle away from the trajectory of waypoints.
# with this simple controller, by the time it recomupted the trajectory and gave new twists commands to the car,
# the car had already sort of wandered away from the waypoints, and so the car then steers sort of suddenly back to the waypoints.
#
# To fix that, go into waypoint_follower cpp files and check the code and make sure that it's updating all the time.
# There's a function in there that checks if you're following the waypoints, and if you're not following the waypoints then it updates.
# We can modify that code to check basically to follow the waypoints all the time.
#
# The other things we can do is to change our yaw controller to dampen the steering a little bit.
# We could add some dampening terms that take current velocity and target angular velocity, look into the difference between them.
# if they are the same, we don't do anything, if they are very different, we might add some dampening to look in.

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.35 # 0.3
        ki = 0.1
        kd = 0.22 # 0.
        mn = 0.0 # Minimum throttle value
        mx = 0.3 # 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, dbw_enabled, current_vel, linear_vel, angular_vel):
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = 700 # 400 # N*m - to hold the car in place if we are stopped at light. Acceleration - 1m/s^2
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m

        return throttle, brake, steering
