#!/usr/bin/env python3
"""Lightweight deterministic lunar terrain and differential-rover simulator.

The terrain is selected by the `~scenario` parameter. `mixed` is the original
hand-tuned map and is the regression baseline, so its coefficients must not drift.
The other four are the typical-terrain classes the task book asks the comparison
experiments to cover, and their parameters are derived from the traversability
thresholds in config/lunar_system.yaml rather than picked by eye -- a scenario
whose features all fall below the lethal thresholds would measure nothing.
"""

import math
import threading

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Float32MultiArray, Header, MultiArrayDimension


def _dense_rocks():
    """Jittered 4x3 grid of boulders with corridors that provably admit a solution.

    The 4.0 m pitch is derived, not tuned. The offsets are written out rather than drawn
    from an RNG so the layout can be checked by reading it: the closest two centres are
    3.52 m apart, perception smears a boulder over its 3-cell step-height window so the
    effective lethal radius is about 0.5 + 0.3 m, and that leaves a 1.92 m gap against a
    1.5 m body -- tight, but never closed. A field that could seal shut would make
    规划成功率 measure the layout instead of the planner.
    """
    return [(-6.25, -9.78, 0.5, 0.5), (-5.72, -6.19, 0.5, 0.5), (-6.18, -1.75, 0.5, 0.5),
            (-1.78, -10.24, 0.5, 0.5), (-2.21, -5.80, 0.5, 0.5), (-1.73, -2.22, 0.5, 0.5),
            (2.19, -9.75, 0.5, 0.5), (1.76, -6.26, 0.5, 0.5), (2.24, -1.79, 0.5, 0.5),
            (5.75, -10.18, 0.5, 0.5), (6.22, -5.77, 0.5, 0.5), (5.80, -2.20, 0.5, 0.5)]


# crater tuple: (x, y, radius, depth, rim_height, sharpness, hazard_radius)
# rock tuple:   (x, y, radius, height)
SCENARIOS = {
    # Byte-for-byte the terrain every earlier measurement was taken on.
    "mixed": {
        "base": (0.025, 0.15, 0.12),          # tilt_x, sine_amp, sine_k (in y)
        "ripple": (0.05, 0.7, 0.5),           # amp, kx, ky
        # hazard_radius 2.0 is where the bowl wall passes the 20 deg limit
        # (|dz/dr| = 1.8*4r^3/3.2^4*exp(-(r/3.2)^4) reaches tan20 near r=1.85);
        # the 3.2 in the profile is the shape radius, which is much wider.
        "craters": [(5.0, 2.0, 3.2, 1.8, 0.55, 4, 2.0)],
        "rocks": [(0.0, -2.0, 0.65, 0.55),
                  (8.0, -5.0, 0.9, 0.75),
                  (-3.0, 5.0, 0.5, 0.40)],
        "start": (-10.0, -6.0, 0.0),
    },
    # 平坦区域: everywhere traversable. The baseline for 路径长度偏差 -- the detour
    # ratio here should be ~0, which is the only direct evidence of search optimality.
    "flat": {
        "base": (0.01, 0.02, 0.10),
        "ripple": (0.03, 0.4, 0.35),
        "craters": [],
        "rocks": [],
        "start": (-10.0, -6.0, 0.0),
    },
    # 密集障碍区: the main test of 避障成功率. The start is west of the field and on the
    # y=-8.0 corridor centreline, so the rover begins outside the boulders and has to
    # thread them rather than starting already boxed in.
    "dense": {
        "base": (0.01, 0.03, 0.10),
        "ripple": (0.03, 0.4, 0.35),
        "craters": [],
        "rocks": _dense_rocks(),
        "start": (-11.0, -8.0, 0.0),
    },
    # 大坡度起伏区: amp*k = 1.2*0.35 = 0.42 -> 22.8 deg, straddling the 20 deg limit,
    # so some slopes are climbable and some are not and routing actually matters. That
    # also means the start cannot be the shared (-10,-6): the gradient there is 0.541
    # (28.4 deg), outside the limit, and the rover would be unable to move at all. The
    # start sits in the largest passable component instead (x -5..5, y -12..1), at a
    # point measuring 15.2 deg.
    "slope": {
        "base": (0.15, 0.30, 0.09),
        "ripple": (1.2, 0.35, 0.28),
        "craters": [],
        "rocks": [],
        "start": (0.0, -5.5, 0.0),
    },
    # 负障碍物集中区: super-Gaussian (sharpness 6) gives near-vertical walls, well over
    # the 0.20 m step limit measured across a 3-cell 0.15 m window. Note the perception
    # stack cannot tell a pit from a boulder -- GroundSegmentation computes step_height
    # as an unsigned max-min -- so this scenario measures avoidance, not classification.
    "negative": {
        "base": (0.01, 0.04, 0.10),
        "ripple": (0.03, 0.4, 0.35),
        "craters": [(-4.0, -3.0, 1.5, 1.2, 0.0, 6, 1.5),
                    (-1.0, -6.5, 1.6, 1.2, 0.0, 6, 1.6),
                    (2.0, -3.5, 1.4, 1.0, 0.0, 6, 1.4),
                    (-5.5, -8.5, 1.5, 1.1, 0.0, 6, 1.5),
                    (1.0, -9.0, 1.5, 1.2, 0.0, 6, 1.5)],
        "rocks": [],
        "start": (-10.0, -6.0, 0.0),
    },
}


class LunarSurfaceSim:
    def __init__(self):
        self.lock = threading.Lock()
        name = rospy.get_param("~scenario", "mixed")
        if name not in SCENARIOS:
            rospy.logwarn("unknown scenario '%s', falling back to 'mixed'", name)
            name = "mixed"
        self.scenario_name = name
        self.scenario = SCENARIOS[name]
        sx, sy, syaw = self.scenario["start"]
        self.x = float(rospy.get_param("~initial_x", sx))
        self.y = float(rospy.get_param("~initial_y", sy))
        self.yaw = float(rospy.get_param("~initial_yaw", syaw))
        self.v = self.w = 0.0
        self.sensor_height = float(rospy.get_param("~sensor_height", 1.0))
        self.cloud_radius = float(rospy.get_param("~cloud_radius", 25.0))
        self.cloud_spacing = float(rospy.get_param("~cloud_spacing", 0.20))
        self.noise_std = float(rospy.get_param("~noise_std", 0.001))
        # Command saturation (raised for the 5 km/h = 1.39 m/s operating domain).
        self.v_cmd_limit = float(rospy.get_param("~v_cmd_limit", 1.5))
        self.w_cmd_limit = float(rospy.get_param("~w_cmd_limit", 0.8))
        # Skid-steer slip "nominal ground truth" injected for identification. Defaults are
        # identity (ideal differential drive) so existing demos are unchanged; the identify
        # launch overrides these with known values that identify_skidsteer.py must recover.
        self.alpha_v = float(rospy.get_param("~alpha_v", 1.0))
        self.alpha_w = float(rospy.get_param("~alpha_w", 1.0))
        self.x_icr = float(rospy.get_param("~x_icr", 0.0))
        self.slope_slip_gain = float(rospy.get_param("~slope_slip_gain", 0.0))
        self.slope_grade_gain = float(rospy.get_param("~slope_grade_gain", 0.0))
        self.rng = np.random.default_rng(42)
        self.cloud_pub = rospy.Publisher("/sensors/velodyne_points", PointCloud2, queue_size=1)
        self.odom_pub = rospy.Publisher("/localization/odometry/filtered_map", Odometry, queue_size=1)
        self.obstacle_pub = rospy.Publisher("/lunar_sim/obstacles", Float32MultiArray,
                                            queue_size=1, latch=True)
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.last = rospy.Time.now()
        self.publish_obstacles()
        rospy.loginfo("lunar_surface_sim: scenario=%s rocks=%d craters=%d start=(%.1f,%.1f)",
                      name, len(self.scenario["rocks"]), len(self.scenario["craters"]),
                      self.x, self.y)
        rospy.Timer(rospy.Duration(0.02), self.update)
        rospy.Timer(rospy.Duration(0.20), self.publish_cloud)

    def publish_obstacles(self):
        """Ground-truth hazard footprints, latched, four floats each: x, y, radius, height.

        These are what the metrics harness measures 安全距离 and collisions against.
        Using ground truth rather than the published costmap keeps "did the planner avoid
        the obstacle" separate from "did perception see it".

        `radius` is the hazard radius, not the shape radius: for a boulder the two coincide,
        but for a bowl the lethal ring is where the wall exceeds the slope limit, well inside
        the radius that defines the bowl's profile. Height is signed so a consumer can tell a
        pit from a boulder even though the perception stack cannot.
        """
        data = []
        for rx, ry, radius, h in self.scenario["rocks"]:
            data += [rx, ry, radius, h]
        for cx, cy, _r, depth, _rim, _sharp, hazard_r in self.scenario["craters"]:
            data += [cx, cy, hazard_r, -depth]
        msg = Float32MultiArray()
        msg.layout.dim = [MultiArrayDimension(label="obstacle", size=len(data) // 4,
                                              stride=len(data)),
                          MultiArrayDimension(label="xyrh", size=4, stride=4)]
        msg.data = data
        self.obstacle_pub.publish(msg)

    def terrain(self, x, y):
        tilt, sine_amp, sine_k = self.scenario["base"]
        r_amp, r_kx, r_ky = self.scenario["ripple"]
        z = tilt*x + sine_amp*np.sin(sine_k*y) + r_amp*np.sin(r_kx*x)*np.sin(r_ky*y)
        for cx, cy, radius, depth, rim_h, sharpness, _hazard_r in self.scenario["craters"]:
            r = np.hypot(x - cx, y - cy)
            z = z - depth*np.exp(-(r/radius)**sharpness)
            if rim_h:
                z = z + rim_h*np.exp(-((r - (radius + 0.6))/0.65)**2)
        return z

    def rocks(self, x, y):
        height = np.zeros_like(x)
        for rx, ry, radius, h in self.scenario["rocks"]:
            d = np.hypot(x-rx, y-ry)
            height = np.maximum(height, h*np.clip(1.0-d/radius, 0.0, 1.0))
        return height

    def terrain_gradient(self, x, y, eps=0.05):
        gx = (self.terrain(np.array(x+eps), np.array(y)) -
              self.terrain(np.array(x-eps), np.array(y))) / (2.0*eps)
        gy = (self.terrain(np.array(x), np.array(y+eps)) -
              self.terrain(np.array(x), np.array(y-eps))) / (2.0*eps)
        return float(gx), float(gy)

    def on_cmd(self, msg):
        with self.lock:
            self.v = float(np.clip(msg.linear.x, -self.v_cmd_limit, self.v_cmd_limit))
            self.w = float(np.clip(msg.angular.z, -self.w_cmd_limit, self.w_cmd_limit))

    def update(self, _event):
        now = rospy.Time.now()
        dt = min(max((now-self.last).to_sec(), 0.0), 0.1)
        self.last = now
        with self.lock:
            v_cmd, w_cmd, yaw = self.v, self.w, self.yaw
            # Terrain-frame gradient projected onto heading (skid-steer slip model,
            # kept in sync with groundgrid::SkidSteerModel::effectiveTwist).
            gx, gy = self.terrain_gradient(self.x, self.y)
            grad_long = gx*math.cos(yaw) + gy*math.sin(yaw)
            grad_lat = -gx*math.sin(yaw) + gy*math.cos(yaw)
            uphill_scale = max(0.0, 1.0 - self.slope_grade_gain*max(0.0, grad_long))
            vx = self.alpha_v*v_cmd*uphill_scale
            omega = self.alpha_w*w_cmd
            vy = -self.x_icr*omega - self.slope_slip_gain*grad_lat
            c, s = math.cos(yaw), math.sin(yaw)
            self.x += (c*vx - s*vy)*dt
            self.y += (s*vx + c*vy)*dt
            self.yaw += omega*dt
            x, y, yaw = self.x, self.y, self.yaw
        z = float(self.terrain(np.array(x), np.array(y)))
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        self.br.sendTransform((x, y, z), q, now, "base_link", "map")
        self.br.sendTransform((0.0, 0.0, self.sensor_height), (0, 0, 0, 1),
                              now, "velodyne", "base_link")
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = x, y, z
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = q[0], q[1]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q[2], q[3]
        self.odom_pub.publish(odom)

    def publish_cloud(self, _event):
        with self.lock:
            x0, y0, yaw = self.x, self.y, self.yaw
        values = np.arange(-self.cloud_radius, self.cloud_radius+self.cloud_spacing,
                           self.cloud_spacing, dtype=np.float32)
        lx, ly = np.meshgrid(values, values, indexing="xy")
        mask = lx*lx + ly*ly <= self.cloud_radius*self.cloud_radius
        lx, ly = lx[mask], ly[mask]
        cy, sy = math.cos(yaw), math.sin(yaw)
        wx, wy = x0 + cy*lx - sy*ly, y0 + sy*lx + cy*ly
        wz = self.terrain(wx, wy) + self.rocks(wx, wy)
        sensor_ground = float(self.terrain(np.array(x0), np.array(y0))) + self.sensor_height
        lz = wz - sensor_ground + self.rng.normal(0.0, self.noise_std, wz.shape)
        rings = np.mod((np.hypot(lx, ly)/self.cloud_spacing).astype(np.uint16), 32)
        points = list(zip(lx.tolist(), ly.tolist(), lz.astype(np.float32).tolist(),
                          np.full(lx.shape, 30.0, np.float32).tolist(), rings.tolist()))
        fields = [PointField("x",0,PointField.FLOAT32,1),
                  PointField("y",4,PointField.FLOAT32,1),
                  PointField("z",8,PointField.FLOAT32,1),
                  PointField("intensity",12,PointField.FLOAT32,1),
                  PointField("ring",16,PointField.UINT16,1)]
        # Use the most recent dynamics/TF timestamp to avoid asking TF for a
        # transform a few milliseconds into the future.
        header = Header(stamp=self.last, frame_id="velodyne")
        self.cloud_pub.publish(point_cloud2.create_cloud(header, fields, points))


if __name__ == "__main__":
    rospy.init_node("lunar_surface_sim")
    LunarSurfaceSim()
    rospy.spin()
