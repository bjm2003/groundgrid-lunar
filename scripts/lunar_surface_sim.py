#!/usr/bin/env python3
"""Lightweight deterministic lunar terrain and differential-rover simulator."""

import math
import threading

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


class LunarSurfaceSim:
    def __init__(self):
        self.lock = threading.Lock()
        self.x = float(rospy.get_param("~initial_x", -10.0))
        self.y = float(rospy.get_param("~initial_y", -6.0))
        self.yaw = float(rospy.get_param("~initial_yaw", 0.0))
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
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.last = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.02), self.update)
        rospy.Timer(rospy.Duration(0.20), self.publish_cloud)

    @staticmethod
    def terrain(x, y):
        base = 0.025 * x + 0.15 * np.sin(0.12 * y)
        crater_r = np.hypot(x - 5.0, y - 2.0)
        crater = -1.8 * np.exp(-(crater_r / 3.2) ** 4)
        rim = 0.55 * np.exp(-((crater_r - 3.8) / 0.65) ** 2)
        ripple = 0.05 * np.sin(0.7 * x) * np.sin(0.5 * y)
        return base + crater + rim + ripple

    @staticmethod
    def rocks(x, y):
        height = np.zeros_like(x)
        for rx, ry, radius, h in [(0.0, -2.0, 0.65, 0.55),
                                  (8.0, -5.0, 0.9, 0.75),
                                  (-3.0, 5.0, 0.5, 0.40)]:
            d = np.hypot(x-rx, y-ry)
            height = np.maximum(height, h*np.clip(1.0-d/radius, 0.0, 1.0))
        return height

    @classmethod
    def terrain_gradient(cls, x, y, eps=0.05):
        gx = (cls.terrain(np.array(x+eps), np.array(y)) -
              cls.terrain(np.array(x-eps), np.array(y))) / (2.0*eps)
        gy = (cls.terrain(np.array(x), np.array(y+eps)) -
              cls.terrain(np.array(x), np.array(y-eps))) / (2.0*eps)
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
