#!/usr/bin/env python3
"""Lightweight deterministic lunar terrain and differential-rover simulator.

The terrain is selected by the `~scenario` parameter. `mixed` is the original
hand-tuned map and is the regression baseline, so its coefficients must not drift.
The other four are the typical-terrain classes the task book asks the comparison
experiments to cover, and their parameters are derived from the traversability
thresholds in config/lunar_system.yaml rather than picked by eye -- a scenario
whose features all fall below the lethal thresholds would measure nothing.

The sensor is a ray-cast rotating multi-beam LiDAR (see `_build_rays`). It replaced a
uniform Cartesian grid of samples, which produced a costmap that was mostly artefact:
GroundGrid gates ground-patch detection on azimuth sample density, and a grid coarser
than the map cell cannot meet that gate at any range.
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

from lunar_terrain import AnalyticLunarTerrain, SCENARIOS


class LunarSurfaceSim:
    def __init__(self):
        self.lock = threading.Lock()
        name = rospy.get_param("~scenario", "mixed")
        if name not in SCENARIOS:
            rospy.logwarn("unknown scenario '%s', falling back to 'mixed'", name)
            name = "mixed"
        self.scenario_name = name
        self.scenario = SCENARIOS[name]
        self.terrain_model = AnalyticLunarTerrain(self.scenario)
        sx, sy, syaw = self.scenario["start"]
        self.x = float(rospy.get_param("~initial_x", sx))
        self.y = float(rospy.get_param("~initial_y", sy))
        self.yaw = float(rospy.get_param("~initial_yaw", syaw))
        self.v = self.w = 0.0
        self.sensor_height = float(rospy.get_param("~sensor_height", 1.0))
        self.cloud_radius = float(rospy.get_param("~cloud_radius", 25.0))
        self.n_rings = int(rospy.get_param("~n_rings", 64))
        self.elev_max_deg = float(rospy.get_param("~elev_max_deg", 2.0))
        self.elev_min_deg = float(rospy.get_param("~elev_min_deg", -24.8))
        self.azimuth_res_deg = float(rospy.get_param("~azimuth_res_deg", 0.2))
        self.march_step = float(rospy.get_param("~march_step", 0.30))
        self.min_range = float(rospy.get_param("~min_range", 1.0))
        self.cloud_period = float(rospy.get_param("~cloud_period", 0.20))
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
        self._build_rays()
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
        rospy.Timer(rospy.Duration(self.cloud_period), self.publish_cloud)

    def _build_rays(self):
        """Beam table for a rotating multi-beam LiDAR, sized from GroundGrid's own gates.

        The azimuth step is not a free knob. GroundSegmentation admits a ground patch only
        when the points in it beat `0.25 * patchSize * atan(1/dist_in_cells)/0.2deg`
        (GroundSegmentation.cpp:420, with the 0.2 deg fixed in GroundSegmentation.h:92),
        which is its estimate of how many azimuth samples span one cell. It then needs a
        non-zero intra-cell variance (:438), so a cell must hold at least two points at
        different bearings. Sampling coarser than 0.2 deg fails both at every range.

        The elevation span is an HDL-64e's, taken as given rather than trimmed to the beams
        that pay off on flat ground. It is tempting to drop everything above -1.8 deg, since
        at the 1.0 m sensor height those beams land past the 25 m cloud radius and return
        nothing -- but which beams return is a property of the terrain, not the sensor. On
        the 8.5 deg base tilt of the slope scenario the -1.8 deg beam lands at 5.5 m, and
        trimming would have capped uphill sensing there without any of it being visible as
        a parameter. Roughly a sixth of the beams are wasted on level ground; that is the
        price of the range not silently depending on the slope.
        """
        elev = np.deg2rad(np.linspace(self.elev_max_deg, self.elev_min_deg, self.n_rings))
        az = np.deg2rad(np.arange(0.0, 360.0, self.azimuth_res_deg))
        self.ray_az = np.tile(az, self.n_rings)
        self.ray_tan = np.repeat(np.tan(elev), az.size)
        self.ray_ring = np.repeat(np.arange(self.n_rings, dtype=np.uint16), az.size)
        rospy.loginfo("lunar_surface_sim: %d beams x %d azimuths = %d rays, march %.2f m",
                      self.n_rings, az.size, self.ray_az.size, self.march_step)

    def publish_obstacles(self):
        """Ground-truth hazard footprints, latched, four floats each: x, y, radius, height.

        These are what the metrics harness measures 安全距离 and collisions against.
        Using ground truth rather than the published costmap keeps "did the planner avoid
        the obstacle" separate from "did perception see it".

        `radius` is the hazard radius, not the shape radius. For a bowl the lethal ring is
        where the wall exceeds the slope limit, well inside the radius that defines the
        profile. A boulder is a cone -- h*(1 - d/radius) -- so the two differ there too: at
        the shape radius the rock is 0 m tall, and the system's own definition of an
        obstacle is `min_obstacle_height`, below which the costmap rates the ground
        drivable and is right to. Publishing the shape radius made the harness score a
        collision whenever the body passed over the rock's outermost few centimetres; on
        the (0,-2) boulder that band is 0.12 m wide, which is under one map cell, so no
        planner working from a 0.15 m grid could have satisfied it. Height is signed so a
        consumer can tell a pit from a boulder even though the perception stack cannot.
        """
        # Must match lunar_traversability's threshold: that node decides what is lethal,
        # and a hazard radius derived from any other number would measure the mismatch.
        thresh = rospy.get_param("/lunar_traversability/min_obstacle_height", 0.10)
        data = []
        for rx, ry, radius, h in self.scenario["rocks"]:
            data += [rx, ry, radius * max(0.0, 1.0 - thresh / h), h]
        for cx, cy, _r, depth, _rim, _sharp, hazard_r in self.scenario["craters"]:
            data += [cx, cy, hazard_r, -depth]
        msg = Float32MultiArray()
        msg.layout.dim = [MultiArrayDimension(label="obstacle", size=len(data) // 4,
                                              stride=len(data)),
                          MultiArrayDimension(label="xyrh", size=4, stride=4)]
        msg.data = data
        self.obstacle_pub.publish(msg)

    def terrain(self, x, y):
        return self.terrain_model.terrain(x, y)

    def height_at(self, x, y):
        return self.terrain_model.height_at(x, y)

    def rocks(self, x, y):
        return self.terrain_model.rocks(x, y)

    def ground_height(self, x, y):
        return self.terrain_model.ground_height(x, y)

    def terrain_gradient(self, x, y, eps=0.05):
        return self.terrain_model.terrain_gradient(x, y, eps)

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
        z = self.height_at(x, y)
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
        """March every beam out from the sensor and keep the first surface it meets.

        Marching outward rather than evaluating the terrain under each sample point is
        what buys occlusion: a beam stopped by a boulder never reports the ground behind
        it, and a shallow beam skims over the far wall of a pit instead of seeing into it.
        The old sampler had every surface visible at all times, which made the negative
        obstacle scenario far easier than it should be.

        The active set only ever shrinks, and steep beams terminate within a few metres,
        so the march costs about a tenth of the naive rays x steps product.
        """
        with self.lock:
            x0, y0, yaw = self.x, self.y, self.yaw
        z_sensor = self.height_at(x0, y0) + self.sensor_height
        caz, saz = np.cos(self.ray_az + yaw), np.sin(self.ray_az + yaw)

        hit = np.full(self.ray_az.size, np.nan)
        rh = rh_prev = self.min_range
        gap = (z_sensor + rh*self.ray_tan) - self.ground_height(x0 + rh*caz, y0 + rh*saz)
        idx = np.flatnonzero(gap > 0.0)
        gap_prev = gap[idx]

        while idx.size and rh + self.march_step <= self.cloud_radius:
            rh += self.march_step
            gap = ((z_sensor + rh*self.ray_tan[idx]) -
                   self.ground_height(x0 + rh*caz[idx], y0 + rh*saz[idx]))
            below = gap <= 0.0
            if below.any():
                # gap_prev > 0 >= gap holds by construction, so the bracket has a root.
                gp, g = gap_prev[below], gap[below]
                hit[idx[below]] = rh_prev + self.march_step*(gp/(gp - g))
                keep = ~below
                idx, gap_prev = idx[keep], gap[keep]
            else:
                gap_prev = gap
            rh_prev = rh

        got = np.flatnonzero(~np.isnan(hit))
        if got.size == 0:
            rospy.logwarn_throttle(5.0, "lunar_surface_sim: no beam returned a point")
            return
        rho = hit[got]
        lx = (rho*np.cos(self.ray_az[got])).astype(np.float32)
        ly = (rho*np.sin(self.ray_az[got])).astype(np.float32)
        lz = (rho*self.ray_tan[got] +
              self.rng.normal(0.0, self.noise_std, rho.shape)).astype(np.float32)
        points = list(zip(lx.tolist(), ly.tolist(), lz.tolist(),
                          np.full(got.size, 30.0, np.float32).tolist(),
                          self.ray_ring[got].tolist()))
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
