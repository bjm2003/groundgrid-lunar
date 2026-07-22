#!/usr/bin/env python3
"""Slope raster export utility for GroundGrid.

Subscribes to the slope outputs of the GroundGrid node and writes them to disk
as raster images. Implements the export side of the slope-aware extension that
follows the height-based slope calculation strategy from TS-SatMVSNet
(arXiv:2501.01049).

For each incoming slope frame the script writes:
  * ``slope_<seq>.tif``       - 32-bit float GeoTIFF in degrees (if rasterio is
                                available), otherwise a 32-bit float PNG.
  * ``slope_<seq>.png``       - 8-bit grayscale PNG (0..255 mapped from 0..clip).
  * ``slope_color_<seq>.png`` - JET color-mapped 8-bit PNG (only if cv2 is
                                available).

Topics consumed (any subset works, only the ones that get data are processed):
  * ``/groundgrid/slope_raster``         (sensor_msgs/Image, 32FC1, units=deg)
  * ``/groundgrid/slope_grid``           (nav_msgs/OccupancyGrid, deg scaled
                                          0..100% of clip range)

Run with:
    rosrun groundgrid save_slope_raster.py _output_dir:=/tmp/groundgrid_slope
"""

import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE = CvBridge()
except Exception as exc:                                       # pragma: no cover
    _CV_BRIDGE = None
    rospy.logwarn("cv_bridge unavailable (%s); raw image saving disabled.", exc)

try:
    import cv2
    _HAS_CV2 = True
except Exception:                                              # pragma: no cover
    _HAS_CV2 = False

try:
    import rasterio                                            # noqa: F401
    from rasterio.transform import from_origin
    _HAS_RASTERIO = True
except Exception:                                              # pragma: no cover
    _HAS_RASTERIO = False


def _ensure_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path, exist_ok=True)


def _save_geotiff(path, slope_deg, resolution, origin_xy):
    """Write a 32-bit float GeoTIFF of the slope raster in degrees."""
    if not _HAS_RASTERIO:
        return False
    height, width = slope_deg.shape
    transform = from_origin(origin_xy[0], origin_xy[1] + height * resolution,
                            resolution, resolution)
    profile = {
        "driver": "GTiff",
        "height": height,
        "width": width,
        "count": 1,
        "dtype": "float32",
        "transform": transform,
        "crs": None,                       # local map frame, no CRS by default
        "nodata": float("nan"),
    }
    with rasterio.open(path, "w", **profile) as dst:
        dst.write(slope_deg.astype(np.float32), 1)
    return True


def _save_png_gray(path, slope_deg, clip_deg):
    """Save an 8-bit grayscale PNG normalized to [0, clip_deg] -> [0, 255]."""
    arr = np.clip(slope_deg, 0.0, clip_deg) / max(clip_deg, 1e-6) * 255.0
    arr = arr.astype(np.uint8)
    if _HAS_CV2:
        cv2.imwrite(path, arr)
        return True
    # Fallback: write a raw .npy beside the requested .png
    np.save(os.path.splitext(path)[0] + ".npy", slope_deg.astype(np.float32))
    return False


def _save_png_color(path, slope_deg, clip_deg):
    """Save a JET color-mapped PNG."""
    if not _HAS_CV2:
        return False
    arr = np.clip(slope_deg, 0.0, clip_deg) / max(clip_deg, 1e-6) * 255.0
    arr = arr.astype(np.uint8)
    color = cv2.applyColorMap(arr, cv2.COLORMAP_JET)
    cv2.imwrite(path, color)
    return True


class SlopeRasterSaver(object):
    def __init__(self):
        self.output_dir = rospy.get_param("~output_dir", "/tmp/groundgrid_slope")
        self.clip_deg = float(rospy.get_param("~clip_deg", 45.0))
        self.grid_max_deg = float(rospy.get_param("~grid_max_deg", 50.0))
        self.save_geotiff = bool(rospy.get_param("~save_geotiff", True))
        self.save_png = bool(rospy.get_param("~save_png", True))
        self.save_color = bool(rospy.get_param("~save_color", True))
        self.subsample = int(rospy.get_param("~subsample", 1))
        _ensure_dir(self.output_dir)

        self._raw_count = 0
        self._grid_count = 0

        rospy.loginfo("save_slope_raster: writing to '%s'", self.output_dir)
        rospy.loginfo("save_slope_raster: clip_deg=%.2f, grid_max_deg=%.2f, subsample=%d",
                      self.clip_deg, self.grid_max_deg, self.subsample)
        if self.save_geotiff and not _HAS_RASTERIO:
            rospy.logwarn("rasterio not installed -> GeoTIFFs will be saved as PNG fallback.")
        if (self.save_png or self.save_color) and not _HAS_CV2:
            rospy.logwarn("cv2 not installed -> PNG outputs disabled, raw .npy fallback only.")

        self.image_sub = rospy.Subscriber("/groundgrid/slope_raster", Image,
                                          self._on_image, queue_size=2)
        self.grid_sub = rospy.Subscriber("/groundgrid/slope_grid", OccupancyGrid,
                                         self._on_grid, queue_size=2)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_image(self, msg):
        self._raw_count += 1
        if self.subsample > 1 and (self._raw_count % self.subsample) != 0:
            return
        if _CV_BRIDGE is None:
            rospy.logwarn_throttle(5.0,
                "cv_bridge unavailable; cannot decode slope_raster topic.")
            return
        try:
            slope_deg = _CV_BRIDGE.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:                              # pragma: no cover
            rospy.logwarn("Failed to decode slope_raster: %s", exc)
            return

        stamp = msg.header.stamp.to_nsec()
        prefix = "slope_{:020d}".format(stamp)

        if self.save_geotiff:
            tif_path = os.path.join(self.output_dir, prefix + ".tif")
            ok = _save_geotiff(tif_path, slope_deg, resolution=1.0,
                               origin_xy=(0.0, 0.0))
            if not ok:
                # GeoTIFF unavailable: save a 32-bit numpy file as lossless fallback
                np.save(os.path.join(self.output_dir, prefix + ".npy"),
                        slope_deg.astype(np.float32))

        if self.save_png:
            png_path = os.path.join(self.output_dir, prefix + ".png")
            _save_png_gray(png_path, slope_deg, self.clip_deg)

        if self.save_color:
            color_path = os.path.join(self.output_dir, prefix + "_color.png")
            _save_png_color(color_path, slope_deg, self.clip_deg)

        if self._raw_count % 50 == 0:
            rospy.loginfo("save_slope_raster: %d slope frames written so far.",
                          self._raw_count)

    def _on_grid(self, msg):
        """Optional path: save the OccupancyGrid as a georeferenced raster.

        Because the OccupancyGrid carries metric origin and resolution we can
        produce a GeoTIFF that is consistent with the map frame.
        """
        self._grid_count += 1
        if self.subsample > 1 and (self._grid_count % self.subsample) != 0:
            return

        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0:
            return
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        # OccupancyGrid uses -1 for unknown; mask those out.
        unknown = data < 0
        # Stored value 0..100 corresponds to slope 0..grid_max_deg. This must
        # match the nodelet's scaling:
        # `slope_norm_max = max(1, slope_max_traversable_deg * 2)`.
        slope_norm_max = max(1.0, self.grid_max_deg)
        slope_deg = (np.clip(data, 0, 100).astype(np.float32) / 100.0) * slope_norm_max
        slope_deg[unknown] = float("nan")

        stamp = msg.header.stamp.to_nsec()
        prefix = "slope_grid_{:020d}".format(stamp)

        if self.save_geotiff and _HAS_RASTERIO:
            tif_path = os.path.join(self.output_dir, prefix + ".tif")
            _save_geotiff(tif_path, slope_deg,
                          resolution=msg.info.resolution,
                          origin_xy=(msg.info.origin.position.x,
                                     msg.info.origin.position.y))
        if self.save_png:
            png_path = os.path.join(self.output_dir, prefix + ".png")
            _save_png_gray(png_path, np.nan_to_num(slope_deg, nan=0.0),
                           self.clip_deg)
        if self.save_color:
            color_path = os.path.join(self.output_dir, prefix + "_color.png")
            _save_png_color(color_path, np.nan_to_num(slope_deg, nan=0.0),
                            self.clip_deg)


def main():
    rospy.init_node("save_slope_raster", anonymous=False)
    SlopeRasterSaver()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
