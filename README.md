# Source code for the article "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation"
This repository contains the source code for the article "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation" published in the IEEE Robotics and Automation Letters ([DOI: 10.1109/LRA.2023.3333233](https://doi.org/10.1109/lra.2023.3333233)).
<p align="center">
  <img src="/res/img/teaser.gif" alt="Ground segmentation results"/>
</p>

# ROS2 Port
An extended ROS2 Jazzy port is available under the [ros2-jazzy](https://github.com/dcmlr/groundgrid/tree/ros2-jazzy) branch.

# Dependencies
- ROS Noetic Ninjemys
- catkin
- roscpp
- geometry_msgs
- sensor_msgs
- std_msgs
- message_generation
- message_runtime
- velodyne_pointcloud
- nodelet
- dynamic_reconfigure
- grid_map_core
- grid_map_ros
- grid_map_cv
- grid_map_loader
- grid_map_msgs
- grid_map_rviz_plugin
- grid_map_visualization
- cv_bridge
- pcl_ros

# Build
```
catkin build -DCMAKE_BUILD_TYPE=Release groundgrid
```

# Launch
## Playback
```
roslaunch groundgrid KITTIPlayback.launch directory:=/path/to/the/SemanticKITTI/dataset sequence:=0
```

The launch file opens a RViz window which displays the segmentation results:
<p align="center">
  <img src="/res/img/rviz.png" alt="SemanticKitti playback Rviz window"/>
</p>


## Ground Segmentation Evaluation
```
roslaunch groundgrid KITTIEvaluate.launch directory:=/path/to/the/SemanticKITTI/dataset sequence:=0
```

This launch file evaluates the ground segmentation performance of GroundGrid and displays the results every 500 processed clouds.
The final results are displayed upon receiving Ctrl+C in the terminal:
```
Stats
Received 4540 point clouds. KITTI sequence 00.
label			nonground %	ground %	nonground	total
unlabeled		88.74%		11.26%		6512861		7339364
outlier			42.51%		57.49%		121616		286056
car			94.42%		5.58%		43078880	45622648
bicycle			89.85%		10.15%		200919		223610
motorcycle		95.39%		4.61%		500522		524684
truck			97.61%		2.39%		416124		426308
other-vehicle		96.34%		3.66%		1199946		1245564
person			95.95%		4.05%		68227		71104
bicyclist		100.00%		0.00%		5		5
motorcyclist		0.00%		100.00%		0		8
road			0.07%		99.93%		68465		95649669
parking			0.45%		99.55%		37828		8450594
sidewalk		0.91%		99.09%		716154		78601664
other-ground		6.43%		93.57%		192		2985
building		97.33%		2.67%		117586234	120810401
fence			88.91%		11.09%		15821127	17793867
other-structure		89.92%		10.08%		713791		793778
lane-marking		0.16%		99.84%		171		109456
vegetation		93.43%		6.57%		121595505	130139604
trunk			97.88%		2.12%		4495649		4592878
terrain			6.68%		93.32%		1939107		29038187
pole			98.14%		1.86%		1819715		1854290
traffic-sign		99.87%		0.13%		248254		248565
other-object		89.59%		10.41%		4577201		5109077
moving-car		96.48%		3.52%		245225		254183
moving-bicyclist	98.23%		1.77%		155263		158054
moving-person		96.51%		3.49%		85024		88099
moving-motorcyclist	94.68%		5.32%		2011		2124
moving-bus		98.26%		1.74%		902		918
moving-other-vehicle	94.24%		5.76%		34090		36175
Precision		96.05%		209090638	8607231
Recall			98.70%		209090638	2761917
F1			97.35%		8607231		2761917
Accuracy		97.24%		400339747	411708895
IoUg			94.84%
```

# Citation
```
@article{steinke2024groundgrid,
  author={Steinke, Nicolai and Goehring, Daniel and Rojas, Raúl},
  journal={IEEE Robotics and Automation Letters},
  title={GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation},
  year={2024},
  volume={9},
  number={1},
  pages={420-426},
  keywords={Sensors;Point cloud compression;Estimation;Laser radar;Image segmentation;Task analysis;Robot sensing systems;Range Sensing;Mapping;Field Robots},
  doi={10.1109/LRA.2023.3333233}}
```

# Slope-aware extension

This fork extends GroundGrid with a slope raster output, inspired by the
height-based slope calculation strategy of *TS-SatMVSNet: Slope Aware Height
Estimation for Large-Scale Earth Terrain Multi-view Stereo*
([arXiv:2501.01049](https://arxiv.org/abs/2501.01049), Zhang et al., 2025).

What was added (paper-to-pipeline mapping):
- **Eq. 1 (height-based slope calculation)**: for every grid cell the 3x3
  neighbourhood is taken from the estimated `ground` layer; the paper's
  `S(x) = |max(p3x3) - H(x)|` is exposed as the new `slope_max_diff` layer and
  the 0..8 directional code (Fig. 4 / Algorithm 1 of the paper) is exposed as
  the new `slope_direction` layer. A conventional gradient-based slope angle in
  degrees is computed via central differences and exposed as `slope`.
- **Eq. 7 (height correction module)**: the 3x3 Gaussian-shaped smoothing
  operator `[[1/16,1/8,1/16],[1/8,1/4,1/8],[1/16,1/8,1/16]]` is applied to the
  ground layer (blended by `groundpatch` confidence so high-confidence cells
  keep their raw height). Result is exposed as the `ground_corrected` layer
  and is what the slope is computed from when
  `height_correction_enable=true`.
- **Slope-guided interval partition (Eq. 3-6)**: adapted from MVS hypothesis
  planes to the ground-point tolerance test in `GroundSegmentation::filter_cloud`.
  When `slope_aware_tolerance_enable=true` the tolerance is inflated by
  `slope_aware_tolerance_factor * tan(slope) * cell_resolution`, so steep
  cells admit more vertical variation before a point is flagged non-ground.
- A binary `traversability` layer is also produced, where 1.0 means
  `slope <= slope_max_traversable_deg` and 0.0 means above the threshold.

New ROS topics (published by `groundgrid/Nodelet`):
| topic | type | description |
| --- | --- | --- |
| `/groundgrid/slope_color` | sensor_msgs/Image (bgr8) | JET-colored slope image (0..2 x max-traversable-deg) |
| `/groundgrid/slope_raster` | sensor_msgs/Image (32FC1) | Raw slope in degrees, lossless |
| `/groundgrid/slope_grid` | nav_msgs/OccupancyGrid | Slope as 0..100 (georeferenced) |
| `/groundgrid/traversability_grid` | nav_msgs/OccupancyGrid | Binary traversability (0 free / 100 blocked) |
| `/groundgrid/grid_map` | grid_map_msgs/GridMap | Now includes `ground_corrected`, `slope`, `slope_max_diff`, `slope_x`, `slope_y`, `slope_direction`, `traversability` layers |

New dynamic-reconfigure parameters (see `cfg/GroundGrid.cfg`):
`slope_enable`, `height_correction_enable`, `height_correction_iterations`,
`height_correction_confidence_blend`, `slope_aware_tolerance_enable`,
`slope_aware_tolerance_factor`, `slope_max_traversable_deg`,
`slope_save_raster`.

## Saving the slope raster to disk

There are two helpers in `scripts/`:

1. **`save_slope_raster.py`** - online ROS node that subscribes to
   `/groundgrid/slope_raster` and `/groundgrid/slope_grid` and writes every
   incoming frame as `slope_<stamp>.tif` (GeoTIFF if `rasterio` is installed),
   `slope_<stamp>.png` (grayscale) and `slope_<stamp>_color.png` (JET).
   ```
   roslaunch groundgrid save_slope_raster.launch \
       output_dir:=/tmp/groundgrid_slope \
       clip_deg:=45.0 \
       grid_max_deg:=50.0
   ```
   or together with the KITTI playback:
   ```
   roslaunch groundgrid KITTIPlayback.launch \
       directory:=/path/to/SemanticKITTI sequence:=0 \
       save_slope_raster:=true slope_output_dir:=/tmp/groundgrid_slope \
       slope_grid_max_deg:=50.0
   ```

2. **`compute_slope_offline.py`** - ROS-free utility that runs the exact same
   algorithm on a saved height raster (`.npy`, `.tif` or `.png`):
   ```
   python scripts/compute_slope_offline.py input_height.npy out_dir \
          --resolution 0.33 --clip_deg 45 --height_correction --iterations 1
   ```
   Useful for debugging or for running the slope step on archived DEMs.

Both scripts gracefully degrade when `rasterio` or `cv2` is missing (they fall
back to `.npy` / grayscale PNG).

# Lunar terrain planning prototype

完整的中文部署、启动、验证和外部仿真器接入流程见
[`LUNAR_DEPLOYMENT_GUIDE.md`](LUNAR_DEPLOYMENT_GUIDE.md)。

This fork also contains a ROS 1 local perception-and-planning pipeline for a
medium differential-drive rover. GroundGrid now preserves measured-vs-
interpolated terrain, publishes observation age, raw/filtered elevation,
roughness, step height and temporally filtered obstacle confidence. Unknown or
stale terrain is deliberately not marked traversable.

The downstream pipeline consists of:

- `lunar_traversability_node`: converts the enriched GridMap into continuous
  terrain cost plus explicit unknown/lethal cells.
- `state_lattice_planner_node`: searches `(x,y,yaw)` using forward, reverse,
  curved and in-place motion primitives. Every primitive is checked across the
  full rover footprint, including direction-dependent longitudinal/lateral
  slope. It publishes an atomic `groundgrid/LunarTrajectory` containing one
  desired effective body twist per path pose; the legacy path and velocity
  topics remain available for RViz and existing tools.
- `lunar_path_follower_node`: a fail-safe differential-drive path follower that
  blends planner yaw-rate feed-forward with geometric feedback, applies skid
  compensation once, and stops on invalid/stale trajectories, stale terrain or
  missing TF. Its latched state is available on `/lunar_path_follower/status`.
- `lunar_surface_sim.py`: a deterministic lightweight crater/rock terrain
  simulator for end-to-end testing without an external simulator.

Run the built-in demo and send a `2D Nav Goal` from RViz:

```bash
roslaunch groundgrid LunarDemo.launch
```

For an external simulator, launch only the pipeline and remap its standard
point cloud, odometry and velocity topics:

```bash
roslaunch groundgrid LunarSystem.launch \
  point_cloud_topic:=/lidar/points \
  odometry_topic:=/odometry/filtered \
  cmd_vel_topic:=/rover/cmd_vel
```

The lunar defaults are in `config/lunar_system.yaml`. Legacy GroundGrid defaults
remain 120 m and 0.33 m; the lunar launch overrides them to 60 m and 0.15 m.
