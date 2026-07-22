/*
Copyright 2023 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

// ros
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

// Pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "velodyne_pointcloud/point_types.h"

// Grid map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/GridMapCvConverter.hpp>

// Config
#include <groundgrid/GroundGridConfig.h>
#include <array>
#include <mutex>


namespace groundgrid {
class GroundSegmentation {
  public:
    typedef velodyne_pointcloud::PointXYZIR PCLPoint;

    GroundSegmentation() {};
    void init(ros::NodeHandle& nodeHandle, const size_t dimension, const float& resolution);
    pcl::PointCloud<PCLPoint>::Ptr filter_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const PCLPoint& cloudOrigin, const geometry_msgs::TransformStamped& mapToBase, const ros::Time& stamp, grid_map::GridMap &map);
    void insert_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const size_t start, const size_t end, const PCLPoint& cloudOrigin, std::vector<std::pair<size_t, grid_map::Index> >& point_index, std::vector<std::pair<size_t, grid_map::Index> >& ignored, std::vector<size_t>& outliers, grid_map::GridMap &map);
    void setConfig(const groundgrid::GroundGridConfig& config);
    // section defines the section  of the map to process (0: top-left, 1: top-right, 2: bottom-left, 3: bottom-right)
    // used for parallel execution
    void detect_ground_patches(grid_map::GridMap &map, unsigned short section) const;
    template<int S> void detect_ground_patch(grid_map::GridMap &map, size_t i, size_t j) const;
    void spiral_ground_interpolation(grid_map::GridMap &map, const geometry_msgs::TransformStamped &toBase) const;
    void interpolate_cell(grid_map::GridMap &map, const size_t x, const size_t y) const;

    // ------------------------------------------------------------------
    // Slope-aware extensions (TS-SatMVSNet inspired, arXiv:2501.01049)
    // ------------------------------------------------------------------
    // Apply the 3x3 learnable Gaussian smoothing operator from Eq. 7 of the
    // paper (here as a fixed-weight kernel) onto the "ground" layer and write
    // the smoothed result into the "ground_corrected" layer.
    void compute_height_correction(grid_map::GridMap &map) const;

    // Compute the slope raster from the (optionally corrected) ground height
    // layer using the height-based slope calculation strategy of Eq. 1 of the
    // paper. Populates:
    //   - "slope"            : slope magnitude in degrees [0, 90]
    //   - "slope_max_diff"   : paper-style |max(p3x3) - H(x)| (meters)
    //   - "slope_x"          : dz/dx central difference (m/m)
    //   - "slope_y"          : dz/dy central difference (m/m)
    //   - "slope_direction"  : 0..8 directional code per Algorithm 1
    //   - "traversability"   : 1.0 where slope <= threshold, 0.0 otherwise
    void compute_slope_map(grid_map::GridMap &map) const;

protected:
    groundgrid::GroundGridConfig mConfig;
    grid_map::Matrix expectedPoints;
    mutable std::array<std::mutex, 4096> mCellMutexes;
    ros::Time mLastStamp;

    // velodyne 128: Average distance in rad on the unit circle of the appr. 220k points per round/128 Lasers
    const float verticalPointAngDist = 0.00174532925*2; // 0.2 degrees HDL-64e //0.00174532925; // 0.1 degrees //(2*M_PI)/(220000.0/128.0); // ca. 0.00365567:
    const float minDistSquared = 12.0f;
};
}
