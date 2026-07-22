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

#include <groundgrid/GroundGrid.h>

#include <chrono>
#include <limits>

// Ros package for package path resolution
#include <ros/package.h>

// Grid map
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_core/GridMapMath.hpp>

// Tf
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace groundgrid;

GroundGrid::GroundGrid() : mTf2_listener(mTfBuffer)
{}

GroundGrid::~GroundGrid() {}

void GroundGrid::setConfig(groundgrid::GroundGridConfig & config) { config_ = config; }

void GroundGrid::configureGeometry(double dimension, double resolution,
                                   const std::string& mapFrame,
                                   const std::string& baseFrame)
{
    mDimension = static_cast<float>(dimension);
    mResolution = static_cast<float>(resolution);
    mDetectionRadius = dimension * 0.5;
    mMapFrame = mapFrame;
    mBaseFrame = baseFrame;
}

void GroundGrid::initGroundGrid(const nav_msgs::OdometryConstPtr &inOdom)
{
    auto start = std::chrono::steady_clock::now();
    geometry_msgs::PoseWithCovarianceStamped odomPose, mapPose;

    mMap_ptr = std::make_shared<grid_map::GridMap, const std::vector< std::string >>({
        "points", "ground", "groundpatch", "minGroundHeight", "maxGroundHeight",
        // Slope-aware extensions (TS-SatMVSNet inspired, arXiv:2501.01049)
        "ground_corrected", "slope", "slope_max_diff", "slope_x", "slope_y",
        "slope_direction", "traversability", "observed", "observation_age",
        "elevation_raw", "elevation_filtered", "elevation_residual",
        "roughness", "step_height", "obstacle_height", "obstacle_confidence"
    });
    grid_map::GridMap& map = *mMap_ptr;
    map.setFrameId(mMapFrame);
    map.setGeometry(grid_map::Length(mDimension, mDimension), mResolution, grid_map::Position(inOdom->pose.pose.position.x,inOdom->pose.pose.position.y));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map.getLength().x(), map.getLength().y(),
             map.getSize()(0), map.getSize()(1));


    odomPose.pose = inOdom->pose;
    odomPose.header = inOdom->header;
    std::vector<grid_map::BufferRegion> damage;
    map.move(grid_map::Position(odomPose.pose.pose.position.x, odomPose.pose.pose.position.y), damage);
    grid_map::BufferRegion region(grid_map::Index(0,0), map.getSize(), grid_map::BufferRegion::Quadrant(0));


    map["points"].setZero();
    map["ground"].setConstant(inOdom->pose.pose.position.z);
    map["groundpatch"].setConstant(0.0000001);
    map["minGroundHeight"].setConstant(100.0);
    map["maxGroundHeight"].setConstant(-100.0);

    // Slope-aware layers init
    map["ground_corrected"].setConstant(inOdom->pose.pose.position.z);
    map["slope"].setZero();
    map["slope_max_diff"].setZero();
    map["slope_x"].setZero();
    map["slope_y"].setZero();
    // 4 = "vertical" code (center is max) per paper Fig. 4
    map["slope_direction"].setConstant(4.0f);
    map["traversability"].setConstant(1.0f);
    map["observed"].setZero();
    map["observation_age"].setConstant(std::numeric_limits<float>::infinity());
    map["elevation_raw"].setConstant(std::numeric_limits<float>::quiet_NaN());
    map["elevation_filtered"].setConstant(inOdom->pose.pose.position.z);
    map["elevation_residual"].setConstant(std::numeric_limits<float>::quiet_NaN());
    map["roughness"].setConstant(std::numeric_limits<float>::quiet_NaN());
    map["step_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
    map["obstacle_height"].setZero();
    map["obstacle_confidence"].setZero();

    auto end = std::chrono::steady_clock::now();
    ROS_DEBUG_STREAM("transforms lookup " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");
    mLastPose = odomPose;
}


std::shared_ptr<grid_map::GridMap> GroundGrid::update(const nav_msgs::OdometryConstPtr &inOdom)
{
    if(!mMap_ptr){
        initGroundGrid(inOdom);
        return mMap_ptr;
    }

    auto start = std::chrono::steady_clock::now();
    grid_map::GridMap& map = *mMap_ptr;

    geometry_msgs::PoseWithCovarianceStamped poseDiff;
    poseDiff.pose.pose.position.x = inOdom->pose.pose.position.x - mLastPose.pose.pose.position.x;
    poseDiff.pose.pose.position.y = inOdom->pose.pose.position.y - mLastPose.pose.pose.position.y;
    std::vector<grid_map::BufferRegion> damage;
    map.move(grid_map::Position(inOdom->pose.pose.position.x, inOdom->pose.pose.position.y), damage);

    // static so if the new transform is not yet available, we can use the last one
    static geometry_msgs::TransformStamped base_to_map;

    try{
        base_to_map = mTfBuffer.lookupTransform(mBaseFrame, mMapFrame, inOdom->header.stamp);
    }
    catch (tf2::LookupException& e)
    {
        // potentially degraded performance
        ROS_WARN("no transform? -> error: %s", e.what());
    }
    catch (tf2::ExtrapolationException& e)
    {
        // can happen when new transform has not yet been published, we can use the old one instead
        ROS_DEBUG("need to extrapolate a transform? -> error: %s", e.what());
    }

    geometry_msgs::PointStamped ps;
    ps.header = inOdom->header;
    ps.header.frame_id = mMapFrame;
    grid_map::Position pos;

    for(auto region : damage){
        for(auto it = grid_map::SubmapIterator(map, region); !it.isPastEnd(); ++it){
            auto idx = *it;

	    map.getPosition(idx, pos);
	    ps.point.x = pos(0);
	    ps.point.y = pos(1);
	    ps.point.z = 0;
            tf2::doTransform(ps, ps, base_to_map);
            map.at("ground", idx) = -ps.point.z;
            map.at("groundpatch", idx) = 0.0;
            // Reset slope-aware layers in newly-entered regions
            map.at("ground_corrected", idx) = -ps.point.z;
            map.at("slope", idx) = 0.0;
            map.at("slope_max_diff", idx) = 0.0;
            map.at("slope_x", idx) = 0.0;
            map.at("slope_y", idx) = 0.0;
            map.at("slope_direction", idx) = 4.0f;
            map.at("traversability", idx) = 1.0f;
            map.at("observed", idx) = 0.0f;
            map.at("observation_age", idx) = std::numeric_limits<float>::infinity();
            map.at("elevation_raw", idx) = std::numeric_limits<float>::quiet_NaN();
            map.at("elevation_filtered", idx) = -ps.point.z;
            map.at("elevation_residual", idx) = std::numeric_limits<float>::quiet_NaN();
            map.at("roughness", idx) = std::numeric_limits<float>::quiet_NaN();
            map.at("step_height", idx) = std::numeric_limits<float>::quiet_NaN();
            map.at("obstacle_height", idx) = 0.0f;
            map.at("obstacle_confidence", idx) = 0.0f;
        }
    }

    // We havent moved so we have nothing to do
    if(damage.empty())
        return mMap_ptr;


    mLastPose.pose = inOdom->pose;
    mLastPose.header = inOdom->header;

    map.convertToDefaultStartIndex();
    auto end = std::chrono::steady_clock::now();
    ROS_DEBUG_STREAM("total " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");
    return mMap_ptr;
}
