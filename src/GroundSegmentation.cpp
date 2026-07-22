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

#include <groundgrid/GroundSegmentation.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <unordered_map>

using namespace groundgrid;


void GroundSegmentation::init(ros::NodeHandle& nodeHandle, const size_t dimension, const float& resolution){
    const size_t cellCount = std::round(dimension/resolution);

    expectedPoints.resize(cellCount, cellCount);
    for(size_t i=0; i<cellCount; ++i){
        for(size_t j=0; j<cellCount; ++j){
            const float& dist = std::hypot(i-cellCount/2.0,j-cellCount/2.0);
            expectedPoints(i,j) = std::atan(1/dist)/verticalPointAngDist;
        }
    }
    Eigen::initParallel();
}

pcl::PointCloud<GroundSegmentation::PCLPoint>::Ptr GroundSegmentation::filter_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const PCLPoint& cloudOrigin, const geometry_msgs::TransformStamped& mapToBase, const ros::Time& stamp, grid_map::GridMap &map)
{
    auto start = std::chrono::steady_clock::now();
    static double avg_insertion_time = 0.0;
    static double avg_detection_time = 0.0;
    static double avg_segmentation_time = 0.0;
    static unsigned int time_vals = 0;

    pcl::PointCloud<PCLPoint>::Ptr filtered_cloud (new pcl::PointCloud<PCLPoint>);
    filtered_cloud->points.reserve(cloud->points.size());

    map.add("groundCandidates", 0.0);
    map.add("planeDist", 0.0);
    map.add("m2", 0.0);
    map.add("meanVariance", 0.0);

    // raw point count layer for the evaluation
    map.add("pointsRaw", 0.0);


    map["groundCandidates"].setZero();
    map["points"].setZero();
    map["pointsRaw"].setZero();
    map["planeDist"].setZero();
    map["m2"].setZero();
    map["meanVariance"].setZero();
    map["minGroundHeight"].setConstant(std::numeric_limits<float>::max());
    map["maxGroundHeight"].setConstant(std::numeric_limits<float>::lowest());

    map.add("variance", 0.0);
    static const grid_map::Matrix& ggv = map["variance"];
    static grid_map::Matrix& gpl = map["points"];
    static grid_map::Matrix& ggl = map["ground"];
    const auto& size = map.getSize();
    const size_t threadcount = mConfig.thread_count;


    std::vector<std::pair<size_t, grid_map::Index> > point_index;
    point_index.reserve(cloud->points.size());
    std::vector<std::vector<std::pair<size_t, grid_map::Index> > > point_index_list;
    point_index_list.resize(threadcount);

    // Collect all outliers for the outlier detection evaluation
    std::vector<size_t> outliers;
    std::vector<std::vector<size_t> > outliers_list;
    outliers_list.resize(threadcount);

    // store ignored points to re-add them afterwards
    std::vector<std::pair<size_t, grid_map::Index> > ignored;
    std::vector<std::vector<std::pair<size_t, grid_map::Index> > > ignored_list;
    ignored_list.resize(threadcount);

    // Divide the point cloud into threadcount sections for threaded calculations
    std::vector<std::thread> threads;

    for(size_t i=0; i<threadcount; ++i){
        const size_t start = std::floor((i*cloud->points.size())/threadcount);
        const size_t end = std::ceil(((i+1)*cloud->points.size())/threadcount);
        threads.push_back(std::thread(&GroundSegmentation::insert_cloud, this, cloud, start, end, std::cref(cloudOrigin), std::ref(point_index_list[i]), std::ref(ignored_list[i]),
                                      std::ref(outliers_list[i]), std::ref(map)));
    }

    // wait for results
    std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));

    // join results
    for(const auto& point_index_part : point_index_list)
        point_index.insert(point_index.end(), point_index_part.begin(), point_index_part.end());
    for(const auto& outlier_index_part : outliers_list)
        outliers.insert(outliers.end(), outlier_index_part.begin(), outlier_index_part.end());
    for(const auto& ignored_part : ignored_list)
        ignored.insert(ignored.end(), ignored_part.begin(), ignored_part.end());

    // Preserve the distinction between measured and interpolated terrain.
    const double dt = mLastStamp.isZero() ? 0.0 : std::max(0.0, (stamp - mLastStamp).toSec());
    mLastStamp = stamp;
    for(grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it){
        const grid_map::Index idx(*it);
        const bool interior = idx(0) > 0 && idx(1) > 0 &&
                              idx(0) + 1 < map.getSize()(0) && idx(1) + 1 < map.getSize()(1);
        const bool covered = map.at("pointsRaw", idx) > 0.0f ||
            (interior && map["pointsRaw"].block<3,3>(idx(0)-1, idx(1)-1).maxCoeff() > 0.0f);
        if(covered){
            map.at("observed", idx) = 1.0f;
            map.at("observation_age", idx) = 0.0f;
            if(map.at("pointsRaw", idx) > 0.0f)
                map.at("elevation_raw", idx) = map.at("minGroundHeight", idx);
        } else if(map.at("observed", idx) > 0.5f && std::isfinite(map.at("observation_age", idx))){
            map.at("observation_age", idx) += static_cast<float>(dt);
        }
    }


    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    const double milliseconds = elapsed_seconds.count() * 1000;
    avg_insertion_time = (milliseconds + time_vals * avg_insertion_time)/(time_vals+1);
    ROS_DEBUG_STREAM("ground point rasterization took " << milliseconds << "ms (avg " << avg_insertion_time << " ms)");

    start = std::chrono::steady_clock::now();

    // Calculate shared variance once. Patch updates are kept sequential because
    // neighbouring patches read each other's persistent confidence and height.
    map["variance"] = map["m2"].array().cwiseQuotient(
        map["points"].array() + std::numeric_limits<float>::min());
    for(unsigned short section=0; section<4; ++section)
        detect_ground_patches(map, section);
    end = std::chrono::steady_clock::now();
    elapsed_seconds = end-start;
    avg_detection_time = (elapsed_seconds.count() * 1000 + time_vals * avg_detection_time)/(time_vals+1);
    ROS_DEBUG_STREAM("ground patch detection took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms (avg " << avg_detection_time << " ms)");
    ++time_vals;

    start = std::chrono::steady_clock::now();
    spiral_ground_interpolation(map, mapToBase);
    end = std::chrono::steady_clock::now();
    ROS_DEBUG_STREAM("ground interpolation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");

    // ----------------------------------------------------------------------
    // Slope-aware extensions (TS-SatMVSNet, arXiv:2501.01049)
    // ----------------------------------------------------------------------
    if(mConfig.slope_enable){
        start = std::chrono::steady_clock::now();
        if(mConfig.height_correction_enable && mConfig.height_correction_iterations > 0)
            compute_height_correction(map);
        else
            map["ground_corrected"] = map["ground"];
        compute_slope_map(map);
        end = std::chrono::steady_clock::now();
        ROS_DEBUG_STREAM("slope computation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");
    }

    start = std::chrono::steady_clock::now();
    map["points"].setConstant(0.0);

    // Re-add ignored points
    point_index.insert(point_index.end(), ignored.begin(), ignored.end());


    // Debugging statistics
    const double& min_dist_fac = mConfig.minimum_distance_factor*5;
    const double& min_point_height_thres = mConfig.miminum_point_height_threshold;
    const double& min_point_height_obs_thres = mConfig.minimum_point_height_obstacle_threshold;

    // Slope-aware tolerance pre-fetch
    const bool slope_aware = mConfig.slope_enable && mConfig.slope_aware_tolerance_enable;
    const double slope_tol_factor = mConfig.slope_aware_tolerance_factor;
    const double cell_resolution = map.getResolution();
    const grid_map::Matrix& slope_layer = map["slope"];

    for(const std::pair<size_t, grid_map::Index>& entry : point_index)
    {
        const PCLPoint& point = cloud->points[entry.first];
        const grid_map::Index& gi = entry.second;
        const double& groundheight = ggl(gi(0),gi(1));

        // copy the points intensity because it get's overwritten for evaluation purposes
        const float& variance = ggv(gi(0),gi(1));

        if(size(0) <= gi(0)+3 || size(1) <= gi(1)+3)
            continue;

        const float dist = std::hypot(point.x-cloudOrigin.x, point.y-cloudOrigin.y);
        double tolerance = std::max(std::min((min_dist_fac*dist)/variance * min_point_height_thres, min_point_height_thres), min_point_height_obs_thres);

        // Slope-aware tolerance: on steep terrain a single cell may span more
        // height than min_point_height_threshold suggests; inflate by the
        // height drop expected over one cell at the local slope.
        if(slope_aware){
            const float slope_deg = slope_layer(gi(0), gi(1));
            if(slope_deg > 0.0f && std::isfinite(slope_deg)){
                const double slope_rad = slope_deg * M_PI / 180.0;
                tolerance += slope_tol_factor * std::tan(slope_rad) * cell_resolution;
            }
        }

        if(tolerance+groundheight < point.z){ // non-ground points
            PCLPoint& segmented_point = filtered_cloud->points.emplace_back(point);
            segmented_point.intensity = 99;
            gpl(gi(0),gi(1)) += 1.0f;
        }
        else{
            PCLPoint& segmented_point = filtered_cloud->points.emplace_back(point); // ground point
            segmented_point.intensity = 49;
        }
    }

    // Re-add outliers to cloud
   for(size_t i : outliers){
        const PCLPoint& point = cloud->points[i];
        PCLPoint& segmented_point = filtered_cloud->points.emplace_back(point); //ground point
        segmented_point.intensity = 49;
    }

    end = std::chrono::steady_clock::now();
    elapsed_seconds = end-start;
    avg_segmentation_time = (elapsed_seconds.count() * 1000 + (time_vals-1) * avg_segmentation_time)/time_vals;
    ROS_DEBUG_STREAM("point cloud segmentation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms (avg " << avg_segmentation_time << " ms)");

    return filtered_cloud;
}


void GroundSegmentation::insert_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const size_t start, const size_t end, const PCLPoint& cloudOrigin, std::vector<std::pair<size_t, grid_map::Index> >& point_index,
                                      std::vector<std::pair<size_t, grid_map::Index> >& ignored, std::vector<size_t>& outliers, grid_map::GridMap &map)
{
    static const grid_map::Matrix& ggp = map["groundpatch"];

    static grid_map::Matrix& gpr = map["pointsRaw"];
    static grid_map::Matrix& gpl = map["points"];
    static grid_map::Matrix& ggl = map["ground"];
    static grid_map::Matrix& gmg = map["groundCandidates"];
    static grid_map::Matrix& gmm = map["meanVariance"];
    static grid_map::Matrix& gmx = map["maxGroundHeight"];
    static grid_map::Matrix& gmi = map["minGroundHeight"];
    static grid_map::Matrix& gmd = map["planeDist"];
    static grid_map::Matrix& gm2 = map["m2"];

    const auto& size = map.getSize();

    point_index.reserve(end-start);

    for(size_t i = start; i < end; ++i)
    {
        const PCLPoint& point = cloud->points[i];
        const auto& pos = grid_map::Position(point.x,point.y);
        const float sqdist = std::pow(point.x-cloudOrigin.x, 2.0) + std::pow(point.y-cloudOrigin.y, 2.0);

        bool toSkip=false;

        grid_map::Index gi;
        map.getIndex(pos, gi);

        if(!map.isInside(pos))
            continue;

        const size_t mutex_index = (static_cast<size_t>(gi(0)) * size(1) +
                                    static_cast<size_t>(gi(1))) % mCellMutexes.size();
        std::lock_guard<std::mutex> cell_lock(mCellMutexes[mutex_index]);

        // point count map used for evaluation
        gpr(gi(0), gi(1)) += 1.0f;


        if(point.ring > mConfig.max_ring || sqdist < minDistSquared){
            ignored.push_back(std::make_pair(i, gi));
            continue;
        }

        // Outlier detection test
        const float oldgroundheight = ggl(gi(0), gi(1));
        if(point.z < oldgroundheight-0.2){

            // get direction
            PCLPoint vec;
            vec.x = point.x - cloudOrigin.x;
            vec.y = point.y - cloudOrigin.y;
            vec.z = point.z - cloudOrigin.z;

            float len = std::sqrt(std::pow(vec.x, 2.0f) + std::pow(vec.y, 2.0f) + std::pow(vec.z, 2.0f));
            vec.x /= len;
            vec.y /= len;
            vec.z /= len;

            // check for occlusion
            for(int step=3; (std::pow(step*vec.x, 2.0) + std::pow(step*vec.y, 2.0) + std::pow(step*vec.z,2.0)) < std::pow(len,2.0) && vec.z < -0.01f; ++step){
                grid_map::Index intersection, pointPosIndex;
                grid_map::Position intersecPos(step*(vec.x)+cloudOrigin.x, step*(vec.y)+cloudOrigin.y);
                map.getIndex(intersecPos, intersection);

                // Check if inside map borders
                if(intersection(0) <= 0 || intersection(1) <= 0 || intersection(0) >= size(0)-1 || intersection(1) >= size(1)-1)
                    continue;

                // check if known ground occludes the line of sight
                const auto& block = ggp.block<3,3>(std::max(intersection(0)-1, 2), std::max(intersection(1)-1,2));
                if(block.sum() > mConfig.min_outlier_detection_ground_confidence && ggp(intersection(0),intersection(1)) > 0.01f && ggl(intersection(0),intersection(1)) >= step*vec.z+cloudOrigin.z+mConfig.outlier_tolerance){
                    outliers.push_back(i);
                    toSkip=true;
                    break;
                }
            }
        }


        if(toSkip)
            continue;


        float &groundheight = gmg(gi(0),gi(1));
        float &mean = gmm(gi(0), gi(1));


        float planeDist = 0.0;
        point_index.push_back(std::make_pair(i, gi));

        float &points = gpl(gi(0), gi(1));
        float &maxHeight = gmx(gi(0),gi(1));
        float &minHeight = gmi(gi(0),gi(1));
        float &planeDistMap = gmd(gi(0),gi(1));
        float &m2 = gm2(gi(0),gi(1));

        planeDist = point.z - cloudOrigin.z;
        groundheight = (point.z + points * groundheight)/(points+1.0);

        if(mean == 0.0)
            mean = planeDist;
        if(!std::isnan(planeDist)){
            float delta = planeDist - mean;
            mean += delta/(points+1);
            planeDistMap = (planeDist + points * planeDistMap)/(points+1.0);
            m2 += delta * (planeDist - mean);
        }

        maxHeight = std::max(maxHeight, point.z);
        minHeight =std::min(minHeight, point.z-0.0001f); // to make sure maxHeight > minHeight
        points += 1.0;
    }
}


void GroundSegmentation::detect_ground_patches(grid_map::GridMap &map, unsigned short section) const
{
    const grid_map::Matrix& gcl = map["groundCandidates"];
    const static auto& size = map.getSize();
    const static float resolution = map.getResolution();
    static const grid_map::Matrix& gm2 = map["m2"];
    static const grid_map::Matrix& gpl = map["points"];
    static grid_map::Matrix& ggv = map["variance"];
    int cols_start = 2 + section%2 * (gcl.cols()/2-2);
    int rows_start = section>=2 ? gcl.rows()/2 : 2;
    int cols_end = (gcl.cols())/2 + section%2 * (gcl.cols()/2-2);
    int rows_end = section>=2 ? gcl.rows()-2 : (gcl.rows())/2;

    for(int i=cols_start; i<cols_end; ++i){
        for(int j=rows_start; j<rows_end; ++j){
            const float sqdist = (std::pow(i-(size(0)/2.0),2.0) + std::pow(j-(size(1)/2.0), 2.0)) * std::pow(resolution,2.0);

            if(sqdist <= std::pow(mConfig.patch_size_change_distance, 2.0))
                detect_ground_patch<3>(map, i, j);
            else
                detect_ground_patch<5>(map, i, j);
        }
    }
}


template <int S> void GroundSegmentation::detect_ground_patch(grid_map::GridMap& map, size_t i, size_t j) const
{
    static grid_map::Matrix& ggl = map["ground"];
    static grid_map::Matrix& ggp = map["groundpatch"];
    static grid_map::Matrix& ggv = map["variance"];
    static const grid_map::Matrix& gmi = map["minGroundHeight"];
    static const grid_map::Matrix& gpl = map["points"];
    static const auto& size = map.getSize();
    static const float resolution = map.getResolution();
    const int center_idx = std::floor(S/2);


    const auto& pointsBlock = gpl.block<S,S>(i-center_idx,j-center_idx);
    const float sqdist = (std::pow(i-(size(0)/2.0),2.0) + std::pow(j-(size(1)/2.0), 2.0)) * std::pow(resolution,2.0);
    const int patchSize = S;
    const float& expectedPointCountperLaserperCell = expectedPoints(i,j);
    const float& pointsblockSum = pointsBlock.sum();
    float& oldConfidence = ggp(i,j);
    float& oldGroundheight = ggl(i,j);

    // early skipping of (almost) empty areas
    if(pointsblockSum < std::max(std::floor(mConfig.ground_patch_detection_minimum_point_count_threshold * patchSize * expectedPointCountperLaserperCell), 3.0))
        return;

    // calculation of variance threshold
    // limit the value to the defined minimum and 10 times the defined minimum
    const float varThresholdsq = std::min(std::max(sqdist * std::pow(mConfig.distance_factor,2.0), std::pow(mConfig.minimum_distance_factor,2.0)), std::pow(mConfig.minimum_distance_factor*10, 2.0));
    const auto& varblock = ggv.block<S,S>(i-center_idx,j-center_idx);
    const auto& minblock = gmi.block<S,S>(i-center_idx, j-center_idx);
    const float& variance = varblock(center_idx,center_idx);
    const float& localmin = minblock.minCoeff();
    const float maxVar = pointsBlock(center_idx,center_idx) >= mConfig.point_count_cell_variance_threshold ? variance : pointsBlock.array().cwiseProduct(varblock.array()).sum()/pointsblockSum;
    const float groundlevel = pointsBlock.cwiseProduct(minblock).sum()/pointsblockSum;
    const float groundDiff = std::max((groundlevel - oldGroundheight) * (2.0f*oldConfidence), 1.0f);

    // Do not update known high confidence estimations upward
    if(oldConfidence > 0.5 && groundlevel >= oldGroundheight + mConfig.outlier_tolerance)
        return;

    if(varThresholdsq > std::pow(maxVar, 2.0) && maxVar > 0 && pointsblockSum > (groundDiff * expectedPointCountperLaserperCell * patchSize) * mConfig.ground_patch_detection_minimum_point_count_threshold){
            const float& newConfidence = std::min(pointsblockSum/mConfig.occupied_cells_point_count_factor, 1.0);
            // calculate ground height
            oldGroundheight = (groundlevel*newConfidence + oldConfidence * oldGroundheight*2)/(newConfidence+oldConfidence*2);
            // update confidence
            oldConfidence = std::min((pointsblockSum/(mConfig.occupied_cells_point_count_factor*2.0f) + oldConfidence)/2.0, 1.0);
    }
    else if(localmin < oldGroundheight){
        // update ground height
        oldGroundheight = localmin;
        // update confidence
        oldConfidence = std::min(oldConfidence + 0.1f, 0.5f);
    }
}


void GroundSegmentation::spiral_ground_interpolation(grid_map::GridMap &map, const geometry_msgs::TransformStamped &toBase) const
{
    static grid_map::Matrix& ggl = map["ground"];
    static grid_map::Matrix& gvl = map["groundpatch"];
    const auto& map_size = map.getSize();
    const auto& center_idx = map_size(0)/2-1;

    gvl(center_idx,center_idx) = 1.0f;
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "base_link";
    tf2::doTransform(ps,ps,toBase);

    // Set center to current vehicle height
    ggl(center_idx,center_idx) = ps.point.z;

    for(int i=center_idx-1; i>=1; --i){
        // rectangle_pos = x,y position of rectangle top left corner
        int rectangle_pos = i;

        // rectangle side length
        int side_length = (center_idx-rectangle_pos)*2;

        // top and left side
        for(short side=0; side<2; ++side){
            for(int pos=rectangle_pos; pos<rectangle_pos+side_length; ++pos){
                const int x = side%2 ? pos : rectangle_pos;
                const int y = side%2 ? rectangle_pos : pos;

                interpolate_cell(map, x, y);
            }
        }

        // bottom and right side
        rectangle_pos += side_length;
        for(short side=0; side<2; ++side){
            for(int pos=rectangle_pos; pos>=rectangle_pos-side_length; --pos){
                int x = side%2 ? pos : rectangle_pos;
                int y = side%2 ? rectangle_pos : pos;

                interpolate_cell(map, x, y);
            }
        }
    }
}



void GroundSegmentation::interpolate_cell(grid_map::GridMap &map, const size_t x, const size_t y) const
{
    static const auto& center_idx = map.getSize()(0)/2-1;
    static const size_t blocksize = 3;
    // "groundpatch" layer contains confidence values
    static grid_map::Matrix& gvl = map["groundpatch"];
    // "ground" contains the ground height values
    static grid_map::Matrix& ggl = map["ground"];
    const auto& gvlblock = gvl.block<blocksize,blocksize>(x-blocksize/2,y-blocksize/2);

    float& height = ggl(x,y);
    float& occupied = gvl(x,y);
    const float& gvlSum = gvlblock.sum() + std::numeric_limits<float>::min(); // avoid a possible div by 0
    const float avg = (gvlblock.cwiseProduct(ggl.block<blocksize,blocksize>(x-blocksize/2, y-blocksize/2))).sum()/gvlSum;

    height = (1.0f-occupied) * avg + occupied * height;

    // Only update confidence in cells above min distance
    if((std::pow((float)x-center_idx, 2.0) + std::pow((float)y-center_idx, 2.0)) * std::pow(map.getResolution(), 2.0f) > minDistSquared)
        occupied = std::max(occupied-occupied/mConfig.occupied_cells_decrease_factor, 0.001);
}


void GroundSegmentation::setConfig(const groundgrid::GroundGridConfig &config)
{
    mConfig = config;
}


// ============================================================================
// Slope-aware extensions (TS-SatMVSNet inspired, arXiv:2501.01049)
// ============================================================================

void GroundSegmentation::compute_height_correction(grid_map::GridMap &map) const
{
    // Paper Eq. 7: 3x3 Gaussian-shaped height correction kernel.
    //   K = [[1/16, 1/8, 1/16],
    //        [1/8 , 1/4, 1/8 ],
    //        [1/16, 1/8, 1/16]]
    // The original work uses a *learnable* parameterised version. As a model-
    // free pipeline we use the fixed coefficients above; we still blend by
    // ground-patch confidence so that high-confidence cells (lots of points)
    // are preserved while only sparsely-observed / interpolated cells get
    // smoothed. This avoids erasing real terrain detail in confident regions.
    static constexpr float K[3][3] = {
        {1.0f/16.f, 1.0f/8.f, 1.0f/16.f},
        {1.0f/8.f , 1.0f/4.f, 1.0f/8.f },
        {1.0f/16.f, 1.0f/8.f, 1.0f/16.f}
    };

    grid_map::Matrix& ground     = map["ground"];
    grid_map::Matrix& corrected  = map["ground_corrected"];
    const grid_map::Matrix& conf = map["groundpatch"];
    const auto& size = map.getSize();
    const float conf_exp = static_cast<float>(mConfig.height_correction_confidence_blend);
    const int iters = std::max(0, mConfig.height_correction_iterations);

    corrected = ground;

    grid_map::Matrix work = corrected;
    for(int it = 0; it < iters; ++it){
        work = corrected;
        for(int i = 1; i < size(0)-1; ++i){
            for(int j = 1; j < size(1)-1; ++j){
                float smoothed = 0.0f;
                for(int di = -1; di <= 1; ++di)
                    for(int dj = -1; dj <= 1; ++dj)
                        smoothed += K[di+1][dj+1] * work(i+di, j+dj);

                // High-confidence cells keep more of the raw height.
                float c = std::min(std::max(conf(i, j), 0.0f), 1.0f);
                if(conf_exp > 0.0f) c = std::pow(c, conf_exp);
                corrected(i, j) = c * work(i, j) + (1.0f - c) * smoothed;
            }
        }
    }
}


void GroundSegmentation::compute_slope_map(grid_map::GridMap &map) const
{
    // Use the corrected ground layer if available, otherwise raw ground.
    const std::string src_layer = mConfig.height_correction_enable ? "ground_corrected" : "ground";
    const grid_map::Matrix& ground = map[src_layer];

    grid_map::Matrix& slope_deg     = map["slope"];
    grid_map::Matrix& slope_maxdiff = map["slope_max_diff"];
    grid_map::Matrix& slope_x       = map["slope_x"];
    grid_map::Matrix& slope_y       = map["slope_y"];
    grid_map::Matrix& slope_dir     = map["slope_direction"];
    grid_map::Matrix& travers       = map["traversability"];
    grid_map::Matrix& filtered      = map["elevation_filtered"];
    grid_map::Matrix& residual      = map["elevation_residual"];
    grid_map::Matrix& roughness     = map["roughness"];
    grid_map::Matrix& step_height   = map["step_height"];
    grid_map::Matrix& obstacle_h    = map["obstacle_height"];
    grid_map::Matrix& obstacle_c    = map["obstacle_confidence"];
    const grid_map::Matrix& raw     = map["elevation_raw"];
    const grid_map::Matrix& observed = map["observed"];
    const grid_map::Matrix& max_h   = map["maxGroundHeight"];
    const grid_map::Matrix& raw_count = map["pointsRaw"];
    const auto& size = map.getSize();
    const float resolution = map.getResolution();
    const float slope_thresh_rad =
        static_cast<float>(mConfig.slope_max_traversable_deg) * static_cast<float>(M_PI) / 180.0f;
    const float slope_thresh_tan = std::tan(slope_thresh_rad);

    // Paper Fig. 4 / Alg. 1 directional codes; the 3x3 neighbourhood (di,dj)
    // around the center pixel maps as:
    //   (di=-1,dj=-1)=8 (di=-1,dj=0)=7 (di=-1,dj=+1)=6
    //   (di= 0,dj=-1)=5 (di= 0,dj=0)=4 (di= 0,dj=+1)=3
    //   (di=+1,dj=-1)=2 (di=+1,dj=0)=1 (di=+1,dj=+1)=0
    static constexpr float DIR_CODE[3][3] = {
        {8.0f, 7.0f, 6.0f},
        {5.0f, 4.0f, 3.0f},
        {2.0f, 1.0f, 0.0f}
    };

    slope_deg.setZero();
    slope_maxdiff.setZero();
    slope_x.setZero();
    slope_y.setZero();
    slope_dir.setConstant(4.0f);
    filtered = ground;
    residual.setConstant(std::numeric_limits<float>::quiet_NaN());
    roughness.setConstant(std::numeric_limits<float>::quiet_NaN());
    step_height.setConstant(std::numeric_limits<float>::quiet_NaN());
    travers.setConstant(std::numeric_limits<float>::quiet_NaN());

    const float inv_2res = 1.0f / (2.0f * resolution);

    for(int i = 1; i < size(0)-1; ++i){
        for(int j = 1; j < size(1)-1; ++j){
            const float center = ground(i, j);
            if(!std::isfinite(center))
                continue;

            if(std::isfinite(raw(i, j)))
                residual(i, j) = raw(i, j) - center;

            const auto block = ground.block<3,3>(i-1, j-1);
            const float local_mean = block.mean();
            roughness(i, j) = std::sqrt(
                (block.array() - local_mean).square().sum() / 9.0f);
            step_height(i, j) = block.maxCoeff() - block.minCoeff();

            if(raw_count(i, j) > 0.0f && std::isfinite(max_h(i, j))){
                obstacle_h(i, j) = std::max(0.0f, max_h(i, j) - center);
                const float hit = obstacle_h(i, j) > 0.10f ? 1.0f : 0.0f;
                obstacle_c(i, j) = 0.7f * obstacle_c(i, j) + 0.3f * hit;
            } else {
                obstacle_c(i, j) *= 0.9f;
            }

            // Paper Eq. 1: slope magnitude as |max(p3x3) - center|, plus the
            // 0..8 direction code indicating where the max neighbour sits.
            float max_val = center;
            int   max_di  = 0;
            int   max_dj  = 0;
            for(int di = -1; di <= 1; ++di){
                for(int dj = -1; dj <= 1; ++dj){
                    const float v = ground(i+di, j+dj);
                    if(std::isfinite(v) && v > max_val){
                        max_val = v;
                        max_di  = di;
                        max_dj  = dj;
                    }
                }
            }
            slope_maxdiff(i, j) = max_val - center;          // always >= 0
            slope_dir(i, j)     = DIR_CODE[max_di+1][max_dj+1];

            // Standard gradient-based slope: central differences -> angle.
            // This gives a smooth, physically-meaningful slope raster while
            // the max-diff value above stays true to the paper.
            const float dzdx = (ground(i+1, j) - ground(i-1, j)) * inv_2res;
            const float dzdy = (ground(i, j+1) - ground(i, j-1)) * inv_2res;
            slope_x(i, j) = dzdx;
            slope_y(i, j) = dzdy;

            const float grad_mag = std::sqrt(dzdx*dzdx + dzdy*dzdy);
            const float angle_rad = std::atan(grad_mag);
            slope_deg(i, j) = angle_rad * 180.0f / static_cast<float>(M_PI);

            // Binary traversability raster.
            if(observed(i, j) > 0.5f)
                travers(i, j) = (grad_mag <= slope_thresh_tan) ? 1.0f : 0.0f;
        }
    }
}
