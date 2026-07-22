#include <algorithm>
#include <cmath>
#include <string>
#include <limits>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace groundgrid {

class LunarTraversabilityNode {
public:
    LunarTraversabilityNode() : nh_(), pnh_("~") {
        pnh_.param("max_longitudinal_slope_deg", max_slope_deg_, 20.0);
        pnh_.param("max_roughness", max_roughness_, 0.08);
        pnh_.param("max_step_height", max_step_, 0.20);
        pnh_.param("min_ground_confidence", min_confidence_, 0.20);
        pnh_.param("max_observation_age", max_age_, 1.0);
        pnh_.param("min_obstacle_height", min_obstacle_height_, 0.10);
        pnh_.param("min_obstacle_confidence", min_obstacle_confidence_, 0.45);
        pnh_.param<std::string>("input_topic", input_topic_, "/groundgrid/grid_map");
        pnh_.param<std::string>("grid_map_topic", grid_map_topic_, "/terrain/grid_map");
        pnh_.param<std::string>("costmap_topic", costmap_topic_, "/terrain/costmap");

        map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_, 1, true);
        cost_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(costmap_topic_, 1, true);
        sub_ = nh_.subscribe(input_topic_, 1, &LunarTraversabilityNode::mapCallback, this);
    }

private:
    bool requiredLayers(const grid_map::GridMap& map) const {
        static const std::vector<std::string> layers = {
            "observed", "observation_age", "groundpatch", "slope",
            "roughness", "step_height", "obstacle_height", "obstacle_confidence"
        };
        for(const auto& layer : layers) {
            if(!map.exists(layer)) {
                ROS_ERROR_THROTTLE(2.0, "Terrain map is missing required layer '%s'.", layer.c_str());
                return false;
            }
        }
        return true;
    }

    void mapCallback(const grid_map_msgs::GridMapConstPtr& msg) {
        grid_map::GridMap map;
        grid_map::GridMapRosConverter::fromMessage(*msg, map);
        if(!requiredLayers(map)) return;

        map.add("terrain_cost", std::numeric_limits<float>::quiet_NaN());
        map.add("terrain_lethal", 1.0f);
        map["terrain_cost"].setConstant(std::numeric_limits<float>::quiet_NaN());
        map["terrain_lethal"].setConstant(1.0f);

        for(grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
            const grid_map::Index idx(*it);
            const float observed = map.at("observed", idx);
            const float age = map.at("observation_age", idx);
            const float confidence = map.at("groundpatch", idx);
            const float slope = map.at("slope", idx);
            const float roughness = map.at("roughness", idx);
            const float step = map.at("step_height", idx);
            const float obstacle_height = map.at("obstacle_height", idx);
            const float obstacle_conf = map.at("obstacle_confidence", idx);

            const bool known = observed > 0.5f && std::isfinite(age) && age <= max_age_ &&
                               std::isfinite(confidence) && confidence >= min_confidence_ &&
                               std::isfinite(slope) && std::isfinite(roughness) && std::isfinite(step);
            if(!known) continue;

            const bool obstacle = obstacle_height >= min_obstacle_height_ &&
                                  obstacle_conf >= min_obstacle_confidence_;
            const bool lethal = obstacle || slope > max_slope_deg_ ||
                                roughness > max_roughness_ || step > max_step_;
            if(lethal) {
                map.at("terrain_cost", idx) = 100.0f;
                map.at("terrain_lethal", idx) = 1.0f;
                continue;
            }

            const float slope_cost = std::clamp(slope / static_cast<float>(max_slope_deg_), 0.0f, 1.0f);
            const float rough_cost = std::clamp(roughness / static_cast<float>(max_roughness_), 0.0f, 1.0f);
            const float step_cost = std::clamp(step / static_cast<float>(max_step_), 0.0f, 1.0f);
            const float uncertainty = 1.0f - std::clamp(confidence, 0.0f, 1.0f);
            map.at("terrain_cost", idx) = 99.0f *
                (0.35f * slope_cost + 0.20f * rough_cost +
                 0.25f * step_cost + 0.20f * uncertainty);
            map.at("terrain_lethal", idx) = 0.0f;
        }

        grid_map_msgs::GridMap enriched;
        grid_map::GridMapRosConverter::toMessage(map, enriched);
        enriched.info.header.stamp = msg->info.header.stamp;
        map_pub_.publish(enriched);

        nav_msgs::OccupancyGrid costmap;
        grid_map::GridMapRosConverter::toOccupancyGrid(map, "terrain_cost", 0.0, 100.0, costmap);
        costmap.header.stamp = msg->info.header.stamp;
        costmap.header.frame_id = map.getFrameId();
        cost_pub_.publish(costmap);
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_;
    ros::Publisher map_pub_, cost_pub_;
    std::string input_topic_, grid_map_topic_, costmap_topic_;
    double max_slope_deg_, max_roughness_, max_step_, min_confidence_, max_age_;
    double min_obstacle_height_, min_obstacle_confidence_;
};

}  // namespace groundgrid

int main(int argc, char** argv) {
    ros::init(argc, argv, "lunar_traversability");
    groundgrid::LunarTraversabilityNode node;
    ros::spin();
    return 0;
}
