#include <elevationMap_layer/elevationMap_layer.h>
#include <pluginlib/class_list_macros.h>

using costmap_2d::LETHAL_OBSTACLE;
using namespace std;

namespace costmap_2d
{

ElevationMapLayer::ElevationMapLayer() {}

ElevationMapLayer::~ElevationMapLayer() {}

// Initialize layer
void ElevationMapLayer::onInitialize()
{
    ObstacleLayer::onInitialize();

    ros::NodeHandle nh("~/" + name_);

    std::string source_topic;
    nh.param<std::string>("source_topic", source_topic, std::string("/velodyne_points"));
    nh.param("travers_thresh", travers_thresh, 0.5);

    // subscriber for elevation map
    elevation_map_available_ = false;
    elevation_map_sub_ = nh.subscribe(source_topic, 10, &ElevationMapLayer::elevationMapCB, this);
}


// Elevation map callback
void ElevationMapLayer::elevationMapCB(const grid_map_msgs::GridMapConstPtr& msg)
{
    if (!elevation_map_available_)
    {
        grid_map::GridMapRosConverter::fromMessage(*msg, elevation_map_);
        elevation_map_available_ = true;
    }
}


// update bounds (base function in costmap class)
void ElevationMapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                                double* min_x, double* min_y, double* max_x, double* max_y)
{
    // memset(costmap_, (unsigned char)costmap_2d::NO_INFORMATION, size_x_* size_y_ * sizeof(unsigned char));

    if (rolling_window_)
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    if (!enabled_)
        return;
    useExtraBounds(min_x, min_y, max_x, max_y);

    bool current = true;
    current = elevation_map_available_;
    current_ = true;

    // check if the elevation map is available
    if (elevation_map_available_)
    {
        grid_map::Matrix& travers = elevation_map_["traver"];
        grid_map::Matrix& elevation = elevation_map_["elevation"];
        grid_map::Matrix& frontier = elevation_map_["frontier"];

        for (grid_map::GridMapIterator iterator(elevation_map_); !iterator.isPastEnd(); ++iterator)
        {
            const grid_map::Index gindex(*iterator);
            bool is_obstacle = (travers(gindex(0), gindex(1)) < travers_thresh) && (travers(gindex(0), gindex(1)) != -10);
            bool is_unknown = (elevation(gindex(0), gindex(1)) == -10);
            bool is_frontier = (frontier(gindex(0), gindex(1)) == 1);

            grid_map::Position pos;
            elevation_map_.getPosition(gindex, pos);
            double px = pos.x(), py = pos.y();

            // now we need to compute the map coordinates for the observation
            unsigned int mx, my;
            if (!worldToMap(px, py, mx, my))
            {
                ROS_WARN("Computing map coords failed");
                continue;
            }

            unsigned int index = getIndex(mx, my);

            if(!is_unknown && is_obstacle){
                costmap_[index] = LETHAL_OBSTACLE;
            }else{
                costmap_[index] = FREE_SPACE;
            }
            touch(px, py, min_x, min_y, max_x, max_y);
        }

        elevation_map_available_ = false;
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

} // namespace costmap_2d

PLUGINLIB_EXPORT_CLASS(costmap_2d::ElevationMapLayer, costmap_2d::Layer)
