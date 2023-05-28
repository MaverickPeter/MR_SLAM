#include <pointMap_layer/pointMap_layer.h>
#include <pluginlib/class_list_macros.h>

using namespace std;
using costmap_2d::LETHAL_OBSTACLE;


namespace costmap_2d
{

PointMapLayer::PointMapLayer() {}

PointMapLayer::~PointMapLayer() {}

// Initialize layer
void PointMapLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
    enabled_ = true;

    std::string source_topic;
    nh.param("source_topic", source_topic, std::string("/velodyne_points"));

    ROS_INFO("\033[1;33m Subscribing to pointcloud topic: %s \033[0m", source_topic.c_str());

    // subscriber for point cloud
    point_map_sub_ = nh.subscribe(source_topic, 10, &PointMapLayer::pointMapCB, this);
}


// Point cloud callback
void PointMapLayer::pointMapCB(const sensor_msgs::PointCloud2& msg) 
{
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(msg, pcl_pc);

    PointCloud<Anypoint>::Ptr pointCloud(new PointCloud<Anypoint>);
    pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
    ob_pointCloud = *pointCloud;
}


// update bounds (base function in costmap class)
void PointMapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
    if (rolling_window_)
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    if (!enabled_)
        return;
    useExtraBounds(min_x, min_y, max_x, max_y);

    // iterate all points
    for (int i = 0; i < ob_pointCloud.points.size(); i++)
    {
        auto& point = ob_pointCloud.points[i];

        unsigned int mx, my;
        double px = point.x;
        double py = point.y;

        if (!worldToMap(px, py, mx, my))
        {
            ROS_DEBUG("Computing map coords failed");
            continue;
        }
        
        // get index of the costmap using raw point xy
        unsigned int index = getIndex(mx, my);

        // directly update costmap using obstacle value given by GEM
        if(point.travers > 0.1){
            costmap_[index] = FREE_SPACE;
        }else{
            costmap_[index] = LETHAL_OBSTACLE;
        }

        // update map
        touch(px, py, min_x, min_y, max_x, max_y);
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

} // end namespace

PLUGINLIB_EXPORT_CLASS(costmap_2d::PointMapLayer, costmap_2d::Layer)
