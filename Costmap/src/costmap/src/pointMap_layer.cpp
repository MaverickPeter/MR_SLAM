#include <pointMap_layer/pointMap_layer.h>
#include <pluginlib/class_list_macros.h>
#include <chrono>

using namespace std;
using namespace chrono;
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
    count = 0;
    thresh_type = -1;
    isUpdated = false;
    rolling_window_ = false;
    isOptimizing = false;

    //Record_txt.open("/home/mav-lab/RealCostTimeAndMemUsage.txt");

    std::string source_topic, opt_topic;
    nh.param("source_topic", source_topic, std::string("/velodyne_points"));
    nh.param("opt_topic", opt_topic, std::string("/opt"));
    nh.param("travers_thresh", travers_thresh, -10.0);
    nh.param("z_thresh", z_thresh, -1000.0);

    if(travers_thresh != -10.0 && z_thresh == -1000.0){
        thresh_type = 0;
    }else if(travers_thresh == -10.0 && z_thresh != -1000.0){
        thresh_type = 1;
    }else{
        ROS_ERROR("Please set a thresh type for costmap to determine obstacles!");
    }

    ROS_INFO("\033[1;33m Subscribing to pointcloud topic: %s \033[0m", source_topic.c_str());

    // subscriber for point cloud
    point_map_sub_ = nh.subscribe(source_topic, 100, &PointMapLayer::pointMapCB, this);
    map_opt_sub_ = nh.subscribe(opt_topic, 100, &PointMapLayer::optSignalCB, this);
}


// Point cloud callback
void PointMapLayer::pointMapCB(const sensor_msgs::PointCloud2& msg) 
{
    if(!isUpdated){
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(msg, pcl_pc);

        PointCloud<Anypoint>::Ptr pointCloud(new PointCloud<Anypoint>);
        pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
        ob_pointCloud = *pointCloud;
        isUpdated = true;
    }
}


// Point cloud callback
void PointMapLayer::optSignalCB(const std_msgs::Bool& msg) 
{
    isOptimizing = true;
}


// update bounds (base function in costmap class)
void PointMapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
    auto start = system_clock::now();

    if(count == 0 || isOptimizing){
        memset(costmap_, (unsigned char)costmap_2d::NO_INFORMATION, size_x_* size_y_ * sizeof(unsigned char));
	ROS_WARN("memset whole map");
    }
    count ++;

    if(isUpdated){
        if (rolling_window_)
            updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
        if (!enabled_)
            return;

        *min_x = 1 - size_x_ * resolution_ / 2;
        *min_y = 1 - size_y_ * resolution_ / 2;
        *max_x = size_x_ * resolution_ / 2 - 1;
        *max_y = size_y_ * resolution_ / 2 - 1;

        useExtraBounds(min_x, min_y, max_x, max_y);

        // iterate all points
        for (int i = 0; i < ob_pointCloud.points.size(); i++)
        {
            Anypoint point = ob_pointCloud.points[i];
	    
            unsigned int mx, my;
            double px = point.x;
            double py = point.y;
	        if(isnan(px) || isnan(py)){continue;}
            if (!worldToMap(px, py, mx, my))
            {
                ROS_DEBUG("Computing map coords failed px %lf, py %lf", px, py);
                continue;
            }

            // get index of the costmap using raw point xy
            unsigned int index = getIndex(mx, my);

            // directly update costmap using obstacle value given by GEM
            if(thresh_type == 0){
                if(point.z != -10 && point.z != 10 && point.travers != -10 && point.travers < travers_thresh){
                    costmap_[index] = LETHAL_OBSTACLE;
                }else{
                    costmap_[index] = FREE_SPACE;
                }
            }else if(thresh_type == 1){
                if(point.z != -10 && point.z != 10 && point.travers != -10 && point.z > z_thresh){
                    costmap_[index] = LETHAL_OBSTACLE;
                }else{
                    costmap_[index] = FREE_SPACE;
                }
            }


            // update map
            touch(px, py, min_x, min_y, max_x, max_y);
        }
        
        updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
        isUpdated = false;
    }
    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    ROS_INFO("costmap generation: %lfs", double(duration.count()) * microseconds::period::num / microseconds::period::den);
 
    isOptimizing = false;

}

void PointMapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
      // cout << "set cost i " << i << " j " << j << endl;
   }
  }
}


} // end namespace

PLUGINLIB_EXPORT_CLASS(costmap_2d::PointMapLayer, costmap_2d::Layer)
