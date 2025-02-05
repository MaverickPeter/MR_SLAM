#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::string POINT_CLOUD_TOPIC;
std::string PROJECT_NAME;

std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;


#if IF_OFFICIAL
double L_C_TX;
double L_C_TY;
double L_C_TZ;
double L_C_RX;
double L_C_RY;
double L_C_RZ;
#else
//? mod: lidar -> imu外参
// [R_imu_lidar, t_imu_lidar;
//         0,          1    ]
tf::Transform Transform_imu_lidar;
#endif

int USE_LIDAR;
int LIDAR_SKIP;


void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    n.getParam("vins_config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // project name
    fsSettings["project_name"] >> PROJECT_NAME;
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);

    // sensor topics
    fsSettings["image_topic"]       >> IMAGE_TOPIC;
    fsSettings["imu_topic"]         >> IMU_TOPIC;
    fsSettings["point_cloud_topic"] >> POINT_CLOUD_TOPIC;

    // lidar configurations
    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["lidar_skip"] >> LIDAR_SKIP;

    // feature and image settings
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];

#if IF_OFFICIAL
    L_C_TX = fsSettings["lidar_to_cam_tx"];
    L_C_TY = fsSettings["lidar_to_cam_ty"];
    L_C_TZ = fsSettings["lidar_to_cam_tz"];
    L_C_RX = fsSettings["lidar_to_cam_rx"];
    L_C_RY = fsSettings["lidar_to_cam_ry"];
    L_C_RZ = fsSettings["lidar_to_cam_rz"];
#endif

    // fisheye mask
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
    {
        std::string mask_name;
        fsSettings["fisheye_mask"] >> mask_name;
        FISHEYE_MASK = pkg_path + mask_name;
    }

    // camera config
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();

    //? add: 读取params_lidar.yaml中的参数
#if IF_OFFICIAL

#else
    std::vector<double> t_imu_lidar_V;
    std::vector<double> R_imu_lidar_V;
    n.param<std::vector<double>>(PROJECT_NAME+ "/extrinsicTranslation", t_imu_lidar_V, std::vector<double>());
    n.param<std::vector<double>>(PROJECT_NAME+ "/extrinsicRotation", R_imu_lidar_V, std::vector<double>());
    Eigen::Vector3d t_imu_lidar = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(t_imu_lidar_V.data(), 3, 1);
    Eigen::Matrix3d R_tmp = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(R_imu_lidar_V.data(), 3, 3);
    ROS_ASSERT(abs(R_tmp.determinant()) > 0.9);   // 防止配置文件中写错，这里加一个断言判断一下
    Eigen::Quaterniond Q_imu_lidar = Eigen::Quaterniond(R_tmp).normalized();
    Eigen::Matrix3d R_imu_lidar = Q_imu_lidar.toRotationMatrix();
    
    Transform_imu_lidar = tf::Transform(tf::Quaternion(Q_imu_lidar.x(), Q_imu_lidar.y(), Q_imu_lidar.z(), Q_imu_lidar.w()), 
        tf::Vector3(t_imu_lidar(0), t_imu_lidar(1), t_imu_lidar(2)));

    ROS_WARN_STREAM("=vins-feature_tracker read R_imu_lidar : =====================");
    std::cout << R_imu_lidar << std::endl;
    ROS_WARN_STREAM("=vins-feature_tracker read t_lidar_imu : =====================");
    std::cout << t_imu_lidar(0)  << ", " << t_imu_lidar(1) << ", " << t_imu_lidar(2) << std::endl;
#endif

    usleep(100);
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    if (thisPub->getNumSubscribers() == 0)
        return;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud); 
}