#include "visualization.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry, pub_latest_odometry_ros;
ros::Publisher pub_path;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate", 1000);
    //! 重要：发布T_odom_lidar给LIO系统，用于LIO系统后端scan-to-map的位姿初值估计
    pub_latest_odometry_ros = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 1000);
    pub_path = n.advertise<nav_msgs::Path>(PROJECT_NAME + "/vins/odometry/path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/odometry/point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/odometry/history_cloud", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>(PROJECT_NAME + "/vins/odometry/key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/vins/odometry/camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/odometry/keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/extrinsic", 1000);

    cameraposevisual.setScale(1);
    cameraposevisual.setLineWidth(0.05);
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
}

tf::Transform transformConversion(const tf::StampedTransform &t)
{
    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = t.getOrigin().x();
    yCur = t.getOrigin().y();
    zCur = t.getOrigin().z();
    tf::Matrix3x3 m(t.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    return tf::Transform(tf::createQuaternionFromRPY(rollCur, pitchCur, yawCur), tf::Vector3(xCur, yCur, zCur));
    ;
}


void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, 
    const Eigen::Vector3d &V, const std_msgs::Header &header, const int &failureId,
    const Eigen::Vector3d &t_ic, const Eigen::Quaterniond &q_ic)
{
    static tf::TransformBroadcaster br;
    static tf::TransformListener listener;
    static double last_align_time = -1;

    // Quternion not normalized
    if (Q.x() * Q.x() + Q.y() * Q.y() + Q.z() * Q.z() + Q.w() * Q.w() < 0.99)
        return;


    // imu odometry in camera frame
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.child_frame_id = "vins_body";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);


#if IF_OFFICIAL
    // imu odometry in ROS format (change rotation), used for lidar odometry initial guess
    odometry.pose.covariance[0] = double(failureId); // notify lidar odometry failure
    tf::Quaternion q_odom_cam(Q.x(), Q.y(), Q.z(), Q.w());
    tf::Quaternion q_cam_to_lidar(0, 1, 0, 0); // mark: camera - lidar
    tf::Quaternion q_odom_ros = q_odom_cam * q_cam_to_lidar;
    tf::quaternionTFToMsg(q_odom_ros, odometry.pose.pose.orientation);
    pub_latest_odometry_ros.publish(odometry);

    // TF of camera in vins_world in ROS format (change rotation), used for depth registration
    tf::Transform t_w_body = tf::Transform(q_odom_ros, tf::Vector3(P.x(), P.y(), P.z()));
    tf::StampedTransform trans_world_vinsbody_ros = tf::StampedTransform(
        t_w_body, header.stamp, "vins_world", "vins_body_ros");
    br.sendTransform(trans_world_vinsbody_ros);
#else
    // Step 1: 发布T_odom_lidar给LIO后端的scan-to-map位姿初值估计
    odometry.pose.covariance[0] = double(failureId); // notify lidar odometry failure
    //; R_odom_imu
    tf::Quaternion q_odom_imu(Q.x(), Q.y(), Q.z(), Q.w());   
    Eigen::Quaterniond q_imu_lidar(R_imu_lidar);
    //; R_imu_lidar
    tf::Quaternion q_imu_lidar_tf(q_imu_lidar.x(), q_imu_lidar.y(), q_imu_lidar.z(), q_imu_lidar.w());   
    //; R_odom_lidar = R_odom_imu * R_imu_lidar
    tf::Quaternion q_odom_lidar = q_odom_imu * q_imu_lidar_tf;
    //; t_dodom_lidar = R_odom_imu * t_imu_lidar + t_odom_imu
    Eigen::Vector3d t_odom_lidar = Q * t_imu_lidar + P;  
    odometry.pose.pose.position.x = t_odom_lidar.x();
    odometry.pose.pose.position.y = t_odom_lidar.y();
    odometry.pose.pose.position.z = t_odom_lidar.z();
    tf::quaternionTFToMsg(q_odom_lidar, odometry.pose.pose.orientation);
    pub_latest_odometry_ros.publish(odometry);

    // Step 2: 发布IMU频率下的T_odom_imu的tf位姿变换
    tf::Transform t_w_body = tf::Transform(q_odom_imu, tf::Vector3(P.x(), P.y(), P.z()));
    tf::StampedTransform trans_world_vinsBody = tf::StampedTransform(
        t_w_body, header.stamp, "vins_world", "vins_body_imuhz");
    br.sendTransform(trans_world_vinsBody);

    // Step 3: 发布camera 和 IMU之间的外参，这个在动态估计外参的时候会变换，所以前端深度注册也要使用这个动态外参
    //; 另外这里发布外参的时候直接发布了vins相机的FLU坐标系的变换，因为前端深度注册要用相机的FLU坐标系
    tf::Transform transform;
    tf::Quaternion q;
    //; Eigen::Quaterniond(0.5, 0.5, -0.5, 0.5)对应的旋转矩阵是R_c_cFLU，即相机的
    //; 前左上坐标系 -> 正常的相机右下前坐标系 之间的旋转，结果就是[0, -1, 0; 0, 0, -1; 1, 0, 0]
    //; R_imu_cFLU = R_imu_cam * R_cam_camFLU
    Eigen::Quaterniond q_i_cFLU = q_ic * Eigen::Quaterniond(0.5, 0.5, -0.5, 0.5);
    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setW(q_i_cFLU.w());
    q.setX(q_i_cFLU.x());
    q.setY(q_i_cFLU.y());
    q.setZ(q_i_cFLU.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(
        transform, header.stamp, "vins_body_imuhz", "vins_cameraFLU"));
#endif

    
    if (ALIGN_CAMERA_LIDAR_COORDINATE)
    {
    #if IF_OFFICIAL
        static tf::Transform t_odom_world = tf::Transform(tf::createQuaternionFromRPY(0, 0, M_PI), tf::Vector3(0, 0, 0));
    #else
        //? mod: vins_world坐标系和odom坐标系不再绕着Z轴旋转，而是直接对齐
        static tf::Transform t_odom_world = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    #endif

        if (header.stamp.toSec() - last_align_time > 1.0)
        {
            try
            {
            #if IF_OFFICIAL
                tf::StampedTransform trans_odom_baselink;
                listener.lookupTransform("odom","base_link", ros::Time(0), trans_odom_baselink);
                t_odom_world = transformConversion(trans_odom_baselink) * transformConversion(trans_world_vinsbody_ros).inverse();
                last_align_time = header.stamp.toSec();
            #else
                //; 计算odom坐标系和vins_world之间的变换关系，这个变换是会变化的，因为vins不准确存在漂移，
                //; 这里就用VINS估计的实时位姿和LIO估计的实时位姿对齐，然后把误差分配到odom和vins_world之间的变换上
                //; 即T_odom_lidar = T_vinsworld_lidar，其中T_odom_lidar是LIO估计的结果，T_vinsworld_lidar
                //; 是VINS估计的结果，则T_odom_vinsworld = T_odom_lidar * T_lidar_vinsworld
                tf::StampedTransform T_odom_lidar;
                listener.lookupTransform("odom", "base_link", ros::Time(0), T_odom_lidar);
                tf::Transform t_w_lidar = tf::Transform(q_odom_lidar, tf::Vector3(t_odom_lidar.x(), t_odom_lidar.y(), t_odom_lidar.z()));
                tf::StampedTransform T_vinsworld_lidar = tf::StampedTransform(
                    t_w_lidar, header.stamp, "vinsworld", "lidar");
                //; T_odom_vinsworld = T_odom_lidar * T_vinsworld_lidar.inverse()
                t_odom_world = transformConversion(T_odom_lidar) * 
                        transformConversion(T_vinsworld_lidar).inverse();
                last_align_time = header.stamp.toSec();
            #endif
            }
            catch (tf::TransformException ex)
            {
            }
        }
        br.sendTransform(tf::StampedTransform(
            t_odom_world, header.stamp, "odom", "vins_world"));
    }
    else
    {

    #if IF_OFFICIAL
        static tf::Transform t_static = tf::Transform(tf::createQuaternionFromRPY(0, 0, M_PI), tf::Vector3(0, 0, 0));
    #else
        //? mod: vins_world坐标系和odom坐标系不再绕着Z轴旋转，而是直接对齐
        static tf::Transform t_static = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    #endif

        br.sendTransform(tf::StampedTransform(t_static, header.stamp, "odom", "vins_world"));
    }
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //ROS_DEBUG("calibration result for camera %d", i);
        ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
        ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.child_frame_id = "vins_world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        static double path_save_time = -1;
        if (header.stamp.toSec() - path_save_time > 0.5)
        {
            path_save_time = header.stamp.toSec();
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = header;
            pose_stamped.header.frame_id = "vins_world";
            pose_stamped.pose = odometry.pose.pose;
            path.header = header;
            path.header.frame_id = "vins_world";
            path.poses.push_back(pose_stamped);
            pub_path.publish(path);
        }
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (pub_key_poses.getNumSubscribers() == 0)
        return;

    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "vins_world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    if (pub_camera_pose_visual.getNumSubscribers() == 0)
        return;

    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    if (pub_point_cloud.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud point_cloud;
        point_cloud.header = header;
        point_cloud.header.frame_id = "vins_world";

        sensor_msgs::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
                continue;

            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            point_cloud.points.push_back(p);

            if (it_per_id.lidar_depth_flag == false)
                intensity_channel.values.push_back(0);
            else
                intensity_channel.values.push_back(1);
        }

        point_cloud.channels.push_back(intensity_channel);
        pub_point_cloud.publish(point_cloud);
    }

    // pub margined potin
    if (pub_margin_cloud.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud margin_cloud;
        margin_cloud.header = header;
        margin_cloud.header.frame_id = "vins_world";

        sensor_msgs::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 && it_per_id.solve_flag == 1)
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                margin_cloud.points.push_back(p);

                if (it_per_id.lidar_depth_flag == false)
                    intensity_channel.values.push_back(0);
                else
                    intensity_channel.values.push_back(1);
            }
        }

        margin_cloud.channels.push_back(intensity_channel);
        pub_margin_cloud.publish(margin_cloud);
    }
}

/**
 * @brief 发布估计的TF坐标变换，这个非常重要
 * 
 * @param[in] estimator 
 * @param[in] header 
 */
void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0), correct_t(1), correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(
        transform, header.stamp, "vins_world", "vins_body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(
        transform, header.stamp, "vins_body", "vins_camera"));

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);
}

void pubKeyframe(const Estimator &estimator)
{
    if (pub_keyframe_pose.getNumSubscribers() == 0 && pub_keyframe_point.getNumSubscribers() == 0)
        return;

    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header = estimator.Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_keyframe_pose.publish(odometry);

        sensor_msgs::PointCloud point_cloud;
        point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if (it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }
        }
        pub_keyframe_point.publish(point_cloud);
    }
}