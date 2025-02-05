#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <ros/ros.h>
#include <map>
#include "../feature_manager.h"

using namespace Eigen;
using namespace std;

class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>>& _points, 
                   const vector<float> &_lidar_initialization_info,
                   double _t):
        t{_t}, is_key_frame{false}, reset_id{-1}, gravity{9.805}
        {
            points = _points;
            
            // reset id in case lidar odometry relocate
            reset_id = (int)round(_lidar_initialization_info[0]);
            // Pose
            T.x() = _lidar_initialization_info[1];
            T.y() = _lidar_initialization_info[2];
            T.z() = _lidar_initialization_info[3];
            // Rotation
            Eigen::Quaterniond Q = Eigen::Quaterniond(_lidar_initialization_info[7],
                                                      _lidar_initialization_info[4],
                                                      _lidar_initialization_info[5],
                                                      _lidar_initialization_info[6]);
            R = Q.normalized().toRotationMatrix();
            // Velocity
            V.x() = _lidar_initialization_info[8];
            V.y() = _lidar_initialization_info[9];
            V.z() = _lidar_initialization_info[10];
            // Acceleration bias
            Ba.x() = _lidar_initialization_info[11];
            Ba.y() = _lidar_initialization_info[12];
            Ba.z() = _lidar_initialization_info[13];
            // Gyroscope bias
            Bg.x() = _lidar_initialization_info[14];
            Bg.y() = _lidar_initialization_info[15];
            Bg.z() = _lidar_initialization_info[16];
            // Gravity
            gravity = _lidar_initialization_info[17];
        };

        map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>> > > points;
        double t;
        
        IntegrationBase *pre_integration;
        bool is_key_frame;

        // Lidar odometry info
        int reset_id;
        Vector3d T;
        Matrix3d R;
        Vector3d V;
        Vector3d Ba;
        Vector3d Bg;
        double gravity;
};


bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);


class odometryRegister
{
public:

    ros::NodeHandle n;
    //; R_lidar_imu，即imu -> lidar，这里的变量写的是真难受
    tf::Quaternion q_lidar_to_cam;   
    //; 这个是作者手动把vins_world和参考的世界坐标系odom转了180度，加不加都不影响
    Eigen::Quaterniond q_lidar_to_cam_eigen;  
    
    //? add
    //; t_lidar_imu，即imu -> lidar
    Eigen::Vector3d t_lidar_imu;   

    ros::Publisher pub_latest_odometry; 

#if IF_OFFICIAL
    odometryRegister(ros::NodeHandle n_in):
        n(n_in)
#else
    odometryRegister(ros::NodeHandle n_in, const Eigen::Matrix3d& R_lidar_imu_,
        const Eigen::Vector3d& t_lidar_imu_):
        n(n_in), t_lidar_imu(t_lidar_imu_)
#endif
    {
        
    #if IF_OFFICIAL
        q_lidar_to_cam = tf::Quaternion(0, 1, 0, 0); // rotate orientation // mark: camera - lidar
        q_lidar_to_cam_eigen = Eigen::Quaterniond(0, 0, 0, 1); // rotate position by pi, (w, x, y, z) // mark: camera - lidar
    #else
        //? mod：添加自己的旋转外参变化，即R_lidar_imu
        Eigen::Quaterniond q_lidar_imu(R_lidar_imu_);
        q_lidar_imu.normalize();   // 四元数归一化
        q_lidar_to_cam = tf::Quaternion(q_lidar_imu.x(), q_lidar_imu.y(), q_lidar_imu.z(), q_lidar_imu.w());  
    #endif
    }

    // convert odometry from ROS Lidar frame to VINS camera frame
    vector<float> getOdometry(deque<nav_msgs::Odometry>& odomQueue, double img_time)
    {
        vector<float> odometry_channel;
        odometry_channel.resize(18, -1); // reset id(1), P(3), Q(4), V(3), Ba(3), Bg(3), gravity(1)

        nav_msgs::Odometry odomCur;
        
        // pop old odometry msg
        while (!odomQueue.empty()) 
        {
            if (odomQueue.front().header.stamp.toSec() < img_time - 0.05)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
        {
            return odometry_channel;
        }

        // find the odometry time that is the closest to image time
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            odomCur = odomQueue[i];

            if (odomCur.header.stamp.toSec() < img_time - 0.002) // 500Hz imu
                continue;
            else
                break;
        }

        // time stamp difference still too large
        if (abs(odomCur.header.stamp.toSec() - img_time) > 0.05)
        {
            return odometry_channel;
        }

        // convert odometry rotation from lidar ROS frame to VINS camera frame (only rotation, assume lidar, camera, and IMU are close enough)
        tf::Quaternion q_odom_lidar;
        tf::quaternionMsgToTF(odomCur.pose.pose.orientation, q_odom_lidar);

    #if IF_OFFICIAL
        tf::Quaternion q_odom_cam = tf::createQuaternionFromRPY(0, 0, M_PI) * (q_odom_lidar * q_lidar_to_cam); // global rotate by pi // mark: camera - lidar
    #else
        //? mod: vins_world坐标系和odom坐标系不再绕着Z轴旋转，而是直接对齐，也就是前面不乘 tf::createQuaternionFromRPY(0, 0, M_PI) 了
        //; R_odom_imu = R_odom_lidar * R_lidar_imu
        tf::Quaternion q_odom_cam = q_odom_lidar * q_lidar_to_cam; // global rotate by pi // mark: camera - lidar
    #endif
        tf::quaternionTFToMsg(q_odom_cam, odomCur.pose.pose.orientation);

        // convert odometry position from lidar ROS frame to VINS camera frame
        Eigen::Vector3d p_eigen(odomCur.pose.pose.position.x, odomCur.pose.pose.position.y, odomCur.pose.pose.position.z);
        Eigen::Vector3d v_eigen(odomCur.twist.twist.linear.x, odomCur.twist.twist.linear.y, odomCur.twist.twist.linear.z);
    
    #if IF_OFFICIAL
        Eigen::Vector3d p_eigen_new = q_lidar_to_cam_eigen * p_eigen;
        Eigen::Vector3d v_eigen_new = q_lidar_to_cam_eigen * v_eigen;
    #else
        //? add: 补偿LiDAR和IMU之间的平移
        //; T_odom_imu = T_odom_lidar * T_lidar_imu 
        //;            = [R_odom_lidar, t_odom_lidar] * [R_lidar_imu, t_lidar_imu]
        //;            = [R_odom_lidar * R_lidar_imu, R_odom_lidar * t_lidar_imu + t_odom_lidar]
        Eigen::Quaterniond q_wl(q_odom_lidar.w(), q_odom_lidar.x(), q_odom_lidar.y(), q_odom_lidar.z());
        //; 注意这里位置需要补偿，imu原点和lidar原点的速度虽然并不严格相等，但是初始化阶段相差不大，所以直接赋值即可
        p_eigen += q_wl * t_lidar_imu;
        Eigen::Vector3d p_eigen_new = p_eigen;
        Eigen::Vector3d v_eigen_new = v_eigen;
    #endif

        odomCur.pose.pose.position.x = p_eigen_new.x();
        odomCur.pose.pose.position.y = p_eigen_new.y();
        odomCur.pose.pose.position.z = p_eigen_new.z();

        odomCur.twist.twist.linear.x = v_eigen_new.x();
        odomCur.twist.twist.linear.y = v_eigen_new.y();
        odomCur.twist.twist.linear.z = v_eigen_new.z();

        odometry_channel[0] = odomCur.pose.covariance[0];
        odometry_channel[1] = odomCur.pose.pose.position.x;
        odometry_channel[2] = odomCur.pose.pose.position.y;
        odometry_channel[3] = odomCur.pose.pose.position.z;
        odometry_channel[4] = odomCur.pose.pose.orientation.x;
        odometry_channel[5] = odomCur.pose.pose.orientation.y;
        odometry_channel[6] = odomCur.pose.pose.orientation.z;
        odometry_channel[7] = odomCur.pose.pose.orientation.w;
        odometry_channel[8]  = odomCur.twist.twist.linear.x;
        odometry_channel[9]  = odomCur.twist.twist.linear.y;
        odometry_channel[10] = odomCur.twist.twist.linear.z;
        odometry_channel[11] = odomCur.pose.covariance[1];
        odometry_channel[12] = odomCur.pose.covariance[2];
        odometry_channel[13] = odomCur.pose.covariance[3];
        odometry_channel[14] = odomCur.pose.covariance[4];
        odometry_channel[15] = odomCur.pose.covariance[5];
        odometry_channel[16] = odomCur.pose.covariance[6];
        odometry_channel[17] = odomCur.pose.covariance[7];

        return odometry_channel;
    }
};
