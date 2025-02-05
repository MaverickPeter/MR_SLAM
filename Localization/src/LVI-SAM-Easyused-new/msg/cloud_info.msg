# Cloud Info
Header header 

int32[] startRingIndex
int32[] endRingIndex

int32[]  pointColInd # point column index in range image
float32[] pointRange # point range 

int64 imuAvailable        #; 原始9轴IMU输出的位姿是否可用
int64 vinsOdomAvailable   #; vins发来的里程计是否可用
int64 imuOdomAvailable    #; LIO-SAM的imu预积分节点发来的里程计是否可用

# Attitude for lidar odometry initialization
float32 imuRollInit
float32 imuPitchInit
float32 imuYawInit

# Odometry 
float32 initialGuessX
float32 initialGuessY
float32 initialGuessZ
float32 initialGuessRoll
float32 initialGuessPitch
float32 initialGuessYaw

# Odometry reset ID
int64 vinsOdomResetId   #; vins里程计重启的id，在vins失效时会重启，id+1
int64 imuOdomResetId    #; LIO-SAM的imu预积分里程计重启的id，在imu预积分节点估计的bias过大时会重启，id+1

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature

# 3rd party messages
sensor_msgs/PointCloud2 key_frame_cloud
sensor_msgs/PointCloud2 key_frame_color
sensor_msgs/PointCloud2 key_frame_poses
sensor_msgs/PointCloud2 key_frame_map