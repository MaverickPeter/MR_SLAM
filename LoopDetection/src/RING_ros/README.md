## Loop Detection Experiments
### - Clients [client1]

#### Filter

cd ~/Projects/DiSLAM-RING-centralized/filter/
source devel/setup.bash (sdb)
roslaunch filter_robot_1.launch 
roslaunch filter_robot_2.launch 
roslaunch filter_robot_3.launch 

#### Fake Image

cd ~/Projects/DiSLAM-RING-centralized/fake_img/
python robot_1.py
python robot_2.py
python robot_3.py

#### Elevation Mapping 

cd ~/Projects/DiSLAM-RING-centralized/Mapping_ws/
source devel/setup.bash (sdb)
roslaunch elevation_mapping_demos 4800_1.launch 
roslaunch elevation_mapping_demos 4800_2.launch 
roslaunch elevation_mapping_demos 4800_3.launch 

#### LIO

cd ~/Projects/DiSLAM-RING-centralized/Localization_ws/
source devel/setup.bash (sdb)
roslaunch fast_lio 4800_1.launch
roslaunch fast_lio 4800_2.launch
roslaunch fast_lio 4800_3.launch
(Change the distance threshold of published submaps by modifying the dis_th parameter in 4800_1.launch)

#### Exploration

cd ~/Projects/DiSLAM-RING-centralized/Exploration_ws/
source devel/setup.bash (sdb)
roslaunch move_base move_base_client_sim1.launch