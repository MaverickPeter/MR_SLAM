// std
#include <iostream>
#include <stdlib.h>
#include <string>

// ROS
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>


/*!
* Performs the pose graph nodes marker initialization procedure.
*/
void initGraphNodeMarkers(visualization_msgs::Marker &graphNodeMarker, std::string frame_id);

/*!
* Performs the pose graph edges marker initialization procedure.
*/
void initGraphEdgeMarkers(visualization_msgs::Marker &graphEdgeMarker, std::string frame_id);
