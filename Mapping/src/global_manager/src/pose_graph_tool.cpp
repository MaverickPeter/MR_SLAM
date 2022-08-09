#include <global_manager/pose_graph_tool.hpp>



/*
 * Utility functions: initialize graph marker
 */
void initGraphNodeMarkers(visualization_msgs::Marker &graphNodeMarker, std::string frame_id)
{
    //init headers
    graphNodeMarker.header.frame_id     = frame_id;
    graphNodeMarker.header.stamp        = ros::Time::now();
    graphNodeMarker.ns                  = "graph_node";
    graphNodeMarker.action              = visualization_msgs::Marker::ADD;
    graphNodeMarker.pose.orientation.w  = 1.0;

    // setting id for each marker
    graphNodeMarker.id   = 0;

    // defining types
    graphNodeMarker.type  = visualization_msgs::Marker::LINE_STRIP;

    // setting scale
    graphNodeMarker.scale.x = 1.2;

    // assigning colors
    graphNodeMarker.color.r = 0.0f;
    graphNodeMarker.color.g = 1.0f;
    graphNodeMarker.color.b = 0.0f;
    graphNodeMarker.color.a = 1.0f;
}


/*
 * Utility functions: initialize graph marker
 */
void initGraphEdgeMarkers(visualization_msgs::Marker &graphEdgeMarker, std::string frame_id)
{
    //init headers
    graphEdgeMarker.header.frame_id     = frame_id;
    graphEdgeMarker.header.stamp        = ros::Time::now();
    graphEdgeMarker.ns                  = "graph_edge";
    graphEdgeMarker.action              = visualization_msgs::Marker::ADD;
    graphEdgeMarker.pose.orientation.w  = 1.0;

    // setting id for each marker
    graphEdgeMarker.id   = 1;

    // defining types
    graphEdgeMarker.type  = visualization_msgs::Marker::LINE_LIST;

    // setting scale
    graphEdgeMarker.scale.x = 1.2;

    // assigning colors
    graphEdgeMarker.color.r = 1.0f;
    graphEdgeMarker.color.g = 0.0f;
    graphEdgeMarker.color.b = 0.0f;
    graphEdgeMarker.color.a = 1.0f;
}