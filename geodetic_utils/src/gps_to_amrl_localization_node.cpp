/*
  Use altitude from 'external_altitude' topic if messages are received
  (To enable the messages arriving, publish to the topic by remapping in the launch file
  Otherwise, altitude from GPS is taken
*/


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geodetic_utils/geodetic_conv.hpp>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include <geodetic_utils//visualization.h>

using amrl_msgs::Localization2DMsg;
using Eigen::Vector2f;
using std::vector;


geodetic_converter::GeodeticConverter g_geodetic_converter;


ros::Publisher g_gps_amrl_localization_pub;
ros::Publisher g_gps_amrl_visualization_pub;

//AMRL Localization message
amrl_msgs::Localization2DMsg localization_msg_;
//AMRL Visualization message
amrl_msgs::VisualizationMsg visualization_msg_;


std::string g_frame_id;
std::string g_tf_child_frame_id;
std::string g_amrl_map;

std::shared_ptr<tf::TransformBroadcaster> p_tf_broadcaster;



void gps_callback(const sensor_msgs::NavSatFixConstPtr& msg)
{

  if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
    return;
  }

  if (!g_geodetic_converter.isInitialised()) {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
    return;
  }

  double x, y, z;
  g_geodetic_converter.geodetic2Enu(msg->latitude, msg->longitude, msg->altitude, &x, &y, &z);


  //prepare amrl localization message
  localization_msg_.header.stamp = ros::Time::now();
  localization_msg_.header.frame_id = g_frame_id;
  localization_msg_.map = g_amrl_map;
  localization_msg_.pose.x = x;
  localization_msg_.pose.y = y;
  localization_msg_.pose.theta = 0;  
  g_gps_amrl_localization_pub.publish(localization_msg_);


  

  //prepare nav trace messages
  static vector<Vector2f> trace;
  static std::vector< uint32_t > color;
  static Vector2f lastLoc;
  static bool initialized = false;
  
  const Vector2f curLoc(x, y);
  if (!initialized) {
    trace.push_back(curLoc);
    if (msg->position_covariance[0] < 0.15) color.push_back(0xFF00FF00); //green
    else if (msg->position_covariance[0] < 1.0) color.push_back(0xFFED7014); //orange
    else if (msg->position_covariance[0] < 2.0) color.push_back(0xFF893101); //amber
    else color.push_back(0xFFFF0000); //red
    lastLoc = curLoc;
    initialized = true;
    return;
  }
  if((curLoc-lastLoc).squaredNorm()>0.2236068){
    trace.push_back(curLoc);
    if (msg->position_covariance[0] < 0.15) color.push_back(0xFF00FF00); //green
    else if (msg->position_covariance[0] < 1.0) color.push_back(0xFFED7014); //orange
    else if (msg->position_covariance[0] < 2.0) color.push_back(0xFF893101); //amber
    else color.push_back(0xFFFF0000); //red
    lastLoc = curLoc;
  }


  for(unsigned int i = 0; i + 1 < trace.size(); i++){
    //if((trace[i]-trace[i+1]).squaredNorm()>2.236068)
    //  continue;
    //
    visualization::DrawLine(trace[i], trace[i + 1], color[i+1], visualization_msg_);
  }

  g_gps_amrl_visualization_pub.publish(visualization_msg_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_to_amrl_localization_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Get manual parameters
  ros::param::param<std::string>("~frame_id",
                                 g_frame_id, "map");
  ros::param::param<std::string>("~amrl_map",
                                 g_amrl_map, "UT_CAMPUS");
  ros::param::param<std::string>("~tf_child_frame_id",
                                 g_tf_child_frame_id, "gps_receiver");


  // Wait until GPS reference parameters are initialized.
  double latitude, longitude, altitude;
  do {
    ROS_INFO("Waiting for GPS reference parameters...");
    if (nh.getParam("/gps_ref_latitude", latitude) &&
        nh.getParam("/gps_ref_longitude", longitude) &&
        nh.getParam("/gps_ref_altitude", altitude)) {
      g_geodetic_converter.initialiseReference(latitude, longitude, altitude);
    } else {
      ROS_INFO(
          "GPS reference not ready yet, use set_gps_reference_node to set it");
      ros::Duration(0.5).sleep(); // sleep for half a second
    }
  } while (!g_geodetic_converter.isInitialised());

  // Show reference point
  double initial_latitude, initial_longitude, initial_altitude;
  g_geodetic_converter.getReference(&initial_latitude, &initial_longitude,
                                    &initial_altitude);
  ROS_INFO("GPS reference initialized correctly %f, %f, %f", initial_latitude,
           initial_longitude, initial_altitude);

  // Initialize publishers
  g_gps_amrl_localization_pub =
      nh.advertise<amrl_msgs::Localization2DMsg>("localization", 1, true);
  g_gps_amrl_visualization_pub =
        nh.advertise<amrl_msgs::VisualizationMsg>("visualization", 1, true);
  visualization_msg_ = visualization::NewVisualizationMessage("map", "enml");

  // Subscribe to GPS fixes, and convert in GPS callback
  ros::Subscriber gps_sub = nh.subscribe("gps", 1, &gps_callback);

  ros::spin();
}
