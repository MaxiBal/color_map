#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Temperature.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <utility>

#include <color.hpp>

ros::Publisher color_map_publisher;

const std::string frame_id = "map";

constexpr uint32_t marker_shape = visualization_msgs::Marker::CUBE;

std::vector<visualization_msgs::Marker> markers;

visualization_msgs::Marker create_marker()
{
	visualization_msgs::Marker marker;

	marker.ns = "heatmap";
	marker.id = markers.size();

	marker.type = marker_shape;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.4;
	marker.scale.y = 0.4;
	marker.scale.z = 0.001;

	marker.color.a = 0.3;

	marker.lifetime = ros::Duration();

	return marker;

}

void combine_map(
	const sensor_msgs::TemperaturePtr& temp,
	const geometry_msgs::PoseStampedConstPtr& pose)
{
	float t = temp->temperature;

	visualization_msgs::Marker marker = create_marker();

	marker.pose = pose->pose;

	color_map::ColorHandler ch(t);

	float r, g, b;

	std::tie(
		marker.color.r, 
		marker.color.g, 
		marker.color.b
	) = ch.get_rgb();

	markers.push_back(marker);
	
	visualization_msgs::MarkerArray arr;
	arr.markers = markers;

	color_map_publisher.publish(arr);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "color_map_node");

	ros::NodeHandle nh;

	color_map_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);

	// subscribe to pose, temp

	message_filters::Subscriber<sensor_msgs::Temperature>   temp_sub (nh, "temperature", 1);
	message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub (nh, "slam_out_pose", 1);


	// eventually add in the temperature reading node
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Temperature, geometry_msgs::PoseStamped> MapPoseSyncPolicy;

	message_filters::Synchronizer<MapPoseSyncPolicy> sync(MapPoseSyncPolicy(10), map_sub, pose_sub);

	sync.registerCallback(
		boost::bind(&combine_map, _1, _2, _3)
	);

	ros::spin();

	return 0;
}
