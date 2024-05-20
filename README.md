# color_map
ROS node for displaying temperature data as a MarkerArray

## Usage

Clone this repository into a catkin workspace and run `catkin_make`.  

Run the node using `rosrun color_map color_map`.

The node will publish a `visualization_msgs/MarkerArray` message to the topic: `/visualization_marker_array`

## Dependencies
This node depends on two different ros topics: `/temperature`, and `/slam_out_pose`, where:

- `/temperature` is a `sensor_msgs/Temperature` message

- `/slam_out_pose` is a `geometry_msgs/PoseStamped` message.
