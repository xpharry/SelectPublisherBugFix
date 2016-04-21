# SelectPublisherBugFix
Final Project of EECS476

# compare among
  
  1. /selected_points
  https://github.com/wsnewman/ros_helper_packages/tree/master/rviz_plugin_selected_points_topic

  2. /selected_points_publisher
  https://github.com/tu-rbo/turbo-ros-pkg/tree/master/selected_points_publisher
 
  3. /cliected_point
  https://github.com/ros-visualization/rviz/blob/groovy-devel/src/rviz/default_plugin/tools/point_tool.h

# how to run the program

After you compile everything, run the launch file

	$ roslaunch display_mobot_view display_mobot_view.launch

which gives you a show of a colored point cloud.

Then, you can pick up a tool among the three to select a point / points and observe the results in the screen.
