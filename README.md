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

# some useful links

1. https://github.com/ros-visualization/rviz/issues/844

RViZ selection tool - points not in FixedFrame

2. http://answers.ros.org/question/99796/rviz-storing-point-cloud-selection/

3. http://answers.ros.org/question/12806/interactively-access-to-laser-point-in-rviz/

4. http://answers.ros.org/question/173002/using-selection-in-rviz-plugin/

5. http://docs.ros.org/jade/api/rviz/html/user_guide/

6. https://github.com/ros-visualization/rviz/blob/0d423d6249a39f7f3ddd591007952c6dc940b6ac/src/rviz/default_plugin/point_cloud_common.cpp
