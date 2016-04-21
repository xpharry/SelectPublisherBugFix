#ifndef SELECTED_POINTS_PUBLISHER_H
#define SELECTED_POINTS_PUBLISHER_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/node_handle.h>
# include <ros/publisher.h>

# include "rviz/tool.h"

# include <QCursor>
# include <QObject>
#endif

#include "rviz/default_plugin/tools/selection_tool.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/extract_indices.h>

namespace rviz_plugin_selected_points_publisher
{

class SelectedPointsPublisher;

class SelectedPointsPublisher : public rviz::SelectionTool
{
Q_OBJECT
public:
  SelectedPointsPublisher();
  virtual ~SelectedPointsPublisher();

  /*
   * Hooks on rviz::SelectionTool::processMouseEvent() to get and publish
   * selected points
   */
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  virtual int processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel );

public Q_SLOTS:
  /*
   * Creates the ROS topic
   */
  void updateTopic();

  void PointCloudsCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);

protected:

  int _processSelectedAreaAndFindPoints();
  int _publishAccumulatedPoints();
  ros::NodeHandle nh_;
  ros::Publisher rviz_selected_pub_;
  ros::Publisher real_selected_pub_;
  ros::Publisher partial_pc_pub_;
  ros::Publisher bb_marker_pub_;
  ros::Subscriber pc_subs_;

  std::string tf_frame_;
  std::string rviz_cloud_topic_;
  std::string real_cloud_topic_;
  std::string subs_cloud_topic_;
  std::string bb_marker_topic_;
  bool selecting_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pc_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_segment_pc_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_segment_pc_;

  pcl::ExtractIndices<pcl::PointXYZRGB>::Ptr extract_indices_filter_;

  int num_acc_points_;
  int num_selected_points_;
};
} // end namespace rviz_plugin_selected_points_publisher

#endif // SELECTED_POINTS_PUBLISHER_H

////////////////////////////////////////////////////////////////////////////////

