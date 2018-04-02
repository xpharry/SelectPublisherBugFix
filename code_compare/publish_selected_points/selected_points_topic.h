#ifndef SELECTED_POINTS_TOPIC_H
#define SELECTED_POINTS_TOPIC_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/node_handle.h>
# include <ros/publisher.h>

# include "rviz/tool.h"

# include <QCursor>
# include <QObject>
#endif

#include "rviz/default_plugin/tools/selection_tool.h"

namespace rviz_plugin_selected_points_topic
{

class SelectedPointsTopic;

class SelectedPointsTopic : public rviz::SelectionTool
{
Q_OBJECT
public:
  SelectedPointsTopic();
  virtual ~SelectedPointsTopic();

  /*
   * Hooks on rviz::SelectionTool::processMouseEvent() to get and publish
   * selected points
   */
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

public Q_SLOTS:
  /*
   * Creates the ROS topic
   */
  void updateTopic();

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string tf_frame_;
  std::string cloud_topic_;
  bool selecting_;
};
} // end namespace rviz_plugin_selected_points_topic

#endif // SELECTED_POINTS_TOPIC_H

////////////////////////////////////////////////////////////////////////////////

