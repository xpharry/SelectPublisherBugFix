#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"

#include "publish_selected_patch.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QVariant>

namespace publish_selected_patch
{
PublishSelectedPatch::PublishSelectedPatch()
{
  updateTopic();
}

PublishSelectedPatch::~PublishSelectedPatch()
{
}

void PublishSelectedPatch::updateTopic()
{
  // nh_.param("frame_id", tf_frame_, std::string("/base_link"));
  cloud_topic_ = "/selected_patch";
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>( cloud_topic_.c_str(), 1 );
  // ROS_INFO( "Publishing data on topic %s with frame_id %s.",
  //           nh_.resolveName (cloud_topic_).c_str (),
  //           tf_frame_.c_str() );
}

int PublishSelectedPatch::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  int flags = rviz::SelectionTool::processMouseEvent( event );

  // determine current selection mode
  if( event.alt() )
  {
    selecting_ = false;
  }
  else
  {
    if( event.leftDown() )
    {
      selecting_ = true;
      sel_start_x_ = event.x;
      sel_start_y_ = event.y;
    }
  }

  if( selecting_ )
  {
    if( event.leftUp() )
    {
      rviz::SelectionManager* selection_manager = context_->getSelectionManager();
      rviz::M_Picked selection = selection_manager->getSelection();
      rviz::PropertyTreeModel *model = selection_manager->getPropertyModel();

      std::vector<Ogre::Vector3> result_points;
      bool success = context_->getSelectionManager()->get3DPatch( event.viewport, sel_start_x_, sel_start_y_,
                        1, 1, true, result_points );

      int num_points = result_points.size();
      if( selection.empty() || num_points <= 0 )
      {
        return flags;
      }

      sensor_msgs::PointCloud2 selected_cloud;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl_clr_ptr->points.resize(num_points);
      pcl_clr_ptr->width = num_points;
      pcl_clr_ptr->height = 1;
      pcl_clr_ptr->header.frame_id = context_->getFixedFrame().toStdString();

      pcl::PointXYZRGB the_clr_point;
      for (int ipt = 0; ipt < num_points; ipt++) {
          QModelIndex child_index = model->index(ipt, 0);
          rviz::Property* child = model->getProp(child_index);
          rviz::VectorProperty* subchild = (rviz::VectorProperty*) child->childAt(0);
          Ogre::Vector3 vec = subchild->getVector();
          ROS_INFO("event: %d, %d", event.x, event.y);
          ROS_INFO("VEC: %f, %f, %f", vec.x, vec.y, vec.z);

          the_clr_point.x = result_points[ipt].x;
          the_clr_point.y = result_points[ipt].y;
          the_clr_point.z = result_points[ipt].z;
          the_clr_point.r = 0;
          the_clr_point.g = 0;
          the_clr_point.b = 0;
          pcl_clr_ptr->points[ipt] = the_clr_point;
      }

      pcl::toROSMsg(*pcl_clr_ptr, selected_cloud);

      selected_cloud.header.stamp = ros::Time::now();
      pub_.publish(selected_cloud);
    }
  }

  return flags;
}

} // end namespace publish_selected_patch

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( publish_selected_patch::PublishSelectedPatch, rviz::Tool )
