#include <OgreRay.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#include <OgreMovableObject.h>
#include <OgreRectangle2D.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreMaterialManager.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"

#include "publish_selected_patch.h"

#include "rviz/default_plugin/tools/move_tool.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QVariant>
#include <QKeyEvent>

namespace rviz
{

namespace publish_selected_patch
{

PublishSelectedPatch::PublishSelectedPatch()
  : Tool()
  , move_tool_( new MoveTool() )
  , selecting_( false )
  , sel_start_x_( 0 )
  , sel_start_y_( 0 )
  , moving_( false )
{
  shortcut_key_ = 's';
  access_all_keys_ = true;
}

PublishSelectedPatch::~PublishSelectedPatch()
{
  delete move_tool_;
}

void PublishSelectedPatch::onInitialize()
{
  move_tool_->initialize( context_ );
}

void PublishSelectedPatch::activate()
{
  setStatus( "Click and drag to select objects on the screen." );
  context_->getSelectionManager()->setTextureSize(512);
  selecting_ = false;
  moving_ = false;
//  context_->getSelectionManager()->enableInteraction(true);
}

void PublishSelectedPatch::deactivate()
{
  context_->getSelectionManager()->removeHighlight();
}

void PublishSelectedPatch::update(float wall_dt, float ros_dt)
{
  cloud_topic_ = "/selected_patch";
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>( cloud_topic_.c_str(), 1 );

  SelectionManager* sel_manager = context_->getSelectionManager();

  if (!selecting_)
  {
    sel_manager->removeHighlight();
  }
}

int PublishSelectedPatch::processMouseEvent( ViewportMouseEvent& event )
{
  SelectionManager* sel_manager = context_->getSelectionManager();

  int flags = 0;

  if( event.alt() )
  {
    moving_ = true;
    selecting_ = false;
  }
  else
  {
    moving_ = false;

    if( event.leftDown() )
    {
      selecting_ = true;

      sel_start_x_ = event.x;
      sel_start_y_ = event.y;
    }
  }

  if( selecting_ )
  {
    sel_manager->highlight( event.viewport, sel_start_x_, sel_start_y_, event.x, event.y );

    if( event.leftUp() )
    {
      SelectionManager::SelectType type = SelectionManager::Replace;

      if( event.shift() )
      {
        type = SelectionManager::Add;
      }
      else if( event.control() )
      {
        type = SelectionManager::Remove;
      }

      sel_manager->select( event.viewport, sel_start_x_, sel_start_y_, event.x, event.y, type );

      selecting_ = false;

      ///////////////////////////////////////////////////////////////////////
      rviz::SelectionManager* sel_manager = context_->getSelectionManager();
      rviz::M_Picked selection = sel_manager->getSelection();
      rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
      int num_points = model->rowCount();
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

          the_clr_point.x = vec.x;
          the_clr_point.y = vec.y;
          the_clr_point.z = vec.z;
          the_clr_point.r = 0;
          the_clr_point.g = 0;
          the_clr_point.b = 0;
          pcl_clr_ptr->points[ipt] = the_clr_point;
      }

      pcl::toROSMsg(*pcl_clr_ptr, selected_cloud);

      selected_cloud.header.stamp = ros::Time::now();
      pub_.publish(selected_cloud);
    }

    flags |= Render;
  }
  else if( moving_ )
  {
    sel_manager->removeHighlight();

    flags = move_tool_->processMouseEvent( event );

    if( event.type == QEvent::MouseButtonRelease )
    {
      moving_ = false;
    }
  }
  else
  {
    sel_manager->highlight( event.viewport, event.x, event.y, event.x, event.y );
  }

  return flags;
}

int PublishSelectedPatch::processKeyEvent( QKeyEvent* event, RenderPanel* panel )
{
  SelectionManager* sel_manager = context_->getSelectionManager();

  if( event->key() == Qt::Key_F )
  {
    sel_manager->focusOnSelection();
  }

  return Render;
}

} // end namespace publish_selected_patch

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::publish_selected_patch::PublishSelectedPatch, rviz::Tool )
