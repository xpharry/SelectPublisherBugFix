#include <OGRE/OgreRay.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreVector3.h>

#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"
#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/view_controller.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/view_manager.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/string_property.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

#include "rviz/default_plugin/tools/move_tool.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/filters/impl/box_clipper3D.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>

#include <sstream>
#include <vector>

#include "publish_selected_patch/publish_selected_patch.h"

namespace rviz {

    PublishSelectedPatch::PublishSelectedPatch()
        : Tool()
        , move_tool_(new MoveTool())
        , selecting_(false)
        , sel_start_x_(0)
        , sel_start_y_(0)
        , moving_(false) {

        ROS_INFO("ENTER PublishSelectedPatch");
        shortcut_key_ = 's';
        access_all_keys_ = true;

        topic_property_ = new StringProperty("Topic", "/publish_selected_patch",
                                             "The topic on which to publish points.",
                                             getPropertyContainer(), SLOT( updateTopic()), this);

        updateTopic();
    }

    PublishSelectedPatch::~PublishSelectedPatch() {
        move_tool_->initialize(context_);
    }

    void PublishSelectedPatch::onInitialize() {
        hit_cursor_ = cursor_;
        std_cursor_ = getDefaultCursor();
    }

    void PublishSelectedPatch::activate() {
        setStatus( "Click and drag to select objects on the screen." );
        context_->getSelectionManager()->setTextureSize(512);
        selecting_ = false;
        moving_ = false;
    }

    void PublishSelectedPatch::deactivate() {
        context_->getSelectionManager()->removeHighlight();
    }

    void PublishSelectedPatch::updateTopic() {
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_property_->getStdString(), 1);

        SelectionManager* selection_manager = context_->getSelectionManager();

        if(!selecting_) {
            selection_manager->removeHighlight();
        }
    }

    void PublishSelectedPatch::updateAutoDeactivate() {}

    int PublishSelectedPatch::processMouseEvent(ViewportMouseEvent& event) {

        SelectionManager* selection_manager = context_->getSelectionManager();

        int flags = 0;

        // determine current selection mode
        if(event.alt()) {
            moving_ = true;
            selecting_ = false;
        } else {
            moving_ = false;
            if(event.leftDown()) {
              selecting_ = true;
            }
        }

        if(selecting_) {
            selection_manager->highlight(event.viewport, sel_start_x_, sel_start_y_, event.x, event.y);
            if(event.leftUp()) {
                SelectionManager::SelectType type = SelectionManager::Replace;

                rviz::M_Picked selection = selection_manager->getSelection();
                rviz::PropertyTreeModel* model = selection_manager->getPropertyModel();

                int npts = model->rowCount();

                if(selection.empty() || npts <= 0) {
                    return flags;
                }

                sensor_msgs::PointCloud2 selected_cloud;

                if(event.shift()) {
                    type = SelectionManager::Add;
                } else if(event.control()) {
                    type = SelectionManager::Remove;
                }

                selection_manager->select(event.viewport, sel_start_x_, sel_start_y_, event.x, event.y, type);
                selecting_ = false;

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl_clr_ptr->points.resize(npts);
                pcl_clr_ptr->width = npts;
                pcl_clr_ptr->height = 1;
                pcl_clr_ptr->header.frame_id = context_->getFixedFrame().toStdString(); 
                if(npts == 0) return 0;

                pcl::PointXYZRGB the_clr_point;
                for (int ipt = 0; ipt < npts; ipt++) {
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
        else if( moving_ ) {
            selection_manager->removeHighlight();
            flags = move_tool_->processMouseEvent(event);
            if(event.type == QEvent::MouseButtonRelease) {
                moving_ = false;
            }
        } else {
            selection_manager->highlight(event.viewport, event.x, event.y, event.x, event.y);
        }
        return flags;
    }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::PublishSelectedPatch, rviz::Tool)
