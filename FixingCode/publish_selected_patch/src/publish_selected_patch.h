////////////////////////////////////////////////////////////////////////////////
//
// Filename:      publish_selected_patch.h
// Last change:   2013-11-21
// Authors:       Bartels, Philipp (mail@pBartels.net)
// Documentation: http://docs.ros.org/api/rviz/html/
// Version:       1.0.0
//
//////////////////////////////// DOCUMENTATION /////////////////////////////////
//
// Fork of the rviz::SelectionTool:
// Drag with the left button to select objects in the 3D scene.
// Hold the Alt key to change viewpoint as in the Move tool.
// Additionally publishes selected points on /selected_points topic.
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2013 Robotics & Biology Laboratory (RBO) TU Berlin
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////// CHANGELOG ///////////////////////////////////
//
// Version 1.0.0 (2013-11-21)
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
// TODO:
//
////////////////////////////////////////////////////////////////////////////////

#ifndef PUBLISH_SELECTED_PATCH_H
#define PUBLISH_SELECTED_PATCH_H

#include "rviz/tool.h"
#include "rviz/selection/forwards.h"

#include <vector>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <QCursor>
#include <QObject>
#endif

namespace Ogre
{
class Viewport;
}

namespace rviz
{

class MoveTool;

namespace publish_selected_patch
{

class PublishSelectedPatch;

class PublishSelectedPatch : public Tool
{
Q_OBJECT
public:
  PublishSelectedPatch();
  virtual ~PublishSelectedPatch();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( ViewportMouseEvent& event );
  virtual int processKeyEvent( QKeyEvent* event, RenderPanel* panel );

public Q_SLOTS:
  virtual void update(float wall_dt, float ros_dt);

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string tf_frame_;
  std::string cloud_topic_;

  MoveTool* move_tool_;

  bool selecting_;
  int sel_start_x_;
  int sel_start_y_;

  M_Picked highlight_;

  bool moving_;
};

}

}

#endif

