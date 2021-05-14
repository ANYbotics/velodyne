/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS nodelet transforms raw Velodyne 3D LIDAR packets to a
    PointCloud2 in the /map frame.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "any_velodyne_pointcloud/transform.h"

namespace velodyne_pointcloud
{
  class TransformNodelet: public nodelet::Nodelet
  {
  public:

    TransformNodelet() {}
    ~TransformNodelet() {
        // Support that nodelets are shut down smoothly. Explicit tear down of ROS infrastructure 
        // ensures that nodelet threads leave ROS-time-dependent sleeps.
        // Request shutdown of the ROS node.
        ros::requestShutdown();
        // Shut down ROS time.
        ros::Time::shutdown();
    }

  private:

    virtual void onInit();
    boost::shared_ptr<Transform> tf_;
  };

  /** @brief Nodelet initialization. */
  void TransformNodelet::onInit()
  {
    tf_.reset(new Transform(getNodeHandle(), getPrivateNodeHandle(), getName()));
  }

} // namespace velodyne_pointcloud


// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_pointcloud::TransformNodelet, nodelet::Nodelet)
