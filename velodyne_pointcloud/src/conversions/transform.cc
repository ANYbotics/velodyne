/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class transforms raw Velodyne 3D LIDAR packets to PointCloud2
    in the /map frame of reference.

    @author Jack O'Quin
    @author Jesse Vera
    @author Sebastian Pütz

*/

#include "any_velodyne_pointcloud/transform.h"

#include <any_velodyne_pointcloud/pointcloudXYZIRT.h>
#include <any_velodyne_pointcloud/organized_cloudXYZIRT.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name):
    tf_prefix_(tf::getPrefixParam(private_nh)),
    data_(new velodyne_rawdata::RawData),
    first_rcfg_call(true),
    diagnostics_(node, private_nh, node_name)
  {
    boost::optional<velodyne_pointcloud::Calibration> calibration = data_->setup(private_nh);
    if(calibration)
    {
      ROS_DEBUG_STREAM("Calibration file loaded.");
      config_.num_lasers = static_cast<uint16_t>(calibration.get().num_lasers);
    }
    else
    {
      ROS_ERROR_STREAM("Could not load calibration file!");
    }

    config_.target_frame = config_.fixed_frame = "velodyne";
    tf_ptr_ = boost::make_shared<tf::TransformListener>();

    if(config_.organize_cloud)
    {
      container_ptr = boost::shared_ptr<OrganizedCloudXYZIRT>(
          new OrganizedCloudXYZIRT(config_.max_range, config_.min_range, config_.target_frame, config_.fixed_frame,
                                  config_.num_lasers, data_->scansPerPacket(), tf_ptr_));
    }
    else
    {
      container_ptr = boost::shared_ptr<PointcloudXYZIRT>(
          new PointcloudXYZIRT(config_.max_range, config_.min_range,
                              config_.target_frame, config_.fixed_frame,
                              data_->scansPerPacket(), tf_ptr_));
    }

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1);

    srv_ = boost::make_shared<dynamic_reconfigure::Server<TransformNodeConfig>> (private_nh);
    dynamic_reconfigure::Server<TransformNodeConfig>::CallbackType f;
    f = boost::bind (&Transform::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets using transform filter
    velodyne_scan_.subscribe(node, "velodyne_packets", 10);
    tf_filter_ptr_ = boost::shared_ptr<tf::MessageFilter<any_velodyne_msgs::VelodyneScan> >(
            new tf::MessageFilter<any_velodyne_msgs::VelodyneScan>(velodyne_scan_, *tf_ptr_, config_.target_frame, 10));
    tf_filter_ptr_->registerCallback(boost::bind(&Transform::processScan, this, _1));
    private_nh.param<std::string>("fixed_frame", config_.fixed_frame, "odom");

    // Diagnostics
    diagnostics_.setHardwareID("Velodyne Transform");
    // Arbitrary frequencies since we don't know which RPM is used, and are only
    // concerned about monitoring the frequency.
    diag_min_freq_ = 2.0;
    diag_max_freq_ = 20.0;
    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("velodyne_points", diagnostics_,
                                          FrequencyStatusParam(&diag_min_freq_,
                                                               &diag_max_freq_,
                                                               0.1, 10),
                                          TimeStampStatusParam()));

  }
  
  void Transform::reconfigure_callback(TransformNodeConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Reconfigure request.");
    data_->setParameters(config.min_range, config.max_range,
                         config.view_direction, config.view_width);
    config_.fixed_frame = tf::resolve(tf_prefix_, config.fixed_frame);
    config_.target_frame = tf::resolve(tf_prefix_, config.target_frame);
    ROS_INFO_STREAM("Fixed frame ID now: " << config_.fixed_frame);
    ROS_INFO_STREAM("Target frame ID now: " << config_.target_frame);
    config_.min_range = config.min_range;
    config_.max_range = config.max_range;

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);

    if(first_rcfg_call || config.organize_cloud != config_.organize_cloud){
      first_rcfg_call = false;
      config_.organize_cloud = config.organize_cloud;
      if(config_.organize_cloud)
      {
        ROS_INFO_STREAM("Using the organized cloud format...");
        container_ptr = boost::shared_ptr<OrganizedCloudXYZIRT>(
            new OrganizedCloudXYZIRT(config_.max_range, config_.min_range,
                                    config_.target_frame, config_.fixed_frame,
                                    config_.num_lasers, data_->scansPerPacket()));
      }
      else
      {
        container_ptr = boost::shared_ptr<PointcloudXYZIRT>(
            new PointcloudXYZIRT(config_.max_range, config_.min_range,
                                config_.target_frame, config_.fixed_frame,
                                data_->scansPerPacket()));
      }
    }
    container_ptr->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);
  }

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const any_velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);

    // allocate a point cloud with same time and frame ID as raw data
    container_ptr->setup(scanMsg);

    const ros::Time referencePointCloudTime{scanMsg->packets[0].stamp};
    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      ROS_DEBUG_STREAM("Unpacking and transforming Lidar Data Packet Index: " << i);
      container_ptr->computeTransformation(scanMsg->packets[i].stamp, referencePointCloudTime, scanMsg->header.frame_id);
      data_->unpack(scanMsg->packets[i], *container_ptr,  referencePointCloudTime);
    }

    // reset transformation to de-skew packets.
    container_ptr->resetTransformation();

    // publish the accumulated cloud message
    output_.publish(container_ptr->finishCloud(referencePointCloudTime));

    diag_topic_->tick(referencePointCloudTime);
    diagnostics_.update();
  }

} // namespace velodyne_pointcloud
