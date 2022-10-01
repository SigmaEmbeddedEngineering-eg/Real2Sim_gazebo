/*
 * Desc: Real2Sim LiDAR sensor ROS Gazebo plugin.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 22 October 2022
 *
 * Copyright 2022 sigma embedded engineering-eg. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GAZEBO_ROS_LIDAR_HH
#define GAZEBO_ROS_LIDAR_HH

#include <cv_bridge/cv_bridge.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_plugins/PubQueue.h>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <image_transport/image_transport.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <ctime>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <mutex>  // std::mutex
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sdf/Param.hh>
#include <string>

#include "LinearMath/btMatrix3x3.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "utils/boundingbox_ground_truth.h"
#include "utils/custom_lidar.h"

namespace gazebo {

class RealLiDARSensor : public RayPlugin {
    /// \brief Constructor
   public:
    RealLiDARSensor();

    /// \brief Destructor
   public:
    virtual ~RealLiDARSensor();

    /// \brief Load the plugin
    /// \param take in SDF root element
   public:
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Keep track of number of connctions
   private:
    int laser_connect_count_;

   private:
    void LaserConnect();

   private:
    void LaserDisconnect();

    // Pointer to the model
    GazeboRosPtr gazebo_ros_;

   private:
    std::string world_name_;

   private:
    physics::WorldPtr world_;
    /// \brief The parent sensor
   private:
    sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
   private:
    ros::NodeHandle *rosnode_;

   private:
    ros::Publisher pcl_pub_;
    ros::Publisher pcl_pub_I_;

   private:
    PubQueue<PointCloud>::Ptr pcl_pub_queue_;

    /// \brief topic name
   private:
    std::string topic_name_;

    /// \brief frame transform name, should match link name
   private:
    std::string frame_name_;

    /// \brief tf prefix
   private:
    std::string tf_prefix_;

    /// \brief for setting ROS name space
   private:
    std::string robot_namespace_;

    // deferred load in case ros is blocking
   private:
    sdf::ElementPtr sdf;

   private:
    void LoadThread();

   private:
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    void model_states_callback(gazebo_msgs::ModelStates const &model_states);

   private:
    boost::thread deferred_load_thread_;

   private:
    gazebo::transport::NodePtr gazebo_node_;

   private:
    gazebo::transport::SubscriberPtr laser_scan_sub_;

   private:
    void OnScan(ConstLaserScanStampedPtr &_msg);

   private:
    PubMultiQueue pmq;
    ros::Publisher marker_pub;
    std::string sensor_id;

    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber model_states_subscriber_;
    cv::Mat cam_feed;
    int gt_rate;

    CustomLidar custom_lidar_;
    BoundingBoxGroundTruth bb_gt;
    std::vector<ObjectPolygon> bb_vertcies;

    std::mutex mtx;
    std::mutex gt_mtx;
    std::string cam_topic_name;
};

}  // namespace gazebo

#endif
