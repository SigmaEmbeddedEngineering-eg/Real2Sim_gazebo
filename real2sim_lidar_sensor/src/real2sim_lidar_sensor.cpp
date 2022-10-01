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

#include <assert.h>
#include <gazebo_plugins/gazebo_ros_laser.h>
#include <real2sim_lidar_sensor/real2sim_lidar_sensor.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Rand.hh>
#include <sdf/Param.hh>
#include <sdf/sdf.hh>
#include <string>

namespace gazebo {
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(RealLiDARSensor);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RealLiDARSensor::RealLiDARSensor() : bb_gt() {}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RealLiDARSensor::~RealLiDARSensor() {
    this->rosnode_->shutdown();
    delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////

// Load the plugin
void RealLiDARSensor::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    // load plugin
    RayPlugin::Load(_parent, this->sdf);
    // Get the world name.

    std::string worldName = _parent->WorldName();
    this->world_ = physics::get_world(worldName);
    // save pointers
    this->sdf = _sdf;

    GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
    this->parent_ray_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(_parent);

    if (!this->parent_ray_sensor_)
        ROS_INFO_NAMED("LiDAR",
                       "RealLiDARSensor controller "
                       "requires a Ray Sensor as its parent");

    this->robot_namespace_ = GetRobotNamespace(_parent, _sdf, "Laser");

    if (!this->sdf->HasElement("frameName")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <frameName>, defaults to /world");
        this->frame_name_ = "/world";
    } else
        this->frame_name_ = this->sdf->Get<std::string>("frameName");

    if (!this->sdf->HasElement("topicName")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <topicName>, defaults to /world");
        this->topic_name_ = "/world";
    } else
        this->topic_name_ = this->sdf->Get<std::string>("topicName");

    // Get sensor position
    std::string sensor_pose;
    if (!this->sdf->HasElement("pose")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <pose>, defaults to <0, 0, 0, 0, 0, 0>");
        sensor_pose = "0 0 0 0 0 0";
    } else
        sensor_pose = this->sdf->Get<std::string>("pose");
    this->custom_lidar_.LP.mp.Set(sensor_pose);
    ROS_INFO_NAMED("LiDAR", "x:%f, y:%f,z:%f,roll:%f,pitch:%f,yaw:%f", this->custom_lidar_.LP.mp.x,
                   this->custom_lidar_.LP.mp.y, this->custom_lidar_.LP.mp.z, this->custom_lidar_.LP.mp.roll,
                   this->custom_lidar_.LP.mp.pitch, this->custom_lidar_.LP.mp.yaw);

    if (!this->sdf->HasElement("cam_topic_name")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <cam_topic_name>, fusion disabled");
        this->cam_topic_name = "";
        this->custom_lidar_.LP.enable_fusion = false;
    } else {
        this->cam_topic_name = this->sdf->Get<std::string>("cam_topic_name");
        this->custom_lidar_.LP.enable_fusion = true;
    }

    if (!this->sdf->HasElement("cam_fov")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <cam_fov>, defaults to 0.0001");
        this->custom_lidar_.LP.cam_fov = 0.0001;
    } else {
        std::string s_cam_fov = this->sdf->Get<std::string>("cam_fov");
        this->custom_lidar_.LP.cam_fov = std::stof(s_cam_fov);
    }

    if (!this->sdf->HasElement("gndThreshold")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <gndThreshold>, defaults to 0.11");
        this->custom_lidar_.LP.gndThreshold = 0.11;
    } else {
        std::string s_gndThreshold = this->sdf->Get<std::string>("gndThreshold");
        this->custom_lidar_.LP.gndThreshold = std::stof(s_gndThreshold);
    }

    if (!this->sdf->HasElement("gndSegMorphological")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <gndSegMorphological>, defaults to 1");
        this->custom_lidar_.LP.gndSegMorphological = true;
    } else {
        std::string s_gndSegMorphological = this->sdf->Get<std::string>("gndSegMorphological");
        this->custom_lidar_.LP.gndSegMorphological = s_gndSegMorphological != "0";
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <gndSegMorphological>, to %d",this->custom_lidar_.LP.gndSegMorphological);
    }

    if (!this->sdf->HasElement("numOfClasses")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <numOfClasses>, defaults to 4");
        this->custom_lidar_.LP.numOfClasses = 4;
    } else {
        std::string s_numOfClasses = this->sdf->Get<std::string>("numOfClasses");
        this->custom_lidar_.LP.numOfClasses = std::stof(s_numOfClasses);
    }

    if (!this->sdf->HasElement("maxSensorTrainingRange")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <maxSensorTrainingRange>, defaults to 80");
        this->custom_lidar_.LP.maxSensorTrainingRange = 80;
    } else {
        std::string s_maxSensorTrainingRange = this->sdf->Get<std::string>("maxSensorTrainingRange");
        this->custom_lidar_.LP.maxSensorTrainingRange = std::stof(s_maxSensorTrainingRange);
    }

    if (!this->sdf->HasElement("DnnPath")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <DnnPath>, default");
        this->custom_lidar_.LP.DnnPath = "/home/khalid/catkin_ws/src/Real2Sim_gazebo/real2sim_lidar_sensor/network/frozen_graph_Unet.pb";
    } else {
        std::string s_DnnPath = this->sdf->Get<std::string>("DnnPath");
        this->custom_lidar_.LP.DnnPath = s_DnnPath;
    }

    if (!this->sdf->HasElement("sensorId")) {
        ROS_INFO_NAMED("LiDAR", "Laser plugin missing <sensorId>, defaults to 0");
        this->sensor_id = "0";
    } else {
        this->sensor_id = this->sdf->Get<std::string>("sensorId");
    }

    this->laser_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM_NAMED("LiDAR",
                               "A ROS node for Gazebo has not been initialized, unable to load "
                               "plugin. "
                                   << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
                                      "the gazebo_ros package)");
        return;
    }

    ROS_INFO_NAMED("LiDAR", "Starting Laser Plugin (ns = %s)", this->robot_namespace_.c_str());
    // ros callback queue for processing subscription
    this->deferred_load_thread_ = boost::thread(boost::bind(&RealLiDARSensor::LoadThread, this));
}
////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RealLiDARSensor::LoadThread() {
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);

    this->pmq.startServiceThread();

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
    if (this->tf_prefix_.empty()) {
        this->tf_prefix_ = this->robot_namespace_;
        boost::trim_right_if(this->tf_prefix_, boost::is_any_of("/"));
    }
    ROS_INFO_NAMED("LiDAR", "Laser Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"", this->robot_namespace_.c_str(),
                   this->tf_prefix_.c_str());

    // resolve tf prefix
    this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

    if (this->topic_name_ != "") {
        ros::AdvertiseOptions pcl_ao = ros::AdvertiseOptions::create<PointCloud>(
            "LiDAR" + this->sensor_id + "/pcl", 1000, boost::bind(&RealLiDARSensor::LaserConnect, this),
            boost::bind(&RealLiDARSensor::LaserDisconnect, this), ros::VoidPtr(), NULL);
        this->pcl_pub_ = this->rosnode_->advertise(pcl_ao);
        this->pcl_pub_queue_ = this->pmq.addPub<PointCloud>();
    }

    // sensor generation off by default
    this->parent_ray_sensor_->SetActive(false);

    this->it_ = new image_transport::ImageTransport(*this->rosnode_);
    // Subscrive to input video feed and publish output video feed
    image_sub_ = this->it_->subscribe(this->cam_topic_name, 1, &RealLiDARSensor::imageCb, this);
    model_states_subscriber_ =
        this->rosnode_->subscribe("/gazebo/model_states", 10, &RealLiDARSensor::model_states_callback, this);

    this->marker_pub = this->rosnode_->advertise<visualization_msgs::MarkerArray>("Ground_Truth", 1);
    this->pcl_pub_I_ = this->rosnode_->advertise<PointCloud_I>("LiDAR" + this->sensor_id + "/pcl_I", 1);
    gt_rate = 0;
}

////////////////////////////////////////////////////////////////////////////////
// model states callback
void RealLiDARSensor::model_states_callback(gazebo_msgs::ModelStates const& model_states) {
    if (gt_rate % 100 == 0) {
        bb_gt.modelstates_to_groundtruth(model_states, this->frame_name_);
        visualization_msgs::MarkerArray groundTruthBoundingboxes;
        bb_gt.get_gt_bounded_boxes(groundTruthBoundingboxes);
        gt_mtx.lock();
        bb_gt.get_gt_vertcies(bb_vertcies);
        gt_mtx.unlock();
        this->marker_pub.publish(groundTruthBoundingboxes);
    }
    gt_rate++;
}

////////////////////////////////////////////////////////////////////////////////
// Camera callback
void RealLiDARSensor::imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        mtx.lock();
        this->cam_feed = cv_ptr->image.clone();
        mtx.unlock();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}
////////////////////////////////////////////////////////////////////////////////
// Increment count
void RealLiDARSensor::LaserConnect() {
    this->laser_connect_count_++;
    if (this->laser_connect_count_ == 1)
        this->laser_scan_sub_ =
            this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &RealLiDARSensor::OnScan, this);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void RealLiDARSensor::LaserDisconnect() {
    this->laser_connect_count_--;
    if (this->laser_connect_count_ == 0) this->laser_scan_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void RealLiDARSensor::OnScan(ConstLaserScanStampedPtr& _msg) {
    this->custom_lidar_.LP.min_horizontal_angle = _msg->scan().angle_min();
    this->custom_lidar_.LP.max_horizontal_angle = _msg->scan().angle_max();
    this->custom_lidar_.LP.horizontal_step_angle = _msg->scan().angle_step();

    this->custom_lidar_.LP.min_vertical_angle = _msg->scan().vertical_angle_min();
    this->custom_lidar_.LP.max_vertical_angle = _msg->scan().vertical_angle_max();
    this->custom_lidar_.LP.vertical_step_angle = _msg->scan().vertical_angle_step();

    this->custom_lidar_.LP.vertical_points_count = _msg->scan().vertical_count();
    this->custom_lidar_.LP.horizontal_points_count = _msg->scan().count();

    if (!this->custom_lidar_.LP.parameters_initialized) {
        this->custom_lidar_.init_lidar_params();
    }
    // start the clock
    clock_t begin = clock();

    // copy the ranges of distances from msg data containers, to a vector.
    std::vector<float> ranges;
    ranges.resize(_msg->scan().ranges_size());
    std::copy(_msg->scan().ranges().begin(), _msg->scan().ranges().end(), ranges.begin());

    mtx.lock();
    cv::Mat cam_feed_ = this->cam_feed.clone();
    mtx.unlock();

    // Create a PCL_msg
    PointCloud pcl_msg;
    pcl_msg.header.frame_id = this->frame_name_;
    pcl_conversions::toPCL(ros::Time::now(), pcl_msg.header.stamp);
    pcl_msg.height = this->custom_lidar_.LP.vertical_points_count;
    pcl_msg.width = this->custom_lidar_.LP.horizontal_points_count;

    gt_mtx.lock();
    std::vector<ObjectPolygon> vertcies = bb_vertcies;
    gt_mtx.unlock();

    (void)this->custom_lidar_.construct_lidar_frame(ranges, cam_feed_, vertcies);
    (void)this->custom_lidar_.get_colored_pc(pcl_msg);

    this->pcl_pub_queue_->push(pcl_msg, this->pcl_pub_);

    // Create a PCL_msg
    PointCloud_I pcl_msg_I;
    pcl_msg_I.header.frame_id = this->frame_name_;
    pcl_conversions::toPCL(ros::Time::now(), pcl_msg_I.header.stamp);
    pcl_msg_I.height = this->custom_lidar_.LP.vertical_points_count;
    pcl_msg_I.width = this->custom_lidar_.LP.horizontal_points_count;
    (void)this->custom_lidar_.get_intensity_pc(pcl_msg_I);
    this->pcl_pub_I_.publish(pcl_msg_I);

    // end the clock
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed_time: " << elapsed_secs << " sec, Freq:" << 1 / elapsed_secs << " Hz" << std::endl;
    std::cout << "******************************************************" << std::endl;
}
}  // namespace gazebo
