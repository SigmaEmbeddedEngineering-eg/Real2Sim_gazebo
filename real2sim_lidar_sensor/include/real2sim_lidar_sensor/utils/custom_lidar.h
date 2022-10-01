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

#ifndef CUSTOM_LIDAR_HH
#define CUSTOM_LIDAR_HH

#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "utils/dataContainers.h"

class CustomLidar {
   public:
    /// \brief Constructor
    CustomLidar();

    /// \brief Destructor
    ~CustomLidar();

    /// \brief initialize from first scan
    void init_lidar_params();

    /// \brief Lidar parameters data container
    LidarParameters LP;

    /// \brief create a frame
    ErrorCode construct_lidar_frame(std::vector<float> const& ranges, cv::Mat const& cam_feed_,
                                    std::vector<ObjectPolygon> const& bb_vertcies);

    /// \brief color a point cloud
    ErrorCode get_colored_pc(PointCloud& pcl_msg);

    /// \brief color a point cloud
    ErrorCode get_intensity_pc(PointCloud_I& pcl_msg);

    /// \brief color a point cloud
    ErrorCode color_point(cv::Mat const& cam_feed, int const& azimuthIdx, int const& elevationIdx);

    /// \brief mounting position point compensation
    ErrorCode mount_position_compensation(int const& azimuthIdx, int const& elevationIdx);

   private:
    ErrorCode arange(double const& minVal, double const& maxVal, double const& resolution, std::vector<double>& vec);
    std::unique_ptr<SGM> sgm_ptr;
    PointCloud::Ptr colored_pcl_msg_ptr;
    PointCloud_I::Ptr intensity_pcl_msg_ptr;
    cv::dnn::Net net;
};
#endif