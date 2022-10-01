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
#ifndef BOUNDINGBOX_GROUND_TRUTH_HH
#define BOUNDINGBOX_GROUND_TRUTH_HH

#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <string>

#include "tf/transform_datatypes.h"
#include "utils/dataContainers.h"

class BoundingBoxGroundTruth {
   public:
    /// \brief Constructor
    BoundingBoxGroundTruth();

    /// \brief Destructor
    ~BoundingBoxGroundTruth();

    /// \brief
    ErrorCode modelstates_to_groundtruth(gazebo_msgs::ModelStates const& model_states, std::string frame_name);

    /// \brief
    ErrorCode get_gt_bounded_boxes(visualization_msgs::MarkerArray& groundTruthBoundingboxes);

    /// \brief
    ErrorCode get_gt_vertcies(std::vector<ObjectPolygon>& opponentsVertcies);

   private:
    GTOpponentsInfo gt_info_;
};
#endif