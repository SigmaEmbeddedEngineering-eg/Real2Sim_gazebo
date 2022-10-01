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
#include "utils/boundingbox_ground_truth.h"

BoundingBoxGroundTruth::BoundingBoxGroundTruth() : gt_info_() {}

BoundingBoxGroundTruth::~BoundingBoxGroundTruth() {}

ErrorCode BoundingBoxGroundTruth::modelstates_to_groundtruth(gazebo_msgs::ModelStates const& model_states,
                                                             std::string frame_name) {
    gt_info_.ground_truth = visualization_msgs::MarkerArray();
    gt_info_.ego_gt = visualization_msgs::Marker();
    gt_info_.opponent_vertcies.clear();
    for (int idx = 0; idx < model_states.name.size(); idx++) {
        std::string const model_name = model_states.name[idx];
        if (model_name.find("opponent") != std::string::npos) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_name;
            marker.header.stamp = ros::Time::now();
            if (model_name.find("car") != std::string::npos) {
                marker.ns = "car";
            }
            // TODO[]: add other classes.
            else {
                marker.ns = "";
            }
            marker.id = idx;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::MODIFY;
            marker.text = model_name;

            marker.pose.position = model_states.pose[idx].position;
            marker.pose.orientation = model_states.pose[idx].orientation;
            marker.scale = gt_info_.gt_scale[model_name];
            marker.color = gt_info_.classification_color[model_name];

            marker.lifetime = ros::Duration(0.1);
            gt_info_.ground_truth.markers.push_back(marker);
        }
        if (model_name.find("ego") != std::string::npos) {
            gt_info_.ego_gt.header.frame_id = frame_name;
            gt_info_.ego_gt.header.stamp = ros::Time::now();
            if (model_name.find("ego") != std::string::npos) {
                gt_info_.ego_gt.ns = "car";
            }
            // TODO[]: add other classes.
            else {
                gt_info_.ego_gt.ns = "";
            }
            gt_info_.ego_gt.id = idx;
            gt_info_.ego_gt.type = visualization_msgs::Marker::CUBE;
            gt_info_.ego_gt.action = visualization_msgs::Marker::MODIFY;
            gt_info_.ego_gt.text = model_name;

            gt_info_.ego_gt.pose.position = model_states.pose[idx].position;
            gt_info_.ego_gt.pose.orientation = model_states.pose[idx].orientation;
            gt_info_.ego_gt.scale = gt_info_.gt_scale[model_name];
            gt_info_.ego_gt.color = gt_info_.classification_color[model_name];

            gt_info_.ego_gt.lifetime = ros::Duration(0.1);
            gt_info_.ground_truth.markers.push_back(gt_info_.ego_gt);
        }
    }
    for (visualization_msgs::Marker& marker : gt_info_.ground_truth.markers) {
        tf::Quaternion opponent_quat;
        tf::Quaternion ego_quat;
        tf::quaternionMsgToTF(marker.pose.orientation, opponent_quat);
        tf::quaternionMsgToTF(gt_info_.ego_gt.pose.orientation, ego_quat);
        double opponent_roll, opponent_pitch, opponent_yaw;
        tf::Matrix3x3(opponent_quat).getRPY(opponent_roll, opponent_pitch, opponent_yaw);
        double ego_roll, ego_pitch, ego_yaw;
        tf::Matrix3x3(ego_quat).getRPY(ego_roll, ego_pitch, ego_yaw);
        geometry_msgs::Point position;
        position.x = (marker.pose.position.x - gt_info_.ego_gt.pose.position.x) * cos(ego_yaw) +
                     (marker.pose.position.y - gt_info_.ego_gt.pose.position.y) * sin(ego_yaw);
        position.y = (marker.pose.position.y - gt_info_.ego_gt.pose.position.y) * cos(ego_yaw) -
                     (marker.pose.position.x - gt_info_.ego_gt.pose.position.x) * sin(ego_yaw);
        position.z = marker.pose.position.z - gt_info_.ego_gt.pose.position.z + marker.scale.z / 2 - 0.1;
        marker.pose.position = position;
        tf::Quaternion new_opponent_quat;
        new_opponent_quat.setRPY(opponent_roll - ego_roll, opponent_pitch - ego_pitch, opponent_yaw - ego_yaw);
        geometry_msgs::Quaternion new_opponent_quat_msg;
        tf::quaternionTFToMsg(new_opponent_quat, new_opponent_quat_msg);
        marker.pose.orientation = new_opponent_quat_msg;
        // set the four vertcies here.
        ObjectPolygon poly;
        double yaw = opponent_yaw - ego_yaw;

        // get the min max of the box from them.
        poly.v[0].x = marker.pose.position.x + (marker.scale.x / 2) * cos(yaw) - (marker.scale.y / 2) * sin(yaw);
        poly.v[0].y = (marker.scale.x / 2) * sin(yaw) + marker.pose.position.y + (marker.scale.y / 2) * cos(yaw);

        poly.v[1].x = marker.pose.position.x + (marker.scale.x / 2) * cos(yaw) - (-marker.scale.y / 2) * sin(yaw);
        poly.v[1].y = (marker.scale.x / 2) * sin(yaw) + marker.pose.position.y + (-marker.scale.y / 2) * cos(yaw);

        poly.v[2].x = marker.pose.position.x + (-marker.scale.x / 2) * cos(yaw) - (-marker.scale.y / 2) * sin(yaw);
        poly.v[2].y = (-marker.scale.x / 2) * sin(yaw) + marker.pose.position.y + (-marker.scale.y / 2) * cos(yaw);

        poly.v[3].x = marker.pose.position.x + (-marker.scale.x / 2) * cos(yaw) - (marker.scale.y / 2) * sin(yaw);
        poly.v[3].y = (-marker.scale.x / 2) * sin(yaw) + marker.pose.position.y + (marker.scale.y / 2) * cos(yaw);

        poly.v_max = Eigen::Vector4f(poly.get_max_x(),poly.get_max_y() , marker.pose.position.z + marker.scale.z, 1.0f);

        poly.v_min = Eigen::Vector4f(poly.get_min_x(), poly.get_min_y(), marker.pose.position.z - marker.scale.z, 1.0f);
        poly.classification = gt_info_.gt_classification[marker.ns];

        gt_info_.opponent_vertcies.push_back(poly);
    }
    return ErrorCode::Success;
}

ErrorCode BoundingBoxGroundTruth::get_gt_bounded_boxes(visualization_msgs::MarkerArray& groundTruthBoundingboxes) {
    groundTruthBoundingboxes = gt_info_.ground_truth;
    return ErrorCode::Success;
}

ErrorCode BoundingBoxGroundTruth::get_gt_vertcies(std::vector<ObjectPolygon>& opponentsVertcies) {
    opponentsVertcies = gt_info_.opponent_vertcies;
    return ErrorCode::Success;
}
