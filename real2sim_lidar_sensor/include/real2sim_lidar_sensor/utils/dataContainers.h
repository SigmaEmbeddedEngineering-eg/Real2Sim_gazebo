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

#ifndef DATA_CONTAINER_HH
#define DATA_CONTAINER_HH

#include <geometry_msgs/Vector3.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud_I;
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloud;

class mountingPositions {
   public:
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    void Set(std::string sensor_pose) {
        std::stringstream ss(sensor_pose);
        ss >> this->x;
        ss >> this->y;
        ss >> this->z;
        ss >> this->roll;
        ss >> this->pitch;
        ss >> this->yaw;
    };
};

class LidarParameters {
   public:
    float min_vertical_angle;
    float max_vertical_angle;
    float vertical_step_angle;

    float min_horizontal_angle;
    float max_horizontal_angle;
    float horizontal_step_angle;

    float horizontal_points_count;
    float vertical_points_count;

    bool parameters_initialized;

    float cam_fov;
    bool enable_fusion;

    std::string DnnPath;
    float numOfClasses;
    float maxSensorTrainingRange;

    float gndThreshold;
    bool gndSegMorphological;

    mountingPositions mp;

    std::vector<double> azimuthVec, elevationVec;
};

class ObjectPolygon {
   public:
    cv::Point2f v[4];
    Eigen::Vector4f v_min;
    Eigen::Vector4f v_max;
    int classification;

    float get_min_x() {
        float min_x = this->v[0].x;
        for (int vertexidx = 0; vertexidx < 4; vertexidx++)
            if (min_x > this->v[vertexidx].x) min_x = this->v[vertexidx].x;
        return min_x;
    }

    float get_max_x() {
        float max_x = this->v[0].x;
        for (int vertexidx = 0; vertexidx < 4; vertexidx++)
            if (max_x < this->v[vertexidx].x) max_x = this->v[vertexidx].x;
        return max_x;
    }

    float get_min_y() {
        float min_y = this->v[0].y;
        for (int vertexidx = 0; vertexidx < 4; vertexidx++)
            if (min_y > this->v[vertexidx].y) min_y = this->v[vertexidx].y;
        return min_y;
    }

    float get_max_y() {
        float max_y = this->v[0].y;
        for (int vertexidx = 0; vertexidx < 4; vertexidx++)
            if (max_y < this->v[vertexidx].y) max_y = this->v[vertexidx].y;
        return max_y;
    }
};

class GTOpponentsInfo {
   public:
    visualization_msgs::MarkerArray ground_truth;
    visualization_msgs::Marker ego_gt;
    std::vector<ObjectPolygon> opponent_vertcies;

    std::map<std::string, int> gt_classification;
    std::map<std::string, geometry_msgs::Vector3> gt_scale;
    std::map<std::string, std_msgs::ColorRGBA> classification_color;
    GTOpponentsInfo() {
        gt_scale["opponent_car_hatchback_blue"] = construct_scale(4, 2.5, 2.5);
        classification_color["opponent_car_hatchback_blue"] = construct_color(0, 1, 0, 0.5);

        gt_scale["opponent_car_hatchback_red"] = construct_scale(4, 2.5, 2.5);
        classification_color["opponent_car_hatchback_red"] = construct_color(0, 1, 0, 0.5);

        gt_scale["opponent_car_hatchback_grey"] = construct_scale(4, 2.5, 2.5);
        classification_color["opponent_car_hatchback_grey"] = construct_color(0, 1, 0, 0.5);

        gt_scale["ego_car"] = construct_scale(1, 1, 1);
        classification_color["ego"] = construct_color(1, 1, 1, 1);

        gt_classification["car"] = 1;
        gt_classification["cyclist"] = 2;
        gt_classification["pedestrian"] = 3;
    }

   private:
    geometry_msgs::Vector3 construct_scale(float x, float y, float z) {
        geometry_msgs::Vector3 scale;
        scale.x = x;
        scale.y = y;
        scale.z = z;
        return scale;
    }
    std_msgs::ColorRGBA construct_color(float r, float g, float b, float a) {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }
};

class SGM {
   public:
    cv::Mat x, y, z, r, g, b, d, i, a;
    int height, width;
    SGM(int height, int width) {
        this->x = cv::Mat(height, width, CV_32F);
        this->y = cv::Mat(height, width, CV_32F);
        this->z = cv::Mat(height, width, CV_32F);
        this->r = cv::Mat(height, width, CV_32F);
        this->g = cv::Mat(height, width, CV_32F);
        this->b = cv::Mat(height, width, CV_32F);
        this->d = cv::Mat(height, width, CV_32F);
        this->i = cv::Mat(height, width, CV_32F);
        this->a = cv::Mat(height, width, CV_32F);
        this->height = height;
        this->width = width;
    }

    cv::Mat spherical_grid_map_slopes() {
        // slope up direction
        int rows = this->x.rows;
        int cols = this->x.cols;
        cv::Mat deltaX = cv::Mat::zeros(this->x.rows, this->x.cols, CV_32F);

        cv::absdiff(this->x(cv::Range(1, rows), cv::Range::all()), this->x(cv::Range(0, rows - 1), cv::Range::all()),
                    deltaX(cv::Range(1, rows), cv::Range::all()));
        cv::pow(deltaX, 2, deltaX);

        cv::Mat deltaY = cv::Mat::zeros(this->x.rows, this->x.cols, CV_32F);
        cv::absdiff(this->y(cv::Range(1, rows), cv::Range::all()), this->y(cv::Range(0, rows - 1), cv::Range::all()),
                    deltaY(cv::Range(1, rows), cv::Range::all()));
        cv::pow(deltaY, 2, deltaY);
        cv::Mat deltaXY = cv::Mat::zeros(this->x.rows, this->x.cols, CV_32F);
        deltaXY = deltaX + deltaY;
        cv::sqrt(deltaXY, deltaXY);

        cv::Mat deltaZ = cv::Mat::zeros(this->x.rows, this->x.cols, CV_32F);
        cv::absdiff(this->z(cv::Range(1, rows), cv::Range::all()), this->z(cv::Range(0, rows - 1), cv::Range::all()),
                    deltaZ(cv::Range(1, rows), cv::Range::all()));
        // cv::pow(deltaZ, 2, deltaZ);
        // cv::sqrt(deltaZ, deltaZ);

        cv::Mat slope = deltaZ / deltaXY;
        return slope;
    }

    cv::Mat ground_segmentation(float const& threshold, bool const& use_morphological_filter) {
        cv::Mat ground_label;
        cv::Mat slope = this->spherical_grid_map_slopes();
        // taking threshold for the ground vectors
        cv::threshold(slope, ground_label, threshold, 1, 0);
        if (use_morphological_filter) {
            cv::Mat element = cv::Mat::ones(1, 1, CV_8U);
            cv::erode(ground_label, ground_label, element, cv::Point(-1, -1));
            cv::dilate(ground_label, ground_label, element, cv::Point(-1, -1));
        }

        return ground_label;
    }
};

enum ErrorCode { Success, Failed, Unknown, BadArgument };

#endif
