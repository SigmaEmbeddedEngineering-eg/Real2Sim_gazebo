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

#include "utils/custom_lidar.h"

CustomLidar::CustomLidar() {
    this->LP.parameters_initialized = false;
}
CustomLidar::~CustomLidar() {}
void CustomLidar::init_lidar_params() {
    this->LP.parameters_initialized = true;
    (void)arange(this->LP.min_horizontal_angle, this->LP.max_horizontal_angle + this->LP.horizontal_step_angle,
                 this->LP.horizontal_step_angle, this->LP.azimuthVec);
    (void)arange(this->LP.min_vertical_angle, this->LP.max_vertical_angle + this->LP.vertical_step_angle,
                 this->LP.vertical_step_angle, this->LP.elevationVec);
    this->net = cv::dnn::readNet(this->LP.DnnPath);
}

ErrorCode CustomLidar::construct_lidar_frame(std::vector<float> const& ranges, cv::Mat const& cam_feed_,
                                             std::vector<ObjectPolygon> const& bb_vertcies) {
    if (ranges.empty()) {
        return ErrorCode::BadArgument;
    }
    colored_pcl_msg_ptr = PointCloud::Ptr(new PointCloud());
    intensity_pcl_msg_ptr = PointCloud_I::Ptr(new PointCloud_I());
    sgm_ptr = std::make_unique<SGM>(this->LP.vertical_points_count, this->LP.horizontal_points_count);
    for (int pointIdx = 0; pointIdx < ranges.size(); pointIdx++) {
        int azimuthIdx = pointIdx % static_cast<int>(this->LP.horizontal_points_count);
        int elevationIdx = pointIdx / static_cast<int>(this->LP.horizontal_points_count);
        float r{0};
        if (std::numeric_limits<float>::infinity() == ranges[pointIdx])
            r = 0;
        else
            r = ranges[pointIdx];

        // Polar to cartisian coordinates transformation from LIDAR frame of reference.
        sgm_ptr->x.at<float>(elevationIdx, azimuthIdx) =
            r * cos(this->LP.azimuthVec[azimuthIdx]) * cos(this->LP.elevationVec[elevationIdx]);
        sgm_ptr->y.at<float>(elevationIdx, azimuthIdx) =
            r * sin(this->LP.azimuthVec[azimuthIdx]) * cos(this->LP.elevationVec[elevationIdx]);
        sgm_ptr->z.at<float>(elevationIdx, azimuthIdx) = r * sin(this->LP.elevationVec[elevationIdx]);
        sgm_ptr->d.at<float>(elevationIdx, azimuthIdx) = r / this->LP.maxSensorTrainingRange;

        // Extract Point color from LIDAR frame of reference.
        (void)color_point(cam_feed_, azimuthIdx, elevationIdx);
        // Compensate the mounting position of the LiDAR sensor to the Frame of reference of the vehicle.
        (void)mount_position_compensation(azimuthIdx, elevationIdx);
        pcl::PointXYZRGBL colored_pt = pcl::PointXYZRGBL();
        colored_pt.x = sgm_ptr->x.at<float>(elevationIdx, azimuthIdx);
        colored_pt.y = sgm_ptr->y.at<float>(elevationIdx, azimuthIdx);
        colored_pt.z = sgm_ptr->z.at<float>(elevationIdx, azimuthIdx);
        colored_pt.r = sgm_ptr->r.at<float>(elevationIdx, azimuthIdx) * 255.0;
        colored_pt.g = sgm_ptr->g.at<float>(elevationIdx, azimuthIdx) * 255.0;
        colored_pt.b = sgm_ptr->b.at<float>(elevationIdx, azimuthIdx) * 255.0;
        colored_pt.label = 0;
        colored_pcl_msg_ptr->points.push_back(colored_pt);
        pcl::PointXYZI Intensity_pt = pcl::PointXYZI();
        Intensity_pt.x = colored_pt.x;
        Intensity_pt.y = colored_pt.y;
        Intensity_pt.z = colored_pt.z;
        Intensity_pt.intensity = 0;
        intensity_pcl_msg_ptr->points.push_back(Intensity_pt);
    }
    cv::Mat ground_label = sgm_ptr->ground_segmentation(this->LP.gndThreshold, this->LP.gndSegMorphological);

    // Apply Cropbox
    pcl::CropBox<pcl::PointXYZRGBL> cropBox(true);
    for (ObjectPolygon bb_vertex : bb_vertcies) {
        cropBox.setMin(bb_vertex.v_min);
        cropBox.setMax(bb_vertex.v_max);
        cropBox.setInputCloud(colored_pcl_msg_ptr);

        // Indices
        std::vector<int> indices;
        cropBox.filter(indices);

        for (int pointIdx : indices) {
            int azimuthIdx = pointIdx % static_cast<int>(this->LP.horizontal_points_count);
            int elevationIdx = pointIdx / static_cast<int>(this->LP.horizontal_points_count);

            if (ground_label.at<float>(elevationIdx, azimuthIdx)) {
                sgm_ptr->a.at<float>(elevationIdx, azimuthIdx) = bb_vertex.classification / this->LP.numOfClasses;
                colored_pcl_msg_ptr->points[pointIdx].label = bb_vertex.classification;
            }
        }
    }

    std::vector<cv::Mat> channels = {sgm_ptr->d, sgm_ptr->a, sgm_ptr->r, sgm_ptr->g, sgm_ptr->b};
    cv::Mat inputBlob = cv::dnn::blobFromImages(channels, 1.0,      // scale factor
                                                sgm_ptr->d.size(),  // spatial size for output image
                                                cv::Scalar(0),      // mean
                                                false,              // swapRB: BGR to RGB
                                                false,              // crop
                                                CV_32F              // Depth of output blob. Choose CV_32F or CV_8U.
    );
    inputBlob = inputBlob.reshape(1, std::vector<int>({1, 5, sgm_ptr->height, sgm_ptr->width}));
    inputBlob.setTo(0, inputBlob < 0);
    inputBlob.setTo(1, inputBlob > 1);
    cv::patchNaNs(inputBlob, 0.0);

    net.setInput(inputBlob);
    cv::Mat output = net.forward();

    std::vector<cv::Mat> outputVectors;
    cv::dnn::imagesFromBlob(output, outputVectors);

    for (int pointIdx = 0; pointIdx < intensity_pcl_msg_ptr->points.size(); pointIdx++) {
        int azimuthIdx = pointIdx % static_cast<int>(this->LP.horizontal_points_count);
        int elevationIdx = pointIdx / static_cast<int>(this->LP.horizontal_points_count);
        // Limiting of the intensity from 0-1.
        if (outputVectors[0].at<float>(elevationIdx, azimuthIdx) < 0)
            outputVectors[0].at<float>(elevationIdx, azimuthIdx) = 0;
        if (outputVectors[0].at<float>(elevationIdx, azimuthIdx) > 1)
            outputVectors[0].at<float>(elevationIdx, azimuthIdx) = 1;
        intensity_pcl_msg_ptr->points[pointIdx].intensity = outputVectors[0].at<float>(elevationIdx, azimuthIdx);
    }

    return ErrorCode::Success;
}

ErrorCode CustomLidar::get_colored_pc(PointCloud& pcl_msg) {
    pcl_msg.points = colored_pcl_msg_ptr->points;
    return ErrorCode::Success;
}

ErrorCode CustomLidar::get_intensity_pc(PointCloud_I& pcl_msg) {
    pcl_msg.points = intensity_pcl_msg_ptr->points;
    return ErrorCode::Success;
}

ErrorCode CustomLidar::color_point(cv::Mat const& cam_feed, int const& azimuthIdx, int const& elevationIdx) {
    std::uint8_t R{0}, G{0}, B{0};
    if (this->LP.enable_fusion) {
        // Fuse camera with lidar point cloud.
        float focalLength = (cam_feed.cols / 2) / tan(this->LP.cam_fov / 2);
        float cx = cam_feed.cols / 2;
        float cy = cam_feed.rows / 2;

        // Camera x-axis is LiDAR -1*y-axis, camera y-axis is LiDAR -1*z-axis, and
        // camera z-axis(deoth) is LiDAR x_axis.
        float X_cam = -sgm_ptr->y.at<float>(elevationIdx, azimuthIdx);
        float Y_cam = -sgm_ptr->z.at<float>(elevationIdx, azimuthIdx);
        float Z_cam = sgm_ptr->x.at<float>(elevationIdx, azimuthIdx);
        X_cam = static_cast<int>(cx + X_cam * focalLength / Z_cam);
        Y_cam = static_cast<int>(cy + Y_cam * focalLength / Z_cam);
        if (Y_cam < cy * 2 - 1 && Y_cam > 0 && X_cam < cx * 2 - 1 && X_cam > 0 && Z_cam > 0) {
            sgm_ptr->b.at<float>(elevationIdx, azimuthIdx) = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[0] / 255.0;
            sgm_ptr->g.at<float>(elevationIdx, azimuthIdx) = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[1] / 255.0;
            sgm_ptr->r.at<float>(elevationIdx, azimuthIdx) = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[2] / 255.0;
        }
        return ErrorCode::Success;
    } else {
        return ErrorCode::Failed;
    }
}
ErrorCode CustomLidar::mount_position_compensation(int const& azimuthIdx, int const& elevationIdx) {
    float x = sgm_ptr->x.at<float>(elevationIdx, azimuthIdx) * cos(this->LP.mp.yaw) * cos(this->LP.mp.pitch) +
              sgm_ptr->y.at<float>(elevationIdx, azimuthIdx) *
                  (cos(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * sin(this->LP.mp.roll) -
                   sin(this->LP.mp.yaw) * cos(this->LP.mp.roll)) +
              sgm_ptr->z.at<float>(elevationIdx, azimuthIdx) *
                  (cos(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * cos(this->LP.mp.roll) +
                   sin(this->LP.mp.yaw) * sin(this->LP.mp.roll)) +
              this->LP.mp.x;
    float y = sgm_ptr->x.at<float>(elevationIdx, azimuthIdx) * sin(this->LP.mp.yaw) * cos(this->LP.mp.pitch) +
              sgm_ptr->y.at<float>(elevationIdx, azimuthIdx) *
                  (sin(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * sin(this->LP.mp.roll) +
                   cos(this->LP.mp.yaw) * cos(this->LP.mp.roll)) +
              sgm_ptr->z.at<float>(elevationIdx, azimuthIdx) *
                  (sin(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * cos(this->LP.mp.roll) -
                   cos(this->LP.mp.yaw) * sin(this->LP.mp.roll)) +
              this->LP.mp.y;
    float z = sgm_ptr->x.at<float>(elevationIdx, azimuthIdx) * (-sin(this->LP.mp.pitch)) +
              sgm_ptr->y.at<float>(elevationIdx, azimuthIdx) * cos(this->LP.mp.pitch) * sin(this->LP.mp.roll) +
              sgm_ptr->z.at<float>(elevationIdx, azimuthIdx) * cos(this->LP.mp.pitch) * cos(this->LP.mp.roll) +
              this->LP.mp.z;
    sgm_ptr->x.at<float>(elevationIdx, azimuthIdx) = x;
    sgm_ptr->y.at<float>(elevationIdx, azimuthIdx) = y;
    sgm_ptr->z.at<float>(elevationIdx, azimuthIdx) = z;
    return ErrorCode::Success;
}
ErrorCode CustomLidar::arange(double const& minVal, double const& maxVal, double const& resolution,
                              std::vector<double>& vec) {
    // validate input arguments.
    if (!vec.empty()) {
        return ErrorCode::BadArgument;
    }
    // We get a vector varying from min val to max val with a step resolution.
    double minimum = std::min(minVal, maxVal);
    double maximum = std::max(minVal, maxVal);
    for (double i = minimum; i < maximum; i += resolution) {
        vec.push_back(i);
    }
    return ErrorCode::Success;
}
