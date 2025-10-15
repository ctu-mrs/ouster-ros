/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_tf_publisher.h
 * @brief ...
 */

#pragma once

#include <tf2_ros/static_transform_broadcaster.h>

namespace ouster_ros {

template <typename NodeT>
class OusterStaticTransformsBroadcaster {
   public:
    OusterStaticTransformsBroadcaster(NodeT* parent)
        : node(parent), tf_bcast(parent) {}

    void declare_parameters() {
        node->declare_parameter("sensor_frame", "os_sensor");
        node->declare_parameter("lidar_frame", "os_lidar");
        node->declare_parameter("imu_frame", "os_imu");
        node->declare_parameter("point_cloud_frame", "");
        node->declare_parameter("pub_static_tf", true);
        node->declare_parameter("use_namespace_as_frame_id_prefix", false);
        node->declare_parameter("frame_id_prefix", "");
    }

    void parse_parameters() {
        sensor_frame = node->get_parameter("sensor_frame").as_string();
        lidar_frame = node->get_parameter("lidar_frame").as_string();
        imu_frame = node->get_parameter("imu_frame").as_string();
        point_cloud_frame =
            node->get_parameter("point_cloud_frame").as_string();
        pub_static_tf = node->get_parameter("pub_static_tf").as_bool();

        // validate point_cloud_frame
        if (point_cloud_frame.empty()) {
            point_cloud_frame =
                lidar_frame;  // for ROS1 we'd still use sensor_frame
        } else if (point_cloud_frame != sensor_frame &&
                   point_cloud_frame != lidar_frame) {
            RCLCPP_WARN(node->get_logger(),
                        "point_cloud_frame value needs to match the value of "
                        "either sensor_frame or lidar_frame but a different "
                        "value was supplied, using lidar_frame's value as the "
                        "value for point_cloud_frame");
            point_cloud_frame = lidar_frame;
        }

        std::string frame_id_prefix = node->get_parameter("frame_id_prefix").as_string();
        bool use_namespace_as_frame_id_prefix = node->get_parameter("use_namespace_as_frame_id_prefix").as_bool();

        if(!frame_id_prefix.empty()){
            RCLCPP_INFO(node->get_logger(), "frame id prefix taken from 'frame_id_prefix' param: %s", (frame_id_prefix).c_str());
            sensor_frame = frame_id_prefix + "/" + sensor_frame;
            lidar_frame = frame_id_prefix + "/" + lidar_frame;
            imu_frame = frame_id_prefix + "/" + imu_frame;
            point_cloud_frame = frame_id_prefix + "/" + point_cloud_frame;
        }
        // else if(use_namespace_as_frame_id_prefix){
        //     RCLCPP_INFO(node->get_logger(), "frame id prefix taken from node namespace: %s", node->get_namespace());
        //     sensor_frame = std::string(node->get_namespace()) + "/" + sensor_frame;
        //     lidar_frame = std::string(node->get_namespace()) + "/" + lidar_frame;
        //     imu_frame = std::string(node->get_namespace()) + "/" + imu_frame;
        //     point_cloud_frame = std::string(node->get_namespace()) + "/" + point_cloud_frame;
        // }
        // else RCLCPP_WARN(node->get_logger(), "'frame_id_prefix' param is empty and 'use_namespace_as_frame_id_prefix' is false. Frame ids will be without prefix");
    }

    void broadcast_transforms(const sensor::sensor_info& info) {
        auto now = node->get_clock()->now();
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame, now));
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.imu_to_sensor_transform, sensor_frame, imu_frame, now));
    }

    const std::string& imu_frame_id() const { return imu_frame; }
    const std::string& lidar_frame_id() const { return lidar_frame; }
    const std::string& sensor_frame_id() const { return sensor_frame; }

    const std::string& point_cloud_frame_id() const {
        return point_cloud_frame;
    }
    bool apply_lidar_to_sensor_transform() const {
        return point_cloud_frame == sensor_frame;
    }
    bool publish_static_tf() const { return pub_static_tf; }

   private:
    NodeT* node;
    tf2_ros::StaticTransformBroadcaster tf_bcast;
    std::string imu_frame;
    std::string lidar_frame;
    std::string sensor_frame;
    std::string point_cloud_frame;
    bool pub_static_tf;
};

}  // namespace ouster_ros
