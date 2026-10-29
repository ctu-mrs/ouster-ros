/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Higher-level functions to read data from the ouster sensors as ROS
 * messages
 */

#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include <string>
#include <vector>

#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/point.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
using Cloud = pcl::PointCloud<Point>;
using ns = std::chrono::nanoseconds;

/**
 * An axis-aligned box of points to be discarded from the scans.
 */
struct ExclusionBox {
    Eigen::Vector3f min_pt;
    Eigen::Vector3f max_pt;
};

/**
 * Boxes of points to be discarded from the scans (typically the returns from the robot carrying the
 * sensor), together with the transformation from the sensor frame to the frame in which the boxes
 * are defined.
 *
 * The points have to be discarded before the scan is deskewed. Deskewing assumes a static world, so
 * it displaces everything that is static relative to the sensor - the robot's own body - by the
 * sensor's motion during the scan, after which such returns no longer lie within the boxes.
 *
 * The clouds are organized, so an excluded point cannot be dropped. It is zeroed out instead,
 * exactly like a point from a lost column: zero coordinates and zero range.
 */
struct ExclusionBoxes {
    std::vector<ExclusionBox> boxes;
    // transforms a point from the sensor frame to the frame in which the boxes are defined
    Eigen::Affine3f sensor_to_boxes_tf = Eigen::Affine3f::Identity();

    bool empty() const { return boxes.empty(); }

    bool excludes(const Eigen::Vector3f& point) const {
        const Eigen::Vector3f pt = sensor_to_boxes_tf * point;
        for (const auto& box : boxes)
            if ((pt.array() >= box.min_pt.array()).all() &&
                (pt.array() <= box.max_pt.array()).all())
                return true;
        return false;
    }
};

/**
 * Read an imu packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param[in] cli the sensor client
 * @param[out] pm the destination packet message
 * @param[in] pf the packet format
 * @return whether reading was successful
 */
bool read_imu_packet(const sensor::client& cli, PacketMsg& pm,
                     const sensor::packet_format& pf);

/**
 * Read a lidar packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param[in] cli the sensor client
 * @param[out] pm the destination packet message
 * @param[in] pf the packet format
 * @return whether reading was successful
 */
bool read_lidar_packet(const sensor::client& cli, PacketMsg& pm,
                       const sensor::packet_format& pf);

/**
 * Parse an imu packet message into a ROS imu message
 * @param[in] pm packet message populated by read_imu_packet
 * @param[in] timestamp the timestamp to give the resulting ROS message
 * @param[in] frame the frame to set in the resulting ROS message
 * @param[in] pf the packet format
 * @return ROS sensor message with fields populated from the packet
 */
sensor_msgs::Imu packet_to_imu_msg(const PacketMsg& pm,
                                   const ros::Time& timestamp,
                                   const std::string& frame,
                                   const sensor::packet_format& pf);

/**
 * Parse an imu packet message into a ROS imu message
 * @param[in] pm packet message populated by read_imu_packet
 * @param[in] frame the frame to set in the resulting ROS message
 * @param[in] pf the packet format
 * @return ROS sensor message with fields populated from the packet
 */
[[deprecated]] sensor_msgs::Imu packet_to_imu_msg(
    const PacketMsg& pm, const std::string& frame,
    const sensor::packet_format& pf);

/**
 * Populate a PCL point cloud from a LidarScan
 * @param[in] xyz_lut lookup table from sensor beam angles (see lidar_scan.h)
 * @param[in] ls input lidar data
 * @param[out] cloud output pcl pointcloud to populate
 * @param[out] cloud_stamp_out output timestamp of the cloud message to populate
 * @param[in] return_index index of return desired starting at 0
 * @param[in] exclusion boxes of points to be zeroed out (see ExclusionBoxes)
 */
bool scan_to_cloud(const ouster::XYZLut& xyz_lut,
                   const ouster::LidarScan& ls,
                   ouster_ros::Cloud& cloud_out,
                   ros::Time& cloud_stamp_out,
                   int return_index = 0,
                   const ExclusionBoxes& exclusion = ExclusionBoxes{});

/**
 * Populate a PCL point cloud from a LidarScan
 * @param[in] xyz_lut lookup table from sensor beam angles (see lidar_scan.h)
 * @param[in] ls input lidar data
 * @param[out] cloud output pcl pointcloud to populate
 * @param[out] cloud_stamp_out output timestamp of the cloud message to populate
 * @param[in] return_index index of return desired starting at 0
 * @param[in] tf_buffer the ROS TF buffer for looking up transforms
 * @param[in] sensor_frame name of the sensor's frame to be used as the source frame of the TF
 * @param[in] target_frame name of the target frame to transform the data to
 * @param[in] fixed_frame name of a world-fixed frame to be used as the middle frame for the TF
 * @param[in] lookup_timeout how long to wait for the transformation to become available
 * @param[in] exclusion boxes of points to be zeroed out before the deskewing (see ExclusionBoxes)
 * @return true if the deskewed cloud was populated; false if deskewing was not possible (no valid column
 * timestamps or the transformation is not available within lookup_timeout), in which case cloud_out is
 * not populated - falling back to scan_to_cloud() is up to the caller
 */
bool scan_to_cloud_deskewed(
                   const ouster::XYZLut& xyz_lut,
                   const ouster::LidarScan& ls,
                   ouster_ros::Cloud& cloud_out,
                   ros::Time& cloud_stamp_out,
                   int return_index,
                   tf2_ros::Buffer & tf_buffer,
                   const std::string & sensor_frame,
                   const std::string & target_frame,
                   const std::string & fixed_frame,
                   const ros::Duration& lookup_timeout = ros::Duration(0.05),
                   const ExclusionBoxes& exclusion = ExclusionBoxes{});

/**
 * Serialize a PCL point cloud to a ROS message
 * @param[in] cloud the PCL point cloud to convert
 * @param[in] timestamp the timestamp to apply to the resulting ROS message
 * @param[in] frame the frame to set in the resulting ROS message
 * @return a ROS message containing the point cloud
 */
sensor_msgs::PointCloud2 cloud_to_cloud_msg(const Cloud& cloud,
                                            const ros::Time& timestamp,
                                            const std::string& frame);

/**
 * Serialize a PCL point cloud to a ROS message
 * @param[in] cloud the PCL point cloud to convert
 * @param[in] timestamp the timestamp to apply to the resulting ROS message
 * @param[in] frame the frame to set in the resulting ROS message
 * @return a ROS message containing the point cloud
 */
[[deprecated]] sensor_msgs::PointCloud2 cloud_to_cloud_msg(
    const Cloud& cloud, ns timestamp, const std::string& frame);

/**
 * Convert transformation matrix return by sensor to ROS transform
 * @param[in] mat transformation matrix return by sensor
 * @param[in] frame the parent frame of the published transform
 * @param[in] child_frame the child frame of the published transform
 * @return ROS message suitable for publishing as a transform
 */
geometry_msgs::TransformStamped transform_to_tf_msg(
    const ouster::mat4d& mat, const std::string& frame,
    const std::string& child_frame);
}  // namespace ouster_ros
