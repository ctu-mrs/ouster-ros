/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster_ros/ros.h"

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>

#include <chrono>
#include <string>
#include <vector>
#include <optional>

namespace ouster_ros {

namespace sensor = ouster::sensor;

bool read_imu_packet(const sensor::client& cli, PacketMsg& pm,
                     const sensor::packet_format& pf) {
    pm.buf.resize(pf.imu_packet_size + 1);
    return read_imu_packet(cli, pm.buf.data(), pf);
}

bool read_lidar_packet(const sensor::client& cli, PacketMsg& pm,
                       const sensor::packet_format& pf) {
    pm.buf.resize(pf.lidar_packet_size + 1);
    return read_lidar_packet(cli, pm.buf.data(), pf);
}

sensor_msgs::Imu packet_to_imu_msg(const PacketMsg& pm,
                                   const ros::Time& timestamp,
                                   const std::string& frame,
                                   const sensor::packet_format& pf) {
    const double standard_g = 9.80665;
    sensor_msgs::Imu m;
    const uint8_t* buf = pm.buf.data();

    m.header.stamp = timestamp;
    m.header.frame_id = frame;

    m.orientation.x = 0;
    m.orientation.y = 0;
    m.orientation.z = 0;
    m.orientation.w = 0;

    m.linear_acceleration.x = pf.imu_la_x(buf) * standard_g;
    m.linear_acceleration.y = pf.imu_la_y(buf) * standard_g;
    m.linear_acceleration.z = pf.imu_la_z(buf) * standard_g;

    m.angular_velocity.x = pf.imu_av_x(buf) * M_PI / 180.0;
    m.angular_velocity.y = pf.imu_av_y(buf) * M_PI / 180.0;
    m.angular_velocity.z = pf.imu_av_z(buf) * M_PI / 180.0;

    for (int i = 0; i < 9; i++) {
        m.orientation_covariance[i] = -1;
        m.angular_velocity_covariance[i] = 0;
        m.linear_acceleration_covariance[i] = 0;
    }
    for (int i = 0; i < 9; i += 4) {
        m.linear_acceleration_covariance[i] = 0.01;
        m.angular_velocity_covariance[i] = 6e-4;
    }

    return m;
}

sensor_msgs::Imu packet_to_imu_msg(const PacketMsg& pm,
                                   const std::string& frame,
                                   const sensor::packet_format& pf) {
    ros::Time timestamp;
    timestamp.fromNSec(pf.imu_gyro_ts(pm.buf.data()));
    return packet_to_imu_msg(pm, timestamp, frame, pf);
}

struct read_and_cast {
    template <typename T, typename U>
    void operator()(Eigen::Ref<const ouster::img_t<T>> field,
                    ouster::img_t<U>& dest) {
        dest = field.template cast<U>();
    }
};

sensor::ChanField suitable_return(sensor::ChanField input_field, bool second) {
    switch (input_field) {
        case sensor::ChanField::RANGE:
        case sensor::ChanField::RANGE2:
            return second ? sensor::ChanField::RANGE2
                          : sensor::ChanField::RANGE;
        case sensor::ChanField::SIGNAL:
        case sensor::ChanField::SIGNAL2:
            return second ? sensor::ChanField::SIGNAL2
                          : sensor::ChanField::SIGNAL;
        case sensor::ChanField::REFLECTIVITY:
        case sensor::ChanField::REFLECTIVITY2:
            return second ? sensor::ChanField::REFLECTIVITY2
                          : sensor::ChanField::REFLECTIVITY;
        case sensor::ChanField::NEAR_IR:
            return sensor::ChanField::NEAR_IR;
        default:
            throw std::runtime_error("Unreachable");
    }
}

template <typename T>
inline ouster::img_t<T> get_or_fill_zero(sensor::ChanField f,
                                         const ouster::LidarScan& ls) {
    ouster::img_t<T> result{ls.h, ls.w};
    if (ls.field_type(f)) {
        ouster::impl::visit_field(ls, f, read_and_cast(), result);
    } else {
        result = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic,
                               Eigen::RowMajor>::Zero(ls.h, ls.w);
    }
    return result;
}

void scan_to_cloud(const ouster::XYZLut& xyz_lut,
                   const ouster::LidarScan& ls,
                   ouster_ros::Cloud& cloud,
                   ros::Time& cloud_stamp_out,
                   int return_index) {
    bool second = (return_index == 1);
    cloud.resize(ls.w * ls.h);

    ouster::img_t<uint16_t> near_ir = get_or_fill_zero<uint16_t>(
        suitable_return(sensor::ChanField::NEAR_IR, second), ls);

    ouster::img_t<uint32_t> range = get_or_fill_zero<uint32_t>(
        suitable_return(sensor::ChanField::RANGE, second), ls);

    ouster::img_t<uint32_t> signal = get_or_fill_zero<uint32_t>(
        suitable_return(sensor::ChanField::SIGNAL, second), ls);

    ouster::img_t<uint16_t> reflectivity = get_or_fill_zero<uint16_t>(
        suitable_return(sensor::ChanField::REFLECTIVITY, second), ls);

    const auto points = ouster::cartesian(range, xyz_lut);
    const auto& ts_raw = ls.timestamp();
    const auto* first_stamp_ptr = std::find_if(ts_raw.data(), ts_raw.data() + ts_raw.size(),
                            [](uint64_t h) { return h != 0; });
    if (first_stamp_ptr == ts_raw.data() + ts_raw.size())
    {
      for (auto u = 0; u < ls.h; u++)
          for (auto v = 0; v < ls.w; v++)
            cloud(v, u) = 
            {
                {{0.0f, 0.0f, 0.0f, 1.0f}},
                0.0f,
                0u,
                0u,
                0u,
                0u,
                0u
            }; // a "correct" zero point
      return;
    }
    const auto first_stamp_internal = *first_stamp_ptr;

    for (auto u = 0; u < ls.h; u++) {
        for (auto v = 0; v < ls.w; v++) {
            const auto xyz = points.row(u * ls.w + v);
            const auto ts = ts_raw[v] == 0u ? 0u : ts_raw[v] - first_stamp_internal;
            cloud(v, u) = ouster_ros::Point{
                {{static_cast<float>(xyz(0)), static_cast<float>(xyz(1)),
                  static_cast<float>(xyz(2)), 1.0f}},
                static_cast<float>(signal(u, v)),
                static_cast<uint32_t>(ts),
                static_cast<uint16_t>(reflectivity(u, v)),
                static_cast<uint8_t>(u),
                static_cast<uint16_t>(near_ir(u, v)),
                static_cast<uint32_t>(range(u, v))};
        }
    }

    cloud_stamp_out.fromNSec(first_stamp_internal);
}

void scan_to_cloud_deskewed(
                   const ouster::XYZLut& xyz_lut,
                   const ouster::LidarScan& ls,
                   ouster_ros::Cloud& cloud_out,
                   ros::Time& cloud_stamp_out,
                   int return_index,
                   tf2_ros::Buffer & tf_buffer,
                   const std::string & sensor_frame,
                   const std::string & target_frame,
                   const std::string & fixed_frame,
                   const ros::Duration& lookup_timeout)
{
    bool second = (return_index == 1);
    cloud_out.resize(ls.w * ls.h);

    ouster::img_t<uint16_t> near_ir = get_or_fill_zero<uint16_t>(
        suitable_return(sensor::ChanField::NEAR_IR, second), ls);

    ouster::img_t<uint32_t> range = get_or_fill_zero<uint32_t>(
        suitable_return(sensor::ChanField::RANGE, second), ls);

    ouster::img_t<uint32_t> signal = get_or_fill_zero<uint32_t>(
        suitable_return(sensor::ChanField::SIGNAL, second), ls);

    ouster::img_t<uint16_t> reflectivity = get_or_fill_zero<uint16_t>(
        suitable_return(sensor::ChanField::REFLECTIVITY, second), ls);

    const auto points = ouster::cartesian(range, xyz_lut);
    const auto& ts_raw = ls.timestamp();

    std::optional<uint64_t> start_ts = std::nullopt;
    std::optional<uint64_t> end_ts = std::nullopt;
    for (auto col_it = 0; col_it < ls.w; col_it++)
    {
      const auto& stamp = ts_raw(col_it);
      if (stamp == 0)
        continue;

      end_ts = stamp;
      if (!start_ts.has_value())
        start_ts = stamp;
    }

    if (!start_ts.has_value() || !end_ts.has_value())
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[OusterCloud]: Cloud has no timestamps! Not deskewing.");
      scan_to_cloud(xyz_lut, ls, cloud_out, cloud_stamp_out, return_index);
      return;
    }

    const auto first_stamp_internal = start_ts.value();
    const auto last_stamp_internal = end_ts.value();
    const auto first_stamp_ros = ros::Time().fromNSec(first_stamp_internal);
    const auto last_stamp_ros = ros::Time().fromNSec(last_stamp_internal);
    geometry_msgs::TransformStamped fallback_tf;
    try
    {
      fallback_tf = tf_buffer.lookupTransform(
          target_frame,
          first_stamp_ros,
          sensor_frame,
          last_stamp_ros,
          fixed_frame,
          lookup_timeout);
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[OusterCloud]: Cannot deskew: " << ex.what());
      scan_to_cloud(xyz_lut, ls, cloud_out, cloud_stamp_out, return_index);
      return;
    }

    int invalid_tfs = 0;
    int dropped_cols = 0;
    for (auto col_it = 0; col_it < ls.w; col_it++)
    {
        const auto col_stamp_internal = ts_raw(col_it);
        // check for dropped packets
        if (col_stamp_internal == 0)
        {
          dropped_cols++;
          // set all the points in this cloud to zero (the cloud is reused, not zeroed by default)
          for (auto point_it = 0; point_it < ls.h; point_it++)
          {
            cloud_out(col_it, point_it) = 
            {
                {{0.0f, 0.0f, 0.0f, 1.0f}},
                static_cast<float>(signal(point_it, col_it)),
                0u,
                static_cast<uint16_t>(reflectivity(point_it, col_it)),
                static_cast<uint8_t>(point_it),
                static_cast<uint16_t>(near_ir(point_it, col_it)),
                0u
            }; // a "correct" zero point
          }
          // then skip the rest of the code in this loop for this column
          continue;
        }
        const auto col_dt = col_stamp_internal - first_stamp_internal;
        const auto pt_stamp = first_stamp_ros + ros::Duration().fromNSec(col_dt);

        // find the corresponding transformation
        geometry_msgs::TransformStamped tf;
        try
        {
/* virtual geometry_msgs::TransformStamped
 * lookupTransform (
 * const std::string &target_frame,
 * const ros::Time &target_time,
 * const std::string &source_frame,
 * const ros::Time &source_time,
 * const std::string &fixed_frame,
 * const ros::Duration timeout
 * ) const */
            tf = tf_buffer.lookupTransform(
                target_frame,
                first_stamp_ros,
                sensor_frame,
                pt_stamp,
                fixed_frame);
        }
        catch (const tf2::TransformException& ex)
        {
            invalid_tfs++;
            tf = fallback_tf;
        }
        // interpret the transformation as an eigen matrix
        const Eigen::Affine3d tf_eig = tf2::transformToEigen(tf.transform);

        // create a temporary matrix of points in the scan column to be transformed
        Eigen::Matrix<double, 3, Eigen::Dynamic> col_points;
        col_points.resize(3, ls.h);
        for (auto point_it = 0; point_it < ls.h; point_it++)
            col_points.col(point_it) = points.row(point_it * ls.w + col_it);
        // transform the points in the matrix
        col_points = tf_eig * col_points;

        for (auto point_it = 0; point_it < ls.h; point_it++)
        {
            const auto pt_range = range(point_it, col_it);
            // each column in col_points represents one point corresponding to one row of the current column
            // from the LiDAR's scan with index (row_it, col_it) ... a bit confusing, I know
            // make sureo to preserve zero points correctly
            const Eigen::Vector3f pt_eig = pt_range == 0 ? Eigen::Vector3f::Zero() : Eigen::Vector3f(col_points.col(point_it).cast<float>());
            const ouster_ros::Point point
            {
                {{pt_eig.x(), pt_eig.y(), pt_eig.z(), 1.0f}},
                static_cast<float>(signal(point_it, col_it)),
                static_cast<uint32_t>(col_dt),
                static_cast<uint16_t>(reflectivity(point_it, col_it)),
                static_cast<uint8_t>(point_it),
                static_cast<uint16_t>(near_ir(point_it, col_it)),
                static_cast<uint32_t>(pt_range)
            };
            cloud_out(col_it, point_it) = point;
        }
    }
    cloud_stamp_out = first_stamp_ros;

    if (invalid_tfs > 0)
    {
        ROS_WARN_STREAM_THROTTLE(1.0, "Could not estimate motion from \"" << sensor_frame << "\" to \"" << target_frame << "\" through fixed frame \"" << fixed_frame << "\" - some points (" << invalid_tfs*cloud_out.height << "/" << cloud_out.size() << ") are not corrected based on motion.");
    }

    if (dropped_cols > 0)
    {
        ROS_WARN_STREAM_THROTTLE(1.0, "Some packets were lost (" << dropped_cols << " columns are missing) - some points (" << dropped_cols*cloud_out.height << "/" << cloud_out.size() << ") are missing.");
    }
}

sensor_msgs::PointCloud2 cloud_to_cloud_msg(const Cloud& cloud,
                                            const ros::Time& timestamp,
                                            const std::string& frame) {
    sensor_msgs::PointCloud2 msg{};
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame;
    msg.header.stamp = timestamp;
    return msg;
}

sensor_msgs::PointCloud2 cloud_to_cloud_msg(const Cloud& cloud, ns ts,
                                            const std::string& frame) {
    ros::Time timestamp;
    timestamp.fromNSec(ts.count());
    return cloud_to_cloud_msg(cloud, timestamp, frame);
}

geometry_msgs::TransformStamped transform_to_tf_msg(
    const ouster::mat4d& mat, const std::string& frame,
    const std::string& child_frame) {
    Eigen::Affine3d aff;
    aff.linear() = mat.block<3, 3>(0, 0);
    aff.translation() = mat.block<3, 1>(0, 3) * 1e-3;

    geometry_msgs::TransformStamped msg = tf2::eigenToTransform(aff);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame;
    msg.child_frame_id = child_frame;

    return msg;
}
}  // namespace ouster_ros
