//
// Created by liuhao on 2021/11/23.
//

#ifndef USV_TF2_TF2_USV_MSGS_HPP
#define USV_TF2_TF2_USV_MSGS_HPP
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

#include <common/types.hpp>
#include <kdl/frames.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <usv_msgs/msg/bounding_box.hpp>
#include <usv_msgs/msg/bounding_box_array.hpp>
#include <usv_msgs/msg/quaternion32.hpp>


using usv::common::types::float32_t;
using usv::common::types::float64_t;
using BoundingBoxArray = usv_msgs::msg::BoundingBoxArray;
using BoundingBox = usv_msgs::msg::BoundingBox;

namespace tf2
{
/*************/
/** Point32 **/
/*************/

/** \brief Apply a geometry_msgs TransformStamped to a geometry_msgs Point32 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a Point32 message.
 * \param t_out The transformed point, as a Point32 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const geometry_msgs::msg::Point32 & t_in,
  geometry_msgs::msg::Point32 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  const KDL::Vector v_out =
    gmTransformToKDL(transform) *
    KDL::Vector(
      static_cast<double>(t_in.x), static_cast<double>(t_in.y), static_cast<double>(t_in.z));
  t_out.x = static_cast<float>(v_out[0]);
  t_out.y = static_cast<float>(v_out[1]);
  t_out.z = static_cast<float>(v_out[2]);
}


/*************/
/** Polygon **/
/*************/

/** \brief Apply a geometry_msgs TransformStamped to a geometry_msgs Polygon type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The polygon to transform.
 * \param t_out The transformed polygon.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const geometry_msgs::msg::Polygon & t_in,
  geometry_msgs::msg::Polygon & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  // Don't call the Point32 doTransform to avoid doing this conversion every time
  const auto kdl_frame = gmTransformToKDL(transform);
  // We don't use std::back_inserter to allow aliasing between t_in and t_out
  t_out.points.resize(t_in.points.size());
  for (size_t i = 0; i < t_in.points.size(); ++i) {
    const KDL::Vector v_out = kdl_frame * KDL::Vector(
                                            static_cast<double>(t_in.points[i].x),
                                            static_cast<double>(t_in.points[i].y),
                                            static_cast<double>(t_in.points[i].z));
    t_out.points[i].x = static_cast<float>(v_out[0]);
    t_out.points[i].y = static_cast<float>(v_out[1]);
    t_out.points[i].z = static_cast<float>(v_out[2]);
  }
}

/******************/
/** Quaternion32 **/
/******************/

/** \brief Apply a geometry_msgs TransformStamped to an usv_auto_geometry_msgs Quaternion32
 * type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The Quaternion32 message to transform.
 * \param t_out The transformed Quaternion32 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const usv_msgs::msg::Quaternion32 & t_in,
  usv_msgs::msg::Quaternion32 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  KDL::Rotation r_in = KDL::Rotation::Quaternion(
    static_cast<double>(t_in.x),
    static_cast<double>(t_in.y),
    static_cast<double>(t_in.z),
    static_cast<double>(t_in.w));
  KDL::Rotation out = gmTransformToKDL(transform).M * r_in;

  double qx, qy, qz, qw;
  out.GetQuaternion(qx, qy, qz, qw);
  t_out.x = static_cast<float32_t>(qx);
  t_out.y = static_cast<float32_t>(qy);
  t_out.z = static_cast<float32_t>(qz);
  t_out.w = static_cast<float32_t>(qw);
}


/******************/
/** BoundingBox **/
/******************/

/** \brief Apply a geometry_msgs TransformStamped to an usv_auto_perception_msgs BoundingBox
 * type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The BoundingBox message to transform.
 * \param t_out The transformed BoundingBox message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const BoundingBox & t_in,
  BoundingBox & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;
  doTransform(t_in.orientation, t_out.orientation, transform);
  doTransform(t_in.centroid, t_out.centroid, transform);
  doTransform(t_in.corners[0], t_out.corners[0], transform);
  doTransform(t_in.corners[1], t_out.corners[1], transform);
  doTransform(t_in.corners[2], t_out.corners[2], transform);
  doTransform(t_in.corners[3], t_out.corners[3], transform);
  // TODO(jitrc): add conversion for other fields of BoundingBox, such as heading, variance, size
}


/**********************/
/** BoundingBoxArray **/
/**********************/

/** \brief Extract a timestamp from the header of a BoundingBoxArray message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t A timestamped BoundingBoxArray message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template <> inline tf2::TimePoint getTimestamp(const BoundingBoxArray & t)
{
  return tf2_ros::fromMsg(t.header.stamp);
}

/** \brief Extract a frame ID from the header of a BoundingBoxArray message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t A timestamped BoundingBoxArray message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template <> inline std::string getFrameId(const BoundingBoxArray & t)
{
  return t.header.frame_id;
}

/** \brief Apply a geometry_msgs TransformStamped to an usv_auto_perception_msgs
 * BoundingBoxArray type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The BoundingBoxArray to transform, as a timestamped BoundingBoxArray message.
 * \param t_out The transformed BoundingBoxArray, as a timestamped BoundingBoxArray message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(
  const BoundingBoxArray & t_in,
  BoundingBoxArray & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;
  for (auto idx = 0U; idx < t_in.boxes.size(); idx++) {
    doTransform(t_out.boxes[idx], t_out.boxes[idx], transform);
  }
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

}  // namespace tf2
#endif  // USV_TF2_TF2_USV_MSGS_HPP
