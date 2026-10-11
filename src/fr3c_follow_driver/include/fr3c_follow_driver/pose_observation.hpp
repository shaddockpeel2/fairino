#pragma once
#include <cstdint>
#include <nlohmann/json.hpp>
#include "fr3c_follow_driver/pose_geometry.hpp"

namespace fr3c_follow_driver {
using ObservationJson=nlohmann::json;
inline std::int64_t exact_stamp(const ObservationJson & stamp) {
  if (!stamp.is_object() || !stamp.contains("sec") || !stamp.contains("nanosec") ||
      !stamp["sec"].is_number_integer() || !stamp["nanosec"].is_number_integer())
    throw std::invalid_argument("missing_exact_image_stamp");
  const auto sec=stamp["sec"].get<std::int64_t>(),nano=stamp["nanosec"].get<std::int64_t>();
  if (sec<0 || sec>9000000000LL || nano<0 || nano>=1000000000LL || (sec==0 && nano==0))
    throw std::invalid_argument("invalid_image_stamp");
  return sec*1000000000LL+nano;
}
struct PoseObservation {
  bool trusted=false, soft=false;
  std::int64_t stamp=0;
  std::string reason;
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
};
inline PoseObservation parse_observation(const ObservationJson & row,int target_id) {
  if (!row.is_object() || row.value("schema_version",0)!=1 || row.value("frame_id","")!="base_link" ||
      !row.contains("motion_commands_enabled") || row["motion_commands_enabled"]!=false ||
      !row.contains("orientation_control_enabled") || row["orientation_control_enabled"]!=false)
    throw std::invalid_argument("invalid_pose_observation");
  PoseObservation out;
  if (row.contains("stamp") && !row["stamp"].is_null()) out.stamp=exact_stamp(row["stamp"]);
  out.reason=row.value("status",std::string("invalid_pose_observation"));
  out.soft=out.reason=="orientation_ambiguous" || out.reason=="acquiring_orientation";
  if (row.value("valid_orientation",false)!=true) {
    if (out.reason=="valid") throw std::invalid_argument("invalid_pose_observation");
    return out;
  }
  if (out.stamp==0 || out.reason!="valid" || row.value("valid_position",false)!=true ||
      row.value("orientation_measurement_valid",false)!=true || row.value("target_id",-1)!=target_id ||
      row.value("source_frame_id","")!="camera_color_optical_frame" ||
      row.value("continuity_frame","")!="base_link" || row.value("confirmation_count",0)<2 ||
      !row.contains("quality") || !row["quality"].is_object())
    throw std::invalid_argument("invalid_trusted_pose");
  const auto & quality=row["quality"];
  if (exact_stamp(quality.at("stamp"))!=out.stamp || quality.value("status","")!="valid" ||
      quality.value("frame_id","")!="camera_color_optical_frame" || quality.value("target_id",-1)!=target_id ||
      quality.value("valid_position",false)!=true || quality.value("orientation_ambiguous",true)!=false ||
      quality.value("orientation_measurement_valid",false)!=true || quality.value("solver","")!="IPPE_SQUARE")
    throw std::invalid_argument("invalid_paired_pose_quality");
  const int count=quality.value("candidate_count",0),selected=quality.value("selected_candidate_index",-1);
  const auto & errors=quality.at("candidate_errors_px");
  if (count<1 || selected<0 || selected>=count || !errors.is_array() || errors.size()!=static_cast<unsigned>(count))
    throw std::invalid_argument("invalid_candidate_quality");
  for (const auto & error:errors)
    if (!error.is_number() || !std::isfinite(error.get<double>()) || error.get<double>()<0)
      throw std::invalid_argument("invalid_candidate_quality");
  const double tolerance=quality.value("orientation_candidate_tolerance_deg",5.);
  if (!std::isfinite(tolerance) || tolerance<0 || tolerance>45)
    throw std::invalid_argument("invalid_orientation_candidate_tolerance");
  if (count>1) {
    const double separation=quality.at("candidate_rotation_distance_deg").get<double>();
    if (!std::isfinite(separation) || separation<0 || separation>180 || (selected!=0 && separation>tolerance) ||
        (selected!=0 && separation>5 && quality.value("orientation_approximate",false)!=true))
      throw std::invalid_argument("invalid_candidate_quality");
  }
  const auto p=row.at("position_m").get<std::vector<double>>(),q=row.at("quaternion_xyzw").get<std::vector<double>>();
  if (p.size()!=3 || q.size()!=4) throw std::invalid_argument("invalid_target_pose");
  Eigen::Vector3d xyz(p.data()); Eigen::Quaterniond quaternion(q[3],q[0],q[1],q[2]);
  if (!xyz.allFinite() || !quaternion.coeffs().allFinite() || !std::isfinite(quaternion.norm()) || quaternion.norm()<1e-9)
    throw std::invalid_argument("invalid_target_pose");
  quaternion.normalize(); out.pose.translation()=xyz; out.pose.linear()=quaternion.toRotationMatrix();
  out.trusted=true; out.reason.clear(); return out;
}
}  // namespace fr3c_follow_driver
