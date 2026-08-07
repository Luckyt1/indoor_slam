// #include <so3_math.h>
#include <malloc.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <timing_utils.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <mutex>
#include <unordered_map>

#include "li_initialization.h"

using namespace std;

#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

int time_log_counter = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool flg_reset = false, flg_exit = false;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

auto LOGGER = rclcpp::get_logger("laserMapping");

void SigHandle(int sig)
{
  flg_exit = true;
  RCLCPP_WARN(LOGGER, "catch sig %d", sig);
  sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE * fp)
{
  V3D rot_ang;
  if (!use_imu_as_input) {
    rot_ang = SO3ToEuler(kf_output.x_.rot);
  } else {
    rot_ang = SO3ToEuler(kf_input.x_.rot);
  }

  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
  if (use_imu_as_input) {
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // omega
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                               // Acc
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));  // Bias_g
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1),
      kf_input.x_.gravity(2));  // Bias_a
  } else {
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // omega
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // Acc
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));  // Bias_g
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
      kf_output.x_.gravity(2));  // Bias_a
  }
  fprintf(fp, "\r\n");
  fflush(fp);
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
    }
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

bool keepPointForRearSector(const PointType & point_body)
{
  if (!filter_rear_points) {
    return true;
  }

  const double rear_sector_rad = std::clamp(rear_filter_angle_deg, 0.0, 360.0) * PI_M / 180.0;
  if (rear_sector_rad <= 0.0) {
    return true;
  }

  const double angle_from_rear = std::abs(std::atan2(point_body.y, -point_body.x));
  return angle_from_rear > 0.5 * rear_sector_rad;
}

static std::vector<std::uint8_t> mapping_keep_mask;
static std::vector<double> mapping_nearest_ranges;
static std::vector<int> mapping_point_bins;
static std::vector<double> mapping_point_ranges;
static PointCloudXYZI::Ptr mapping_filtered_world(new PointCloudXYZI());

void updateMappingKeepMask()
{
  const std::size_t count = feats_down_body->size();
  mapping_keep_mask.assign(count, 1U);
  if (count == 0) {
    return;
  }

  for (std::size_t index = 0; index < count; ++index) {
    if (!keepPointForRearSector(feats_down_body->points[index])) {
      mapping_keep_mask[index] = 0U;
    }
  }
  if (!occlusion_filter_en) {
    return;
  }

  const double azimuth_resolution =
    std::clamp(occlusion_azimuth_resolution_deg, 0.25, 10.0);
  const double elevation_resolution =
    std::clamp(occlusion_elevation_resolution_deg, 0.5, 10.0);
  const int azimuth_bins = static_cast<int>(std::ceil(360.0 / azimuth_resolution));
  const int elevation_bins = static_cast<int>(std::ceil(180.0 / elevation_resolution));
  const double infinity = std::numeric_limits<double>::infinity();
  const std::size_t bin_count = static_cast<std::size_t>(azimuth_bins * elevation_bins);
  mapping_nearest_ranges.assign(bin_count, infinity);
  mapping_point_bins.assign(count, -1);
  mapping_point_ranges.assign(count, infinity);

  for (std::size_t index = 0; index < count; ++index) {
    if (mapping_keep_mask[index] == 0U) {
      continue;
    }
    const auto & point = feats_down_body->points[index];
    const double horizontal = std::hypot(point.x, point.y);
    const double range = std::hypot(horizontal, static_cast<double>(point.z));
    if (!std::isfinite(range) || range <= 0.05) {
      mapping_keep_mask[index] = 0U;
      continue;
    }
    double azimuth = std::atan2(point.y, point.x) * 180.0 / PI_M;
    if (azimuth < 0.0) {
      azimuth += 360.0;
    }
    const double elevation =
      std::atan2(point.z, horizontal) * 180.0 / PI_M + 90.0;
    const int azimuth_bin = std::clamp(
      static_cast<int>(azimuth / azimuth_resolution), 0, azimuth_bins - 1);
    const int elevation_bin = std::clamp(
      static_cast<int>(elevation / elevation_resolution), 0, elevation_bins - 1);
    const int bin = elevation_bin * azimuth_bins + azimuth_bin;
    mapping_point_bins[index] = bin;
    mapping_point_ranges[index] = range;
    mapping_nearest_ranges[static_cast<std::size_t>(bin)] =
      std::min(mapping_nearest_ranges[static_cast<std::size_t>(bin)], range);
  }

  const double depth_margin = std::max(0.2, occlusion_depth_margin_m);
  const double surface_tolerance = std::max(0.1, occlusion_surface_tolerance_m);
  for (std::size_t index = 0; index < count; ++index) {
    const int bin = mapping_point_bins[index];
    if (bin < 0 || mapping_keep_mask[index] == 0U) {
      continue;
    }
    const int elevation_bin = bin / azimuth_bins;
    const int azimuth_bin = bin % azimuth_bins;
    const double own_nearest = mapping_nearest_ranges[static_cast<std::size_t>(bin)];
    if (mapping_point_ranges[index] > own_nearest + depth_margin) {
      mapping_keep_mask[index] = 0U;
      continue;
    }
    const int left_azimuth = (azimuth_bin + azimuth_bins - 1) % azimuth_bins;
    const int right_azimuth = (azimuth_bin + 1) % azimuth_bins;
    const double left = mapping_nearest_ranges[static_cast<std::size_t>(
      elevation_bin * azimuth_bins + left_azimuth)];
    const double right = mapping_nearest_ranges[static_cast<std::size_t>(
      elevation_bin * azimuth_bins + right_azimuth)];
    if (
      std::isfinite(left) && std::isfinite(right) &&
      std::abs(left - right) <= surface_tolerance &&
      mapping_point_ranges[index] > std::max(left, right) + depth_margin)
    {
      mapping_keep_mask[index] = 0U;
    }
  }
}

bool keepPointForMapping(std::size_t index)
{
  if (index < mapping_keep_mask.size()) {
    return mapping_keep_mask[index] != 0U;
  }
  return index < feats_down_body->size() &&
         keepPointForRearSector(feats_down_body->points[index]);
}

PointCloudXYZI::Ptr buildWorldCloudForMapping()
{
  if (!filter_rear_points && !occlusion_filter_en) {
    return feats_down_world;
  }

  PointCloudXYZI::Ptr filtered = mapping_filtered_world;
  filtered->clear();
  const size_t point_count = std::min(feats_down_world->size(), feats_down_body->size());
  filtered->points.reserve(point_count);

  for (size_t i = 0; i < point_count; ++i) {
    if (keepPointForMapping(i)) {
      filtered->points.push_back(feats_down_world->points[i]);
    }
  }

  filtered->width = filtered->points.size();
  filtered->height = 1;
  filtered->is_dense = feats_down_world->is_dense;
  return filtered;
}

void MapIncremental()
{
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (size_t i = 0; i < cur_pts; ++i) {
    if (i < feats_down_body->size() && !keepPointForMapping(i)) {
      continue;
    }

    /* decide if need add to map */
    PointType & point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector & points_near = Nearest_Points[i];

      Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) *
        filter_size_map_min;
      bool need_add = true;
      for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
        Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
        if (
          fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  ivox_->AddPoints(points_to_add);
}

void publish_init_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  int size_init_map = init_feats_world->size();

  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  pcl::toROSMsg(*init_feats_world, laserCloudmsg);

  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = world_frame;
  pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
static std::mutex pcd_save_mutex;
static bool app_mapping_active = false;
static bool app_mapping_paused = false;

void resetPcdSaveCloud()
{
  pcl_wait_save->clear();
}

bool removePreviousScansPcd(std::string * message)
{
  const std::filesystem::path pcd_path =
    std::filesystem::path(ROOT_DIR) / "PCD" / "scans.pcd";
  std::error_code error;
  std::filesystem::remove(pcd_path, error);
  if (!error) {
    return true;
  }
  if (message) {
    *message = "failed to remove previous scans.pcd: " + error.message();
  }
  RCLCPP_ERROR(
    LOGGER, "Failed to remove previous PCD map %s: %s",
    pcd_path.c_str(), error.message().c_str());
  return false;
}

void accumulatePcdSaveCloud(const PointCloudXYZI & cloud)
{
  // Persist the same filtered world-frame cloud that is published to the live
  // mapping UI. Save-time loop closure or visibility filtering changes point
  // coordinates/removes points and makes the final PCD/PGM disagree with what
  // the operator approved while mapping.
  *pcl_wait_save += cloud;
}

PointCloudXYZI::Ptr buildPcdSaveCloud()
{
  return pcl_wait_save;
}

bool saveScansPcd(const PointCloudXYZI & cloud)
{
  const std::filesystem::path pcd_path = std::filesystem::path(ROOT_DIR) / "PCD" / "scans.pcd";

  try {
    std::filesystem::create_directories(pcd_path.parent_path());
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(pcd_path.string(), cloud);
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(LOGGER, "Failed to save PCD map to %s: %s", pcd_path.c_str(), ex.what());
    return false;
  }
}

bool saveCurrentPcdMap(std::string * message)
{
  PointCloudXYZI::Ptr pcd_cloud = buildPcdSaveCloud();
  if (pcd_cloud->empty()) {
    if (message) {
      *message = "no points to save";
    }
    return false;
  }
  if (!saveScansPcd(*pcd_cloud)) {
    if (message) {
      *message = "failed to write src/Point-LIO/PCD/scans.pcd";
    }
    return false;
  }
  if (message) {
    *message = "saved src/Point-LIO/PCD/scans.pcd";
  }
  return true;
}

void publish_frame_world(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  PointCloudXYZI::Ptr live_cloud = feats_down_world;
  {
    std::lock_guard<std::mutex> lock(pcd_save_mutex);
    if (app_mapping_active) {
      live_cloud = buildWorldCloudForMapping();
    }
  }
  if (scan_pub_en) {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    // Mapping preview matches the filtered saved map. Outside an App mapping
    // session, navigation keeps the complete 360-degree safety scan.
    pcl::toROSMsg(*live_cloud, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = world_frame;
    pubLaserCloudFullRes->publish(laserCloudmsg);
  }

  //--------------------------save map-----------------------------------
  // 1. make sure you have enough memories
  // 2. noted that pcd save will influence the real-time performances
  if (pcd_save_en) {
    std::lock_guard<std::mutex> lock(pcd_save_mutex);
    if (app_mapping_active && !app_mapping_paused) {
      accumulatePcdSaveCloud(*live_cloud);
    }
  }
}

void publish_frame_body(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = body_frame;
  pubLaserCloudFull_body->publish(laserCloudmsg);
}

template <typename T>
void set_posestamp(T & out)
{
  if (!use_imu_as_input) {
    out.position.x = kf_output.x_.pos(0);
    out.position.y = kf_output.x_.pos(1);
    out.position.z = kf_output.x_.pos(2);
    Eigen::Quaterniond q = kf_output.x_.rot.quaternion();
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  } else {
    out.position.x = kf_input.x_.pos(0);
    out.position.y = kf_input.x_.pos(1);
    out.position.z = kf_input.x_.pos(2);
    Eigen::Quaterniond q = kf_input.x_.rot.quaternion();
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  }
}

void publish_odometry(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped,
  std::shared_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
  odomAftMapped.header.frame_id = world_frame;
  odomAftMapped.child_frame_id = body_frame;
  if (publish_odometry_without_downsample) {
    odomAftMapped.header.stamp = get_ros_time(time_current);
  } else {
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
  }
  set_posestamp(odomAftMapped.pose.pose);

  // nav_msgs/Odometry twist is expressed in child_frame_id. Point-LIO already
  // estimates velocity in its filter state, so publish that estimate instead
  // of forcing downstream consumers to differentiate and low-pass the pose.
  V3D body_velocity;
  V3D body_angular_velocity;
  if (!use_imu_as_input) {
    body_velocity = kf_output.x_.rot.matrix().transpose() * kf_output.x_.vel;
    body_angular_velocity = kf_output.x_.omg;
  } else {
    body_velocity = kf_input.x_.rot.matrix().transpose() * kf_input.x_.vel;
    body_angular_velocity = input_in.gyro - kf_input.x_.bg;
  }
  odomAftMapped.twist.twist.linear.x = body_velocity.x();
  odomAftMapped.twist.twist.linear.y = body_velocity.y();
  odomAftMapped.twist.twist.linear.z = body_velocity.z();
  odomAftMapped.twist.twist.angular.x = body_angular_velocity.x();
  odomAftMapped.twist.twist.angular.y = body_angular_velocity.y();
  odomAftMapped.twist.twist.angular.z = body_angular_velocity.z();

  pubOdomAftMapped->publish(odomAftMapped);

  if (tf_send_en) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = world_frame;
    transform.child_frame_id = body_frame;
    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transform.header.stamp = odomAftMapped.header.stamp;
    tf_br->sendTransform(transform);
  }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
  set_posestamp(msg_body_pose.pose);
  // msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
  msg_body_pose.header.frame_id = world_frame;
  static int jjj = 0;
  jjj++;
  // if (jjj % 2 == 0) // if path is too large, the rvis will crash
  {
    path.poses.emplace_back(msg_body_pose);
    pubPath->publish(path);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);

  readParameters(nh);
  std::cout << "lidar_type: " << lidar_type << '\n';
  ivox_ = std::make_shared<IVoxType>(ivox_options_);

  path.header.stamp = get_ros_time(lidar_end_time);
  path.header.frame_id = world_frame;

  /*** variables definition for counting ***/
  int frame_num = 0;
  double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0,
         aver_time_solve = 0, aver_time_propag = 0;

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  // 设置体素滤波器参数
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    } else {
      kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    }
  }

  p_imu->lidar_type = p_pre->lidar_type = lidar_type;
  p_imu->imu_en = imu_en;

  kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
  kf_output.init_dyn_share_modified_3h(
    get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
  Eigen::Matrix<double, 24, 24> P_init;  // = MD(18, 18)::Identity() * 0.1;
  reset_cov(P_init);
  kf_input.change_P(P_init);
  Eigen::Matrix<double, 30, 30> P_init_output;  // = MD(24, 24)::Identity() * 0.01;
  reset_cov_output(P_init_output);
  kf_output.change_P(P_init_output);
  Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
  Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
  /*** debug record ***/
  FILE * fp = nullptr;
  if (runtime_pos_log) {
    std::error_code log_dir_error;
    const std::filesystem::path log_dir = std::filesystem::path(root_dir) / "Log";
    std::filesystem::create_directories(log_dir, log_dir_error);
    if (log_dir_error) {
      RCLCPP_WARN(
        LOGGER, "Disabling Point-LIO runtime logs: cannot create %s: %s",
        log_dir.c_str(), log_dir_error.message().c_str());
      runtime_pos_log = false;
    } else {
      const std::filesystem::path pos_log_path = log_dir / "pos_log.txt";
      fp = fopen(pos_log_path.string().c_str(), "w");
      if (fp == nullptr || !open_file()) {
        RCLCPP_WARN(
          LOGGER, "Disabling Point-LIO runtime logs: cannot open files under %s",
          log_dir.c_str());
        if (fp != nullptr) {
          fclose(fp);
          fp = nullptr;
        }
        runtime_pos_log = false;
      }
    }
  }

  /*** ROS subscribe initialization ***/
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
  if (p_pre->lidar_type == AVIA) {
    sub_pcl_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lid_topic, rclcpp::SensorDataQoS(),
      [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { livox_pcl_cbk(msg); });
  } else {
    sub_pcl_pc = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      lid_topic, rclcpp::SensorDataQoS(),
      [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { standard_pcl_cbk(msg); });
  }
  auto sub_imu =
    nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);
  auto pub_laser_cloud_full_res =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_registered", rclcpp::SensorDataQoS().keep_last(2));
  auto pub_laser_cloud_full_res_body =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_registered_body", rclcpp::SensorDataQoS().keep_last(2));
  auto pub_laser_cloud_effect =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_effected", 1000);
  auto pub_laser_cloud_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map", 1000);
  auto pub_odom_aft_mapped =
    nh->create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init", 1000);
  auto pub_path = nh->create_publisher<nav_msgs::msg::Path>("path", 1000);
  auto mapping_start_srv = nh->create_service<std_srvs::srv::Trigger>("/point_lio/mapping/start",
    [](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::lock_guard<std::mutex> lock(pcd_save_mutex);
      if (!pcd_save_en) {
        response->success = false;
        response->message = "pcd_save.pcd_save_en is false";
        return;
      }
      resetPcdSaveCloud();
      app_mapping_active = false;
      app_mapping_paused = false;
      std::string cleanup_message;
      if (!removePreviousScansPcd(&cleanup_message)) {
        response->success = false;
        response->message = cleanup_message;
        return;
      }
      app_mapping_active = true;
      app_mapping_paused = false;
      response->success = true;
      response->message = "Point-LIO mapping accumulation started";
    });
  auto mapping_pause_srv = nh->create_service<std_srvs::srv::SetBool>("/point_lio/mapping/pause",
    [](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      std::lock_guard<std::mutex> lock(pcd_save_mutex);
      if (!app_mapping_active) {
        response->success = false;
        response->message = "no active Point-LIO mapping accumulation";
        return;
      }
      app_mapping_paused = request->data;
      response->success = true;
      response->message = app_mapping_paused ? "Point-LIO mapping paused" :
        "Point-LIO mapping resumed";
    });
  auto mapping_save_srv = nh->create_service<std_srvs::srv::Trigger>("/point_lio/mapping/save",
    [](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::lock_guard<std::mutex> lock(pcd_save_mutex);
      std::string message;
      response->success = saveCurrentPcdMap(&message);
      response->message = message;
      if (response->success) {
        app_mapping_active = false;
        app_mapping_paused = false;
      }
    });
  auto mapping_cancel_srv = nh->create_service<std_srvs::srv::Trigger>("/point_lio/mapping/cancel",
    [](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::lock_guard<std::mutex> lock(pcd_save_mutex);
      resetPcdSaveCloud();
      app_mapping_active = false;
      app_mapping_paused = false;
      response->success = true;
      response->message = "Point-LIO mapping accumulation cancelled";
    });
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);

  //------------------------------------------------------------------------------------------------------
  signal(SIGINT, SigHandle);
  rclcpp::Rate rate(500);
  while (rclcpp::ok()) {
    if (flg_exit) break;
    executor.spin_some();
    if (sync_packages(Measures)) {
      if (flg_reset) {
        RCLCPP_WARN(LOGGER, "reset when rosbag play back");
        p_imu->Reset();
        feats_undistort.reset(new PointCloudXYZI());
        if (use_imu_as_input) {
          // state_in = kf_input.get_x();
          state_in = state_input();
          kf_input.change_P(P_init);
        } else {
          // state_out = kf_output.get_x();
          state_out = state_output();
          kf_output.change_P(P_init_output);
        }
        flg_first_scan = true;
        is_first_frame = true;
        flg_reset = false;
        init_map = false;

        {
          ivox_.reset(new IVoxType(ivox_options_));
        }
      }

      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        flg_first_scan = false;
        if (first_imu_time < 1) {
          first_imu_time = get_time_sec(imu_next.header.stamp);
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        if (imu_en) {
          // imu_next = *(imu_deque.front());
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc *= -1;

          {
            while (Measures.lidar_beg_time >
                   get_time_sec(imu_next.header.stamp))  // if it is needed for the new map?
            {
              imu_deque.pop_front();
              if (imu_deque.empty()) {
                break;
              }
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_deque.pop();
            }
            //时间同步
          }
        } else {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);   // _init);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);  //_init);
          kf_output.x_.acc << VEC_FROM_ARRAY(gravity);      //_init);
          kf_output.x_.acc *= -1;
          p_imu->imu_need_init_ = false;
          // p_imu->after_imu_init_ = true;
        }
        G_m_s2 =
          std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
      }

      double t0, t1, t2, t3, t4, t5, match_start, solve_start;
      match_time = 0;
      solve_time = 0; 
      propag_time = 0;
      update_time = 0;
      t0 = point_lio::wall_time();

      /*** downsample the feature points in a scan ***/
      t1 = point_lio::wall_time();
      p_imu->Process(Measures, feats_undistort);
      if (space_down_sample) {
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      } else {
        feats_down_body = Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      }
      {
        time_seq = time_compressing<int>(feats_down_body);
        feats_down_size = feats_down_body->points.size();
      } 

      if (!p_imu->after_imu_init_)  // !p_imu->UseLIInit &&
      {
        if (!p_imu->imu_need_init_) {
          V3D tmp_gravity;
          if (imu_en) {
            tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
          } else {
            tmp_gravity << VEC_FROM_ARRAY(gravity_init);
            p_imu->after_imu_init_ = true;
          }
          // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);
          kf_input.x_.rot = rot_init;
          kf_output.x_.rot = rot_init;
          // kf_input.x_.rot; //.normalize();
          // kf_output.x_.rot; //.normalize();
          kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
        } else {
          continue;
        }
      }
      /*** initialize the map ***/
      if (!init_map) {
        feats_down_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++) {
          {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
          }
        }
        for (size_t i = 0; i < feats_down_world->size(); i++) {
          if (
            i < feats_undistort->size() &&
            !keepPointForRearSector(feats_undistort->points[i]))
          {
            continue;
          }
          init_feats_world->points.emplace_back(feats_down_world->points[i]);
        }
        if (init_feats_world->size() < init_map_size) {
          init_map = false;
        } else {
          ivox_->AddPoints(init_feats_world->points);
          publish_init_map(pub_laser_cloud_map);  //(pubLaserCloudFullRes);

          init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
        }
        continue;
      }

      /*** ICP and Kalman filter update ***/
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      Nearest_Points.resize(feats_down_size);

      t2 = point_lio::wall_time();

      /*** iterated state estimation ***/
      crossmat_list.reserve(feats_down_size);
      pbody_list.reserve(feats_down_size);
      // pbody_ext_list.reserve(feats_down_size);

      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(
          feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z);
        pbody_list[i] = point_this;
        if (!extrinsic_est_en)
        // {
        //     if (!use_imu_as_input)
        //     {
        //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
        //     }
        //     else
        //     {
        //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
        //     }
        // }
        // else
        {
          point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          crossmat_list[i] = point_crossmat;
        }
      }
      if (!use_imu_as_input) {
        bool imu_upda_cov = false;
        effct_feat_num = 0;
        /**** point by point update ****/
        if (time_seq.size() > 0) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];

            time_current = point_body.curvature / 1000.0 + pcl_beg_time;

            if (is_first_frame) {
              if (imu_en) {
                while (time_current > get_time_sec(imu_next.header.stamp)) {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
              }
              is_first_frame = false;
              imu_upda_cov = true;
              time_update_last = time_current;
              time_predict_last_const = time_current;
            }
            if (imu_en && !imu_deque.empty()) {
              bool last_imu = get_time_sec(imu_next.header.stamp) ==
                              get_time_sec(imu_deque.front()->header.stamp);
              while (get_time_sec(imu_next.header.stamp) < time_predict_last_const &&
                     !imu_deque.empty()) {
                if (!last_imu) {
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                  break;
                } else {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
              }
              bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              while (imu_comes) {
                imu_upda_cov = true;
                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;

                /*** covariance update ***/
                double dt = get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                kf_output.predict(dt, Q_output, input_in, true, false);
                time_predict_last_const = get_time_sec(imu_next.header.stamp);  // big problem

                {
                  double dt_cov = get_time_sec(imu_next.header.stamp) - time_update_last;

                  if (dt_cov > 0.0) {
                    time_update_last = get_time_sec(imu_next.header.stamp);
                    double propag_imu_start = point_lio::wall_time();

                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                    propag_time += point_lio::wall_time() - propag_imu_start;
                    double solve_imu_start = point_lio::wall_time();
                    kf_output.update_iterated_dyn_share_IMU();
                    solve_time += point_lio::wall_time() - solve_imu_start;
                  }
                }
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              }
            }
            if (flg_reset) {
              break;
            }

            double dt = time_current - time_predict_last_const;
            double propag_state_start = point_lio::wall_time();
            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_output.predict(dt, Q_output, input_in, true, false);
            propag_time += point_lio::wall_time() - propag_state_start;
            time_predict_last_const = time_current;
            double t_update_start = point_lio::wall_time();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");
              idx += time_seq[k];
              continue;
            }
            if (!kf_output.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }
            solve_start = point_lio::wall_time();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_output.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                         << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose()
                         << " " << kf_output.x_.acc.transpose() << " "
                         << kf_output.x_.gravity.transpose() << " " << kf_output.x_.bg.transpose()
                         << " " << kf_output.x_.ba.transpose() << " "
                         << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }

            solve_time += point_lio::wall_time() - solve_start;

            update_time += point_lio::wall_time() - t_update_start;
            idx += time_seq[k];
            // std::cout << "pbp output effect feat num:" << effct_feat_num << '\n';
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());

            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }
                  break;
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;

                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;

                imu_upda_cov = true;
                time_update_last = time_current;
                time_predict_last_const = time_current;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - time_predict_last_const;
                {
                  double dt_cov = time_current - time_update_last;
                  if (dt_cov > 0.0) {
                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                    time_update_last = time_current;
                  }
                  kf_output.predict(dt, Q_output, input_in, true, false);
                }

                time_predict_last_const = time_current;

                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                kf_output.update_iterated_dyn_share_IMU();
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      } else {
        bool imu_prop_cov = false;
        effct_feat_num = 0;
        if (time_seq.size() > 0) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];
            time_current = point_body.curvature / 1000.0 + pcl_beg_time;
            if (is_first_frame) {
              while (time_current > get_time_sec(imu_next.header.stamp)) {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
              imu_prop_cov = true;

              is_first_frame = false;
              t_last = time_current;
              time_update_last = time_current;
              {
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              }
            }

            while (time_current > get_time_sec(imu_next.header.stamp))  // && !imu_deque.empty())
            {
              imu_deque.pop_front();

              input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                imu_last.angular_velocity.z;
              input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                imu_last.linear_acceleration.z;
              input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              double dt = get_time_sec(imu_last.header.stamp) - t_last;

              double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = get_time_sec(imu_last.header.stamp);  //time_current;
              }
              kf_input.predict(dt, Q_input, input_in, true, false);
              t_last = get_time_sec(imu_last.header.stamp);
              imu_prop_cov = true;

              if (imu_deque.empty()) break;
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_upda_cov = true;
            }
            if (flg_reset) {
              break;
            }
            double dt = time_current - t_last;
            t_last = time_current;
            double propag_start = point_lio::wall_time();

            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_input.predict(dt, Q_input, input_in, true, false);

            propag_time += point_lio::wall_time() - propag_start;

            double t_update_start = point_lio::wall_time();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");

              idx += time_seq[k];
              continue;
            }
            if (!kf_input.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }

            solve_start = point_lio::wall_time();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_input.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                         << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                         << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose()
                         << " " << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }
            solve_time += point_lio::wall_time() - solve_start;

            update_time += point_lio::wall_time() - t_update_start;
            idx = idx + time_seq[k];
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }

                  break;
                }
                imu_prop_cov = true;

                t_last = time_current;
                time_update_last = time_current;
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - t_last;

                double dt_cov = time_current - time_update_last;
                if (dt_cov > 0.0) {
                  // kf_input.predict(dt_cov, Q_input, input_in, false, true);
                  time_update_last = get_time_sec(imu_next.header.stamp);  //time_current;
                }
                // kf_input.predict(dt, Q_input, input_in, true, false);

                t_last = get_time_sec(imu_next.header.stamp);

                input_in.gyro << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                input_in.acc << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      }
      // M3D rot_cur_lidar;
      // {
      //     rot_cur_lidar = state.rot_end;
      // }
      // euler_cur = RotMtoEuler(rot_cur_lidar);
      // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
      //                     (euler_cur(0), euler_cur(1), euler_cur(2));
      /******* Publish odometry downsample *******/
      if (!publish_odometry_without_downsample) {
        publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
      }

      /*** add the feature points to map ***/
      t3 = point_lio::wall_time();

      if (feats_down_size > 4) {
        updateMappingKeepMask();
        MapIncremental();
      } else {
        mapping_keep_mask.clear();
      }

      t5 = point_lio::wall_time();
      /******* Publish points *******/
      if (path_en) publish_path(pub_path);
      if (scan_pub_en || pcd_save_en) publish_frame_world(pub_laser_cloud_full_res);
      if (scan_pub_en && scan_body_pub_en) publish_frame_body(pub_laser_cloud_full_res_body);

      /*** Debug variables Logging ***/
      if (runtime_pos_log) {
        frame_num++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        {
          aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
        }
        aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
        aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
        aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
        T1[time_log_counter] = Measures.lidar_beg_time;
        s_plot[time_log_counter] = t5 - t0;
        s_plot2[time_log_counter] = feats_undistort->points.size();
        s_plot3[time_log_counter] = aver_time_consu;
        time_log_counter++;
        printf(
          "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: "
          "%0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
          t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu,
          aver_time_icp, aver_time_propag);
        if (!publish_odometry_without_downsample) {
          if (!use_imu_as_input) {
            euler_cur = SO3ToEuler(kf_output.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                     << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                     << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose()
                     << " " << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose()
                     << " " << feats_undistort->points.size() << '\n';
          } else {
            euler_cur = SO3ToEuler(kf_input.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                     << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                     << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                     << feats_undistort->points.size() << '\n';
          }
        }
        dump_lio_state_to_log(fp);
      }
    }
    rate.sleep();
  }
  //--------------------------save map-----------------------------------
  /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en) {
    std::lock_guard<std::mutex> lock(pcd_save_mutex);
    if (app_mapping_active) {
      std::string message;
      if (!saveCurrentPcdMap(&message)) {
        RCLCPP_WARN(LOGGER, "%s", message.c_str());
      }
    }
  }
  if (fp != nullptr) fclose(fp);
  if (fout_out.is_open()) fout_out.close();
  if (fout_imu_pbp.is_open()) fout_imu_pbp.close();
  return 0;
}
