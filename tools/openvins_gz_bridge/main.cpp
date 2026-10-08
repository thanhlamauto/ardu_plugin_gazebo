// Adapter for evaluating upstream OpenVINS with Gazebo RGB + raw IMU samples.
// The flight origin is the declared hover point (0, 0, 3 m); no Gazebo pose is read.
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <time.h>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include <opencv2/imgproc.hpp>
#include <opencv2/geometry/3d.hpp>

#include "cam/CamRadtan.h"
#include "core/VioManager.h"
#include "state/State.h"
#include "utils/quat_ops.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

namespace {
std::atomic<bool> running{true};
double mono_seconds() {
  timespec ts{};
#ifdef __APPLE__
  // Python time.monotonic() uses CLOCK_UPTIME_RAW on macOS.
  clock_gettime(CLOCK_UPTIME_RAW, &ts);
#else
  clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
  return ts.tv_sec + 1e-9 * ts.tv_nsec;
}
double stamp(const gz::msgs::Header &h) {
  return h.stamp().sec() + 1e-9 * h.stamp().nsec();
}
void stop(int) { running = false; }
}

int main(int argc, char **argv) {
  if (argc < 2 || argc > 4) {
    std::cerr << "usage: openvins_gz_bridge OUTPUT_CSV [--zupt-after-motion] [--debug-zupt]\n";
    return 2;
  }
  bool zupt_after_motion = false, debug_zupt = false;
  for (int i = 2; i < argc; ++i) {
    const std::string flag(argv[i]);
    if (flag == "--zupt-after-motion" && !zupt_after_motion) zupt_after_motion = true;
    else if (flag == "--debug-zupt" && !debug_zupt) debug_zupt = true;
    else {
      std::cerr << "unknown or duplicate flag: " << flag << '\n';
      return 2;
    }
  }
  if (debug_zupt) ov_core::Printer::setPrintLevel("DEBUG");
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  std::ofstream csv(argv[1]);
  csv << std::fixed << std::setprecision(9);
  csv << "stamp,initialized,x,y,z,qx,qy,qz,qw,imu_count,image_count,"
         "image_received_mono_s,image_processing_start_mono_s,"
         "vio_done_mono_s,pose_published_mono_s,imu_alignment_ms,"
         "vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,active_tracks,msckf_updates,"
         "odom_qx,odom_qy,odom_qz,odom_qw\n";
  csv.flush();

  ov_msckf::VioManagerOptions options;
  options.use_aruco = false;
  options.num_pts = 250;
  options.track_frequency = 10.0;
  options.num_opencv_threads = 2;
  // Starting at a verified hover should use OpenVINS' stationary initializer;
  // otherwise it waits for a new takeoff jerk that this experiment never gives.
  options.try_zupt = true;
  options.zupt_only_at_beginning = !zupt_after_motion;
  if (zupt_after_motion) {
    options.zupt_max_velocity = 0.1;
    options.zupt_max_disparity = 0.5;
    options.zupt_noise_multiplier = 10.0;
  }
  std::cerr << "ZUPT after motion: " << (zupt_after_motion ? "enabled" : "disabled") << '\n';
  options.init_options.init_window_time = 1.0;
  options.init_options.init_imu_thresh = 1.0;
  options.init_options.init_max_features = 150;
  // SDF IMU uses 100 Hz independent Gaussian sample noise. At this rate,
  // sample stddev = continuous noise density * sqrt(100 Hz).
  options.imu_noises.sigma_w = 1.6968e-4;
  options.imu_noises.sigma_a = 2e-3;
  // The SDF has no modeled bias walk; these are numerical floors only.
  options.imu_noises.sigma_wb = 1e-7;
  options.imu_noises.sigma_ab = 1e-6;
  std::cerr << "IMU_PROFILE gaussian_sdf_100hz sigma_w=" << options.imu_noises.sigma_w
            << " sigma_a=" << options.imu_noises.sigma_a
            << " sigma_wb=" << options.imu_noises.sigma_wb
            << " sigma_ab=" << options.imu_noises.sigma_ab << '\n';
  // OpenVINS stores lower-triangular inverse IMU scale matrices in Kalibr
  // order. Zero would erase the IMU acceleration and cause free-fall drift.
  options.vec_dw << 1, 0, 0, 1, 0, 1;
  options.vec_da << 1, 0, 0, 1, 0, 1;
  options.vec_tg.setZero();
  options.q_GYROtoIMU << 0, 0, 0, 1;
  options.q_ACCtoIMU << 0, 0, 0, 1;
  auto camera = std::make_shared<ov_core::CamRadtan>(640, 360);
  Eigen::Matrix<double, 8, 1> intrinsics;
  const double f = 640.0 / (2.0 * std::tan(1.3962634 / 2.0));
  intrinsics << f, f, 319.5, 179.5, 0, 0, 0, 0;
  camera->set_value(intrinsics);
  options.camera_intrinsics[0] = camera;
  // Gazebo link: +X forward, +Y left, +Z up. Image optical: +Z forward,
  // +X right, +Y down. Camera optical center is 0.09 m ahead of the IMU.
  Eigen::Matrix3d r_camera_to_imu;
  r_camera_to_imu << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Eigen::Matrix<double, 7, 1> extrinsics;
  extrinsics.head<4>() = ov_core::rot_2_quat(r_camera_to_imu.transpose());
  extrinsics.tail<3>() = -r_camera_to_imu.transpose() * Eigen::Vector3d(0.09, 0, 0);
  std::cerr << "CALIB width=640 height=360 hfov=1.3962634 fx=" << f
            << " imu_to_camera_flu=0.09,0,0 camera_to_imu_rotation_flu_optical="
            << "0,0,1,-1,0,0,0,-1,0\n";
  options.camera_extrinsics[0] = extrinsics;
  options.init_options.camera_intrinsics[0] = camera;
  options.init_options.camera_extrinsics[0] = extrinsics;
  ov_msckf::VioManager estimator(options);

  gz::transport::Node node;
  auto publisher = node.Advertise<gz::msgs::Odometry>("/perception/visual_odometry");
  std::mutex mutex;
  std::deque<gz::msgs::IMU> imu_queue;
  struct TimedImage { gz::msgs::Image msg; double received_mono_s; };
  std::deque<TimedImage> image_queue;
  std::deque<gz::msgs::IMU> imu_history;
  bool have_imu = false;
  std::size_t imu_count = 0, image_count = 0;
  node.Subscribe<gz::msgs::IMU>("/sensor_suite/imu", std::function<void(const gz::msgs::IMU &)>([&](const gz::msgs::IMU &m) {
    std::lock_guard<std::mutex> guard(mutex);
    imu_queue.push_back(m);
    if (imu_queue.size() > 1000) imu_queue.pop_front();
  }));
  node.Subscribe<gz::msgs::Image>("/sensor_suite/rgb", std::function<void(const gz::msgs::Image &)>([&](const gz::msgs::Image &m) {
    std::lock_guard<std::mutex> guard(mutex);
    image_queue.push_back({m, mono_seconds()});
    if (image_queue.size() > 20) image_queue.pop_front();
  }));
  bool anchored = false;
  bool seeded = false;
  std::deque<ov_core::ImuData> imu_window;
  Eigen::Matrix3d r_vio_to_enu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_vio0 = Eigen::Vector3d::Zero();
  double last_imu_stamp = -1, last_image_stamp = -1;
  while (running) {
    std::vector<gz::msgs::IMU> imus;
    std::vector<TimedImage> images;
    {
      std::lock_guard<std::mutex> guard(mutex);
      while (!imu_queue.empty()) {
        imus.push_back(std::move(imu_queue.front()));
        imu_queue.pop_front();
      }
      while (!image_queue.empty()) {
        images.push_back(std::move(image_queue.front()));
        image_queue.pop_front();
      }
    }
    if (imus.empty() && images.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
      continue;
    }
    std::sort(imus.begin(), imus.end(), [](const auto &a, const auto &b) { return stamp(a.header()) < stamp(b.header()); });
    for (const auto &m : imus) {
      double t = stamp(m.header());
      if (t <= last_imu_stamp) continue;
      last_imu_stamp = t;
      have_imu = true;
      imu_history.push_back(m);
      while (!imu_history.empty() && t - stamp(imu_history.front().header()) > 2.0)
        imu_history.pop_front();
      ov_core::ImuData data;
      data.timestamp = t;
      data.wm << m.angular_velocity().x(), m.angular_velocity().y(), m.angular_velocity().z();
      data.am << m.linear_acceleration().x(), m.linear_acceleration().y(), m.linear_acceleration().z();
      estimator.feed_measurement_imu(data);
      imu_window.push_back(data);
      while (!imu_window.empty() && data.timestamp - imu_window.front().timestamp > 1.2)
        imu_window.pop_front();
      ++imu_count;
    }
    std::sort(images.begin(), images.end(), [](const auto &a, const auto &b) { return stamp(a.msg.header()) < stamp(b.msg.header()); });
    for (const auto &item : images) {
      const auto &m = item.msg;
      double t = stamp(m.header());
      if (!have_imu || t <= last_image_stamp || t > last_imu_stamp ||
          m.width() != 640 || m.height() != 360 || m.step() < 640 * 3 ||
          m.data().size() < m.step() * m.height()) continue;
      const auto closest_imu = std::min_element(imu_history.begin(), imu_history.end(),
          [t](const auto &a, const auto &b) {
            return std::abs(stamp(a.header()) - t) < std::abs(stamp(b.header()) - t);
          });
      if (closest_imu == imu_history.end()) continue;
      const double imu_alignment_ms = 1000.0 * (stamp(closest_imu->header()) - t);
      if (std::abs(imu_alignment_ms) > 20.0) continue;
      last_image_stamp = t;
      const double processing_start_mono_s = mono_seconds();
      cv::Mat rgb(m.height(), m.width(), CV_8UC3, const_cast<char *>(m.data().data()), m.step());
      cv::Mat gray;
      cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
      ov_core::CameraData data;
      data.timestamp = t;
      data.sensor_ids = {0};
      data.images = {gray};
      // OpenVINS uses 255 for masked-out pixels; zero permits feature tracking.
      data.masks = {cv::Mat::zeros(gray.size(), CV_8UC1)};
      if (!seeded && imu_window.size() >= 80 &&
          imu_window.back().timestamp - imu_window.front().timestamp >= 1.0) {
        Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel = Eigen::Vector3d::Zero();
        for (const auto &sample : imu_window) {
          gyro += sample.wm;
          accel += sample.am;
        }
        gyro /= static_cast<double>(imu_window.size());
        accel /= static_cast<double>(imu_window.size());
        const auto &orientation = closest_imu->orientation();
        Eigen::Quaterniond q_imu_to_enu(orientation.w(), orientation.x(),
                                         orientation.y(), orientation.z());
        Eigen::Matrix3d r_enu_to_imu = q_imu_to_enu.normalized().toRotationMatrix().transpose();
        Eigen::Matrix<double, 17, 1> hover_state = Eigen::Matrix<double, 17, 1>::Zero();
        hover_state(0) = t - 0.02;
        hover_state.segment<4>(1) = ov_core::rot_2_quat(r_enu_to_imu);
        hover_state.segment<3>(11) = gyro;
        hover_state.segment<3>(14) = accel - r_enu_to_imu * Eigen::Vector3d(0, 0, 9.81);
        // This upstream method is named initialize_with_gt, but every value
        // above comes from the IMU and the declared stationary hover. No GT
        // pose, velocity or Gazebo odometry is passed to it.
        estimator.initialize_with_gt(hover_state);
        seeded = true;
        std::cerr << "OpenVINS seeded from stationary IMU at t=" << t << '\n';
      }
      estimator.feed_measurement_camera(data);
      const double vio_done_mono_s = mono_seconds();
      ++image_count;
      if (!estimator.initialized()) {
        if (image_count % 30 == 0) {
          std::cerr << "OpenVINS pending: images=" << image_count
                    << " state_stamp=" << estimator.get_state()->_timestamp
                    << " init_time=" << estimator.initialized_time() << '\n';
        }
        csv << t << ",0,,,,,,,," << imu_count << ',' << image_count << ','
            << item.received_mono_s << ',' << processing_start_mono_s << ','
            << vio_done_mono_s << ",," << imu_alignment_ms << std::string(15, ',') << '\n';
        csv.flush();
        continue;
      }
      auto state = estimator.get_state();
      auto position = state->_imu->pos();
      auto q = state->_imu->quat();
      if (!anchored) {
        const auto &s = closest_imu->orientation();
        Eigen::Quaterniond imu_to_enu(s.w(), s.x(), s.y(), s.z());
        r_vio_to_enu = imu_to_enu.normalized().toRotationMatrix() * ov_core::quat_2_Rot(q);
        p_vio0 = position;
        anchored = true;
        std::cerr << "OpenVINS initialized at t=" << t << " after " << image_count << " images\n";
      }
      Eigen::Vector3d p = Eigen::Vector3d(0, 0, 3) + r_vio_to_enu * (position - p_vio0);
      const Eigen::Matrix3d r_i_to_enu = r_vio_to_enu * ov_core::quat_2_Rot(q).transpose();
      Eigen::Quaterniond q_i_to_enu(r_i_to_enu);
      q_i_to_enu.normalize();
      double tracks_time = 0;
      std::unordered_map<size_t, Eigen::Vector3d> tracks_pos, tracks_uvd;
      estimator.get_active_tracks(tracks_time, tracks_pos, tracks_uvd);
      const auto velocity = state->_imu->vel();
      const auto bg = state->_imu->bias_g();
      const auto ba = state->_imu->bias_a();
      gz::msgs::Odometry odom;
      odom.mutable_header()->CopyFrom(m.header());
      odom.mutable_pose()->mutable_position()->set_x(p.x());
      odom.mutable_pose()->mutable_position()->set_y(p.y());
      odom.mutable_pose()->mutable_position()->set_z(p.z());
      auto *orientation = odom.mutable_pose()->mutable_orientation();
      orientation->set_x(q_i_to_enu.x());
      orientation->set_y(q_i_to_enu.y());
      orientation->set_z(q_i_to_enu.z());
      orientation->set_w(q_i_to_enu.w());
      publisher.Publish(odom);
      const double pose_published_mono_s = mono_seconds();
      csv << t << ",1," << p.x() << ',' << p.y() << ',' << p.z() << ','
          << q(0) << ',' << q(1) << ',' << q(2) << ',' << q(3) << ','
          << imu_count << ',' << image_count << ',' << item.received_mono_s << ','
          << processing_start_mono_s << ',' << vio_done_mono_s << ','
          << pose_published_mono_s << ',' << imu_alignment_ms << ','
          << velocity.x() << ',' << velocity.y() << ',' << velocity.z() << ','
          << bg.x() << ',' << bg.y() << ',' << bg.z() << ','
          << ba.x() << ',' << ba.y() << ',' << ba.z() << ','
          << tracks_uvd.size() << ',' << estimator.get_good_features_MSCKF().size() << ','
          << q_i_to_enu.x() << ',' << q_i_to_enu.y() << ','
          << q_i_to_enu.z() << ',' << q_i_to_enu.w() << '\n';
      csv.flush();
    }
  }
  std::cerr << "OpenVINS bridge stopped: " << imu_count << " IMU and " << image_count << " images\n";
  return 0;
}
