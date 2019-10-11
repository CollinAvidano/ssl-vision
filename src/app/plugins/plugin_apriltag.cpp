#include "plugin_apriltag.h"
#include "messages_robocup_ssl_detection.pb.h"
#include <Eigen/Dense>
#include <apriltag_pose.h>
#include <array>
#include <cstdio>
#include <image.h>
#include <iostream>
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom20h7.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>

#define DBG_MACRO_DISABLE
#include <dbg.h>

using HammHist = std::array<int, 10>;
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

PluginAprilTag::PluginAprilTag(FrameBuffer *buffer,
                               const CameraParameters &camera_params)
    : VisionPlugin(buffer), settings(new VarList("AprilTag")),
      v_enable(new VarBool("enable", true)), v_iters(new VarInt("iters", 1)),
      v_threads(new VarInt("threads", 1)), v_hamming(new VarInt("hamming", 1)),
      v_decimate(new VarDouble("decimate", 2.0)),
      v_blur(new VarDouble("blur", 0.0)),
      v_refine_edges(new VarBool("refine-edges", true)),
      detections{nullptr, &apriltag_detections_destroy},
      camera_params(camera_params) {

  v_family.reset(new VarStringEnum("Tag Family", "tagCircle21h7"));
  v_family->addItem("tag36h11");
  v_family->addItem("tag16h5");
  v_family->addItem("tagCircle21h7");
  v_family->addItem("tagCircle49h12");
  v_family->addItem("tagStandard41h12");
  v_family->addItem("tagStandard52h13");
  v_family->addItem("tagCustom48h12");
  v_family->addItem("tagCustom20h7");

  settings->addChild(v_enable.get());
  settings->addChild(v_family.get());
  settings->addChild(v_iters.get());
  settings->addChild(v_threads.get());
  settings->addChild(v_hamming.get());
  settings->addChild(v_decimate.get());
  settings->addChild(v_blur.get());
  settings->addChild(v_refine_edges.get());

  // construct apriltag detector with these settings
  makeTagFamily();
  makeTagDetector();
}

ProcessResult PluginAprilTag::process(FrameData *data, RenderOptions *options) {
  if (!v_enable->getBool()) {
    return ProcessingOk;
  }
  if (!tag_family || v_family->getString() != tag_family->name) {
    makeTagFamily();
    if (!tag_family) {
      std::cerr << "AprilTag: Failed to create tag family '"
                << v_family->getString() << "'\n";
      return ProcessingFailed;
    }

    apriltag_detector_clear_families(tag_detector.get());
    apriltag_detector_add_family_bits(tag_detector.get(), tag_family.get(),
                                      v_hamming->getInt());
  }

  if (!tag_detector) {
    makeTagDetector();
    if (!tag_detector) {
      std::cerr << "AprilTag: Failed to create tag detector\n";
      return ProcessingFailed;
    }
  }

  // TODO(dschwab): Add option for debug output
  tag_detector->quad_decimate = v_decimate->getDouble();
  tag_detector->quad_sigma = v_blur->getDouble();
  tag_detector->nthreads = v_threads->getInt();
  tag_detector->refine_edges = v_refine_edges->getBool();

  Image<raw8> *img_greyscale =
      reinterpret_cast<Image<raw8> *>(data->map.get("greyscale"));
  if (img_greyscale == nullptr) {
    std::cerr << "AprilTag: No greyscale image in frame data\n";
    return ProcessingFailed;
  }

  const int maxiters = v_iters->getInt();
  for (int iter = 0; iter < maxiters; iter++) {
    if (maxiters > 1) {
      std::cout << "iter " << iter + 1 << " / " << maxiters << "\n";
    }

    // zero-copy. Just use the already allocated buffer data.
    image_u8_t im{.width = img_greyscale->getWidth(),
                  .height = img_greyscale->getHeight(),
                  .stride = img_greyscale->getWidth(),
                  .buf = img_greyscale->getData()};

    detections.reset(apriltag_detector_detect(tag_detector.get(), &im));
    data->map.update("apriltag_detections", detections.get());

    // add the detections to the robot list so that the positions will
    // be published
    SSL_DetectionFrame *detection_frame = 0;
    detection_frame =
        (SSL_DetectionFrame *)data->map.get("ssl_detection_frame");
    if (detection_frame == 0)
      detection_frame = (SSL_DetectionFrame *)data->map.insert(
          "ssl_detection_frame", new SSL_DetectionFrame());

    // add the detections to the detected robots list
    apriltag_detection_info_t info;
    info.tagsize = 0.07; // in meters
    info.fx = camera_params.focal_length->getDouble();
    info.fy = camera_params.focal_length->getDouble();
    info.cx = camera_params.principal_point_x->getDouble();
    info.cy = camera_params.principal_point_y->getDouble();

    const Isometry3d field_to_cam_transform = [&]() {
      const Vector3d cam_pos{camera_params.tx->getDouble(),
                             camera_params.ty->getDouble(),
                             camera_params.tz->getDouble()};
      Quaterniond cam_quat{
          camera_params.q0->getDouble(), camera_params.q1->getDouble(),
          camera_params.q2->getDouble(), camera_params.q3->getDouble()};
      cam_quat.norm();

      Isometry3d cam_transform;
      cam_transform.fromPositionOrientationScale(
          cam_pos, cam_quat, /*cam_scale*/ Vector3d::Constant(1));
      return cam_transform;
    }();

    const AngleAxisd cam_to_aprilcam_rot{M_PI, Vector3d::UnitZ()};
    const Isometry3d field_to_aprilcam_transform =
        field_to_cam_transform * cam_to_aprilcam_rot;
    const Isometry3d aprilcam_to_field_transform =
        field_to_aprilcam_transform.inverse();
    const Vector3d tag_to_floor_trans{0, 0, 0};

    // // DEBUG ROBOTS
    // // Used to check coordinate transforms in vision client as fixed references

    // // CP0
    // {
    //   auto robot_detection = detection_frame->add_robots_yellow();
    //   robot_detection->set_confidence(100);
    //   robot_detection->set_robot_id(0);
    //   robot_detection->set_x(-2577);
    //   robot_detection->set_y(-1655);
    //   robot_detection->set_orientation(0);
    //   robot_detection->set_pixel_x(0);
    //   robot_detection->set_pixel_y(0);
    // }
    // // CP1
    // {
    //   auto robot_detection = detection_frame->add_robots_yellow();
    //   robot_detection->set_confidence(100);
    //   robot_detection->set_robot_id(1);
    //   robot_detection->set_x(0);
    //   robot_detection->set_y(-1655);
    //   robot_detection->set_orientation(M_PI / 3);
    //   robot_detection->set_pixel_x(0);
    //   robot_detection->set_pixel_y(0);
    // }
    // // CP2
    // {
    //   auto robot_detection = detection_frame->add_robots_yellow();
    //   robot_detection->set_confidence(100);
    //   robot_detection->set_robot_id(2);
    //   robot_detection->set_x(2577);
    //   robot_detection->set_y(-1655);
    //   robot_detection->set_orientation(2 * M_PI / 3);
    //   robot_detection->set_pixel_x(0);
    //   robot_detection->set_pixel_y(0);
    // }
    // // CP3
    // {
    //   auto robot_detection = detection_frame->add_robots_yellow();
    //   robot_detection->set_confidence(100);
    //   robot_detection->set_robot_id(3);
    //   robot_detection->set_x(2577);
    //   robot_detection->set_y(0);
    //   robot_detection->set_orientation(M_PI);
    //   robot_detection->set_pixel_x(0);
    //   robot_detection->set_pixel_y(0);
    // }
    // // CP4
    // {
    //   auto robot_detection = detection_frame->add_robots_yellow();
    //   robot_detection->set_confidence(100);
    //   robot_detection->set_robot_id(4);
    //   robot_detection->set_x(0);
    //   robot_detection->set_y(0);
    //   robot_detection->set_orientation(4 * M_PI / 3);
    //   robot_detection->set_pixel_x(0);
    //   robot_detection->set_pixel_y(0);
    // }
    // // CP5
    // {
    //   auto robot_detection = detection_frame->add_robots_yellow();
    //   robot_detection->set_confidence(100);
    //   robot_detection->set_robot_id(5);
    //   robot_detection->set_x(-2577);
    //   robot_detection->set_y(0);
    //   robot_detection->set_orientation(5 * M_PI / 3);
    //   robot_detection->set_pixel_x(0);
    //   robot_detection->set_pixel_y(0);
    // }
    // std::cout << "camera translation:\n"
    //           << cam_transform.translation() << "\ncam rotation:\n"
    //           << cam_transform.rotation() << "\n";

    for (int i = 0; i < zarray_size(detections.get()); ++i) {
      apriltag_detection_t *det;
      zarray_get(detections.get(), i, &det);

      // TODO(dschwab): Maybe I should use the included apriltags pose
      // detection? If it is using the quad info and not just the
      // image center we would probably get better accuracy when the
      // camera isn't perfectly orthogonal to the tag.
      info.det = det;
      apriltag_pose_t pose;
      estimate_tag_pose(&info, &pose);
      dbg(det->id);

      const Isometry3d aprilcam_to_tag_transform = [&]() {
        const Vector3d tag_pos{1000.0 * pose.t->data[0],
                               1000.0 * pose.t->data[1],
                               1000.0 * pose.t->data[2]};
        Matrix3d tag_rot;
        tag_rot << pose.R->data[0], pose.R->data[1], pose.R->data[2],
            pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.R->data[6],
            pose.R->data[7], pose.R->data[8];

        Isometry3d tag_transform;
        tag_transform.fromPositionOrientationScale(
            tag_pos, tag_rot,
            /*tag_scale*/ Vector3d::Constant(1));
        return tag_transform;
      }();

      const Vector3d tag_pos_in_aprilcam{1000.0 * pose.t->data[0],
                                         1000.0 * pose.t->data[1],
                                         1000.0 * pose.t->data[2]};

      Matrix3d tag_rot_in_aprilcam;
      tag_rot_in_aprilcam << pose.R->data[0], pose.R->data[1], pose.R->data[2],
          pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.R->data[6],
          pose.R->data[7], pose.R->data[8];
      const Matrix3d tag_rot_in_field =
          aprilcam_to_field_transform.linear() * tag_rot_in_aprilcam;

      const Vector3d tag_x_in_field = tag_rot_in_field.col(0);
      dbg(tag_rot_in_field);
      dbg(tag_x_in_field);

      const double angle =
          atan2(tag_x_in_field[1], tag_x_in_field[0]) + M_PI / 2.0;

      vector2d reg_img_center(det->c[0], det->c[1]);
      vector3d reg_center3d;
      // TODO(dschwab): Actually get the robot height from the
      // configured team. For now, this is hardcoded to cmdragons size
      camera_params.image2field(reg_center3d, reg_img_center, 140);
      
      // TODO(dschwab): add all the detections to team blue for
      // now. Should add a config option to assign id to teams and
      // then use that info here.
      auto robot_detection = detection_frame->add_robots_blue();
      // printf("%d: hamming: %d, decision_margin: %f\n", det->id, det->hamming,
      //        det->decision_margin);
      robot_detection->set_confidence(
          det->decision_margin); // TODO(dschwab): What value should I put here?
      robot_detection->set_robot_id(det->id);
      // robot_detection->set_x(tag_pos_in_field.x());
      // robot_detection->set_y(tag_pos_in_field.y());
      robot_detection->set_x(reg_center3d.x);
      robot_detection->set_y(reg_center3d.y);
      robot_detection->set_orientation(angle);
      robot_detection->set_pixel_x(det->c[0]);
      robot_detection->set_pixel_y(det->c[1]);
      robot_detection->set_height(140); // TODO(dschwab): Use actual team height

      // printf("Tag %d at (%.3f, %.3f, %.3f)\n", det->id, reg_center3d.x,
      //        reg_center3d.y, reg_center3d.z);
      // printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin
      //            % 8.3f\n ", i,
      //              det->family->nbits,
      //        det->family->h, det->id, det->hamming, det->decision_margin);
    }

    // printf("apriltag found %d blue robots\n",
    //        detection_frame->robots_blue().size());
    // printf("apriltag found %d yellow robots\n",
    //        detection_frame->robots_yellow().size());
  }

  return ProcessingOk;
}

VarList *PluginAprilTag::getSettings() { return settings.get(); }

std::string PluginAprilTag::getName() { return "AprilTag"; }

void PluginAprilTag::makeTagFamily() {
  const auto tag_name = v_family->getString();
  if (tag_name == "tag36h11") {
    tag_family = std::unique_ptr<apriltag_family_t,
                                 std::function<void(apriltag_family_t *)>>{
        tag36h11_create(), &tag36h11_destroy};
  } else if (tag_name == "tag25h9") {
    tag_family = std::unique_ptr<apriltag_family_t,
                                 std::function<void(apriltag_family_t *)>>{
        tag25h9_create(), &tag25h9_destroy};
  } else if (tag_name == "tag16h5") {
    tag_family = std::unique_ptr<apriltag_family_t,
                                 std::function<void(apriltag_family_t *)>>{
        tag16h5_create(), &tag16h5_destroy};
  } else if (tag_name == "tagCircle21h7") {
    tag_family = std::unique_ptr<apriltag_family_t,
                                 std::function<void(apriltag_family_t *)>>{
        tagCircle21h7_create(), &tagCircle21h7_destroy};
  } else if (tag_name == "tagCircle49h12") {
    tag_family = std::unique_ptr<apriltag_family_t,
                                 std::function<void(apriltag_family_t *)>>{
        tagCircle49h12_create(), &tagCircle49h12_destroy};
  } else if (tag_name == "tagStandard41h12") {
    tag_family = std::unique_ptr<apriltag_family_t,
                                 std::function<void(apriltag_family_t *)>>{
        tagStandard41h12_create(), &tagStandard41h12_destroy};
  } else if (tag_name == "tagStandard52h13") {
    tag_family = std::unique_ptr<apriltag_family_t,
                                 std::function<void(apriltag_family_t *)>>{
        tagStandard52h13_create(), &tagStandard52h13_destroy};
  } else if (tag_name == "tagCustom48h12") {
    tag_family = std::unique_ptr<apriltag_family_t,
                                 std::function<void(apriltag_family_t *)>>{
        tagCustom48h12_create(), &tagCustom48h12_destroy};
  } else if (tag_name == "tagCustom20h7") {
    tag_family = std::unique_ptr<apriltag_family_t,
                                 std::function<void(apriltag_family_t *)>>{
        tagCustom20h7_create(), &tagCustom20h7_destroy};
  } else {
    std::cerr << "AprilTag: Failed to create apriltag family. Unknown family '"
              << tag_name << "'\n";
    tag_family = std::unique_ptr<apriltag_family_t,
                                 std::function<void(apriltag_family_t *)>>{
        nullptr, [](apriltag_family_t *) {}};
  }
}

void PluginAprilTag::makeTagDetector() {
  if (!tag_family) {
    std::cerr
        << "AprilTag: Cannot construct tag detector because tag family has not "
           "been created\n";
    tag_detector = std::unique_ptr<apriltag_detector_t,
                                   std::function<void(apriltag_detector_t *)>>{
        nullptr, [](apriltag_detector_t *) {}};
    return;
  }

  tag_detector = std::unique_ptr<apriltag_detector_t,
                                 std::function<void(apriltag_detector_t *)>>{
      apriltag_detector_create(), &apriltag_detector_destroy};

  apriltag_detector_add_family_bits(tag_detector.get(), tag_family.get(),
                                    v_hamming->getInt());
}
