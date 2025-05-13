#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <functional>
#include <limits>
#include <memory>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "Constants.h"

class Vision {
 public:
  /**
   * @param estConsumer Lamba that will accept a pose estimate and pass it to
   * your desired SwerveDrivePoseEstimator.
   */
  Vision(std::function<void(frc::Pose2d, units::second_t,
                            Eigen::Matrix<double, 3, 1>)>
             estConsumer)
      : estConsumer{estConsumer} {
    photonEstimator.SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);

    // if (frc::RobotBase::IsSimulation()) {
    //   visionSim = std::make_unique<photon::VisionSystemSim>("main");

    //   visionSim->AddAprilTags(constants::Vision::kTagLayout);

    //   cameraProp = std::make_unique<photon::SimCameraProperties>();

    //   cameraProp->SetCalibration(960, 720, frc::Rotation2d{90_deg});
    //   cameraProp->SetCalibError(.35, .10);
    //   cameraProp->SetFPS(15_Hz);
    //   cameraProp->SetAvgLatency(50_ms);
    //   cameraProp->SetLatencyStdDev(15_ms);

    //   cameraSim =
    //       std::make_shared<photon::PhotonCameraSim>(&camera, *cameraProp.get());

    //   visionSim->AddCamera(cameraSim.get(), constants::Vision::kRobotToCam);
    //   cameraSim->EnableDrawWireframe(true);
    // }
  }

  photon::PhotonPipelineResult GetLatestResult() { return m_latestResult; }

  void Periodic() {
    // Run each new pipeline result through our pose estimator
    for (const auto& result : camera.GetAllUnreadResults()) {
      // cache result and update pose estimator
      auto visionEst = photonEstimator.Update(result);
      m_latestResult = result;

      // In sim only, add our vision estimate to the sim debug field
      // if (frc::RobotBase::IsSimulation()) {
      //   if (visionEst) {
      //     GetSimDebugField()
      //         .GetObject("VisionEstimation")
      //         ->SetPose(visionEst->estimatedPose.ToPose2d());
      //   } else {
      //     GetSimDebugField().GetObject("VisionEstimation")->SetPoses({});
      //   }
      // }

      if (visionEst) {
        estConsumer(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp,
                    GetEstimationStdDevs(visionEst->estimatedPose.ToPose2d()));
      }
    }
  }

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs =
        Constants::kSingleTagStdDevs;
    auto targets = GetLatestResult().GetTargets();
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto& tgt : targets) {
      auto tagPose =
          photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose) {
        numTags++;
        avgDist += tagPose->ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }
    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = Constants::kMultiTagStdDevs;
    }
    if (numTags == 1 && avgDist > 4_m) {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                       .finished();
    } else {
      estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
  }

  void SimPeriodic(frc::Pose2d robotSimPose) {
    visionSim->Update(robotSimPose);
  }

  void ResetSimPose(frc::Pose2d pose) {
  //   if (frc::RobotBase::IsSimulation()) {
  //     visionSim->ResetRobotPose(pose);
  //   }
  }

  frc::Field2d& GetSimDebugField() { return visionSim->GetDebugField(); }

 private:
  photon::PhotonPoseEstimator photonEstimator{
      Constants::kTagLayout,
      photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      Constants::kRobotToCam};
  photon::PhotonCamera camera{"Arducam OV9281 USB Camera 001"};
  std::unique_ptr<photon::VisionSystemSim> visionSim;
  std::unique_ptr<photon::SimCameraProperties> cameraProp;
  std::shared_ptr<photon::PhotonCameraSim> cameraSim;

  // The most recent result, cached for calculating std devs
  photon::PhotonPipelineResult m_latestResult;
  std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)>
      estConsumer;
};