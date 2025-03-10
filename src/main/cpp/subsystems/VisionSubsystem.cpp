#include "subsystems/VisionSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <photon/PhotonCamera.h>
#include <Constants.h>


VisionSubsystem::VisionSubsystem() : camera1("Arducam OV9281 USB Camera 001"), camera2("Arducam OV9281 USB Camera 002") {
    camera1.SetDriverMode(false); // Driver mode is an unfiltered / normal view of the camera to be used while driving the robot.
    camera2.SetDriverMode(false); // Driver mode is an unfiltered / normal view of the camera to be used while driving the robot.

    frc::Shuffleboard::GetTab(Constants::kDriverTabName)
        .AddCamera("Climber Camera", "Arducam OV9281 USB Camera 001", std::vector<std::string>{"http://photonvision.local:1182/stream.mjpg"}) //http://photonvision.local:1182/stream.mjpg
        .WithSize(4, 4)
        .WithPosition(6, 0);

    frc::Shuffleboard::GetTab(Constants::kDriverTabName)
        .AddCamera("Claw Camera", "Arducam OV9281 USB Camera 002", std::vector<std::string>{"http://photonvision.local:1184/stream.mjpg"})
        .WithSize(4, 4)
        .WithPosition(2, 0);

  }


void VisionSubsystem::Periodic() {
    #if 0
    auto results1 = camera1.GetAllUnreadResults();
    auto result1 = results1.empty() ? photon::PhotonPipelineResult() : results1.back();
    auto results2 = camera2.GetAllUnreadResults();
    auto result2 = results2.empty() ? photon::PhotonPipelineResult() : results2.back();

    frc::SmartDashboard::PutBoolean("Camera1 Has Target", result1.HasTargets());
    frc::SmartDashboard::PutBoolean("Camera2 Has Target", result2.HasTargets());
    #endif
}