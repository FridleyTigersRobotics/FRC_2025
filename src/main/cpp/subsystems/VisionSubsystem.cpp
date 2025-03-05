#include "subsystems/VisionSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>


VisionSubsystem::VisionSubsystem() : camera1("Arducam OV9281 USB Camera 001"), camera2("Arducam OV9281 USB Camera 002") {
    frc::CameraServer::StartAutomaticCapture(0);
    frc::CameraServer::StartAutomaticCapture(1);

    frc::Shuffleboard::GetTab("Vision")
        .AddCamera("Camera 1", "Arducam OV9281 USB Camera 001", std::vector<std::string>{"http://photonvision.local:1181/stream.mjpg"})//could be 1182
        .WithSize(3, 3)
        .WithPosition(0, 0);

    frc::Shuffleboard::GetTab("Vision")
        .AddCamera("Camera 2", "Arducam OV9281 USB Camera 002", std::vector<std::string>{"http://photonvision.local:1183/stream.mjpg"})//could be 1184
        .WithSize(3, 3)
        .WithPosition(3, 0);
  }


void VisionSubsystem::Periodic() {
    auto results1 = camera1.GetAllUnreadResults();
    auto result1 = results1.empty() ? photon::PhotonPipelineResult() : results1.back();
    auto results2 = camera2.GetAllUnreadResults();
    auto result2 = results2.empty() ? photon::PhotonPipelineResult() : results2.back();

    frc::SmartDashboard::PutBoolean("Camera1 Has Target", result1.HasTargets());
    frc::SmartDashboard::PutBoolean("Camera2 Has Target", result2.HasTargets());
}