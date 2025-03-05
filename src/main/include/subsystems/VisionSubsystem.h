#include "frc/shuffleboard/Shuffleboard.h"
#include "photon/PhotonCamera.h"
#include "frc/TimedRobot.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandScheduler.h"
#include "frc2/command/SubsystemBase.h"
#include <cameraserver/CameraServer.h>

class VisionSubsystem : public frc2::SubsystemBase {
 public:
    VisionSubsystem();
    void Periodic() override;

 private:
    photon::PhotonCamera camera1;
    photon::PhotonCamera camera2;
};