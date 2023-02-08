#pragma once

#include <thread>
#include <photonLib/PhotonUtils.h>
#include <photonLib/PhotonCamera.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"
#include <iostream>

class VisionContainer
{
  private:
    
    photonlib::PhotonCamera *camera;

  public:
    
    std::string cameraName;
    volatile int pipelineIndex;
    volatile bool hasTarget;
    volatile double yaw;
    volatile double pitch;
    volatile double area;
    volatile double skew;
    volatile bool stopThread = false;
    int count = 0;
    // volatile double *pose;

    void VisionThread() {

      camera = new photonlib::PhotonCamera(cameraName);
      camera->SetPipelineIndex(pipelineIndex);
      while (!stopThread) {
        
        ProcessCamera();
        
        usleep(15000);
      }
    }

    void ProcessCamera() {

      photonlib::PhotonPipelineResult result = camera->GetLatestResult();
      hasTarget = result.HasTargets();

      if (result.HasTargets()) {

        photonlib::PhotonTrackedTarget target = result.GetBestTarget();

        // set volatile variables here
        yaw = target.GetYaw();
        pitch = target.GetPitch();
        area = target.GetArea();
        skew = target.GetSkew();
      }
      frc::SmartDashboard::PutBoolean(cameraName + ":hasTarget", hasTarget);
      frc::SmartDashboard::PutNumber(cameraName + ":yaw", yaw);
      frc::SmartDashboard::PutNumber(cameraName + ":-pitch", pitch);
    }

  public:

    VisionContainer(std::string cameraName, int pipelineIndex){
      this->cameraName = cameraName;
      this->pipelineIndex = pipelineIndex;
    }

    bool getHasTarget() { return hasTarget; }
    double getYaw() { return yaw; }
    double getPitch() { return pitch; }
    double getArea() { return area; }
    double getSkew() { return skew; }

    //need to tune the distance calc if we are going to use it
    ///////////////////////////////////////////////////////////////
    double getDistance() {
      return photonlib::PhotonUtils::CalculateDistanceToTarget(
                Camerapos::cam_height_meters,
                Camerapos::goal_height_meters,
                units::degree_t(Camerapos::cam_angle_degrees),
                units::degree_t(pitch))
          .value();
    }

    void start()
    {
      std::thread m_thread(&VisionContainer::VisionThread, this);
      m_thread.detach();
    }

    void stop() { stopThread = true; }
};