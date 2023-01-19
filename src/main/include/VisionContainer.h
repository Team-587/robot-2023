#pragma once

#include <thread>
#include "Constants.h"
#include <photonLib/PhotonUtils.h>
#include <photonLib/PhotonCamera.h>

class VisionContainer {

    private:

    volatile double yaw;
    volatile double pitch;
    volatile double area;
    volatile double skew;
    volatile double pose;



    void VisionThread() {
        photonlib::PhotonCamera m_camera{"PhotonVision"};
        while(true){
            
            photonlib::PhotonPipelineResult result = m_camera.GetLatestResult();

            if(result.HasTargets()){

                photonlib::PhotonTrackingTarget target = result.GetBestTarget();


            }

            sleep(100);
        }
    }

    public:



    VisionContainer() {



    }

    void setPipeline(int pipeline) {
        m_camera.pipelineIndex(pipeline);
    }

    double getYaw() {
        return yaw;
    }

    double getPitch() {
        return pitch;
    }

    double getArea() {
        return area;
    }

    double getSkew() {
        return skew;
    }

    double getPose() {
        return pose;
    }
    
    double getDistance() {
        return photonlib::PhotonUtils::CalculateDistanceToTarget(
            Camerapos::cam_height_meters, 
            Camerapos::goal_height_meters, 
            units::degree_t(Camerapos::cam_angle_degrees), 
            units::degree_t(pitch)
            ).value();
    }

    

    void start() {
        std::thread m_thread(&VisionContainer::VisionThread, this);
        m_thread.detach();
    }



}