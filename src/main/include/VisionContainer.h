#pragma once

#include <thread>
#include <photonLib/PhotonUtils.h>
#include <photonLib/PhotonCamera.h>
#include <mutex>

#include "Constants.h"

class VisionContainer {

    private:

        photonlib::PhotonCamera *camera;

        std::mutex cameraLock;

        volatile int pipelineIndex;
        volatile bool hasTarget;
        volatile double yaw;
        volatile double pitch;
        volatile double area;
        volatile double skew;
        //volatile double *pose;


        void VisionThread() {
            
            camera = new photonlib::PhotonCamera("PhotonVisionCam1");
            camera->SetPipelineIndex(pipelineIndex);

            while(true){
                
                ProcessCamera();
                
                sleep(100);

            }
        }

        void ProcessCamera() {

            std::lock_guard<std::mutex> guard(cameraLock);

            photonlib::PhotonPipelineResult result = camera->GetLatestResult();

            hasTarget = result.HasTargets();

            if(result.HasTargets()){

                photonlib::PhotonTrackedTarget target = result.GetBestTarget();
                
                //set volatile variables here
                yaw = target.GetYaw();
                pitch = target.GetPitch();
                area = target.GetArea();
                skew = target.GetSkew();

            }
        }

    public:

        VisionContainer(int pipelineIndex = 0) {
            this->pipelineIndex = pipelineIndex;
        }

        VisionContainer(const VisionContainer&) { }

        //void setPipeline(int pipeline) {
        //    std::lock_guard<std::mutex> guard(cameraLock);
        //    pipelineIndex = pipeline;
        //    camera->SetPipelineIndex(pipeline);
        //}

        bool getHasTarget() {
            return hasTarget;
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

        //double* getPose() {
        //    return pose;
        //}
        
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



};