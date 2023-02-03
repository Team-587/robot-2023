#pragma once

#include <thread>
#include <photonLib/PhotonUtils.h>
#include <photonLib/PhotonCamera.h>
#include <mutex>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"
#include <iostream>

class VisionContainer {

    private:

        photonlib::PhotonCamera *camera;

        //std::mutex cameraLock;

public:
        volatile int pipelineIndex;
        volatile bool hasTarget;
        volatile double yaw;
        volatile double pitch;
        volatile double area;
        volatile double skew;
        volatile bool stopThread = false;
        int count = 0; 
        //volatile double *pose;


        void VisionThread() {
            
           
           // frc::SmartDashboard::PutString("VisionState", "ThreadStart");
           std::cout<<"ThreadStart"<<"\n";
            camera = new photonlib::PhotonCamera("HD_Pro_Webcam_C920");
           std::cout<<"GetCamera"<<"\n";
           // frc::SmartDashboard::PutString("VisionState", "GetCamera");
            camera->SetPipelineIndex(pipelineIndex);
           std::cout<<"SetPipeline"<<"\n";
            //frc::SmartDashboard::PutString("VisionState", "SetPipoeline");

            while(!stopThread){
           std::cout<<"ProcessCamera"<<"\n";
            //    frc::SmartDashboard::PutString("VisionState", "ProcessCamera");
                ProcessCamera();
           //std::cout<<"Processed"<<"\n";
              // frc::SmartDashboard::PutString("VisionState", "Processed");
             //  std::cout<<count++<<"\n";
            
                usleep(15000);
    

            }
        }

        void ProcessCamera() {

//frc::SmartDashboard::PutString("VisionState", "PreLock");
            //std::lock_guard<std::mutex> guard(cameraLock);
//frc::SmartDashboard::PutString("VisionState", "LockAquired");

//           std::cout<<"ProcessCameraStart"<<"\n";
            photonlib::PhotonPipelineResult result = camera->GetLatestResult();
           //std::cout<<"ProcessGotTarget"<<"\n";
//frc::SmartDashboard::PutString("VisionState", "ResultAcquired");
            hasTarget = result.HasTargets();

            if(result.HasTargets()){

                photonlib::PhotonTrackedTarget target = result.GetBestTarget();
                
                //set volatile variables here
                yaw = target.GetYaw();
                pitch = target.GetPitch();
                area = target.GetArea();
                skew = target.GetSkew();


            }
            frc::SmartDashboard::PutBoolean("hasTarget", hasTarget);
            frc::SmartDashboard::PutNumber("yaw", yaw);
            frc::SmartDashboard::PutNumber("pitch", pitch);


        }

    public:

        VisionContainer(int pipelineIndex = 0) {
            this->pipelineIndex = pipelineIndex;
        }

        //VisionContainer(const VisionContainer&) { }

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

        void stop() { stopThread = true; }



};