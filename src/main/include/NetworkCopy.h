
#pragma once

#include <ntcore.h>


class NetworkCopy {
  
  private:
    
    nt::NetworkTableEntry read;
    nt::NetworkTableEntry write;

  public:
  
    NetworkCopy() {
      
      nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
      auto rootTable = inst.GetTable("photonvision");
      auto subTable = rootTable->GetSubTable("PhotonVisionCam1");
      
      read = subTable->GetEntry("rawBytes");
      write = subTable->GetEntry("raw");

    }

    void Periodic() {
      nt::RawSubscriber::ParamType defaultValue { };
      auto value = read.GetRaw(defaultValue);
      write.SetRaw(std::span{value.begin(), value.end()});
      
    }
};