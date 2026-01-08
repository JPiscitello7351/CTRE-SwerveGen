#pragma once

#include <map>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <choreo/trajectory/EventMarker.h>


// TODO: Class for ChoreoEventManager here

class ChoreoEventManager {

// TODO: Create string to pointer map

public:
    // Functions
    ChoreoEventManager();

    void AddKey(std::string key, frc2::CommandPtr command);
    void DeleteKey(std::string key);
    frc2::CommandPtr* GetCommand(const std::string key);
    const std::unordered_map<std::string, frc2::CommandPtr> &GetMap();
    //std::vector<frc2::Command*> GetActiveEvents(const std::vector<choreo::EventMarker> &events, units::second_t timestamp, units::millisecond_t offsetThreshold);
private:
    // Member Variables
    std::unordered_map<std::string, frc2::CommandPtr> m_stringToCommandPtrMap;


};