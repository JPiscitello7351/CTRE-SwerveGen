#include "summer_lib/choreo_event_manager.h"

#include <choreo/trajectory/EventMarker.h>

ChoreoEventManager::ChoreoEventManager(){}

void ChoreoEventManager::AddKey(std::string key, frc2::CommandPtr command){
    m_stringToCommandPtrMap.emplace(std::make_pair(key, command.get()));
}

void ChoreoEventManager::DeleteKey(std::string key){
    auto toDelete = m_stringToCommandPtrMap.find(key);
    if (toDelete != m_stringToCommandPtrMap.end())
    {
        m_stringToCommandPtrMap.erase(toDelete);
    }
}

const std::map<std::string, frc2::Command*> &ChoreoEventManager::GetMap(){return m_stringToCommandPtrMap;}

frc2::Command* ChoreoEventManager::GetCommand(const std::string key){
    auto command = m_stringToCommandPtrMap.find(key);
    if (command != m_stringToCommandPtrMap.end())
    {
        return command->second;
    }
    return nullptr;
}

/*
std::vector<frc2::Command*> ChoreoEventManager::GetActiveEvents(const std::vector<choreo::EventMarker> &events, units::second_t timestamp, units::millisecond_t offsetThreshold)
{
    std::vector<frc2::Command*> activeCommands;
    for (const choreo::EventMarker &event : events)
    {
        if (event.timestamp >= timestamp - offsetThreshold && event.timestamp <= timestamp + offsetThreshold)
        {
            // Access string with event.event --> Map that to a command pointer from our map member
            auto command = m_stringToCommandPtrMap.find(event.event);
            if (command != m_stringToCommandPtrMap.end())
            {
                // Element found - do something
                activeCommands.emplace_back(command->second);
            }
        }
    }
    return activeCommands;
}
*/