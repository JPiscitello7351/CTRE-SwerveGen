#include "summer_lib/choreo_event_runner.h"
#include <frc2/command/CommandScheduler.h>

ChoreoEventRunner::ChoreoEventRunner(ChoreoEventManager &eventManager) : m_eventmanager{eventManager} {}

void ChoreoEventRunner::ScheduleActiveCommands(const std::vector<choreo::EventMarker>& events,
                                                units::second_t timestamp,
                                                units::millisecond_t offsetThreshold)
{
    
    for (const choreo::EventMarker &event : events)
    {
        if (event.timestamp >= timestamp - offsetThreshold && event.timestamp <= timestamp + offsetThreshold)
        {
            // Check if this event has already been triggered
            auto it = std::find_if(m_triggeredEvents.begin(), m_triggeredEvents.end(),
                                   [&event](const choreo::EventMarker &e) { return e.event == event.event; });
            if (it == m_triggeredEvents.end())
            {
                auto command = m_eventmanager.GetCommand(event.event);
                if (command != nullptr)
                {
                    frc2::CommandScheduler& scheduler = frc2::CommandScheduler::GetInstance();
                    scheduler.Schedule(command); 
                    m_triggeredEvents.push_back(event); // Mark this event as triggered
                }
            }
        }
    }
}

void ChoreoEventRunner::ClearTriggeredEvents()
{
    m_triggeredEvents.clear();
}