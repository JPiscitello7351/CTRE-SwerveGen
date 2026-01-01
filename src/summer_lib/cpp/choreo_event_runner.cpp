#include "summer_lib/choreo_event_runner.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

ChoreoEventRunner::ChoreoEventRunner(ChoreoEventManager &eventManager) : m_eventmanager{eventManager} {}

void ChoreoEventRunner::ScheduleActiveCommands(const std::vector<choreo::EventMarker>& events,
                                                units::second_t timestamp,
                                                units::millisecond_t offsetThreshold)
{
    frc::SmartDashboard::PutString("First Event:", events[0].event);
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
                    frc::SmartDashboard::PutString("Event Start Status", "Scheduling event: " + event.event);
                    
                    // Get Command Scheduler instance and run using scheduler command
                        //frc2::CommandScheduler& scheduler = frc2::CommandScheduler::GetInstance();
                        //scheduler.Schedule(command);

                    // Using command -> schedule
                        //command->Schedule();

                    m_triggeredEvents.push_back(event); // Mark this event as triggered
                    frc::SmartDashboard::PutString("Event Complete Status", "Triggered event: " + event.event);
                }
            }
        }
    }
}

void ChoreoEventRunner::ClearTriggeredEvents()
{
    m_triggeredEvents.clear();
}