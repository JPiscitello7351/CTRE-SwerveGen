#include "summer_lib/choreo_event_runner.h"

ChoreoEventRunner::ChoreoEventRunner(const ChoreoEventManager &eventManager) : m_eventmanager{eventManager} {}

void ChoreoEventRunner::ScheduleActiveCommands(const std::vector<choreo::EventMarker>& events,
                                                units::second_t timestamp,
                                                units::millisecond_t offsetThreshold)
{
    //std::vector<frc2::Command *> commands = m_eventmanager.GetActiveEvents(events, timestamp, offsetThreshold);
}