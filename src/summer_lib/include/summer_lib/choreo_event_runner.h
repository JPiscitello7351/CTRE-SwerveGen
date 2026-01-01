#pragma once

#include "summer_lib/choreo_event_manager.h"
#include <frc2/command/Command.h>
#include <choreo/trajectory/EventMarker.h>
#include <frc2/command/CommandScheduler.h>
#include <vector>

class ChoreoEventRunner {
 public:
  ChoreoEventRunner(ChoreoEventManager &eventManager);

  void ScheduleActiveCommands(const std::vector<choreo::EventMarker>& events,
                              units::second_t timestamp,
                              units::millisecond_t offsetThreshold);

  void ClearTriggeredEvents();
 private:
    std::vector<choreo::EventMarker> m_triggeredEvents;
    ChoreoEventManager &m_eventmanager;
};