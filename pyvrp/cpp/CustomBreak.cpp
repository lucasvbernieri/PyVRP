#include "CustomBreak.h"

using pyvrp::CustomBreak;

CustomBreak::CustomBreak(size_t id,
                         std::vector<std::pair<Duration, Duration>> tws,
                         Duration service,
                         CustomBreakTrigger trigger,
                         Duration triggerValue,
                         CustomBreakReset reset,
                         bool mandatory,
                         Duration conditionMinRouteS,
                         int priority,
                         std::vector<size_t> supersedes,
                         bool twsRelative,
                         bool relaxable)
    : id(id),
      tws(std::move(tws)),
      service(service),
      trigger(trigger),
      triggerValue(triggerValue),
      reset(reset),
      mandatory(mandatory),
      conditionMinRouteS(conditionMinRouteS),
      priority(priority),
      supersedes(std::move(supersedes)),
      twsRelative(twsRelative),
      relaxable(relaxable)
{
}

bool CustomBreak::isValidStart(Duration arrival) const
{
    for (auto const &[open, close] : tws)
        if (arrival >= open && arrival < close)
            return true;

    return false;
}
