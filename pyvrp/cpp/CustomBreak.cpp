#include "CustomBreak.h"

#include <limits>

using pyvrp::BreakRule;
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

BreakRule pyvrp::makeBreakRule(CustomBreak const &brk, Duration twEarlyAnchor)
{
    bool const hasWindow = !brk.tws.empty();

    int64_t openAbs = 0;
    int64_t closeAbs = std::numeric_limits<int64_t>::max();
    if (hasWindow)
    {
        int64_t const anchor
            = brk.twsRelative ? static_cast<int64_t>(twEarlyAnchor.get()) : 0;
        openAbs = anchor + static_cast<int64_t>(brk.tws.front().first.get());
        closeAbs = anchor + static_cast<int64_t>(brk.tws.back().second.get());
    }

    uint16_t supersedesMask = 0;
    for (auto const sid : brk.supersedes)
        supersedesMask |= static_cast<uint16_t>(1u) << (sid & 0xF);

    return BreakRule{
        static_cast<int64_t>(brk.triggerValue.get()),
        openAbs,
        closeAbs,
        static_cast<int64_t>(brk.conditionMinRouteS.get()),
        static_cast<int64_t>(brk.service.get()),
        brk.id,
        static_cast<uint16_t>(static_cast<uint16_t>(1u) << (brk.id & 0xF)),
        supersedesMask,
        brk.trigger,
        brk.reset,
        brk.mandatory,
        hasWindow,
        brk.twsRelative,
        brk.relaxable,
    };
}
