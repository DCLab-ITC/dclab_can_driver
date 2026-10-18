// Gives a CANable (candleLight) back to Linux's CAN driver: the C++ twin of
// can_driver/canable_reclaim.py (see there). Returns how many it gave back;
// log(message) reports each one.
#pragma once

#include <functional>
#include <string>

namespace dclab
{
int reclaim_canables(const std::function<void(const std::string &)> & log);
}  // namespace dclab
