#pragma once

#include <chrono>

namespace point_lio
{

inline double wall_time()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

}  // namespace point_lio
