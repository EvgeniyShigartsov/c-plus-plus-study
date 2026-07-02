#pragma once
#include "types.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual Target getTarget(const int targetIndex) = 0;
  virtual bool isLoadSucces() = 0;
  virtual bool isThreadReady() const = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void run() = 0;
  virtual ~ITargetProvider() = default;
};
