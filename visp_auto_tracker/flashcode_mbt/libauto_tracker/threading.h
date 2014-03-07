#ifndef __THREADING_H__
#define __THREADING_H__
#include "tracking.h"

class TrackerThread{
private:
  tracking::Tracker& tracker_;
public:
  TrackerThread(tracking::Tracker& tracker);
  void operator()();
};
#endif
