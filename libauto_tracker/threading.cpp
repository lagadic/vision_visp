#include "threading.h"


TrackerThread::TrackerThread(tracking::Tracker& tracker) : tracker_(tracker){
}

void TrackerThread::operator()(){
  tracker_.start(); //start state machine
}
