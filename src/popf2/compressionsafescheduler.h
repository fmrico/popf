#ifndef COMPRESSIONSAFESCHEDULER_H
#define COMPRESSIONSAFESCHEDULER_H

#include "minimalstate.h"
#include "FFEvent.h"

namespace Planner {

class CompressionSafeScheduler
{

private:
    static bool safeToUseThis;
    
public:
    static void assignTimestamps(const MinimalState & s,
                                 list<FFEvent> & header,
                                 list<FFEvent> & now);
    
    static bool canUseThisScheduler();
};

};

#endif // COMPRESSIONSAFESCHEDULER_H
