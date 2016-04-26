#ifndef FFEVENT_H
#define FFEVENT_H

#include <ptree.h>

namespace Inst {
    class instantiatedOp;
};

using Inst::instantiatedOp;

#include <set>
using std::set;

namespace Planner {

#ifdef STOCHASTICDURATIONS
class StochasticTimestampData;
#endif


class FFEvent
{

public:

    static int tilLimit;

    instantiatedOp* action;
    VAL::time_spec time_spec;
    double minDuration;
    double maxDuration;
    int pairWithStep;
//  ScheduleNode* wait;
    bool getEffects;
    double lpTimestamp;
    double lpMinTimestamp;
    double lpMaxTimestamp;
    int divisionID;
    set<int> needToFinish;
    
    #ifdef STOCHASTICDURATIONS
    StochasticTimestampData * stochasticTimestamp;
    #endif
    
    FFEvent(instantiatedOp* a, const double & dMin, const double & dMax);
    FFEvent(instantiatedOp* a, const int & pw, const double & dMin, const double & dMax);
    FFEvent(instantiatedOp* a, const int & s, const int & pw, const double & dMin, const double & dMax);
    FFEvent(const int & t);

    virtual ~FFEvent();
    
    virtual void passInMinMax(const double & a, const double & b) {
        lpMinTimestamp = a;
        lpMaxTimestamp = b;
    }

    FFEvent(const FFEvent & f);
//  FFEvent(ScheduleNode* s, const bool & b);
    FFEvent();
    FFEvent & operator=(const FFEvent & f);
    bool operator==(const FFEvent & f) const {
        if (time_spec != VAL::E_AT_START && pairWithStep != f.pairWithStep) return false;
        return (action == f.action && time_spec == f.time_spec && minDuration == f.minDuration && maxDuration == f.maxDuration && pairWithStep == f.pairWithStep && getEffects == f.getEffects && divisionID == f.divisionID);
    }


    static void printPlan(const list<FFEvent> & toPrint);

};

};

#endif
