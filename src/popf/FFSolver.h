/************************************************************************
 * Copyright 2010, Strathclyde Planning Group,
 * Department of Computer and Information Sciences,
 * University of Strathclyde, Glasgow, UK
 * http://planning.cis.strath.ac.uk/
 *
 * Andrew Coles, Amanda Coles - Code for POPF
 * Maria Fox, Richard Howey and Derek Long - Code from VAL
 * Stephen Cresswell - PDDL Parser
 *
 * This file is part of the planner POPF.
 *
 * POPF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * POPF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with POPF.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************/

#ifndef __FFSOLVER
#define __FFSOLVER

#include "RPGBuilder.h"

#include <ptree.h>

#include <map>
#include <list>

using std::map;
using std::list;


namespace Planner
{

class SearchQueueItem;
class ParentData;

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

    FFEvent(instantiatedOp* a, const double & dMin, const double & dMax);
    FFEvent(instantiatedOp* a, const int & pw, const double & dMin, const double & dMax);
    FFEvent(instantiatedOp* a, const int & s, const int & pw, const double & dMin, const double & dMax);
    FFEvent(const int & t);

    virtual ~FFEvent() {};
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

};

struct StartEvent {
    int actID;
    int divisionsApplied;
    int stepID;
    double advancingDuration;
    double minDuration;
    double maxDuration;
    double elapsed;
    double minAdvance;
//  ScheduleNode* compulsaryEnd;
    bool terminated;
    bool ignore;
    int fanIn;
    set<int> endComesBefore;
    set<int> endComesBeforePair;

    set<int> endComesAfter;
    set<int> endComesAfterPair;


    inline set<int> & getEndComesBefore() {
        return endComesBefore;
    };
    inline set<int> & getEndComesAfter() {
        return endComesAfter;
    };
    inline set<int> & getEndComesAfterPair() {
        return endComesAfterPair;
    };
    inline set<int> & getEndComesBeforePair() {
        return endComesBeforePair;
    };


    double lpMinTimestamp;
    double lpMaxTimestamp;


    StartEvent(const int & a, const int & da, const int & s, const double & mind, const double & maxd, const double &e) : actID(a), divisionsApplied(da), stepID(s), advancingDuration(mind), minDuration(mind), maxDuration(maxd), elapsed(e), minAdvance(DBL_MAX), terminated(false), ignore(false), fanIn(0), lpMinTimestamp(0.0), lpMaxTimestamp(DBL_MAX) {};

    bool operator ==(const StartEvent & e) const {
        return (actID == e.actID &&
                divisionsApplied == e.divisionsApplied &&
                stepID == e.stepID &&
                fabs(minDuration - e.minDuration) < 0.0005 &&
                fabs(maxDuration - e.maxDuration) < 0.0005 &&
                fabs(elapsed - e.elapsed) < 0.0005 &&
                fabs(advancingDuration - e.advancingDuration) < 0.0005 &&
//          compulsaryEnd == e.compulsaryEnd &&
                terminated == e.terminated &&
                fanIn == e.fanIn &&
                endComesBefore == e.endComesBefore);
    }

    void endMustComeAfter(const int & i) {
        assert(i >= 0); endComesAfter.insert(i);
    }
    void endMustComeAfterPair(const int & i) {
        assert(i >= 0); endComesAfterPair.insert(i);
    }

    void actionHasFinished(const int & i) {
        assert(i >= 0); endComesAfter.erase(i);
    }

};

class FakeFFEvent : public FFEvent
{
private:
    StartEvent * toUpdate;
public:
    FakeFFEvent(StartEvent * const e, instantiatedOp * a, const int & pw, const double & dMin, const double &
                dMax)
            : FFEvent(a, pw, dMin, dMax), toUpdate(e) {
        lpMinTimestamp = e->lpMinTimestamp;
        lpMaxTimestamp = e->lpMaxTimestamp;
    };

    virtual ~FakeFFEvent() {
//      cout << "~FakeFFEvent, moving bounds of [" << lpMinTimestamp << ",";
//      if (lpMaxTimestamp == DBL_MAX) {
//          cout << "inf";
//      } else {
//          cout << lpMaxTimestamp;
//      }
//      cout << "] for end of " << *(action) << " back to SEQ entry\n";
        toUpdate->lpMinTimestamp = lpMinTimestamp;
        toUpdate->lpMaxTimestamp = lpMaxTimestamp;
    }

    virtual void passInMinMax(const double & a, const double & b) {
        toUpdate->lpMinTimestamp = lpMinTimestamp = a;
        toUpdate->lpMaxTimestamp = lpMaxTimestamp = b;
    }


};
/*
class ImplicitFFEvent : public FFEvent {
private:
    FFEvent * toUpdate;
public:
        ImplicitFFEvent(FFEvent * const e, instantiatedOp * a, const int & pw, const double & dMin, const double & dMax)
        : FFEvent(a,pw,dMin,dMax), toUpdate(e)
    {
        lpMinTimestamp = e->lpMinTimestamp + e->minDuration;
        lpMaxTimestamp = e->lpMaxTimestamp;
                if (lpMaxTimestamp != DBL_MAX) {
                    if (e->maxDuration == DBL_MAX) {
                        lpMaxTimestamp = DBL_MAX;
                    } else {
                        lpMaxTimestamp += e->maxDuration;
                    }
                }
    };

        void pushToStart();

        ~ImplicitFFEvent() {
//      cout << "~FakeFFEvent, moving bounds of [" << lpMinTimestamp << ",";
//      if (lpMaxTimestamp == DBL_MAX) {
//          cout << "inf";
//      } else {
//          cout << lpMaxTimestamp;
//      }
//      cout << "] for end of " << *(action) << " back to SEQ entry\n";
                pushToStart();
    }

    virtual void passInMinMax(const double & a, const double & b) {
        lpMinTimestamp = a;
        lpMaxTimestamp = b;
                pushToStart();
    }


};
*/

class ExtendedMinimalState
{

private:
    bool operator ==(ExtendedMinimalState &) {
        assert(false);
        return false;
    }

protected:
    MinimalState decorated;
public:

    list<StartEvent> startEventQueue;
    map<int, list<list<StartEvent>::iterator > > entriesForAction;

    double timeStamp;
    int stepBeforeTIL;
    int tilFanIn;
    list<int> tilComesBefore;

    ExtendedMinimalState(const set<int> & f, const vector<double> & sMin, const vector<double> & sMax, const map<int, set<int> > & sa, const double & ts, const int & nt, const unsigned int & pl) : decorated(f, sMin, sMax, sa, nt, pl), timeStamp(ts), stepBeforeTIL(-1), tilFanIn(0) {};
    ExtendedMinimalState() : timeStamp(0.0), stepBeforeTIL(-1), tilFanIn(0) {};

    ExtendedMinimalState(const ExtendedMinimalState & e) : decorated(e.decorated), startEventQueue(e.startEventQueue), timeStamp(e.timeStamp), stepBeforeTIL(e.stepBeforeTIL), tilFanIn(e.tilFanIn), tilComesBefore(e.tilComesBefore)  {

//      factsIfWeFinishActions = e.factsIfWeFinishActions;

        list<StartEvent>::iterator bqItr = startEventQueue.begin();
        const list<StartEvent>::iterator bqEnd = startEventQueue.end();

        for (; bqItr != bqEnd; ++bqItr) {
            entriesForAction[bqItr->actID].push_back(bqItr);
        }

    }

    ExtendedMinimalState(const ExtendedMinimalState & e, const MinimalState & ms) : decorated(ms), startEventQueue(e.startEventQueue), timeStamp(e.timeStamp), stepBeforeTIL(e.stepBeforeTIL), tilFanIn(e.tilFanIn), tilComesBefore(e.tilComesBefore)  {

        //      factsIfWeFinishActions = e.factsIfWeFinishActions;

        list<StartEvent>::iterator bqItr = startEventQueue.begin();
        const list<StartEvent>::iterator bqEnd = startEventQueue.end();

        for (; bqItr != bqEnd; ++bqItr) {
            entriesForAction[bqItr->actID].push_back(bqItr);
        }

    }


    ExtendedMinimalState & operator=(const ExtendedMinimalState & e);

    virtual ~ExtendedMinimalState() {
#ifdef STATEHASHDEBUG
        cout << "Deleting state at " << &(decorated) << std::endl;
#endif
    }

    static bool queueEqual(const list<StartEvent> & a, const list<StartEvent> & b) {
        list<StartEvent>::const_iterator aItr = a.begin();
        const list<StartEvent>::const_iterator aEnd = a.end();

        list<StartEvent>::const_iterator bItr = b.begin();
        const list<StartEvent>::const_iterator bEnd = b.end();

        for (; aItr != aEnd && bItr != bEnd; ++aItr, ++bItr) {
            if (!(*aItr == *bItr)) return false;
        }

        return ((aItr == aEnd) == (bItr == bEnd));

    }

//  virtual bool operator==(const ExtendedMinimalState & o) const {
//      return (nextTIL == o.nextTIL && first == o.first && secondMin == o.secondMin && secondMax == o.secondMax && startedActions == o.startedActions && invariants == o.invariants && fluentInvariants == o.fluentInvariants && stepBeforeTIL == o.stepBeforeTIL && tilFanIn == o.tilFanIn && tilComesBefore == o.tilComesBefore && queueEqual(startEventQueue, o.startEventQueue) && fabs(timeStamp - o.timeStamp) < 0.0005);
//  }

    virtual void deQueueFirstOf(const int & actID, const int & divID);
    virtual void deQueueStep(const int & actID, const int & stepID);

    MinimalState & getEditableInnerState() {
        return decorated;
    }

    const MinimalState & getInnerState() const {
        return decorated;
    }

    ExtendedMinimalState * applyAction(const ActionSegment & a, double minDur = 0.0, double maxDur = 0.0) const {
        return new ExtendedMinimalState(*this, decorated.applyAction(a, minDur, maxDur));
    }

    void applyActionLocally(const ActionSegment & a, double minDur = 0.0, double maxDur = 0.0) {
        decorated.applyActionLocally(a, minDur, maxDur);
    }


};

struct SecondaryExtendedStateEquality {
    bool operator()(const ExtendedMinimalState & a, const ExtendedMinimalState & b) const;
};


struct WeakExtendedStateEquality {
    bool operator()(const ExtendedMinimalState & a, const ExtendedMinimalState & b) const;
};

struct SecondaryExtendedStateLessThan {
    bool operator()(const ExtendedMinimalState & a, const ExtendedMinimalState & b) const;
    bool operator()(const ExtendedMinimalState * const a, const ExtendedMinimalState * const b) const;
};


struct WeakExtendedStateLessThan {
    bool operator()(const ExtendedMinimalState & a, const ExtendedMinimalState & b) const;
    bool operator()(const ExtendedMinimalState * const a, const ExtendedMinimalState * const b) const;
};


class FF
{

public:

    class HTrio
    {

    public:

        double heuristicValue;
        double makespan;
//        double makespanEstimate;
        double qbreak;

#ifndef NDEBUG
        const char * diagnosis;
#endif

        HTrio() {};
        HTrio(const double & hvalue, const double & msIn, const double &, const int & planLength, const char *
#ifndef NDEBUG
              diagnosisIn
#endif
             )
                : heuristicValue(hvalue), makespan(msIn)//, makespanEstimate(mseIn)
#ifndef NDEBUG
                , diagnosis(diagnosisIn)
#endif

        {
            if (FF::WAStar) {
                if (FF::biasD) {
                    qbreak = planLength + 1;
                } else if (FF::biasG) {
                    qbreak = heuristicValue;
                } else {
                    qbreak = 0;
                }
            } else {
                qbreak = planLength + 1;
            }
        }

        HTrio(const HTrio & h) : heuristicValue(h.heuristicValue), makespan(h.makespan),/* makespanEstimate(h.makespanEstimate),*/ qbreak(h.qbreak)
#ifndef NDEBUG
                , diagnosis(h.diagnosis)
#endif
        {};

        HTrio & operator =(const HTrio & h) {
            heuristicValue = h.heuristicValue;
            makespan = h.makespan;
//            makespanEstimate = h.makespanEstimate;
            qbreak = h.qbreak;
#ifndef NDEBUG
            diagnosis = h.diagnosis;
#endif
            return *this;
        }

        bool operator<(const HTrio & other) const {
            if (qbreak < other.qbreak) return true;
            if (qbreak > other.qbreak) return false;

            if (!FF::makespanTieBreak) return false;

            if ((makespan - other.makespan) < -0.0001) return true;
            if ((makespan - other.makespan) > 0.0001) return false;

//            if ((makespanEstimate - other.makespanEstimate) < -0.0001) return true;
//            if ((makespanEstimate - other.makespanEstimate) > 0.0001) return false;


            return false;
        }

    };

private:

    static bool scheduleToMetric;
    static bool skipRPG;

    static HTrio calculateHeuristicAndSchedule(ExtendedMinimalState & theState, ExtendedMinimalState * prevState, set<int> & goals, set<int> & goalFluents, ParentData * const p, list<ActionSegment> & helpfulActions, list<FFEvent> & header, list<FFEvent> & now, const int & stepID, bool considerCache = false, map<double, list<pair<int, int> > > * justApplied = 0, double tilFrom = 0.001);

    static ExtendedMinimalState * applyActionToState(ActionSegment & theAction, const ExtendedMinimalState & parent);

    static void evaluateStateAndUpdatePlan(auto_ptr<SearchQueueItem> & succ, ExtendedMinimalState & state, ExtendedMinimalState * prevState, set<int> & goals, set<int> & goalFluents, ParentData * const incrementalData, list<ActionSegment> & helpfulActionsExport, const ActionSegment & actID, list<FFEvent> & header);

//  static void justEvaluateNotReuse(auto_ptr<SearchQueueItem> & succ, RPGHeuristic* rpg, ExtendedMinimalState & state, ExtendedMinimalState * prevState, set<int> & goals, set<int> & goalFluents, list<ActionSegment> & helpfulActionsExport, list<FFEvent> & extraEvents, list<FFEvent> & header, HTrio & bestNodeLimitHeuristic, list<FFEvent> *& bestNodeLimitPlan, bool & bestNodeLimitGoal, bool & stagnant, map<double, list<pair<int,int> > > * justApplied, double tilFrom=0.001);


//  static bool checkTSTemporalSoundness(RPGHeuristic* const rpg, ExtendedMinimalState & theState, const int & theAction, const VAL::time_spec & ts, const double & incr, int oldTIL=-1);
    static bool precedingActions(ExtendedMinimalState & theState, const ActionSegment & actionSeg, list<ActionSegment> & alsoMustDo, int oldTIL = -1, double moveOn = 0.001);

    static bool checkTemporalSoundness(ExtendedMinimalState & theState, const ActionSegment & actionSeg, int oldTIL = -1, double moveOn = 0.001);

    static void makeJustApplied(map<double, list<pair<int, int> > > & justApplied, double & tilFrom, ExtendedMinimalState & state, const bool & lastIsSpecial);


public:

    static bool steepestDescent;
    static bool bestFirstSearch;
    static bool helpfulActions;
    static bool pruneMemoised;
    static bool firstImprover;
    static bool incrementalExpansion;
    static bool skipEHC;
    static bool zealousEHC;
    static bool startsBeforeEnds;
    static bool invariantRPG;
    static bool tsChecking;
    static bool timeWAStar;
    static bool WAStar;
    static double doubleU;
    static bool biasG;
    static bool biasD;
    static bool makespanTieBreak;
    static bool planMustSucceed;
    static bool nonDeletorsFirst;
    //static list<instantiatedOp*> * solveSubproblem(LiteralSet & startingState, vector<pair<PNE*, double> > & startingFluents, SubProblem* const s);
    static pair<list<FFEvent>*, TemporalConstraints*> search(bool & reachedGoal);

    static list<FFEvent> * doBenchmark(bool & reachedGoal, list<FFEvent> * soln, const bool doLoops = true);
    static list<FFEvent> * reprocessPlan(list<FFEvent> * soln, TemporalConstraints * cons);
};


};

#endif
