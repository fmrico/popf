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

#include "FFSolver.h"
#include "temporalconstraints.h"
#include "globals.h"
#include "numericanalysis.h"
#include "lpscheduler.h"

#include <float.h>

#include <sys/times.h>
#include <sys/time.h>
#include <sys/unistd.h>

namespace Planner
{

int FFEvent::tilLimit = 0;

list<pair<double, list<ActionSegment> > > FFcache_relaxedPlan;
list<ActionSegment> FFcache_helpfulActions;
int FFcache_h;
bool FFcache_upToDate;
bool FFheader_upToDate;
bool FFonly_one_successor;
double FFcache_makespanEstimate;
bool FF::steepestDescent = true;

double FF::doubleU = 5.0;
bool FF::WAStar = false;
bool FF::timeWAStar = false;

bool FF::bestFirstSearch = true;
bool FF::helpfulActions = true;

bool FF::biasG = false;
bool FF::biasD = false;

bool FF::pruneMemoised = true;

bool FF::firstImprover = false;

bool FF::incrementalExpansion = false;
bool FF::skipEHC = false;
bool FF::zealousEHC = true;
bool FF::startsBeforeEnds = true;
bool FF::invariantRPG = true;
bool FF::tsChecking = true;
bool FF::scheduleToMetric = false;
bool FF::makespanTieBreak = true;
bool FF::nonDeletorsFirst = false;

bool FF::skipRPG = false;

using std::endl;

ExtendedMinimalState & ExtendedMinimalState::operator=(const ExtendedMinimalState & e)
{

    decorated = e.decorated;
    startEventQueue = e.startEventQueue;
    timeStamp = e.timeStamp;
    stepBeforeTIL = e.stepBeforeTIL;
    tilFanIn = e.tilFanIn;
    tilComesBefore = e.tilComesBefore;
    entriesForAction.clear();
    list<StartEvent>::iterator bqItr = startEventQueue.begin();
    const list<StartEvent>::iterator bqEnd = startEventQueue.end();

    for (; bqItr != bqEnd; ++bqItr) {
        entriesForAction[bqItr->actID].push_back(bqItr);
    }


    return *this;
}


/*void ImplicitFFEvent::pushToStart() {
//    cout << "Pushing back from implicit end with t in [" << lpMinTimestamp << "," << lpMaxTimestamp << "\n";
    const double newStartMin = lpMinTimestamp - toUpdate->maxDuration;
    if (newStartMin > toUpdate->lpMinTimestamp) {
        toUpdate->lpMinTimestamp = newStartMin;
    }
    double newStartMax = lpMaxTimestamp;
    if (newStartMax != DBL_MAX) newStartMax -= toUpdate->minDuration;
    if (newStartMax < toUpdate->lpMaxTimestamp) {
        toUpdate->lpMaxTimestamp = newStartMax;
    }
    if (toUpdate->lpEndTimestamp < lpMinTimestamp) {
        toUpdate->lpEndTimestamp = lpMinTimestamp;
        //cout << "Pushing " << lpMinTimestamp << " back as timestamp of end of skippable action\n";
    }
}*/

void printState(MinimalState & e)
{

    cout << e;

    cout << "State Finished\n";
};

void printState(ExtendedMinimalState & e)
{

    cout << e.getInnerState();

    cout << "\nStart event queue:";
    {
        list<StartEvent>::iterator aItr = e.startEventQueue.begin();
        const list<StartEvent>::iterator aEnd = e.startEventQueue.end();
        for (; aItr != aEnd; ++aItr) {
            cout << aItr->stepID << ": " << aItr->actID << "\n";
        }
    }

    cout << "State Finished\n";

};

namespace CSBase
{

vector<bool> ignorableFluents;

int compareSets(const set<int> & a, const set<int> & b)
{

    const bool aEmpty = a.empty();
    const bool bEmpty = b.empty();
    if (aEmpty && bEmpty) return 0;
    if (aEmpty && !bEmpty) return -1;
    if (bEmpty && !aEmpty) return 1;

    set<int>::const_iterator aItr = a.begin();
    const set<int>::const_iterator aEnd = a.end();

    set<int>::const_iterator bItr = b.begin();
    const set<int>::const_iterator bEnd = b.end();

    int av, bv;

    for (; aItr != aEnd && bItr != bEnd; ++aItr, ++bItr) {
        av = *aItr;
        bv = *bItr;
        if (av < bv) return 1;
        if (av > bv) return -1;
    }

    if (aItr == aEnd && bItr != bEnd) return 1;
    if (aItr != aEnd && bItr == bEnd) return -1;

    return 0;
}

int compareSets(const set<StepAndBeforeOrAfter> & a, const set<StepAndBeforeOrAfter> & b)
{

    const bool aEmpty = a.empty();
    const bool bEmpty = b.empty();
    if (aEmpty && bEmpty) return 0;
    if (aEmpty && !bEmpty) return -1;
    if (bEmpty && !aEmpty) return 1;

    set<StepAndBeforeOrAfter>::const_iterator aItr = a.begin();
    const set<StepAndBeforeOrAfter>::const_iterator aEnd = a.end();

    set<StepAndBeforeOrAfter>::const_iterator bItr = b.begin();
    const set<StepAndBeforeOrAfter>::const_iterator bEnd = b.end();

    for (; aItr != aEnd && bItr != bEnd; ++aItr, ++bItr) {
        const StepAndBeforeOrAfter & av = *aItr;
        const StepAndBeforeOrAfter & bv = *bItr;
        if (av < bv) return 1;
        if (av > bv) return -1;
    }

    if (aItr == aEnd && bItr != bEnd) return 1;
    if (aItr != aEnd && bItr == bEnd) return -1;

    return 0;
}

int compareSets(const map<StepAndBeforeOrAfter, bool> & a, const map<StepAndBeforeOrAfter, bool> & b)
{

    const bool aEmpty = a.empty();
    const bool bEmpty = b.empty();
    if (aEmpty && bEmpty) return 0;
    if (aEmpty && !bEmpty) return -1;
    if (bEmpty && !aEmpty) return 1;

    map<StepAndBeforeOrAfter, bool>::const_iterator aItr = a.begin();
    const map<StepAndBeforeOrAfter, bool>::const_iterator aEnd = a.end();

    map<StepAndBeforeOrAfter, bool>::const_iterator bItr = b.begin();
    const map<StepAndBeforeOrAfter, bool>::const_iterator bEnd = b.end();

    for (; aItr != aEnd && bItr != bEnd; ++aItr, ++bItr) {
        const StepAndBeforeOrAfter & av = aItr->first;
        const StepAndBeforeOrAfter & bv = bItr->first;
        if (av < bv) return 1;
        if (av > bv) return -1;

        if (!aItr->second && bItr->second) return 1;
        if (aItr->second && !bItr->second) return -1;
    }

    if (aItr == aEnd && bItr != bEnd) return 1;
    if (aItr != aEnd && bItr == bEnd) return -1;

    return 0;
}

int compareAnnotations(const map<int, PropositionAnnotation> & a, const map<int, PropositionAnnotation> & b)
{
    map<int, PropositionAnnotation>::const_iterator aItr = a.begin();
    const map<int, PropositionAnnotation>::const_iterator aEnd = a.end();

    map<int, PropositionAnnotation>::const_iterator bItr = b.begin();

    int cMaps;
    for (; aItr != aEnd; ++aItr, ++bItr) {
        assert(aItr->first == bItr->first);
        {
            const StepAndBeforeOrAfter & av = aItr->second.availableFrom;
            const StepAndBeforeOrAfter & bv = bItr->second.availableFrom;
            if (av < bv) return 1;
            if (av > bv) return -1;
        }

        {
            const map<StepAndBeforeOrAfter, bool> & av = aItr->second.deletableFrom;
            const map<StepAndBeforeOrAfter, bool> & bv = bItr->second.deletableFrom;

            cMaps = compareSets(av, bv);
#ifdef STATEHASHDEBUG
            assert(cMaps * -1 == compareSets(bv, av));
#endif

            if (cMaps != 0) return cMaps;

        }

    }

    return 0;
}

int compareSets(const map<int, PropositionAnnotation> & a, const map<int, PropositionAnnotation> & b)
{

    const bool aEmpty = a.empty();
    const bool bEmpty = b.empty();
    if (aEmpty && bEmpty) return 0;
    if (aEmpty && !bEmpty) return -1;
    if (bEmpty && !aEmpty) return 1;

    map<int, PropositionAnnotation>::const_iterator aItr = a.begin();
    const map<int, PropositionAnnotation>::const_iterator aEnd = a.end();

    map<int, PropositionAnnotation>::const_iterator bItr = b.begin();
    const map<int, PropositionAnnotation>::const_iterator bEnd = b.end();

    int av, bv;
    for (; aItr != aEnd && bItr != bEnd; ++aItr, ++bItr) {

        av = aItr->first;
        bv = bItr->first;
        if (av < bv) return 1;
        if (av > bv) return -1;

    }

    if (aItr == aEnd && bItr != bEnd) return 1;
    if (aItr != aEnd && bItr == bEnd) return -1;

    return 0;
}

void skipTerminates(list<StartEvent>::const_iterator & itr, const list<StartEvent>::const_iterator & itrEnd)
{
    while (itr != itrEnd && itr->terminated) ++itr;
}

int compareLists(const list<StartEvent> & a, const list<StartEvent> & b)
{

    list<StartEvent>::const_iterator aItr = a.begin();
    const list<StartEvent>::const_iterator aEnd = a.end();

    list<StartEvent>::const_iterator bItr = b.begin();
    const list<StartEvent>::const_iterator bEnd = b.end();

    skipTerminates(aItr, aEnd);
    skipTerminates(bItr, bEnd);

    while (aItr != aEnd && bItr != bEnd) {
        const StartEvent& av = *aItr;
        const StartEvent& bv = *bItr;
        if (av.actID < bv.actID) return 1;
        if (av.actID > bv.actID) return -1;

        //          if (av.stepID < bv.stepID) return 1;
        //          if (av.stepID > bv.stepID) return -1;

        ++aItr;
        ++bItr;

        skipTerminates(aItr, aEnd);
        skipTerminates(bItr, bEnd);

    }

    if (aItr == aEnd && bItr != bEnd) return 1;
    if (aItr != aEnd && bItr == bEnd) return -1;

    return 0;
}


int compareMaps(const map<int, int> & a, const map<int, int> & b)
{

    const bool aEmpty = a.empty();
    const bool bEmpty = b.empty();
    if (aEmpty && bEmpty) return 0;
    if (aEmpty && !bEmpty) return -1;
    if (bEmpty && !aEmpty) return 1;

    map<int, int>::const_iterator aItr = a.begin();
    const map<int, int>::const_iterator aEnd = a.end();

    map<int, int>::const_iterator bItr = b.begin();
    const map<int, int>::const_iterator bEnd = b.end();

    int av, bv, av2, bv2;
    for (; aItr != aEnd && bItr != bEnd; ++aItr, ++bItr) {
        av = aItr->first;
        bv = bItr->first;
        if (av < bv) return 1;
        if (av > bv) return -1;
        av2 = aItr->second;
        bv2 = bItr->second;
        if (av2 < bv2) return 1;
        if (av2 > bv2) return -1;
    }

    if (aItr == aEnd && bItr != bEnd) return 1;
    if (aItr != aEnd && bItr == bEnd) return -1;

    return 0;
}

int compareMaps(const map<int, map<int, int> > & a, const map<int, map<int, int> > & b)
{

    const bool aEmpty = a.empty();
    const bool bEmpty = b.empty();
    if (aEmpty && bEmpty) return 0;
    if (aEmpty && !bEmpty) return -1;
    if (bEmpty && !aEmpty) return 1;

    map<int, map<int, int> >::const_iterator aItr = a.begin();
    const map<int, map<int, int> >::const_iterator aEnd = a.end();

    map<int, map<int, int> >::const_iterator bItr = b.begin();
    const map<int, map<int, int> >::const_iterator bEnd = b.end();

    int av, bv;
    for (; aItr != aEnd && bItr != bEnd; ++aItr, ++bItr) {
        av = aItr->first;
        bv = bItr->first;
        if (av < bv) return 1;
        if (av > bv) return -1;
        const int cMaps = compareMaps(aItr->second, bItr->second);
#ifdef STATEHASHDEBUG
        assert(-1 * cMaps == compareMaps(bItr->second, aItr->second));
#endif
        if (cMaps != 0) return cMaps;
    }

    if (aItr == aEnd && bItr != bEnd) return 1;
    if (aItr != aEnd && bItr == bEnd) return -1;

    return 0;
}

int compareMaps(const map<int, set<int> > & a, const map<int, set<int> > & b)
{

    const bool aEmpty = a.empty();
    const bool bEmpty = b.empty();
    if (aEmpty && bEmpty) return 0;
    if (aEmpty && !bEmpty) return -1;
    if (bEmpty && !aEmpty) return 1;

    map<int, set<int> >::const_iterator aItr = a.begin();
    const map<int, set<int> >::const_iterator aEnd = a.end();

    map<int, set<int> >::const_iterator bItr = b.begin();
    const map<int, set<int> >::const_iterator bEnd = b.end();

    int av, bv, cMaps;
    for (; aItr != aEnd && bItr != bEnd; ++aItr, ++bItr) {
        av = aItr->first;
        bv = bItr->first;
        if (av < bv) return 1;
        if (av > bv) return -1;
        //          const int av2 = aItr->second;
        //          const int bv2 = bItr->second;
        //          if (av2 < bv2) return 1;
        //          if (av2 > bv2) return -1;
        cMaps = compareSets(aItr->second, bItr->second);
#ifdef STATEHASHDEBUG
        assert(-1 * cMaps == compareSets(bItr->second, aItr->second));
#endif
        if (cMaps != 0) return cMaps;
    }

    if (aItr == aEnd && bItr != bEnd) return 1;
    if (aItr != aEnd && bItr == bEnd) return -1;

    return 0;
}

int compareVecs(const vector<double> & a, const vector<double> & b)
{
    const int aSize = a.size();
    const int bSize = b.size();
    if (!aSize && !bSize) return 0;
    if (aSize < bSize) return 1;
    if (bSize > aSize) return -1;

    double av, bv;
    for (int i = 0; i < aSize; ++i) {
        if (!ignorableFluents[i]) {
            av = a[i];
            bv = b[i];
            if (av < bv) return 1;
            if (av > bv) return -1;
        }
    }

    return 0;
}

/*  int compareLists(const list<int> & a, const list<int> & b) {

        const bool aEmpty = a.empty();
        const bool bEmpty = b.empty();
        if (aEmpty && bEmpty) return 0;
        if (aEmpty && !bEmpty) return -1;
        if (bEmpty && !aEmpty) return 1;

        list<int>::const_iterator aItr = a.begin();
        const list<int>::const_iterator aEnd = a.end();

        list<int>::const_iterator bItr = b.begin();
        const list<int>::const_iterator bEnd = b.end();

        for (; aItr != aEnd && bItr != bEnd; ++aItr, ++bItr) {
            const int av = *aItr;
            const int bv = *bItr;
            if (av < bv) return 1;
            if (av > bv) return -1;
        }

        if (aItr == aEnd && bItr != bEnd) return 1;
        if (aItr != aEnd && bItr == bEnd) return -1;

        return 0;

    }*/

bool baseLessThan(const ExtendedMinimalState & ae, const ExtendedMinimalState & be)
{

    const MinimalState & a = ae.getInnerState();
    const MinimalState & b = be.getInnerState();

#ifdef STATEHASHDEBUG
    const bool shdVerbose = false;
    if (shdVerbose) {
        cout << "Seeing if " << &ae << " < " << &be;
        cout.flush();
    }
#endif

    const int csVal = CSBase::compareSets(a.first, b.first);
#ifdef STATEHASHDEBUG
    assert(-1*csVal == CSBase::compareSets(b.first, a.first));
#endif
    if (csVal > 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - yes 1\n";
        }
#endif
        return true;
    } else if (csVal < 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - no 1\n";
        }
#endif
        return false;
    }

    const int cv1Val = CSBase::compareVecs(a.secondMin, b.secondMin);
#ifdef STATEHASHDEBUG
    assert(-1 * cv1Val == CSBase::compareVecs(b.secondMin, a.secondMin));
#endif
    if (cv1Val > 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - yes 2\n";
        }
#endif
        return true;
    } else if (cv1Val < 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - no 2\n";
        }
#endif
        return false;
    }

    const int cv2Val = CSBase::compareVecs(a.secondMax, b.secondMax);
#ifdef STATEHASHDEBUG
    assert(-1 * cv2Val == CSBase::compareVecs(b.secondMax, a.secondMax));
#endif
    if (cv2Val > 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - yes 3\n";
        }
#endif
        return true;
    } else if (cv2Val < 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - no 3\n";
        }
#endif
        return false;
    }


    const int saVal = CSBase::compareMaps(a.startedActions, b.startedActions);
#ifdef STATEHASHDEBUG
    assert(-1 * saVal == CSBase::compareMaps(b.startedActions, a.startedActions));
#endif
    if (saVal > 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - yes 4\n";
        }
#endif
        return true;
    } else if (saVal < 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - no 4\n";
        }
#endif
        return false;
    }

    if (a.nextTIL < b.nextTIL) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - yes 5\n";
        }
#endif
        return true;
    } else if (a.nextTIL > b.nextTIL) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - no 5\n";
        }
#endif
        return false;
    }

#ifdef STATEHASHDEBUG
    if (shdVerbose) {
        cout << " - identical\n";
    }
#endif



    return false;
}

bool secondaryLessThan(const ExtendedMinimalState & ae, const ExtendedMinimalState & be)
{

    const MinimalState & a = ae.getInnerState();
    const MinimalState & b = be.getInnerState();

#ifdef STATEHASHDEBUG
    const bool shdVerbose = false;
    if (shdVerbose) {
        cout << "Secondary map - seeing if " << &ae << " < " << &be;
        cout.flush();
    }
#endif

    const int caVal = CSBase::compareAnnotations(a.first, b.first);

#ifdef STATEHASHDEBUG
    assert(-1*caVal == CSBase::compareAnnotations(b.first, a.first));
#endif

    if (caVal > 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - yes s1\n";
        }
#endif
        return true;
    } else if (caVal < 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - no s1\n";
        }
#endif
        return false;
    }


    const int ceVal = CSBase::compareLists(ae.startEventQueue, be.startEventQueue);
#ifdef STATEHASHDEBUG
    assert(-1 * ceVal == -1 * CSBase::compareLists(be.startEventQueue, ae.startEventQueue));
#endif
    if (ceVal > 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - yes s2\n";
        }
#endif
        return true;
    } else if (ceVal < 0) {
#ifdef STATEHASHDEBUG
        if (shdVerbose) {
            cout << " - no s2\n";
        }
#endif
        return false;
    }
#ifdef STATEHASHDEBUG
    if (shdVerbose) {
        cout << " - identical\n";
    }
#endif

    return false;

}


};

bool SecondaryExtendedStateLessThan::operator()(const ExtendedMinimalState * const ae, const ExtendedMinimalState * const be) const
{
    return operator()(*ae, *be);
}

bool SecondaryExtendedStateLessThan::operator()(const ExtendedMinimalState & ae, const ExtendedMinimalState & be) const
{
    return CSBase::secondaryLessThan(ae, be);
}


bool WeakExtendedStateLessThan::operator()(const ExtendedMinimalState * const ae, const ExtendedMinimalState * const be) const
{
    return operator()(*ae, *be);
}

bool WeakExtendedStateLessThan::operator()(const ExtendedMinimalState & ae, const ExtendedMinimalState & be) const
{
    return CSBase::baseLessThan(ae, be);
}

FFEvent::FFEvent(instantiatedOp* a, const double & dMin, const double & dMax)
        : action(a), time_spec(VAL::E_AT_START), minDuration(dMin), maxDuration(dMax), pairWithStep(-1),
        getEffects(true), lpTimestamp(-1.0), /*lpEndTimestamp(-1.0), */lpMinTimestamp(-1.0), lpMaxTimestamp(DBL_MAX), divisionID(-1)
{
    //cout << "FFEvent start\n";
}

FFEvent::FFEvent(instantiatedOp* a, const int & pw, const double & dMin, const double & dMax)
        : action(a), time_spec(VAL::E_AT_END), minDuration(dMin), maxDuration(dMax), pairWithStep(pw),
        getEffects(true), lpTimestamp(-1.0), /*lpEndTimestamp(-1.0), */lpMinTimestamp(-1.0), lpMaxTimestamp(DBL_MAX), divisionID(-1)
{
    //cout << "FFEvent end\n";
}

FFEvent::FFEvent(instantiatedOp* a, const int & s, const int & pw, const double & dMin, const double & dMax)
        : action(a), time_spec(VAL::E_OVER_ALL), minDuration(dMin), maxDuration(dMax), pairWithStep(pw),
        getEffects(true), lpTimestamp(-1.0), /*lpEndTimestamp(-1.0), */lpMinTimestamp(-1.0), lpMaxTimestamp(DBL_MAX), divisionID(s)
{
    //cout << "FFEvent end\n";
}

FFEvent::FFEvent(const int & t)
        : action(0), time_spec(VAL::E_AT), minDuration(-1.0), maxDuration(-1.0), pairWithStep(-1), getEffects(true),
        lpTimestamp(-1.0), /*lpEndTimestamp(-1.0), */lpMinTimestamp(-1.0), lpMaxTimestamp(DBL_MAX), divisionID(t)
{
    assert(divisionID <= tilLimit);
    //cout << "FFEvent start\n";
}


FFEvent::FFEvent(const FFEvent & f)
        : action(f.action), time_spec(f.time_spec), minDuration(f.minDuration), maxDuration(f.maxDuration),
        pairWithStep(f.pairWithStep), getEffects(f.getEffects) , lpTimestamp(f.lpTimestamp),
        lpMinTimestamp(f.lpMinTimestamp), lpMaxTimestamp(f.lpMaxTimestamp), divisionID(f.divisionID), needToFinish(f.needToFinish)
{
    //cout << "FFEvent copy\n";
}

FFEvent::FFEvent()
        : action(0), time_spec(VAL::E_AT_START), minDuration(0.0), maxDuration(0.0), lpTimestamp(-1.0),
        lpMinTimestamp(-1.0), lpMaxTimestamp(DBL_MAX), divisionID(-1)
{
    //cout << "FFEvent default\n";
}

FFEvent & FFEvent::operator=(const FFEvent & f)
{
    //cout << "FFEvent assignment op\n";
    action = f.action;
    time_spec = f.time_spec;
    minDuration = f.minDuration;
    maxDuration = f.maxDuration;
    pairWithStep = f.pairWithStep;
    getEffects = f.getEffects;
    lpTimestamp = f.lpTimestamp;
    lpMinTimestamp = f.lpMinTimestamp;
    lpMaxTimestamp = f.lpMaxTimestamp;
    divisionID = f.divisionID;
    needToFinish = f.needToFinish;
    return *this;
}

class SearchQueueItem
{

private:
    ExtendedMinimalState * internalState;
    bool ownState;

public:
#ifdef STATEHASHDEBUG
    bool mustNotDeleteState;
#endif


    inline ExtendedMinimalState * state() {
        return internalState;
    }

    list<FFEvent> plan;

    list<ActionSegment> helpfulActions;
    FF::HTrio heuristicValue;

    SearchQueueItem()
            : internalState(0), ownState(false) {
#ifdef STATEHASHDEBUG
        mustNotDeleteState = false;
#endif
    }



    /**
     *  Create a search queue item for the specified state.
     *
     *  @param sIn              The state to store in the search queue item
     *  @param clearIfDeleted   If <code>true</code>, mark that <code>sIn</code> should be deleted
     *                          if the search queue item is deleted (unless <code>releaseState()</code>
     *                          is called first).
     */
    SearchQueueItem(ExtendedMinimalState * const sIn, const bool clearIfDeleted)
            : internalState(sIn), ownState(clearIfDeleted) {
#ifdef STATEHASHDEBUG
        mustNotDeleteState = false;
#endif
    }

    ~SearchQueueItem() {
        if (ownState) {
#ifdef STATEHASHDEBUG
            assert(!mustNotDeleteState);
#endif
            delete internalState;
        }
    }


    /**
     *  Return the state held in this search queue item, flagging that it should not be deleted by
     *  the search queue item's destructor.
     *
     *  @return The state held in this search queue item
     */
    ExtendedMinimalState * releaseState() {
        assert(ownState);
        ownState = false;
        return internalState;
    }


    void printPlan() {
        if (Globals::globalVerbosity & 2) {
            list<FFEvent>::iterator planItr = plan.begin();
            const list<FFEvent>::iterator planEnd = plan.end();

            for (int i = 0; planItr != planEnd; ++planItr, ++i) {
                if (!planItr->getEffects) cout << "(( ";
                if (planItr->action) {
                    cout << i << ": " << *(planItr->action) << ", " << (planItr->time_spec == VAL::E_AT_START ? "start" : "end");
                } else if (planItr->time_spec == VAL::E_AT) {
                    cout << i << ": TIL " << planItr->divisionID;

                } else {
                    cout << i << ": null node!";
                    assert(false);
                }
                if (!planItr->getEffects) cout << " ))";
                cout << "\n";
            }
        }
    }

};

class SearchQueue
{

private:

    map<double, list<SearchQueueItem*> > qOne;
    map<double, list<SearchQueueItem*> > qTwo;
public:

    ~SearchQueue() {
        clear();
    }

    void clear() {
        for (int pass = 0; pass < 2; ++pass) {

            map<double, list<SearchQueueItem*> > & currMap = (pass ? qTwo : qOne);
            map<double, list<SearchQueueItem*> >::iterator cmItr = currMap.begin();
            const map<double, list<SearchQueueItem*> >::iterator cmEnd = currMap.end();

            for (; cmItr != cmEnd; ++cmItr) {
                list<SearchQueueItem*>::iterator qItr = cmItr->second.begin();
                const list<SearchQueueItem*>::iterator qEnd = cmItr->second.end();

                for (; qItr != qEnd; ++qItr) delete *qItr;
            }
            currMap.clear();

        }
    }

    SearchQueueItem* pop_front() {
        static int lastTime = 0;
        if (!qOne.empty()) {
            if (lastTime != 1) {
                lastTime = 1;
                if (Globals::globalVerbosity & 1) {
                    cout << "\n1: ";
                    cout.flush();
                }
            }
            map<double, list<SearchQueueItem*> >::iterator mItr = qOne.begin();
            SearchQueueItem* const toReturn = mItr->second.front();
            mItr->second.pop_front();
            if (mItr->second.empty()) qOne.erase(mItr);
            return toReturn;
        } else {
            if (lastTime != 2) {
                lastTime = 2;
                if (Globals::globalVerbosity & 1) {
                    cout << "\n2: ";
                    cout.flush();
                }
            }
            map<double, list<SearchQueueItem*> >::iterator mItr = qTwo.begin();
            SearchQueueItem* const toReturn = mItr->second.front();
            mItr->second.pop_front();
            if (mItr->second.empty()) qTwo.erase(mItr);
            return toReturn;
        }
    }

    SearchQueueItem * back() {
        if (qTwo.empty()) {
            return qOne.rbegin()->second.back();
        } else {
            return qTwo.rbegin()->second.back();
        }
    }

    void push_back(SearchQueueItem* p, const int category = 1) {
        list<SearchQueueItem*> & q = (category == 1 ? qOne[p->heuristicValue.qbreak] : qTwo[p->heuristicValue.qbreak]);

        list<SearchQueueItem*>::iterator qItr = q.begin();
        const list<SearchQueueItem*>::iterator qEnd = q.end();
        for (; qItr != qEnd; ++qItr) {
            if (p->heuristicValue < (*qItr)->heuristicValue) {
                q.insert(qItr, p);
                return;
            }
        }
        q.push_back(p);

    }

    void insert(SearchQueueItem* p, const int category = 1) {

        double prim = p->heuristicValue.heuristicValue;
        if (FF::WAStar) {
            prim *= FF::doubleU;
            if (FF::timeWAStar) {
                prim += p->state()->timeStamp;
            } else {
                const int pps = p->state()->getInnerState().planLength - p->state()->getInnerState().actionsExecuting;
                prim += pps;
            }
        }
        list<SearchQueueItem*> & q = (category == 1 ? qOne[prim] : qTwo[prim]);
        if (q.empty()) {
            q.push_back(p);
            return;
        }

        list<SearchQueueItem*>::iterator qItr = q.begin();
        const list<SearchQueueItem*>::iterator qEnd = q.end();
        for (; qItr != qEnd; ++qItr) {
            if (p->heuristicValue < (*qItr)->heuristicValue) {
                q.insert(qItr, p);
                return;
            }
        }
        q.push_back(p);

    }


    bool empty() const {
        return (qOne.empty() && qTwo.empty());
    }

};

void populateTimestamps(vector<double> & minTimestamps, double & makespan, list<FFEvent> & header, list<FFEvent> & now)
{
    int stepID = 0;

    for (int pass = 0; pass < 2; ++pass) {
        list<FFEvent>::iterator planItr = (pass ? now.begin() : header.begin());
        const list<FFEvent>::iterator planEnd = (pass ? now.end() : header.end());

        for (; planItr != planEnd; ++planItr, ++stepID) {
            const double curr = planItr->lpMinTimestamp;
            minTimestamps[stepID] = curr;
            if (curr > makespan) makespan = curr;
        }
    }
}


FF::HTrio FF::calculateHeuristicAndSchedule(ExtendedMinimalState & theState, ExtendedMinimalState * prevState, set<int> & goals, set<int> & goalFluents, ParentData * const incrementalData, list<ActionSegment> & helpfulActions, list<FFEvent> & header, list<FFEvent> & now, const int & stepID, bool considerCache, map<double, list<pair<int, int> > > * justApplied, double tilFrom)
{

    //cout << "Evaluating a state reached by " << header.size() + now.size() << " snap actions\n";
    //cout << "Has " << theState.startedActions.size() << " sorts of actions on the go\n";
//  LPScheduler tryToSchedule(header, now, theState.entriesForAction, (prevState ? &prevState->secondMin : 0), (prevState ? &prevState->secondMax : 0));

    LPScheduler tryToSchedule(theState.getInnerState(), header, now, stepID, theState.startEventQueue, incrementalData, theState.entriesForAction, (prevState ? &prevState->getInnerState().secondMin : 0), (prevState ? &prevState->getInnerState().secondMax : 0), &(theState.tilComesBefore), scheduleToMetric);

    if (scheduleToMetric) {
        HTrio toReturn(0, 0.0, 0.0, theState.getInnerState().planLength - theState.getInnerState().actionsExecuting, "Evaluated Successfully");        
        return toReturn;        
    }
    
    if (!tryToSchedule.isSolved()) return HTrio(-1.0, DBL_MAX, DBL_MAX, INT_MAX, "LP deemed action choice invalid");

    tryToSchedule.updateStateFluents(theState.getEditableInnerState().secondMin, theState.getEditableInnerState().secondMax);

    if (skipRPG) return HTrio(-2.0, -2.0, -2.0, INT_MAX, "skipRPG specified - skipping RPG evaluation");

    vector<double> minTimestamps(theState.getInnerState().planLength);
    double makespan = 0.0;

    populateTimestamps(minTimestamps, makespan, header, now);

    list<pair<double, list<ActionSegment> > > relaxedPlan;

    //printState(theState);
    static int oldBestH = INT_MAX;

    int h;
    double makespanEstimate = 0.0;
    if (considerCache) {
        if (FFcache_upToDate) {
            relaxedPlan = FFcache_relaxedPlan;
            helpfulActions.insert(helpfulActions.end(), FFcache_helpfulActions.begin(), FFcache_helpfulActions.end());
            h = FFcache_h;
            makespanEstimate = FFcache_makespanEstimate;
            cout << "*";
            cout.flush();
        } else {
            h = RPGBuilder::getHeuristic()->getRelaxedPlan(theState.getInnerState(), minTimestamps, theState.timeStamp, helpfulActions, relaxedPlan, makespanEstimate, justApplied, tilFrom);

            FFcache_relaxedPlan = relaxedPlan;
            FFcache_helpfulActions = helpfulActions;
            FFcache_h = h;
            FFcache_makespanEstimate = makespanEstimate;
            FFcache_upToDate = true;
        }
    } else {
        //printState(theState);
        h = RPGBuilder::getHeuristic()->getRelaxedPlan(theState.getInnerState(), minTimestamps, theState.timeStamp, helpfulActions, relaxedPlan, makespanEstimate, justApplied, tilFrom);

    }

    if (h < oldBestH) {
        oldBestH = h;
    }

    if (h < 0) return HTrio(-1.0, DBL_MAX, DBL_MAX, INT_MAX, "RPG heuristic detected a deadend");

    bool cycle = false;

    if (FF::incrementalExpansion && !FFonly_one_successor) {

        assert(false);
    } else {
        const bool oldVal = FF::incrementalExpansion;
        FF::incrementalExpansion = false;
        cycle = !tryToSchedule.addRelaxedPlan(header, now, relaxedPlan);
        FF::incrementalExpansion = oldVal;
    }

    if (h == 0) {
        h += (tryToSchedule.isSolution(theState.getInnerState(), header, now) ? 0 : 1);
        if (h == 0 && false) {
            int s = 0;
            for (int pass = 0; pass < 2; ++pass) {
                list<FFEvent>::iterator pItr = (pass ? now.begin() : header.begin());
                list<FFEvent>::iterator pEnd = (pass ? now.end() : header.end());
                for (; pItr != pEnd; ++pItr) {
                    cout << s << ": " << pItr->lpTimestamp << "\n";
                    ++s;
                }
            }
        }
    }


    if (cycle) {
        if (Globals::globalVerbosity & 1) cout << "c";
        return HTrio(-1.0, DBL_MAX, DBL_MAX, INT_MAX, "Cycle detected when adding relaxed plan data to LP");
    }

    HTrio toReturn(h, makespan, makespanEstimate, theState.getInnerState().planLength - theState.getInnerState().actionsExecuting, "Evaluated Successfully");

    return toReturn;
}


ExtendedMinimalState * FF::applyActionToState(ActionSegment & actionToApply, const ExtendedMinimalState & parent)
{

//  static const double EPSILON = 0.001;
    const bool localDebug = false;

    if (localDebug) {
        if (actionToApply.second == VAL::E_AT) {
            cout << "Applying TIL " << actionToApply.divisionID << " to state:\n";
        } else {
            cout << "Applying action " << *(actionToApply.first);
            if (actionToApply.second == VAL::E_AT_START) {
                cout << " start";
            } else {
                cout << " end";
            }
            cout << " to state:\n";
        }
        //printState(theState);
    }

//  assert(actionToApply.needToFinish.empty());

    assert(!actionToApply.first || !RPGBuilder::rogueActions[actionToApply.first->getID()]);
    assert(RPGBuilder::getHeuristic()->testApplicability(parent.getInnerState(), parent.timeStamp, actionToApply, true));

    if (actionToApply.second == VAL::E_AT) { // til actions
        assert(actionToApply.divisionID == parent.getInnerState().nextTIL);
        return parent.applyAction(actionToApply);

    }

    double minDur = 0.0;
    double maxDur = 0.0;

    bool nonTemporal = false;

    RPGBuilder::LinearEffects * const lEffs = RPGBuilder::getLinearDiscretisation()[actionToApply.first->getID()];

    if (actionToApply.second != VAL::E_AT_END) {

        pair<double, double> actDur(RPGBuilder::getOpDuration(actionToApply.first->getID(), (actionToApply.second == VAL::E_OVER_ALL ? actionToApply.divisionID + 1 : 0), parent.getInnerState().secondMin, parent.getInnerState().secondMax));

        minDur = actDur.first;
        maxDur = actDur.second;

        nonTemporal = RPGBuilder::getRPGDEs(actionToApply.first->getID()).empty();

        if (Globals::globalVerbosity & 4096 && !nonTemporal) {
            cout << "- Calculated duration of new action as being in the range [" << minDur << "," << maxDur << "]\n";
        }

        if (!nonTemporal) {
            if (minDur > maxDur) return 0;
        }

    } else {
        map<int, list<list<StartEvent>::iterator > >::const_iterator tsiOld = parent.entriesForAction.find(actionToApply.first->getID());
        assert(tsiOld != parent.entriesForAction.end());

        const list<StartEvent>::iterator pairWith = tsiOld->second.front();
        minDur = pairWith->minDuration;
        maxDur = pairWith->maxDuration;

    }


    ExtendedMinimalState * const toReturn = parent.applyAction(actionToApply, minDur, maxDur);

    if (actionToApply.second == VAL::E_AT_START) {

        if (!nonTemporal) {
            if (RPGBuilder::canSkipToEnd(actionToApply.first->getID())) {
                toReturn->startEventQueue.push_back(StartEvent(actionToApply.first->getID(), 0, toReturn->getInnerState().planLength - 2, minDur, maxDur, 0.0));
            } else {
                toReturn->startEventQueue.push_back(StartEvent(actionToApply.first->getID(), 0, toReturn->getInnerState().planLength - 2, minDur, maxDur, 0.0));
                list<StartEvent>::iterator backItr = toReturn->startEventQueue.end();
                --backItr;
                toReturn->entriesForAction[actionToApply.first->getID()].push_back(backItr);
                //cout << "Now " << theState.startedActions[actionToApply.first->getID()][0] << " instances of " << *(actionToApply.first) << " going\n";

            }
        }

    } else if (actionToApply.second == VAL::E_AT_END) {
        map<int, list<list<StartEvent>::iterator > >::iterator efaItr = toReturn->entriesForAction.find(actionToApply.first->getID());
        assert(efaItr != toReturn->entriesForAction.end());

        list<list<StartEvent>::iterator >::iterator efaMatch = efaItr->second.begin();
        const list<list<StartEvent>::iterator >::iterator efaEnd = efaItr->second.end();

        if (lEffs) {

            const int intToMatch = lEffs->divisions - 1;

            for (; efaMatch != efaEnd; ++efaMatch) {
                if ((*efaMatch)->divisionsApplied == intToMatch) break;
            }

            assert(efaMatch != efaEnd);


        }

        (*efaMatch)->terminated = true;

    } else {
        assert(false); // is neither a start nor an end
    }

    return toReturn;
}

void ExtendedMinimalState::deQueueFirstOf(const int & actID, const int & divID)
{

    map<int, list<list<StartEvent>::iterator > >::iterator tsiOld = entriesForAction.find(actID);
    assert(tsiOld != entriesForAction.end());

    list<list<StartEvent>::iterator >::iterator pwItr = tsiOld->second.begin();
    const list<list<StartEvent>::iterator >::iterator pwEnd = tsiOld->second.end();

    list<StartEvent>::iterator pairWith;

    bool removed = false;
    for (; pwItr != pwEnd; ++pwItr) {

        if ((*pwItr)->divisionsApplied == divID) {
            pairWith = *pwItr;
            tsiOld->second.erase(pwItr);
            removed = true;
            break;
        }
    }

    assert(removed);

    if (tsiOld->second.empty()) entriesForAction.erase(tsiOld);

    startEventQueue.erase(pairWith);

}

void ExtendedMinimalState::deQueueStep(const int & actID, const int & stepID)
{

    if (stepID == -1) return;

    map<int, list<list<StartEvent>::iterator > >::iterator tsiOld = entriesForAction.find(actID);
    assert(tsiOld != entriesForAction.end());

    list<list<StartEvent>::iterator >::iterator pwItr = tsiOld->second.begin();
    const list<list<StartEvent>::iterator >::iterator pwEnd = tsiOld->second.end();

    for (; pwItr != pwEnd; ++pwItr) {
        if ((*pwItr)->stepID == stepID) {
            startEventQueue.erase(*pwItr);
            tsiOld->second.erase(pwItr);
            if (tsiOld->second.empty()) entriesForAction.erase(tsiOld);
            return;
        }
    }

    assert(pwItr != pwEnd);

}
void FF::evaluateStateAndUpdatePlan(auto_ptr<SearchQueueItem> & succ, ExtendedMinimalState & state, ExtendedMinimalState * prevState, set<int> & goals, set<int> & goalFluents, ParentData * const incrementalData, list<ActionSegment> & helpfulActionsExport, const ActionSegment & actID, list<FFEvent> & header)
{

    list<ActionSegment> helpfulActions;


    FFEvent extraEvent;
    FFEvent extraEventTwo;

    FFEvent * reusedEndEvent = 0;

    bool eventOneDefined = false;
    bool eventTwoDefined = false;

//  pair<int, ScheduleNode*> newCEN(0, 0);

    const bool rawDebug = false;

    map<double, list<pair<int, int> > > * justApplied = 0;
    map<double, list<pair<int, int> > > actualJustApplied;
    double tilFrom = 0.001;

    succ->plan = header;

    int stepID = -1;

    if (actID.second == VAL::E_AT_START) {
        if (rawDebug) cout << "RAW start\n";
        extraEvent = FFEvent(actID.first, state.startEventQueue.back().minDuration, state.startEventQueue.back().maxDuration);
        eventOneDefined = true;

        assert(extraEvent.time_spec == VAL::E_AT_START);
        //makeJustApplied(actualJustApplied, tilFrom, state, true);
        //if (!actualJustApplied.empty()) justApplied = &actualJustApplied;

        if (!RPGBuilder::getRPGDEs(actID.first->getID()).empty()) { // if it's not a non-temporal action

            const int endStepID = state.getInnerState().planLength - 1;
            const int startStepID = endStepID - 1;
            extraEventTwo = FFEvent(actID.first, startStepID, state.startEventQueue.back().minDuration, state.startEventQueue.back().maxDuration);
            extraEvent.pairWithStep = endStepID;
            eventTwoDefined = true;

            if (!RPGBuilder::canSkipToEnd(actID.first->getID())) {
                extraEventTwo.getEffects = false;
            }

            stepID = startStepID;
        } else {
            stepID = state.getInnerState().planLength - 1;
        }

    } else if (actID.second == VAL::E_AT_END) {
        if (rawDebug) cout << "RAW end\n";
        map<int, list<list<StartEvent>::iterator > >::iterator tsiOld = state.entriesForAction.find(actID.first->getID());
        assert(tsiOld != state.entriesForAction.end());

        list<StartEvent>::iterator pairWith = tsiOld->second.front();
        tsiOld->second.pop_front();
        if (tsiOld->second.empty()) state.entriesForAction.erase(tsiOld);

        if (rawDebug || Globals::globalVerbosity & 1048576) cout << "Pairing with start at step " << pairWith->stepID << ", so activating end at " << pairWith->stepID + 1 << "\n";

        stepID = pairWith->stepID + 1;

        {
            list<FFEvent>::iterator pwItr = succ->plan.begin();
            for (int sID = 0; sID <= pairWith->stepID; ++sID, ++pwItr) ;
            pwItr->getEffects = true;
            reusedEndEvent = &(*pwItr);
        }

        state.startEventQueue.erase(pairWith);

        // makeJustApplied(actualJustApplied, tilFrom, state, false);
        // if (!actualJustApplied.empty()) justApplied = &actualJustApplied;
    } else {
        extraEvent = FFEvent(actID.divisionID);
        eventOneDefined = true;
        stepID = state.getInnerState().planLength - 1;
    }


    FFcache_upToDate = false;

    list<FFEvent> nowList;

    //  extraEvent.needToFinish = actID.needToFinish;

    if (eventOneDefined) nowList.push_back(extraEvent);
    if (eventTwoDefined) nowList.push_back(extraEventTwo);

    assert(stepID != -1);

    HTrio h1(calculateHeuristicAndSchedule(state, prevState, goals, goalFluents, incrementalData, helpfulActions, succ->plan, nowList, stepID, true, justApplied, tilFrom));

    if (eventTwoDefined) {
        extraEventTwo = nowList.back();
        nowList.pop_back();
    }

    if (eventOneDefined) {
        extraEvent = nowList.back();
    }

    HTrio hcurr(h1);
    //int actionsSaved = 0;



    helpfulActionsExport = helpfulActions;
    succ->heuristicValue = hcurr;
    // succ->plan = header;
    if (eventOneDefined) {
        succ->plan.push_back(extraEvent);
    }

    if (eventTwoDefined) {
        succ->plan.push_back(extraEventTwo);
    }


    if (actID.second == VAL::E_AT_START // If it's the start of an action...
            && !RPGBuilder::getRPGDEs(actID.first->getID()).empty() // that is temporal..
            && RPGBuilder::canSkipToEnd(actID.first->getID())) { // and compression-safe

        // we can pull it off the start event queue as we don't need to worry about applying it

        state.startEventQueue.pop_back();
    }
}


bool FF::precedingActions(ExtendedMinimalState & theState, const ActionSegment & actionSeg, list<ActionSegment> & alsoMustDo, const int oldTIL, const double moveOn)
{

//  static const double EPSILON = 0.001;
    static const bool tsDebug = false;
    static vector<double> tilStamps;
    static vector<list<int> > tilNegativeEffects;
    static vector<list<int> > tilPermanentNegativeEffects;
    static int tilCount;
    static bool haveTILstamps = false;

    if (!haveTILstamps) {
        haveTILstamps = true;
        vector<RPGBuilder::FakeTILAction*> & tilVec = RPGBuilder::getTILVec();
        tilCount = tilVec.size();
        tilStamps = vector<double>(tilCount);
        tilNegativeEffects = vector<list<int> >(tilCount);
        tilPermanentNegativeEffects = vector<list<int> >(tilCount);
        set<int> addedLater;
        for (int i = tilCount - 1; i >= 0; --i) {
            tilStamps[i] = tilVec[i]->duration;
            {
                list<Literal*>::iterator effItr = tilVec[i]->addEffects.begin();
                const list<Literal*>::iterator effEnd = tilVec[i]->addEffects.end();

                for (; effItr != effEnd; ++effItr) {
                    const int currEffID = (*effItr)->getID();
                    //tilEffects[i].push_back(currEffID);
                    addedLater.insert(currEffID);
                }
            }

            {
                list<Literal*>::iterator effItr = tilVec[i]->delEffects.begin();
                const list<Literal*>::iterator effEnd = tilVec[i]->delEffects.end();

                for (; effItr != effEnd; ++effItr) {
                    const int currEffID = (*effItr)->getID();
                    tilNegativeEffects[i].push_back(currEffID);
                    if (RPGBuilder::getEffectsToActions(currEffID).empty() && addedLater.find(currEffID) == addedLater.end()) {
                        tilPermanentNegativeEffects[i].push_back(currEffID);
                    }
                }
            }
        }
    }


    const VAL::time_spec ts = actionSeg.second;

    if (ts == VAL::E_AT_START) {


        if (tsDebug) cout << "Considering start of " << *(actionSeg.first) << "\n";

        {
            list<StartEvent>::iterator tsiItr = theState.startEventQueue.begin();
            const list<StartEvent>::iterator tsiEnd = theState.startEventQueue.end();

            for (; tsiItr != tsiEnd; ++tsiItr) {
                if (!tsiItr->terminated) {
                    const double wouldBe = tsiItr->elapsed + moveOn;
                    if (wouldBe > tsiItr->maxDuration) {
                        if (RPGBuilder::canSkipToEnd(tsiItr->actID)) {
                            alsoMustDo.push_back(ActionSegment(RPGBuilder::getInstantiatedOp(tsiItr->actID), VAL::E_AT_END, -16, RPGHeuristic::emptyIntList));
                            cout << "Also must do " << *(RPGBuilder::getInstantiatedOp(tsiItr->actID)) << "\n";
                        } else {
                            if (tsDebug) cout << "Now " << tsiItr->elapsed << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->actID)) << " pt. " << tsiItr->divisionsApplied << ", this exceeds duration\n";
                            return false;
                        }
                    }
                }
            }
        }

    } else if (ts == VAL::E_AT_END) {

        if (tsDebug) cout << "Considering end of " << *(actionSeg.first) << "\n";

        map<int, list<list<StartEvent>::iterator > >::iterator tsiOld = theState.entriesForAction.find(actionSeg.first->getID());
        assert(tsiOld != theState.entriesForAction.end());

        list<StartEvent>::iterator pairWith;

        {
            RPGBuilder::LinearEffects * leffs = RPGBuilder::getLinearDiscretisation()[actionSeg.first->getID()];
            if (leffs) {
                const int toMatch = leffs->divisions - 1;

                list<list<StartEvent>::iterator >::iterator tiItr = tsiOld->second.begin();
                const list<list<StartEvent>::iterator >::iterator tiEnd = tsiOld->second.end();

                for (; tiItr != tiEnd; ++tiItr) {
                    if ((*tiItr)->divisionsApplied == toMatch) {
                        pairWith = *tiItr;
                        break;
                    }
                }

                assert(tiItr != tiEnd);

            } else {
                pairWith = tsiOld->second.front();
            }
        }

        if (tsDebug) cout << "Started " << pairWith->elapsed << " ago, of duration [" << pairWith->minDuration << "," << pairWith->maxDuration << "\n";

        double extraTime = pairWith->advancingDuration;

        //if (pairWith->minDuration < pairWith->maxDuration) {
        if (extraTime < moveOn) extraTime = moveOn;
        //}

        if (tsDebug) cout << "extra time = " << extraTime << "\n";

        //if (extraTime < 0) {
        //  return false;

        //}

        const list<StartEvent>::iterator theRestEnd = theState.startEventQueue.end();

        for (list<StartEvent>::iterator bqItr = theState.startEventQueue.begin(); bqItr != pairWith; ++bqItr) {

            if (!bqItr->terminated) {

                if (tsDebug) cout << "Can advance " << *(RPGBuilder::getInstantiatedOp(bqItr->actID)) << " point " << bqItr->divisionsApplied << "\n";

                const double wouldBe = bqItr->elapsed + extraTime;
                if (wouldBe > bqItr->maxDuration) {
                    if (RPGBuilder::canSkipToEnd(bqItr->actID)) {
                        alsoMustDo.push_back(ActionSegment(RPGBuilder::getInstantiatedOp(bqItr->actID), VAL::E_AT_END, -16, RPGHeuristic::emptyIntList));
                        cout << "Also must do " << *(RPGBuilder::getInstantiatedOp(bqItr->actID)) << "\n";
                    } else {
                        if (tsDebug) cout << "Now " << bqItr->elapsed << " since point " << bqItr->divisionsApplied << " of " << *(RPGBuilder::getInstantiatedOp(bqItr->actID)) << ", this exceeds duration\n";
                        return false;
                    }
                }
            }

        }



        if (theState.getInnerState().nextTIL < tilCount) {
            if (tilStamps[theState.getInnerState().nextTIL] < theState.timeStamp + extraTime) {
                if (tsDebug) cout << "Too late for the next TIL: " << tilStamps[theState.getInnerState().nextTIL] << "\n";
                return false;
            }
        }


        {
            list<StartEvent>::iterator theRestItr = pairWith;

            ++theRestItr;

            for (; theRestItr != theRestEnd; ++theRestItr) {

                if (!theRestItr->terminated) {
                    const double wouldBe = theRestItr->elapsed + moveOn;
                    if (wouldBe > theRestItr->maxDuration) {
                        if (RPGBuilder::canSkipToEnd(theRestItr->actID)) {
                            alsoMustDo.push_back(ActionSegment(RPGBuilder::getInstantiatedOp(theRestItr->actID), VAL::E_AT_END, -1, RPGHeuristic::emptyIntList));
                            cout << "Also must do " << *(RPGBuilder::getInstantiatedOp(theRestItr->actID)) << "\n";
                        } else {
                            if (tsDebug) cout << "Now " << theRestItr->elapsed << " since point " << theRestItr->divisionsApplied << " of " << *(RPGBuilder::getInstantiatedOp(theRestItr->actID)) << ", this exceeds duration\n";
                            return false;
                        }
                    }
                }
            }
        }

    }

    return true;
}

bool FF::checkTemporalSoundness(ExtendedMinimalState & theState, const ActionSegment & actionSeg, const int oldTIL, const double moveOn)
{

    //const int & theAction, const VAL::time_spec & ts


    if (!tsChecking || true) return true;

    static const double EPSILON = 0.001;
    static const bool tsDebug = (Globals::globalVerbosity & 16);
    static vector<double> tilStamps;
    static vector<list<int> > tilNegativeEffects;
    static vector<list<int> > tilPermanentNegativeEffects;
    static int tilCount;
    static bool haveTILstamps = false;

    if (!haveTILstamps) {
        haveTILstamps = true;
        vector<RPGBuilder::FakeTILAction*> & tilVec = RPGBuilder::getTILVec();
        tilCount = tilVec.size();
        tilStamps = vector<double>(tilCount);
        tilNegativeEffects = vector<list<int> >(tilCount);
        tilPermanentNegativeEffects = vector<list<int> >(tilCount);
        set<int> addedLater;
        for (int i = tilCount - 1; i >= 0; --i) {
            tilStamps[i] = tilVec[i]->duration;
            {
                list<Literal*>::iterator effItr = tilVec[i]->addEffects.begin();
                const list<Literal*>::iterator effEnd = tilVec[i]->addEffects.end();

                for (; effItr != effEnd; ++effItr) {
                    const int currEffID = (*effItr)->getID();
                    //tilEffects[i].push_back(currEffID);
                    addedLater.insert(currEffID);
                }
            }

            {
                list<Literal*>::iterator effItr = tilVec[i]->delEffects.begin();
                const list<Literal*>::iterator effEnd = tilVec[i]->delEffects.end();

                for (; effItr != effEnd; ++effItr) {
                    const int currEffID = (*effItr)->getID();
                    tilNegativeEffects[i].push_back(currEffID);
                    if (RPGBuilder::getEffectsToActions(currEffID).empty() && addedLater.find(currEffID) == addedLater.end()) {
                        tilPermanentNegativeEffects[i].push_back(currEffID);
                    }
                }
            }
        }
    }

    const VAL::time_spec ts = actionSeg.second;

    if (ts == VAL::E_AT_START) {


        if (tsDebug) cout << "Considering start of " << *(actionSeg.first) << "\n";
        const bool nonTemporal = RPGBuilder::getRPGDEs(actionSeg.first->getID()).empty();

        {
            list<StartEvent>::iterator tsiItr = theState.startEventQueue.begin();
            const list<StartEvent>::iterator tsiEnd = theState.startEventQueue.end();

            for (; tsiItr != tsiEnd; ++tsiItr) {
                if (!tsiItr->terminated) {
                    const double wouldBe = tsiItr->elapsed + moveOn;
                    if (wouldBe > tsiItr->maxDuration) {
                        return false;
                    } else {
                        tsiItr->elapsed = wouldBe;
                    }
                    if (tsDebug) cout << "Now " << tsiItr->elapsed << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->actID)) << " pt. " << tsiItr->divisionsApplied << "\n";
                    tsiItr->advancingDuration -= moveOn;
                }
            }
        }

        if (tsDebug) cout << "Now " << moveOn << " since start of new action " << *(actionSeg.first) << "\n";

        theState.timeStamp += moveOn;

        if (theState.getInnerState().nextTIL < tilCount) {
            if (tilStamps[theState.getInnerState().nextTIL] < theState.timeStamp) return false;

            const double earliestFinish = theState.timeStamp + (nonTemporal ? 0.0 : theState.startEventQueue.back().advancingDuration);

            list<Literal*> precs;
            list<Literal*> inv;
            list<RPGBuilder::NumericPrecondition> numericPrec;
            list<RPGBuilder::NumericPrecondition> numericInv;

            RPGBuilder::getPrecInv(actionSeg.first, false, precs, inv, numericPrec, numericInv);

            for (int tc = theState.getInnerState().nextTIL; tc < tilCount; ++tc) {
                if (tilStamps[tc] < earliestFinish) { // if TIL has to occur during the execution of this action
                    {
                        list<int>::iterator pnItr = tilPermanentNegativeEffects[tc].begin();
                        const list<int>::iterator pnEnd = tilPermanentNegativeEffects[tc].end();

                        for (; pnItr != pnEnd; ++pnItr) {
                            list<Literal*>::iterator pItr = precs.begin();
                            const list<Literal*>::iterator pEnd = precs.end();

                            for (; pItr != pEnd; ++pItr) {
                                if ((*pItr)->getID() == *pnItr) return false;
                            }
                        }
                    }
                    {
                        list<int>::iterator pnItr = tilNegativeEffects[tc].begin();
                        const list<int>::iterator pnEnd = tilNegativeEffects[tc].end();

                        for (; pnItr != pnEnd; ++pnItr) {
                            list<Literal*>::iterator pItr = inv.begin();
                            const list<Literal*>::iterator pEnd = inv.end();

                            for (; pItr != pEnd; ++pItr) {
                                if ((*pItr)->getID() == *pnItr) return false;
                            }
                        }
                    }
                }
            }
        }

        if (!nonTemporal && !RPGBuilder::canSkipToEnd(actionSeg.first->getID()) && LPScheduler::optimiseOrdering) {
            {
                list<StartEvent>::reverse_iterator tsiItr = theState.startEventQueue.rbegin();
                const list<StartEvent>::reverse_iterator tsiEnd = theState.startEventQueue.rend();

                int & thisFanIn = tsiItr->fanIn;
                int & thisStepID = tsiItr->stepID;
                const double thisIncr = tsiItr->minDuration - tsiItr->elapsed;
                StartEvent & eca = *tsiItr;
                if (thisIncr >= EPSILON) {
                    ++tsiItr;

                    for (; tsiItr != tsiEnd; ++tsiItr) {
                        if (!tsiItr->terminated) {
                            const double consIncr = tsiItr->maxDuration - tsiItr->elapsed;
                            if (consIncr < thisIncr && fabs(thisIncr - consIncr) > 0.0005) {
                                ++thisFanIn;
                                tsiItr->endComesBefore.insert(thisStepID);
                                eca.endComesBeforePair.insert(tsiItr->stepID);
                            }
                        }
                        //cout << "Now " << tsiItr->second << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->first)) << "\n";
                    }
                    if (theState.getInnerState().nextTIL < tilCount) {
                        const double consIncr = tilStamps[theState.getInnerState().nextTIL] - theState.timeStamp;
                        if (consIncr < thisIncr && fabs(thisIncr - consIncr) > 0.0005) {
                            ++thisFanIn;
                            theState.tilComesBefore.push_back(thisStepID);
                        }
                    }
                }
            }

            {
                {
                    list<Literal*> & delEffs =  RPGBuilder::getEndPropositionDeletes()[actionSeg.first->getID()];
                    list<Literal*> & invs    = RPGBuilder::getInvariantPropositionalPreconditions()[actionSeg.first->getID()];
                    LiteralSet & oneShotEndPrecs  = RPGBuilder::getEndOneShots(actionSeg.first->getID());

                    if (!delEffs.empty()) {
                        set<int> deSet;
                        {
                            list<Literal*>::iterator deItr = delEffs.begin();
                            const list<Literal*>::iterator deEnd = delEffs.end();
                            for (; deItr != deEnd; ++deItr) deSet.insert((*deItr)->getID());
                        }

                        list<StartEvent>::reverse_iterator tsiItr = theState.startEventQueue.rbegin();
                        const list<StartEvent>::reverse_iterator tsiEnd = theState.startEventQueue.rend();

                        StartEvent & eca = *tsiItr;

                        ++tsiItr;

                        for (; tsiItr != tsiEnd; ++tsiItr) {
                            if (!tsiItr->terminated) {
                                bool precedes = false;
                                list<Literal*> & openInvs = RPGBuilder::getInvariantPropositionalPreconditions()[tsiItr->actID];
                                list<Literal*>::iterator oiItr = openInvs.begin();
                                const list<Literal*>::iterator oiEnd = openInvs.end();
                                for (; oiItr != oiEnd; ++oiItr) {
                                    if (deSet.find((*oiItr)->getID()) != deSet.end()) {
                                        eca.endMustComeAfter(tsiItr->stepID);
                                        tsiItr->endMustComeAfterPair(eca.stepID);
                                        precedes = true;
                                        break;
                                    }
                                }

                                if (!precedes) {
                                    LiteralSet & thisEndOneShot = RPGBuilder::getEndOneShots(tsiItr->actID);

                                    LiteralSet::iterator apItr = thisEndOneShot.begin();
                                    const LiteralSet::iterator apEnd = thisEndOneShot.end();

                                    set<int>::iterator adItr = deSet.begin();
                                    const set<int>::iterator adEnd = deSet.end();

                                    while (apItr != apEnd && adItr != adEnd) {
                                        const int idOne = (*apItr)->getID();
                                        const int idTwo = *adItr;
                                        if (idOne < idTwo) {
                                            ++apItr;
                                        } else if (idTwo < idOne) {
                                            ++adItr;
                                        } else {
                                            eca.endMustComeAfter(tsiItr->stepID);
                                            tsiItr->endMustComeAfterPair(eca.stepID);
                                            break;
                                        }
                                    };

                                }
                            }
                        }

                    }

                    if (!invs.empty()) {
                        set<int> invSet;
                        {
                            list<Literal*>::iterator deItr = invs.begin();
                            const list<Literal*>::iterator deEnd = invs.end();
                            for (; deItr != deEnd; ++deItr) invSet.insert((*deItr)->getID());
                        }

                        list<StartEvent>::reverse_iterator tsiItr = theState.startEventQueue.rbegin();
                        const list<StartEvent>::reverse_iterator tsiEnd = theState.startEventQueue.rend();

                        int & thisStepID = tsiItr->stepID;

                        StartEvent & eca = *tsiItr;
                        ++tsiItr;

                        for (; tsiItr != tsiEnd; ++tsiItr) {
                            if (!tsiItr->terminated) {
                                list<Literal*> & openDels = RPGBuilder::getEndPropositionDeletes()[tsiItr->actID];
                                list<Literal*>::iterator odItr = openDels.begin();
                                const list<Literal*>::iterator odEnd = openDels.end();
                                for (; odItr != odEnd; ++odItr) {
                                    if (invSet.find((*odItr)->getID()) != invSet.end()) {
                                        tsiItr->endMustComeAfter(thisStepID);
                                        eca.endMustComeAfterPair(tsiItr->stepID);
                                        break;
                                    }
                                }
                            }
                        }

                    }

                    if (!oneShotEndPrecs.empty()) {

                        list<StartEvent>::reverse_iterator tsiItr = theState.startEventQueue.rbegin();
                        const list<StartEvent>::reverse_iterator tsiEnd = theState.startEventQueue.rend();

                        int & thisStepID = tsiItr->stepID;

                        ++tsiItr;

                        for (; tsiItr != tsiEnd; ++tsiItr) {
                            if (!tsiItr->terminated) {
                                list<Literal*> & openDels = RPGBuilder::getEndPropositionDeletes()[tsiItr->actID];
                                list<Literal*>::iterator odItr = openDels.begin();
                                const list<Literal*>::iterator odEnd = openDels.end();
                                for (; odItr != odEnd; ++odItr) {
                                    if (oneShotEndPrecs.find(*odItr) != oneShotEndPrecs.end()) {
                                        tsiItr->endMustComeAfter(thisStepID);
                                        break;
                                    }
                                }
                            }
                        }

                    }


                }
            }
        }

    } else if (ts == VAL::E_OVER_ALL) {
        if (tsDebug) cout << "Considering intermediate point " << actionSeg.divisionID << " within " << *(actionSeg.first) << "\n";


        map<int, list<list<StartEvent>::iterator > >::iterator tsiOld = theState.entriesForAction.find(actionSeg.first->getID());
        assert(tsiOld != theState.entriesForAction.end());



        list<StartEvent>::iterator pairWith;

        {
            RPGBuilder::LinearEffects * leffs = RPGBuilder::getLinearDiscretisation()[actionSeg.first->getID()];
            assert(leffs);
            list<list<StartEvent>::iterator >::iterator tiItr = tsiOld->second.begin();
            const list<list<StartEvent>::iterator >::iterator tiEnd = tsiOld->second.end();

            for (; tiItr != tiEnd; ++tiItr) {
                if ((*tiItr)->divisionsApplied == actionSeg.divisionID) {
                    pairWith = *tiItr;
                    break;
                }
            }
            assert(tiItr != tiEnd);
        }

        if (!pairWith->endComesAfter.empty()) return false;
        if (!pairWith->endComesBeforePair.empty()) return false;

        if (tsDebug) cout << "Started " << pairWith->elapsed << " ago, of duration [" << pairWith->minDuration << "," << pairWith->maxDuration << "\n";

        double extraTime = pairWith->advancingDuration;

        //if (pairWith->minDuration < pairWith->maxDuration) {
        if (extraTime < moveOn) extraTime = moveOn;
        //}

        if (tsDebug) cout << "extra time = " << extraTime << "\n";

        //if (extraTime < 0) {
        //  return false;

        //}

        const list<StartEvent>::iterator theRestEnd = theState.startEventQueue.end();



        for (list<StartEvent>::iterator bqItr = theState.startEventQueue.begin(); bqItr != pairWith; ++bqItr) {

            if (tsDebug) cout << "Can advance " << *(RPGBuilder::getInstantiatedOp(bqItr->actID)) << " point " << bqItr->divisionsApplied << "\n";

            if ((bqItr->elapsed = bqItr->elapsed + extraTime) > bqItr->maxDuration) {
                if (tsDebug) cout << "Now " << bqItr->elapsed << " since point " << bqItr->divisionsApplied << " of " << *(RPGBuilder::getInstantiatedOp(bqItr->actID)) << ", this exceeds duration\n";
                return false;
            }
            bqItr->advancingDuration -= extraTime;

        }


        theState.timeStamp += extraTime;

        if (tsDebug) cout << "State timestamp is now " << theState.timeStamp << "\n";

        if (theState.getInnerState().nextTIL < tilCount) {
            if (tilStamps[theState.getInnerState().nextTIL] < theState.timeStamp) {
                if (tsDebug) cout << "Too late for the next TIL: " << tilStamps[theState.getInnerState().nextTIL] << "\n";
                return false;
            }
        }


        {
            list<StartEvent>::iterator theRestItr = pairWith;

            ++theRestItr;

            for (; theRestItr != theRestEnd; ++theRestItr) {

                if ((theRestItr->elapsed = theRestItr->elapsed + moveOn) > theRestItr->maxDuration) {
                    if (tsDebug) cout << "Now " << theRestItr->elapsed << " since point " << theRestItr->divisionsApplied << " of " << *(RPGBuilder::getInstantiatedOp(theRestItr->actID)) << ", this exceeds duration\n";
                    return false;
                }

                theRestItr->advancingDuration -= extraTime;

            }
        }

        {
            set<int> & beforeList = pairWith->endComesBefore;

            if (!beforeList.empty()) {

                set<int>::iterator cbeItr = beforeList.begin();
                const set<int>::iterator cbeEnd = beforeList.end();

                list<StartEvent>::iterator theRestItr = pairWith;
                ++theRestItr;

                if (*cbeItr == -1) {
                    --(theState.tilFanIn);
                    ++cbeItr;
                }

                if (cbeItr != cbeEnd) {
                    for (; theRestItr != theRestEnd; ++theRestItr) {

                        if (*cbeItr == theRestItr->stepID) {
                            --(theRestItr->fanIn);
                            (theRestItr->endComesBeforePair.erase(pairWith->stepID));
                            ++cbeItr;
                            if (cbeItr == cbeEnd) break;
                            if (*cbeItr == -1) {
                                --(theState.tilFanIn);
                                ++cbeItr;
                            }
                            if (cbeItr == cbeEnd) break;
                        }

                    }
                }

            }

        }

        {
            int & thisFanIn = pairWith->fanIn;

            if (thisFanIn) {

                const int thisStepID = pairWith->stepID;

                for (list<StartEvent>::iterator bqItr = theState.startEventQueue.begin(); thisFanIn && bqItr != pairWith; ++bqItr) {
                    set<int> & cbfList = bqItr->endComesBefore;
                    set<int>::iterator cbfItr = cbfList.begin();
                    const set<int>::iterator cbfEnd = cbfList.end();

                    for (; cbfItr != cbfEnd; ++cbfItr) {
                        const int & deref = *cbfItr;
                        if (deref > thisStepID) {
                            break;
                        } else if (deref == thisStepID) {
                            bqItr->endComesBeforePair.erase(thisStepID);
                            cbfList.erase(cbfItr);
                            --thisFanIn;
                            break;
                        }
                    }

                }

                if (thisFanIn) {
                    list<int> & cbfList = theState.tilComesBefore;
                    list<int>::iterator cbfItr = cbfList.begin();
                    const list<int>::iterator cbfEnd = cbfList.end();

                    for (; cbfItr != cbfEnd; ++cbfItr) {
                        const int & deref = *cbfItr;
                        if (deref > thisStepID) {
                            break;
                        } else if (deref == thisStepID) {
                            cbfList.erase(cbfItr);
                            --thisFanIn;
                            break;
                        }
                    }
                }

            }
        }


        {
            list<StartEvent>::reverse_iterator tsiItr = theState.startEventQueue.rbegin();
            const list<StartEvent>::reverse_iterator tsiEnd = theState.startEventQueue.rend();

            int & thisFanIn = tsiItr->fanIn;
            int & thisStepID = tsiItr->stepID;
            const double thisIncr = tsiItr->minDuration - tsiItr->elapsed;

            StartEvent & eca = *tsiItr;
            if (thisIncr >= EPSILON) {
                ++tsiItr;

                for (; tsiItr != tsiEnd; ++tsiItr) {
                    const double consIncr = tsiItr->maxDuration - tsiItr->elapsed;
                    if (consIncr < thisIncr && fabs(thisIncr - consIncr) > 0.0005) {
                        ++thisFanIn;
                        tsiItr->endComesBefore.insert(thisStepID);
                        eca.endComesBeforePair.insert(tsiItr->stepID);
                    }
                    //cout << "Now " << tsiItr->second << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->first)) << "\n";
                }
                if (theState.getInnerState().nextTIL < tilCount) {
                    const double consIncr = tilStamps[theState.getInnerState().nextTIL] - theState.timeStamp;
                    if (consIncr < thisIncr && fabs(thisIncr - consIncr) > 0.0005) {
                        ++thisFanIn;
                        theState.tilComesBefore.push_back(thisStepID);
                    }
                }
            }
        }

    } else if (ts == VAL::E_AT_END) {

        if (tsDebug) cout << "Considering end of " << *(actionSeg.first) << "\n";

        map<int, list<list<StartEvent>::iterator > >::iterator tsiOld = theState.entriesForAction.find(actionSeg.first->getID());
        assert(tsiOld != theState.entriesForAction.end());



        list<StartEvent>::iterator pairWith;

        {
            RPGBuilder::LinearEffects * leffs = RPGBuilder::getLinearDiscretisation()[actionSeg.first->getID()];
            if (leffs) {
                const int toMatch = leffs->divisions - 1;

                list<list<StartEvent>::iterator >::iterator tiItr = tsiOld->second.begin();
                const list<list<StartEvent>::iterator >::iterator tiEnd = tsiOld->second.end();

                for (; tiItr != tiEnd; ++tiItr) {
                    if ((*tiItr)->divisionsApplied == toMatch) {
                        pairWith = *tiItr;
                        break;
                    }
                }

                assert(tiItr != tiEnd);

            } else {
                pairWith = tsiOld->second.front();
            }
        }

        if (!pairWith->getEndComesAfter().empty()) {
            if (tsDebug) {
                cout << "ECA list is not empty, contains:";
                set<int>::iterator ecaItr = pairWith->getEndComesAfter().begin();
                const set<int>::iterator ecaEnd = pairWith->getEndComesAfter().end();

                for (; ecaItr != ecaEnd; ++ecaItr) {
                    cout << " " << *ecaItr;
                }
                cout << "\n";
            }
            return false;
        }

        if (!pairWith->getEndComesBeforePair().empty()) {
            if (tsDebug) cout << "ECB-pair list is not empty\n";
            return false;
        }

        if (tsDebug) cout << "Started " << pairWith->elapsed << " ago, of duration [" << pairWith->minDuration << "," << pairWith->maxDuration << "\n";

        double extraTime = pairWith->advancingDuration;

        //if (pairWith->minDuration < pairWith->maxDuration) {
        if (extraTime < moveOn) extraTime = moveOn;
        //}

        if (tsDebug) cout << "extra time = " << extraTime << "\n";

        //if (extraTime < 0) {
        //  return false;

        //}

        const list<StartEvent>::iterator theRestEnd = theState.startEventQueue.end();

        for (list<StartEvent>::iterator bqItr = theState.startEventQueue.begin(); bqItr != pairWith; ++bqItr) {

            if (!bqItr->terminated) {

                if (tsDebug) cout << "Can advance " << *(RPGBuilder::getInstantiatedOp(bqItr->actID)) << " point " << bqItr->divisionsApplied << "\n";

                if ((bqItr->elapsed = bqItr->elapsed + extraTime) > bqItr->maxDuration) {
                    if (tsDebug) cout << "Now " << bqItr->elapsed << " since point " << bqItr->divisionsApplied << " of " << *(RPGBuilder::getInstantiatedOp(bqItr->actID)) << ", this exceeds duration\n";
                    return false;
                }
                bqItr->advancingDuration -= extraTime;
            }

        }


        theState.timeStamp += extraTime;

        if (tsDebug) cout << "State timestamp is now " << theState.timeStamp << "\n";

        if (theState.getInnerState().nextTIL < tilCount) {
            if (tilStamps[theState.getInnerState().nextTIL] < theState.timeStamp) {
                if (tsDebug) cout << "Too late for the next TIL: " << tilStamps[theState.getInnerState().nextTIL] << "\n";
                return false;
            }
        }


        {
            list<StartEvent>::iterator theRestItr = pairWith;

            ++theRestItr;

            for (; theRestItr != theRestEnd; ++theRestItr) {

                if (!theRestItr->terminated) {

                    if ((theRestItr->elapsed = theRestItr->elapsed + moveOn) > theRestItr->maxDuration) {
                        if (tsDebug) cout << "Now " << theRestItr->elapsed << " since point " << theRestItr->divisionsApplied << " of " << *(RPGBuilder::getInstantiatedOp(theRestItr->actID)) << ", this exceeds duration\n";
                        return false;
                    }

                    theRestItr->advancingDuration -= extraTime;
                }
            }
        }

        if (LPScheduler::optimiseOrdering) {

            {
                set<int> & beforeList = pairWith->endComesBefore;

                if (!beforeList.empty()) {

                    set<int>::iterator cbeItr = beforeList.begin();
                    const set<int>::iterator cbeEnd = beforeList.end();

                    list<StartEvent>::iterator theRestItr = pairWith;
                    ++theRestItr;

                    if (*cbeItr == -1) {
                        --(theState.tilFanIn);
                        ++cbeItr;
                    }

                    if (cbeItr != cbeEnd) {
                        for (; theRestItr != theRestEnd; ++theRestItr) {

                            if (!theRestItr->terminated) {

                                if (*cbeItr == theRestItr->stepID) {
                                    theRestItr->endComesBeforePair.erase(pairWith->stepID);
                                    --(theRestItr->fanIn);
                                    ++cbeItr;
                                    if (cbeItr == cbeEnd) break;
                                    if (*cbeItr == -1) {
                                        --(theState.tilFanIn);
                                        ++cbeItr;
                                    }
                                    if (cbeItr == cbeEnd) break;
                                }
                            }
                        }
                    }

                }

            }

            {
                int & thisFanIn = pairWith->fanIn;

                if (thisFanIn) {

                    const int thisStepID = pairWith->stepID;

                    for (list<StartEvent>::iterator bqItr = theState.startEventQueue.begin(); thisFanIn && bqItr != pairWith; ++bqItr) {
                        if (!bqItr->terminated) {
                            set<int> & cbfList = bqItr->endComesBefore;
                            set<int>::iterator cbfItr = cbfList.begin();
                            const set<int>::iterator cbfEnd = cbfList.end();

                            for (; cbfItr != cbfEnd; ++cbfItr) {
                                const int & deref = *cbfItr;
                                if (deref > thisStepID) {
                                    break;
                                } else if (deref == thisStepID) {
                                    bqItr->endComesBeforePair.erase(thisStepID);
                                    cbfList.erase(cbfItr);
                                    --thisFanIn;
                                    break;
                                }
                            }
                        }

                    }

                    if (thisFanIn) {
                        list<int> & cbfList = theState.tilComesBefore;
                        list<int>::iterator cbfItr = cbfList.begin();
                        const list<int>::iterator cbfEnd = cbfList.end();

                        for (; cbfItr != cbfEnd; ++cbfItr) {
                            const int & deref = *cbfItr;
                            if (deref > thisStepID) {
                                break;
                            } else if (deref == thisStepID) {
                                cbfList.erase(cbfItr);
                                --thisFanIn;
                                break;
                            }
                        }
                    }

                }
            }

            {
                const int errStepID = pairWith->stepID;

                for (list<StartEvent>::iterator bqItr = theState.startEventQueue.begin(); bqItr != pairWith; ++bqItr) {
                    if (!bqItr->terminated) bqItr->actionHasFinished(errStepID);
                }

                list<StartEvent>::iterator theRestItr = pairWith;

                ++theRestItr;

                for (; theRestItr != theRestEnd; ++theRestItr) {
                    if (!theRestItr->terminated) theRestItr->actionHasFinished(errStepID);

                }
            }
        }
    } else { // TIL
        if (tsDebug) cout << "Applying TIL\n";

        for (int pass = oldTIL; pass <= actionSeg.divisionID; ++pass) {

            const double thisAdvancer = tilStamps[pass] - theState.timeStamp;

            list<StartEvent>::iterator tsiItr = theState.startEventQueue.begin();
            const list<StartEvent>::iterator tsiEnd = theState.startEventQueue.end();

            for (; tsiItr != tsiEnd; ++tsiItr) {
                if (!tsiItr->terminated) {
                    if (tsiItr->stepID < theState.stepBeforeTIL) {
                        if ((tsiItr->elapsed = tsiItr->elapsed + thisAdvancer) > tsiItr->maxDuration) {
                            if (tsDebug) cout << "Now " << tsiItr->elapsed << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->actID)) << ", this exceeds duration\n";
                            return false;
                        }
                        if (tsDebug) cout << "Now " << tsiItr->elapsed << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->actID)) << "\n";
                        tsiItr->advancingDuration -= thisAdvancer;
                    } else {
                        if ((tsiItr->elapsed = tsiItr->elapsed + EPSILON) > tsiItr->maxDuration) {
                            if (tsDebug) cout << "Now " << tsiItr->elapsed << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->actID)) << ", this exceeds duration\n";
                            return false;
                        }
                        if (tsDebug) cout << "Now " << tsiItr->elapsed << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->actID)) << "\n";
                        tsiItr->advancingDuration -= thisAdvancer;
                    }
                }
            }

            if (LPScheduler::optimiseOrdering) {
                {
                    list<int> & beforeList = theState.tilComesBefore;

                    if (!beforeList.empty()) {

                        list<int>::iterator cbeItr = beforeList.begin();
                        const list<int>::iterator cbeEnd = beforeList.end();

                        list<StartEvent>::iterator theRestItr = theState.startEventQueue.begin();
                        const list<StartEvent>::iterator theRestEnd = theState.startEventQueue.end();

                        while (theRestItr != theRestEnd && theRestItr->stepID < theState.stepBeforeTIL) ++theRestItr;

                        for (; theRestItr != theRestEnd; ++theRestItr) {
                            if (!theRestItr->terminated) {
                                if (*cbeItr == theRestItr->stepID) {
                                    --(theRestItr->fanIn);
                                    ++cbeItr;
                                    if (cbeItr == cbeEnd) break;
                                }
                            }

                        }

                    }

                }

                {
                    int & thisFanIn = theState.tilFanIn;

                    if (thisFanIn) {

                        list<StartEvent>::iterator bqItr = theState.startEventQueue.begin();
                        const list<StartEvent>::iterator bqEnd = theState.startEventQueue.end();

                        for (; thisFanIn && bqItr != bqEnd && bqItr->stepID < (theState.stepBeforeTIL - 1); ++bqItr) {
                            if (!bqItr->terminated) {
                                set<int> & cbfList = bqItr->endComesBefore;
                                set<int>::iterator cbfItr = cbfList.begin();
                                const set<int>::iterator cbfEnd = cbfList.end();

                                for (; cbfItr != cbfEnd; ++cbfItr) {
                                    const int & deref = *cbfItr;
                                    if (deref > -1) {
                                        break;
                                    } else if (deref == -1) {
                                        cbfList.erase(cbfItr);
                                        --thisFanIn;
                                        break;
                                    }
                                }
                            }

                        }

                    }
                }
            }

            theState.timeStamp = tilStamps[pass];
            theState.stepBeforeTIL = theState.getInnerState().planLength;
            theState.tilFanIn = 0;
            theState.tilComesBefore.clear();

            if (LPScheduler::optimiseOrdering) {

                if ((pass + 1) < tilCount) {
                    list<StartEvent>::reverse_iterator tsiItr = theState.startEventQueue.rbegin();
                    const list<StartEvent>::reverse_iterator tsiEnd = theState.startEventQueue.rend();

                    int & thisFanIn = theState.tilFanIn;
                    const double thisIncr = tilStamps[pass+1] - tilStamps[pass];
                    if (thisIncr >= EPSILON) {
                        for (; tsiItr != tsiEnd; ++tsiItr) {
                            if (!tsiItr->terminated) {
                                const double consIncr = tsiItr->maxDuration - tsiItr->elapsed;
                                if (consIncr < thisIncr && fabs(thisIncr - consIncr) > 0.0005) {
                                    ++thisFanIn;
                                    tsiItr->endComesBefore.insert(-1);
                                }
                            }
                            //cout << "Now " << tsiItr->second << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->first)) << "\n";
                        }
                    }
                }
            }
//what about the case where the TIL thing is quite small but it has to end before the action because it can't slide around so readily, i.e. no matter how far we shift an action to the left, this is always going to go inside it. i.e. it's timestamp which we can back-infer + min duration > timestamp + this til, we have an ordering.- think about this, it could be good....


        }



    }

    return true;

};


/*
bool FF::checkTSTemporalSoundness(RPGHeuristic* const rpg, ExtendedMinimalState & theState, const int & theAction, const VAL::time_spec & ts, const double & incr, const int oldTIL) {

    static const double EPSILON = 0.001;

    const bool localDebug = false;

    if (ts == VAL::E_AT_START) {

        list<StartEvent>::iterator tsiItr = theState.startEventQueue.begin();
        list<StartEvent>::iterator tsiEnd = theState.startEventQueue.end();

        --tsiEnd;

        for (; tsiItr != tsiEnd; ++tsiItr) {
            if ((tsiItr->elapsed = tsiItr->elapsed + incr) > tsiItr->maxDuration + 0.0005) {
                return false;
            }
            //cout << "Now " << tsiItr->second << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->first)) << "\n";
        }

        theState.timeStamp += EPSILON;


    } else if (ts == VAL::E_AT_END) {

        const double extraTime = incr;

        map<int, list<list<StartEvent>::iterator > >::iterator tsiOld = theState.entriesForAction.find(theAction);
        assert(tsiOld != theState.entriesForAction.end());

        if (localDebug) cout << theAction << " started " << tsiOld->second.front()->elapsed << " ago, of duration " << tsiOld->second.front()->maxDuration << "\n";


        if (localDebug) cout << "extra time = " << extraTime << "\n";

        if (extraTime < -0.0005) {
            return false;

        }

        list<StartEvent>::iterator & pairWith = tsiOld->second.front();


        list<StartEvent>::iterator tsiItr = theState.startEventQueue.begin();
        const list<StartEvent>::iterator tsiEnd = theState.startEventQueue.end();

        for (; tsiItr != pairWith; ++tsiItr) {
            if ((tsiItr->elapsed = tsiItr->elapsed + extraTime) > tsiItr->maxDuration + 0.0005) {

                return false;
            }
        }

        theState.timeStamp += extraTime;
        ++tsiItr;

        for (; tsiItr != tsiEnd; ++tsiItr) {
            if ((tsiItr->elapsed = tsiItr->elapsed + extraTime) > tsiItr->maxDuration + 0.0005) {

                return false;
            }
        }


    } else { // TIL
        for (int pass = oldTIL; pass <= theAction; ++pass) {
            list<StartEvent>::iterator tsiItr = theState.startEventQueue.begin();
            const list<StartEvent>::iterator tsiEnd = theState.startEventQueue.end();

            for (; tsiItr != tsiEnd; ++tsiItr) {
                if ((tsiItr->elapsed = tsiItr->elapsed + EPSILON) > tsiItr->maxDuration + 0.0005) {
                    return false;
                }
                //cout << "Now " << tsiItr->second << " since start of " << *(RPGBuilder::getInstantiatedOp(tsiItr->first)) << "\n";
            }
        }
        theState.timeStamp = RPGBuilder::getTILVec()[theAction]->duration;

    }

    return true;

};
*/
void reorderStartsBeforeEnds(list<ActionSegment > & applicableActions)
{

    if (false) {
        cout << "Before, ordered:\n";
        list<ActionSegment>::iterator postItr = applicableActions.begin();
        const list<ActionSegment>::iterator postEnd = applicableActions.end();

        for (; postItr != postEnd; ++postItr) {
            if (postItr->second != VAL::E_AT) {
                cout << "\t" << *(postItr->first);
                if (postItr->second == VAL::E_AT_START) {
                    cout << " start";
                } else {
                    cout << " end";
                }

                if (postItr->needToFinish.size()) {
                    cout << " ( with";
                    set<int>::iterator ntfItr = postItr->needToFinish.begin();
                    const set<int>::iterator ntfEnd = postItr->needToFinish.end();
                    for (; ntfItr != ntfEnd; ++ntfItr) {
                        cout << " " << *(RPGBuilder::getInstantiatedOp(*ntfItr));
                    }
                    cout << " before)";
                }
                cout << "\n";
            }
        }
    }


    list<ActionSegment > tmp(applicableActions);
    applicableActions.clear();

    list<ActionSegment >::iterator itr = tmp.begin();
    const list<ActionSegment >::iterator itrEnd = tmp.end();

    list<ActionSegment >::iterator firstEnd = applicableActions.end();

    const VAL::time_spec toMatch = (FF::startsBeforeEnds ? VAL::E_AT_START : VAL::E_AT_END);

    for (; itr != itrEnd; ++itr) {
        if (itr->second == toMatch) {
            if (firstEnd == applicableActions.end()) {
                applicableActions.push_back(*itr);
            } else {
                applicableActions.insert(firstEnd, *itr);
            }
        } else {
            applicableActions.push_back(*itr);
            if (firstEnd == applicableActions.end()) {
                --firstEnd;
            }
        }
    }

    if (false) {
        cout << "After, ordered:\n";
        list<ActionSegment>::iterator postItr = applicableActions.begin();
        const list<ActionSegment>::iterator postEnd = applicableActions.end();

        for (; postItr != postEnd; ++postItr) {
            if (postItr->second != VAL::E_AT) {
                cout << "\t" << *(postItr->first);
                if (postItr->second == VAL::E_AT_START) {
                    cout << " start";
                } else {
                    cout << " end";
                }

                if (postItr->needToFinish.size()) {
                    cout << " ( with";
                    set<int>::iterator ntfItr = postItr->needToFinish.begin();
                    const set<int>::iterator ntfEnd = postItr->needToFinish.end();
                    for (; ntfItr != ntfEnd; ++ntfItr) {
                        cout << " " << *(RPGBuilder::getInstantiatedOp(*ntfItr));
                    }
                    cout << " before)";
                }
                cout << "\n";
            }
        }
    }

};

void reorderNonDeletorsFirst(list<ActionSegment > & applicableActions)
{

    list<ActionSegment > tmp(applicableActions);
    applicableActions.clear();

    list<set<int> > preconditionFacts;
    
    {
        
        list<ActionSegment >::iterator itr = tmp.begin();
        const list<ActionSegment >::iterator itrEnd = tmp.end();
     
        for (; itr != itrEnd; ++itr) {
            preconditionFacts.push_back(set<int>());
            
            if (itr->second == VAL::E_AT) continue;
            
            set<int> & toFill = preconditionFacts.back();
            
            const list<Literal*> & pres = (itr->second == VAL::E_AT_START
                                           ? RPGBuilder::getProcessedStartPropositionalPreconditions()[itr->first->getID()]
                                           : RPGBuilder::getEndPropositionalPreconditions()[itr->first->getID()]);
                                           
            list<Literal*>::const_iterator pItr = pres.begin();
            const list<Literal*>::const_iterator pEnd = pres.end();
            
            for (; pItr != pEnd; ++pItr) {
                toFill.insert((*pItr)->getID());
            }
        }
    }
    
    list<int> penalties;
    
    list<set<int> >::const_iterator ownPFItr = preconditionFacts.begin();
    list<ActionSegment >::iterator itr = tmp.begin();
    const list<ActionSegment >::iterator itrEnd = tmp.end();

    for (; itr != itrEnd; ++itr, ++ownPFItr) {
        int penaltyScore = 0;
        if (itr->second != VAL::E_AT) {
            set<int> toFill;
            
            const list<Literal*> & effs = (itr->second == VAL::E_AT_START
                                            ? RPGBuilder::getStartPropositionDeletes()[itr->first->getID()]
                                            : RPGBuilder::getEndPropositionDeletes()[itr->first->getID()]);
                                            
            list<Literal*>::const_iterator eItr = effs.begin();
            const list<Literal*>::const_iterator eEnd = effs.end();

            for (; eItr != eEnd; ++eItr) {
                toFill.insert((*eItr)->getID());
            }
            
            list<set<int> >::const_iterator pfItr = preconditionFacts.begin();
            const list<set<int> >::const_iterator pfEnd = preconditionFacts.end();
            
            for (; pfItr != pfEnd; ++pfItr) {
                if (pfItr == ownPFItr) {
                    continue;
                }
                set<int> overlap;
                
                std::set_intersection(pfItr->begin(), pfItr->end(), toFill.begin(), toFill.end(),
                                      insert_iterator<set<int> >(overlap, overlap.begin()));
                                      
                if (!overlap.empty()) {
                    ++penaltyScore;
                }
            }
        }
        
        list<int>::iterator penItr = penalties.begin();
        list<ActionSegment>::iterator actItr = applicableActions.begin();
        
        for (; actItr != applicableActions.end() && penaltyScore >= *penItr; ++actItr, ++penItr) ;
        
        penalties.insert(penItr, penaltyScore);
        applicableActions.insert(actItr, *itr);
        
    }
    
    if (false) {
        cout << "After, ordered:\n";
        list<ActionSegment>::iterator postItr = applicableActions.begin();
        const list<ActionSegment>::iterator postEnd = applicableActions.end();

        for (; postItr != postEnd; ++postItr) {
            if (postItr->second != VAL::E_AT) {
                cout << "\t" << *(postItr->first);
                if (postItr->second == VAL::E_AT_START) {
                    cout << " start";
                } else {
                    cout << " end";
                }
                cout << "\n";
            }
        }
    }
};

void reorderHelpfulFirst(list<ActionSegment> & applicableActions, list<ActionSegment> & helpfulActions)
{

    list<ActionSegment> tmp(applicableActions);
    applicableActions.clear();

    list<int> weights;

    const list<ActionSegment>::iterator hitrStart = helpfulActions.begin();
    const list<ActionSegment>::iterator hitrEnd = helpfulActions.end();

    list<ActionSegment>::iterator itr = tmp.begin();
    const list<ActionSegment>::iterator itrEnd = tmp.end();

    for (; itr != itrEnd; ++itr) {
        int w = 0;
        for (list<ActionSegment>::iterator hitr = hitrStart; hitr != hitrEnd; ++hitr, ++w) {
            if (itr->first == hitr->first && itr->second == hitr->second && itr->divisionID == hitr->divisionID) {
                break;
            }
        }

        list<ActionSegment>::iterator aaItr = applicableActions.begin();
        const list<ActionSegment>::iterator aaEnd = applicableActions.end();
        list<int>::iterator wItr = weights.begin();

        for (; aaItr != aaEnd && (w >= *wItr); ++aaItr, ++wItr);

        applicableActions.insert(aaItr, *itr);
        weights.insert(wItr, w);

    }


};


void FF::makeJustApplied(map<double, list<pair<int, int> > > & justApplied, double & tilFrom, ExtendedMinimalState & state, const bool & lastIsSpecial)
{

    static const bool debugJA = false;
    static vector<double> tilStamps;
    static int tilCount;
    static bool haveTILstamps = false;
    static const double EPSILON = 0.001;

    if (!haveTILstamps) {
        haveTILstamps = true;
        vector<RPGBuilder::FakeTILAction*> & tilVec = RPGBuilder::getTILVec();
        tilCount = tilVec.size();
        tilStamps = vector<double>(tilCount);
        for (int i = 0; i < tilCount; ++i) {
            tilStamps[i] = tilVec[i]->duration;
        }
    }

    tilFrom = EPSILON;

    if (state.startEventQueue.empty() || !FF::invariantRPG || !FF::tsChecking) return;

    bool visitedTIL = (state.getInnerState().nextTIL >= tilCount);
    double tilAdvance = (visitedTIL ? DBL_MAX : tilStamps[state.getInnerState().nextTIL] - state.timeStamp);

    {
        list<StartEvent>::iterator seItr = state.startEventQueue.begin();
        const list<StartEvent>::iterator seEnd = state.startEventQueue.end();
        for (; seItr != seEnd; ++seItr) seItr->minAdvance = DBL_MAX;
    }

    list<StartEvent>::iterator seItr = state.startEventQueue.begin();
    list<StartEvent>::iterator seEnd = state.startEventQueue.end();

    if (lastIsSpecial) {
        --seEnd;
    }

    while (seItr != seEnd || !visitedTIL) {
        if (!visitedTIL && (seItr == seEnd || seItr->stepID >= state.stepBeforeTIL)) {

            const double refIncr = tilStamps[state.getInnerState().nextTIL] - state.timeStamp;
            double & inheritedIncr = tilAdvance;
            const double maxIncr = (refIncr < inheritedIncr ? refIncr : inheritedIncr);

            if (!state.tilComesBefore.empty()) {
                list<int>::iterator cbfItr = state.tilComesBefore.begin();
                const list<int>::iterator cbfEnd = state.tilComesBefore.end();

                list<StartEvent>::iterator seNext = seItr;

                for (; cbfItr != cbfEnd; ++cbfItr) {
                    int & deref = *cbfItr;
                    while (seNext != seEnd && seNext->stepID != deref) ++seNext;
                    if (seNext != seEnd) {
                        if (seNext->minAdvance > maxIncr) seNext->minAdvance = maxIncr;
                        ++seNext;
                    }
                }

            }

            {
                const double newElapsed = (state.getInnerState().nextTIL == 0 ? state.timeStamp : state.timeStamp - tilStamps[state.getInnerState().nextTIL - 1]) + maxIncr;
                const double equivalentDur = tilStamps[state.getInnerState().nextTIL] - (state.getInnerState().nextTIL == 0 ? 0.0 : tilStamps[state.getInnerState().nextTIL - 1]);
                if (newElapsed >= equivalentDur) {
//                  tilFrom = EPSILON;
                } else {
                    tilFrom = equivalentDur - newElapsed;
                }
            }

            visitedTIL = true;

        } else {
            if (!seItr->ignore) {
                const double refIncr = seItr->maxDuration - seItr->elapsed;
                double & inheritedIncr = seItr->minAdvance;
                const double maxIncr = (refIncr < inheritedIncr ? refIncr : inheritedIncr);

                if (!seItr->endComesBefore.empty()) {
                    set<int>::iterator cbfItr = seItr->endComesBefore.begin();
                    const set<int>::iterator cbfEnd = seItr->endComesBefore.end();

                    list<StartEvent>::iterator seNext = seItr;
                    ++seNext;

                    for (; cbfItr != cbfEnd; ++cbfItr) {
                        const int & deref = *cbfItr;
                        if (deref == -1) {
                            if (tilAdvance > maxIncr) tilAdvance = maxIncr;
                        } else {
                            while (seNext != seEnd && seNext->stepID != deref) ++seNext;
                            if (seNext != seEnd) {
                                if (seNext->minAdvance > maxIncr) seNext->minAdvance = maxIncr;
                                ++seNext;
                            }
                        }
                    }
                }

                {
                    const double newElapsed = seItr->elapsed + maxIncr;

///                 RPGBuilder::LinearEffects * const lEffs = RPGBuilder::getLinearDiscretisation()[seItr->actID];
                    double remainingDur = 0.0;
///                 if (lEffs && lEffs->divisions > 1) {
///                     for (int d = seItr->divisionsApplied + 1; d < lEffs->divisions; ++d) {
///                         remainingDur += lEffs->durations[d];
///                     }
///                 }



                    if (newElapsed >= seItr->minDuration) {
                        justApplied[remainingDur].push_back(pair<int, int>(seItr->actID, seItr->divisionsApplied));
                    } else {
                        const double insLayer = seItr->minDuration - newElapsed + remainingDur;
                        if (insLayer > EPSILON) {
                            justApplied[insLayer].push_back(pair<int, int>(seItr->actID, seItr->divisionsApplied));
                            if (debugJA) {
                                cout << insLayer << " left for step " << seItr->stepID << "\n";
                            }
                        }
                    }
                }
            }
            ++seItr;
        }
    }



    /*  if (state.nextTIL < tilCount) {
            const double tilIncr = tilStamps[state.nextTIL] - state.timeStamp;
            if (tilIncr < maxIncr) maxIncr = tilIncr;
        }


        double maxIncr = 0.0;

        list<StartEvent>::iterator seEnd = state.startEventQueue.end();

        {
            list<StartEvent>::iterator seItr = state.startEventQueue.begin();

            for (; seItr != seEnd; ++seItr) {
                if (!seItr->fanIn) {
                    const double thisIncr = seItr->maxDuration - seItr->elapsed;
                    if (maxIncr < thisIncr) maxIncr = thisIncr;
                }
            }
        }

        if (state.nextTIL < tilCount) {
            const double tilIncr = tilStamps[state.nextTIL] - state.timeStamp;
            if (tilIncr < maxIncr) maxIncr = tilIncr;
        }

        if (debugJA) cout << "Can advance by " << maxIncr << "\n";

        if (lastIsSpecial) --seEnd;

        {
            list<StartEvent>::iterator seItr = state.startEventQueue.begin();

            for (; seItr != seEnd; ++seItr) {
                const double newElapsed = seItr->elapsed + maxIncr;
                if (newElapsed >= seItr->minDuration) {
    //              justApplied[0.0].push_back(seItr->actID);
                } else {
                    justApplied[seItr->minDuration - newElapsed].push_back(seItr->actID);
                    if (debugJA) {
                        cout << seItr->minDuration - newElapsed << " left for step " << seItr->stepID << "\n";
                    }
                }
            }

        }*/

    if (lastIsSpecial) {

        const double newElapsed = seEnd->elapsed;

///     RPGBuilder::LinearEffects * const lEffs = RPGBuilder::getLinearDiscretisation()[seEnd->actID];
        double remainingDur = 0.0;
///     if (lEffs && lEffs->divisions > 1) {
///         for (int d = seEnd->divisionsApplied + 1; d < lEffs->divisions; ++d) {
///             remainingDur += lEffs->durations[d];
///         }
///     }

        if (newElapsed >= seEnd->minDuration) {
            justApplied[remainingDur].push_back(pair<int, int>(seEnd->actID, seEnd->divisionsApplied));
        } else {
            const double insLayer = seItr->minDuration - newElapsed + remainingDur;
            if (insLayer > EPSILON) {
                justApplied[insLayer].push_back(pair<int, int>(seEnd->actID, seEnd->divisionsApplied));
                if (debugJA) {
                    cout << insLayer << " left for step " << seEnd->stepID << " (the last one)\n";
                }
            }

        }
    }

};

class StatesToDelete
{

private:

    list<ExtendedMinimalState*> gc;

public:

    ~StatesToDelete() {
        list<ExtendedMinimalState*>::iterator cItr = gc.begin();
        const list<ExtendedMinimalState*>::iterator cEnd = gc.end();

        for (; cItr != cEnd; ++cItr) {
            delete *cItr;
        }
        gc.clear();
    }

    void alsoCleanUp(ExtendedMinimalState * const c) {
        gc.push_back(c);
    }

    void alsoCleanUp(SearchQueueItem * const s) {
        gc.push_back(s->releaseState());
    }


};

typedef map<ExtendedMinimalState*, double, SecondaryExtendedStateLessThan> InnerMap;
typedef map<ExtendedMinimalState*, pair<double, InnerMap>, WeakExtendedStateLessThan> OuterMap;

class StateHash : protected OuterMap
{

public:

    struct InsertIterator {

        pair<OuterMap::iterator, bool> outerInsertion;
        pair<InnerMap::iterator, bool> innerInsertion;

        InsertIterator() {
        }

        void setTimestampOfThisState(const ExtendedMinimalState * const e) {
            const double & t = e->timeStamp;

            double & previousTS = outerInsertion.first->second.first;
            if (previousTS > t) {
                previousTS = t;
            }
            innerInsertion.first->second = t;
        }
    };

    struct FindIterator {

        OuterMap::const_iterator outerFind;
        InnerMap::const_iterator innerFind;

        OuterMap::const_iterator outerEnd;
        InnerMap::const_iterator innerEnd;


        FindIterator(const OuterMap::const_iterator & itr, const OuterMap::const_iterator & itrEnd)
                : outerFind(itr), outerEnd(itrEnd) {
        }

    };

    StateHash() {
    }

    ~StateHash() {
    }

    InsertIterator insertState(SearchQueueItem * const s, StatesToDelete * const keptStates) {
        static InnerMap emptyInner;
        InsertIterator toReturn;

        toReturn.outerInsertion = insert(make_pair(s->state(), make_pair(s->state()->timeStamp, emptyInner)));
        toReturn.innerInsertion = toReturn.outerInsertion.first->second.second.insert(make_pair(s->state(), s->state()->timeStamp));


        if (toReturn.outerInsertion.second || toReturn.innerInsertion.second) {
            keptStates->alsoCleanUp(s);
#ifdef STATEHASHDEBUG
            s->mustNotDeleteState = true;
#endif
        }

        return toReturn;

    }

    InsertIterator insertState(ExtendedMinimalState * const e) {
        static InnerMap emptyInner;
        InsertIterator toReturn;

        toReturn.outerInsertion = insert(make_pair(e, make_pair(e->timeStamp, emptyInner)));

#ifdef STATEHASHDEBUG
        /*if (toReturn.outerInsertion.second) {
            cout << "State " << e << " is new in the outer set\n";
        }*/
#endif

        toReturn.innerInsertion = toReturn.outerInsertion.first->second.second.insert(make_pair(e, e->timeStamp));

#ifdef STATEHASHDEBUG
        /*if (toReturn.innerInsertion.second) {
            cout << "State " << e << " is new in the inner set\n";
        }*/
#endif

        return toReturn;
    }


    FindIterator findState(ExtendedMinimalState * const e) const {
        FindIterator toReturn(this->OuterMap::find(e), this->OuterMap::end());
        if (toReturn.outerFind != end()) {
            toReturn.innerFind = toReturn.outerFind->second.second.find(e);
            toReturn.innerEnd = toReturn.outerFind->second.second.end();
        }
        return toReturn;
    }

    void clear() {
        this->OuterMap::clear();
    }

};

bool FF::planMustSucceed = false;

list<FFEvent> * FF::doBenchmark(bool & reachedGoal, list<FFEvent> * oldSoln, const bool doLoops)
{

    static bool initCSBase = false;

    if (!initCSBase) {
        initCSBase = true;
        const vector<NumericAnalysis::dominance_constraint> & dcs = NumericAnalysis::getDominanceConstraints();
        const int pneCount = dcs.size();
        CSBase::ignorableFluents = vector<bool>(pneCount);
        for (int i = 0; i < pneCount; ++i) {
            CSBase::ignorableFluents[i] = (dcs[i] == NumericAnalysis::E_METRICTRACKING);
        }
    }

    set<int> goals;
    set<int> numericGoals;
    ExtendedMinimalState initialState;

    initialState.timeStamp = 0.0;

    {
        LiteralSet tinitialState;
        vector<double> tinitialFluents;

        RPGBuilder::getNonStaticInitialState(tinitialState, tinitialFluents);

        initialState.getEditableInnerState().setFacts(tinitialState);
        initialState.getEditableInnerState().setFacts(tinitialFluents);

    }

    {
        list<Literal*>::iterator gsItr = RPGBuilder::getLiteralGoals().begin();
        const list<Literal*>::iterator gsEnd = RPGBuilder::getLiteralGoals().end();

        for (; gsItr != gsEnd; ++gsItr) {
            pair<bool, bool> & currStatic = RPGBuilder::isStatic(*gsItr);
            if (currStatic.first) {
                if (!currStatic.second) {
                    cout << "Static goal " << *(*gsItr) << " resolves to false: no plan can solve this problem\n";
                    if (planMustSucceed && !oldSoln->empty()) {
                        exit(1);
                    }
                    reachedGoal = false;
                    return 0;
                }
            } else {
                goals.insert((*gsItr)->getID());
            }

        }

    }

    {
        list<pair<int, int> >::iterator gsItr = RPGBuilder::getNumericRPGGoals().begin();
        const list<pair<int, int> >::iterator gsEnd = RPGBuilder::getNumericRPGGoals().end();

        for (; gsItr != gsEnd; ++gsItr) {
            if (gsItr->first != -1) {
                numericGoals.insert(gsItr->first);
            }
            if (gsItr->second != -1) {
                numericGoals.insert(gsItr->second);
            }
        }
    }


    list<FFEvent>::iterator osItr = oldSoln->begin();
    const list<FFEvent>::iterator osEnd = oldSoln->end();

    SearchQueueItem * currSQI = new SearchQueueItem(&initialState, false);

    StatesToDelete * const keptStates = new StatesToDelete();

    StateHash visitedStates;
    visitedStates.insertState(&initialState);

    set<int> ignoreStep;

    if (!doLoops) {
        list<FFEvent> tEvent;
        HTrio h1(calculateHeuristicAndSchedule(*(currSQI->state()), 0, goals, numericGoals,
                                               (ParentData*) 0, currSQI->helpfulActions, currSQI->plan, tEvent, -1));
        currSQI->heuristicValue = h1;
        cout << "Heuristic value of initial state: ";
        if (h1.heuristicValue != -1.0) {
            cout << h1.heuristicValue << "\n";
        } else {
            cout << " dead end";
#ifndef NDEBUG
            if (h1.diagnosis) {
                cout << " - " << h1.diagnosis;
            }
#endif
            cout << "\n";
            if (planMustSucceed) {
                exit(1);
            }
        }
    }

    for (int i = 1; osItr != osEnd; ++osItr, ++i) {

        ActionSegment actID(osItr->action, osItr->time_spec, osItr->divisionID, RPGHeuristic::emptyIntList);

        {
            cout << "About to apply ";
            if (actID.second == VAL::E_AT_START) {
                cout << *(actID.first) << " start";
            } else if (actID.second == VAL::E_AT_END) {
                cout << *(actID.first) << " end";
            } else {
                cout << "TIL " << actID.divisionID;
            }
            cout << "\n";
        }

        if (ignoreStep.find(i - 1) != ignoreStep.end()) {
            cout << "\tIs the end of a skippable action\n";
            continue;
        }

        auto_ptr<ParentData> pd(LPScheduler::prime(currSQI->plan, currSQI->state()->getInnerState().temporalConstraints,
                                currSQI->state()->startEventQueue));

        SearchQueueItem * succ = new SearchQueueItem(applyActionToState(actID, *(currSQI->state())), true);

        if (!succ->state()) {
            cout << "Takes us to a null state: trying to start an action whose min duration exceeds its max\n";
        }

        if (!doLoops) {
            StateHash::InsertIterator insResult = visitedStates.insertState(succ, keptStates);

            if (insResult.outerInsertion.second) {
                cout << "Next action takes us to a new state\n";
                insResult.setTimestampOfThisState(succ->state());
            } else {
                cout << "*** Next action takes us to a memoised state";
                if (succ->state()->timeStamp < insResult.innerInsertion.first->second) {
                    cout << " but at a better timestamp\n";
                    insResult.setTimestampOfThisState(succ->state());
                } else {
                    cout << "\n";
                }
            }

            if (!FF::bestFirstSearch) {
                list<ActionSegment>::iterator asItr = currSQI->helpfulActions.begin();
                const list<ActionSegment>::iterator asEnd = currSQI->helpfulActions.end();

                for (; asItr != asEnd; ++asItr) {
                    if (asItr->first == actID.first && asItr->second == actID.second && asItr->divisionID == actID.divisionID) break;
                    assert(!(asItr->first == actID.first && asItr->second == actID.second && asItr->second != VAL::E_AT));
                }
                if (asItr == asEnd) {
                    cout << "--\nAction was not helpful when it should have been.  The helpful actions were:\n";
                    asItr = currSQI->helpfulActions.begin();

                    for (; asItr != asEnd; ++asItr) {
                        cout << "\t";
                        if (asItr->first) {
                            cout << *(asItr->first);
                            if (asItr->second == VAL::E_AT_END) {
                                cout << " end\n";
                            } else {
                                cout << " start\n";
                            }
                        } else {
                            cout << "TIL " << asItr->divisionID << "\n";
                        }
                    }
                    /*              list<ActionSegment> prevHelpful;
                                  list<FFEvent> tEvent;
                                  const int oldVerbosity = Globals::globalVerbosity;
                                  Globals::globalVerbosity = oldVerbosity & 64;
                                  HTrio h1(evaluateStateWRTSchedule(rpg, *(currSQI->state), 0, goals, numericGoals, (ParentData*) 0, prevHelpful, currSQI->plan, tEvent, -1));
                                  Globals::globalVerbosity = oldVerbosity;*/
                }
            }
        }

        FFEvent extraEvent;
        FFEvent extraEventTwo;

        FFEvent * reusedEndEvent = 0;

        bool eventOneDefined = false;
        bool eventTwoDefined = false;


        const bool rawDebug = false;

        map<double, list<pair<int, int> > > actualJustApplied;

        succ->plan = currSQI->plan;

        int stepID = -1;



        if (actID.second == VAL::E_AT_START) {
            if (rawDebug) cout << "RAW start\n";
            extraEvent = FFEvent(actID.first, succ->state()->startEventQueue.back().minDuration, succ->state()->startEventQueue.back().maxDuration);
            eventOneDefined = true;
            assert(extraEvent.time_spec == VAL::E_AT_START);
            if (!RPGBuilder::getRPGDEs(actID.first->getID()).empty()) { // if it's not a non-temporal action

                const int endStepID = succ->state()->getInnerState().planLength - 1;
                const int startStepID = endStepID - 1;
                extraEventTwo = FFEvent(actID.first, startStepID, succ->state()->startEventQueue.back().minDuration, succ->state()->startEventQueue.back().maxDuration);
                extraEvent.pairWithStep = endStepID;
                eventTwoDefined = true;

                if (!RPGBuilder::canSkipToEnd(actID.first->getID())) {
                    extraEventTwo.getEffects = false;
                } else {
                    ignoreStep.insert(osItr->pairWithStep);
                }

                stepID = startStepID;
            } else {
                stepID = succ->state()->getInnerState().planLength - 1;
            }

        } else if (actID.second == VAL::E_AT_END) {
            if (rawDebug) cout << "RAW end\n";
            map<int, list<list<StartEvent>::iterator > >::iterator tsiOld = succ->state()->entriesForAction.find(actID.first->getID());
            assert(tsiOld != succ->state()->entriesForAction.end());

            list<StartEvent>::iterator pairWith = tsiOld->second.front();
            tsiOld->second.pop_front();
            if (tsiOld->second.empty()) succ->state()->entriesForAction.erase(tsiOld);

            stepID = pairWith->stepID + 1;

            {
                list<FFEvent>::iterator pwItr = succ->plan.begin();
                for (int sID = 0; sID <= pairWith->stepID; ++sID, ++pwItr) ;
                pwItr->getEffects = true;
                reusedEndEvent = &(*pwItr);
            }

            succ->state()->startEventQueue.erase(pairWith);


        } else {
            extraEvent = FFEvent(actID.divisionID);
            eventOneDefined = true;
            stepID = succ->state()->getInnerState().planLength - 1;
        }

        FFcache_upToDate = false;

        list<FFEvent> nowList;
        if (eventOneDefined) nowList.push_back(extraEvent);
        if (eventTwoDefined) nowList.push_back(extraEventTwo);


        if (doLoops) cout << i << ",";

        tms before;
        times(&before);


        if (doLoops) {
            for (int rep = 0; rep < 99; ++rep) {
                if (rep) {
                    if (eventOneDefined) nowList.pop_back();
                    if (eventTwoDefined) nowList.pop_back();
                    if (eventOneDefined) nowList.push_back(extraEvent);
                    if (eventTwoDefined) nowList.push_back(extraEventTwo);
                }

                LPScheduler tryToSchedule(succ->state()->getInnerState(), succ->plan, nowList, i - 1, succ->state()->startEventQueue,
                                          pd.get(), succ->state()->entriesForAction, &(currSQI->state()->getInnerState().secondMin),
                                          &(currSQI->state()->getInnerState().secondMax),  &(succ->state()->tilComesBefore), false);

                tryToSchedule.updateStateFluents(succ->state()->getEditableInnerState().secondMin, succ->state()->getEditableInnerState().secondMax);
            }


            if (eventOneDefined) nowList.pop_back();
            if (eventTwoDefined) nowList.pop_back();
            if (eventOneDefined) nowList.push_back(extraEvent);
            if (eventTwoDefined) nowList.push_back(extraEventTwo);
            LPScheduler tryToSchedule(succ->state()->getInnerState(), succ->plan, nowList, i - 1, succ->state()->startEventQueue, pd.get(),
                                      succ->state()->entriesForAction, &(currSQI->state()->getInnerState().secondMin), &(currSQI->state()->getInnerState().secondMax),  &(succ->state()->tilComesBefore), false);

            //assert(tryToSchedule.isSolved());

            tryToSchedule.updateStateFluents(succ->state()->getEditableInnerState().secondMin, succ->state()->getEditableInnerState().secondMax);
        } else {
            HTrio h1(calculateHeuristicAndSchedule(*(succ->state()), currSQI->state(), goals, numericGoals, pd.get(),
                                                   succ->helpfulActions, succ->plan, nowList, stepID, true, 0, 0.001));

#ifndef NDEBUG
            if (h1.heuristicValue < 0.0) {
                cout << "*** ";
            }
            cout << "After step ";
            if (actID.second == VAL::E_AT_START) {
                cout << *(actID.first) << " start";
            } else if (actID.second == VAL::E_AT_END) {
                cout << *(actID.first) << " end";
            } else {
                cout << "TIL " << actID.divisionID;
            }
            if (h1.heuristicValue < 0.0) {
                cout << ", a dead end has been reached";
                if (h1.diagnosis) {
                    cout << ": " << h1.diagnosis;
                }
                cout << "\n";
                if (planMustSucceed) {
                    exit(1);
                }

            } else {
                cout << ", heuristic value is " << h1.heuristicValue << "\n";
            }

#endif

            succ->heuristicValue = h1;
        }

        tms refReturn;
        times(&refReturn);

        if (doLoops) {
            double secs = ((double)refReturn.tms_utime + (double)refReturn.tms_stime - (double)before.tms_utime - (double)before.tms_stime) / ((double) sysconf(_SC_CLK_TCK));

            int threedp = (int)(secs * 1000.0);
            int wholesecs = threedp / 1000;
            int centisecs = threedp % 1000;

            cout << wholesecs << ".";
            if (centisecs < 100) cout << "0";
            if (centisecs < 10) cout << "0";
            cout << centisecs << "\n";
        }

        if (eventTwoDefined) {
            extraEventTwo = nowList.back();
            nowList.pop_back();
        }

        if (eventOneDefined) {
            extraEvent = nowList.back();
        }

        if (eventOneDefined) {
            succ->plan.push_back(extraEvent);
            if (!doLoops) {
                cout << "Minimum timestamp of new step: " << extraEvent.lpMinTimestamp << "\n";
            }
        }

        if (eventTwoDefined) {
            succ->plan.push_back(extraEventTwo);
            if (!doLoops) {
                cout << "Minimum timestamp of its corresponding end: " << extraEventTwo.lpMinTimestamp << "\n";
            }
        }

        if (!doLoops && reusedEndEvent) {
            cout << "Minimum timestamp of reused end: " << reusedEndEvent->lpMinTimestamp << "\n";
        }

        if (actID.second == VAL::E_AT_START && !RPGBuilder::getRPGDEs(actID.first->getID()).empty() && RPGBuilder::canSkipToEnd(actID.first->getID())) {
            succ->state()->startEventQueue.pop_back();
        }


        currSQI = succ;
        if (rawDebug) cout << "Now done up to " << currSQI->plan.size() << " steps\n";

    }

    {
        cout << "digraph Plan {\n";
        cout << "\tnode [shape=rectangle, fontsize=14, color=black, fillcolor=white, fontcolor=black];\n";
        cout << "\tedge [style=solid, color=black];\n\n";

        list<FFEvent>::const_iterator printItr = currSQI->plan.begin();
        const list<FFEvent>::const_iterator printEnd = currSQI->plan.end();


        for (int i = 0; printItr != printEnd; ++printItr, ++i) {
            cout << "\tnode" << i << " [label=\"" << printItr->lpTimestamp << ": ";

            if (printItr->time_spec == VAL::E_AT_START) {
                cout << *(printItr->action);
                if (!RPGBuilder::getRPGDEs(printItr->action->getID()).empty()) {
                    cout << " start\"];\n";
                    assert(printItr->pairWithStep != -1);

                    cout << "\tnode" << i << " -> node" << printItr->pairWithStep << " [label=\"maxduration=" << printItr->maxDuration << "\"];\n";
                    cout << "\tnode" << printItr->pairWithStep << " -> node" << i << " [label=\"-minduration=" << -printItr->minDuration << "\"];\n";

                } else {
                    cout << "\"]\n";
                }
            } else if (printItr->time_spec == VAL::E_AT_END) {
                cout << *(printItr->action) << " end\"];\n";
            } else {
                cout << "TIL @ t=";
                cout << (printItr->lpMinTimestamp);
                cout << "\"];\n";
            }
            const int ignore = printItr->pairWithStep;


            const map<int, bool> * const stepsThatComeBeforeThisOne = currSQI->state()->getInnerState().temporalConstraints->stepsBefore(i);

            if (stepsThatComeBeforeThisOne) {
                map<int, bool>::const_iterator cbfItr = stepsThatComeBeforeThisOne->begin();
                const map<int, bool>::const_iterator cbfEnd = stepsThatComeBeforeThisOne->end();

                for (; cbfItr != cbfEnd; ++cbfItr) {
                    if (cbfItr->first == ignore) continue;
                    cout << "\tnode" << i << " -> node" << (cbfItr->first);
                    if (cbfItr->second) {
                        cout << " [label=\"-e\"];\n";
                    } else {
                        cout << " [label=\"0\"];\n";
                    }
                }
            }


        }
        cout << "}\n";
    }

    return new list<FFEvent>(currSQI->plan);
};

void registerFinished(ExtendedMinimalState & theState, set<int> & haveFinished)
{


    /*
    set<int>::iterator hfItr = haveFinished.begin();
    const set<int>::iterator hfEnd = haveFinished.end();

    for (; hfItr != hfEnd; ++hfItr) {

        map<int, pair<set<int>, set<int> > >::iterator fwfItr = theState.factsIfWeFinishActions.begin();
        const map<int, pair<set<int>, set<int> > >::iterator fwfEnd = theState.factsIfWeFinishActions.end();


        while (fwfItr != fwfEnd) {
            bool changed = false;
            {
                set<int>::iterator caItr = fwfItr->second.first.find(*hfItr);
                if (caItr != fwfItr->second.first.end()) {
    //                  cout << "Fact " << fwfItr->first << " now has no strings attached\n";
                    theState.first.insert(fwfItr->first);
                    fwfItr->second.first.erase(caItr);
                    changed = true;
                }
            }
            {
                set<int>::iterator caItr = fwfItr->second.second.find(*hfItr);
                if (caItr != fwfItr->second.second.end()) {
    //                  cout << "Deregistering invariant on fact " << fwfItr->first << "\n";
                    map<int,int>::iterator saItr = theState.invariants.find(fwfItr->first);
                    if (!(--(saItr->second))) {
                        theState.invariants.erase(saItr);
                    }
                    fwfItr->second.second.erase(caItr);
                    changed = true;
                }
            }
            if (changed && fwfItr->second.first.empty() && fwfItr->second.second.empty()) {
                const map<int, pair<set<int>, set<int> > >::iterator fwfDel = fwfItr++;
                theState.factsIfWeFinishActions.erase(fwfDel);
            } else {
                ++fwfItr;
            }

        }
    }
    */

}

pair<list<FFEvent>*, TemporalConstraints*> FF::search(bool & reachedGoal)
{

    static bool initCSBase = false;

    if (!initCSBase) {
        initCSBase = true;
        const vector<NumericAnalysis::dominance_constraint> & dcs = NumericAnalysis::getDominanceConstraints();
        const int pneCount = dcs.size();
        CSBase::ignorableFluents = vector<bool>(pneCount);
        for (int i = 0; i < pneCount; ++i) {
            CSBase::ignorableFluents[i] = (dcs[i] == NumericAnalysis::E_METRICTRACKING);
        }
    }

    const bool ffDebug = (Globals::globalVerbosity & 2);

    FFheader_upToDate = false;
    FFonly_one_successor = false;
    WAStar = false;
    set<int> goals;
    set<int> numericGoals;
    ExtendedMinimalState initialState;

    {
        LiteralSet tinitialState;
        vector<double> tinitialFluents;

        RPGBuilder::getNonStaticInitialState(tinitialState, tinitialFluents);

        initialState.getEditableInnerState().setFacts(tinitialState);
        initialState.getEditableInnerState().setFacts(tinitialFluents);

        if (ffDebug) {
            cout << "Initial state has " << initialState.getInnerState().first.size() << " propositional facts\n";
        }
    }


    {
        list<Literal*>::iterator gsItr = RPGBuilder::getLiteralGoals().begin();
        const list<Literal*>::iterator gsEnd = RPGBuilder::getLiteralGoals().end();

        for (; gsItr != gsEnd; ++gsItr) {
            pair<bool, bool> & currStatic = RPGBuilder::isStatic(*gsItr);
            if (currStatic.first) {
                if (!currStatic.second) {
                    cout << "Static goal " << *(*gsItr) << " resolves to false: no plan can solve this problem\n";
                    reachedGoal = false;
                    return pair<list<FFEvent>*, TemporalConstraints*>(0,0);
                }
            } else {
                goals.insert((*gsItr)->getID());
            }

        }

    }
    {
        list<pair<int, int> >::iterator gsItr = RPGBuilder::getNumericRPGGoals().begin();
        const list<pair<int, int> >::iterator gsEnd = RPGBuilder::getNumericRPGGoals().end();

        for (; gsItr != gsEnd; ++gsItr) {
            if (gsItr->first != -1) {
                numericGoals.insert(gsItr->first);
            }
            if (gsItr->second != -1) {
                numericGoals.insert(gsItr->second);
            }
        }
    }

    if (ffDebug) {
        cout << "Solving subproblem\n";
    }

    StateHash visitedStates;

    SearchQueue searchQueue;

    HTrio bestHeuristic;
    HTrio initialHeuristic;

    {
        SearchQueueItem * const initialSQI = new SearchQueueItem(&initialState, false);
        list<FFEvent> tEvent;
        FFheader_upToDate = false;
        FFonly_one_successor = true;
        bestHeuristic = calculateHeuristicAndSchedule(initialState, 0, goals, numericGoals, (ParentData*) 0, initialSQI->helpfulActions, initialSQI->plan, tEvent, -1);
        initialSQI->heuristicValue = bestHeuristic;
        initialHeuristic = bestHeuristic;
        searchQueue.push_back(initialSQI, 1);
    }

    auto_ptr<StatesToDelete> statesKept(new StatesToDelete());

    if (ffDebug) cout << "Initial heuristic = " << bestHeuristic.heuristicValue << "\n";

    if (bestHeuristic.heuristicValue == -1.0) {
        reachedGoal = false;
        return pair<list<FFEvent>*, TemporalConstraints*>(0,0);
    }

    if (bestHeuristic.heuristicValue == 0.0) {
        reachedGoal = true;
        return pair<list<FFEvent>*, TemporalConstraints*>(new list<FFEvent>(),0);
    }

    auto_ptr<list<FFEvent> > bestPlan(new list<FFEvent>());
    {

        ExtendedMinimalState * const toHash = new ExtendedMinimalState(initialState);

        toHash->timeStamp = 0.0;

        StateHash::InsertIterator itr = visitedStates.insertState(toHash);
        itr.setTimestampOfThisState(toHash);

        statesKept->alsoCleanUp(toHash);
    }

    if (ffDebug) {
        cout << "(" << bestHeuristic.heuristicValue << ") ";
        cout.flush();
    }

    if (skipEHC) searchQueue.pop_front();



    while (!searchQueue.empty()) {
        if (Globals::globalVerbosity & 2) cout << "\n--\n";
        auto_ptr<SearchQueueItem> currSQI(searchQueue.pop_front());

        bool foundBetter = false;



        list<ActionSegment > maybeApplicableActions;
        list<ActionSegment >::iterator helpfulActsItr;
        list<ActionSegment >::iterator helpfulActsEnd;

        if (!foundBetter) {
            if (helpfulActions) {
                //cout << "(( " << currSQI->helpfulActions.size() << "))";
                RPGBuilder::getHeuristic()->filterApplicableActions(currSQI->state()->getInnerState(), currSQI->state()->timeStamp, currSQI->helpfulActions);
                reorderStartsBeforeEnds(currSQI->helpfulActions);
                if (nonDeletorsFirst) {
                    reorderNonDeletorsFirst(currSQI->helpfulActions);
                }
                helpfulActsItr = currSQI->helpfulActions.begin();
                helpfulActsEnd = currSQI->helpfulActions.end();
                //cout << "(( " << currSQI->helpfulActions.size() << "))";
                //cout.flush();
                FFonly_one_successor = (currSQI->helpfulActions.size() == 1);
            } else {
                RPGBuilder::getHeuristic()->findApplicableActions(currSQI->state()->getInnerState(), currSQI->state()->timeStamp, maybeApplicableActions);
                reorderStartsBeforeEnds(maybeApplicableActions);
                if (nonDeletorsFirst) {
                    reorderNonDeletorsFirst(maybeApplicableActions);
                }
                helpfulActsItr = maybeApplicableActions.begin();
                helpfulActsEnd = maybeApplicableActions.end();
                //cout << "(( " << maybeApplicableActions.size() << "))";
                //cout.flush();
                FFonly_one_successor = (maybeApplicableActions.size() == 1);
            }
        } else {
            helpfulActsItr = helpfulActsEnd = currSQI->helpfulActions.end();
        }

        FFheader_upToDate = false;


        const auto_ptr<ParentData> incrementalData(LPScheduler::prime(currSQI->plan, currSQI->state()->getInnerState().temporalConstraints,
                currSQI->state()->startEventQueue));



        for (; helpfulActsItr != helpfulActsEnd; ++helpfulActsItr) {

            auto_ptr<SearchQueueItem> succ;
            bool tsSound = false;
            const int oldTIL = currSQI->state()->getInnerState().nextTIL;

            if (helpfulActsItr->second == VAL::E_AT) {
                set<int> needToFinish;// = RPGBuilder::TILneedsToHaveFinished(oldTIL, succ->state);
                // TODO: Does this need revisiting?
                // registerFinished(toSolve->rpg, succ->state, needToFinish);
                ActionSegment tempSeg(0, VAL::E_AT, oldTIL, RPGHeuristic::emptyIntList);
                succ = auto_ptr<SearchQueueItem>(new SearchQueueItem(applyActionToState(tempSeg, *(currSQI->state())), true));

                if (succ->state()) {
                    tsSound = checkTemporalSoundness(*(succ->state()), tempSeg, oldTIL);
                } else {
                    tsSound = false;
                }
            } else {
                //registerFinished(*(succ->state), helpfulActsItr->needToFinish);
                succ = auto_ptr<SearchQueueItem>(new SearchQueueItem(applyActionToState(*helpfulActsItr, *(currSQI->state())), true));
                if (succ->state()) {
                    tsSound = checkTemporalSoundness(*(succ->state()), *helpfulActsItr, oldTIL);
                } else {
                    tsSound = false;
                }

            }


            if (!tsSound) {
                if (Globals::globalVerbosity & 1) cout << "t"; cout.flush();
            } else {


                auto_ptr<SearchQueueItem> TILparentAutoDelete(0);
                SearchQueueItem * TILparent = 0;
                bool TILfailure = false;
                bool incrementalIsDead = false;


                if (helpfulActsItr->second == VAL::E_AT) {

                    bool visitTheState = false;

                    if (zealousEHC) {
                        const StateHash::FindIterator lookup = visitedStates.findState(succ->state());
                        visitTheState = (lookup.outerFind == lookup.outerEnd);
                    } else {
                        const StateHash::FindIterator lookup = visitedStates.findState(succ->state());
                        visitTheState = (lookup.outerFind == lookup.outerEnd);
                        if (!visitTheState) {
                            visitTheState = (lookup.innerFind == lookup.innerEnd);

                            if (!visitTheState) {
                                const double & previousTS = lookup.innerFind->second;

                                visitTheState = (fabs(succ->state()->timeStamp - previousTS) > 0.0005
                                                 && succ->state()->timeStamp < previousTS);
                            }
                        }
                    }

                    if (visitTheState) {

                        TILparent = currSQI.get();

                        for (int tn = oldTIL + 1; tn <= helpfulActsItr->divisionID; ++tn) {
                            set<int> needToFinish;// = RPGBuilder::TILneedsToHaveFinished(tn, succ->state);
                            //registerFinished(toSolve->rpg, succ->state, needToFinish);

                            ActionSegment tempSeg(0, VAL::E_AT, tn - 1, RPGHeuristic::emptyIntList);
                            FFcache_upToDate = false;

                            evaluateStateAndUpdatePlan(succ, *(succ->state()), TILparent->state(), goals, numericGoals,
                                                       (incrementalIsDead ? (ParentData*) 0 : incrementalData.get()) ,
                                                       succ->helpfulActions, tempSeg, TILparent->plan);

                            if (succ->heuristicValue.heuristicValue == -1.0) {
                                TILfailure = true;
                                break;
                            }
                            if (TILparent != currSQI.get()) {
                                delete TILparent;
                                TILparentAutoDelete.release();
                            }

                            TILparent = succ.release();
                            TILparentAutoDelete = auto_ptr<SearchQueueItem>(TILparent);


                            tempSeg = ActionSegment(0, VAL::E_AT, tn, RPGHeuristic::emptyIntList);

                            succ = auto_ptr<SearchQueueItem>(new SearchQueueItem(applyActionToState(tempSeg, *(TILparent->state())), true));

                            if (!succ->state() || !checkTemporalSoundness(*(succ->state()), tempSeg, tn - 1)) {
                                succ->heuristicValue.heuristicValue = -1.0;
                                TILfailure = true;
                                break;
                            }
                            incrementalIsDead = true;
                        }

                    }


                }

                StateHash::InsertIterator insResult;
                bool visitTheState = false;

                if (!TILfailure) {

                    ExtendedMinimalState * const hunting = succ->state();
                    insResult = visitedStates.insertState(succ.get(), statesKept.get());
                    visitTheState = (insResult.outerInsertion.second);

                    if (!zealousEHC && !visitTheState) {
                        visitTheState = (insResult.innerInsertion.second);

                        if (!visitTheState) {
                            const double & previousTS = insResult.innerInsertion.first->second;
                            visitTheState = (fabs(hunting->timeStamp - previousTS) > 0.0005
                                             && hunting->timeStamp < previousTS);
                        }

                    }
#ifdef STATEHASHDEBUG
                    if (visitTheState) {
                        succ->mustNotDeleteState = true;
                    }
#endif
                }


                if (!visitTheState) {
                    if (pruneMemoised) {
                        visitTheState = false;
                        if (Globals::globalVerbosity & 1) cout << "s";
                    }
                }

                if (visitTheState) {

                    if (helpfulActsItr->second == VAL::E_AT) {
                        evaluateStateAndUpdatePlan(succ, *(succ->state()), TILparent->state(), goals, numericGoals, (incrementalIsDead ? (ParentData*) 0 : incrementalData.get()), succ->helpfulActions, *helpfulActsItr, TILparent->plan);
                    } else {
                        evaluateStateAndUpdatePlan(succ,  *(succ->state()), currSQI->state(), goals, numericGoals, incrementalData.get(), succ->helpfulActions, *helpfulActsItr, currSQI->plan);
                    }

                    succ->printPlan();
                    if (succ->heuristicValue.heuristicValue != -1.0) {

                        if (succ->heuristicValue.heuristicValue == 0.0) {
                            if (Globals::globalVerbosity & 1) {
                                cout << "g"; cout.flush();
                            }
                            reachedGoal = true;
                            return make_pair(new list<FFEvent>(succ->plan), new TemporalConstraints(*(succ->state()->getInnerState().temporalConstraints)));
                        }

                        insResult.setTimestampOfThisState(succ->state());

                        if ((succ->heuristicValue.heuristicValue < bestHeuristic.heuristicValue)
                                || (FF::makespanTieBreak && (succ->heuristicValue.heuristicValue == bestHeuristic.heuristicValue)
                                    && (succ->heuristicValue.makespan < bestHeuristic.makespan))
                           ) {

                            bestHeuristic = succ->heuristicValue;
                            cout << "b (" << bestHeuristic.heuristicValue << " | " << bestHeuristic.makespan << ")" ; cout.flush();
                            succ->printPlan();
                            searchQueue.clear();
                            searchQueue.push_back(succ.release(), 1);
                            if (!FF::steepestDescent) {
                                foundBetter = true;
                                break;
                            }
                        } else {
                            if (Globals::globalVerbosity & 1) cout << "."; cout.flush();
                            searchQueue.push_back(succ.release(), 1);
                        }
                    } else {
                        if (Globals::globalVerbosity & 1) {
                            cout << "d";
                            cout.flush();
                        }
#ifndef NDEBUG
                        if (Globals::globalVerbosity & 2) {
                            cout << succ->heuristicValue.diagnosis << " ";
                            cout.flush();
                        }
#endif
                    }
                } else {
                    if (Globals::globalVerbosity & 1) cout << "p"; cout.flush();
                }
            }
        }

        FFonly_one_successor = false;
    }

    if (!bestFirstSearch) {
        cout << "\nProblem unsolvable by EHC\n";

        reachedGoal = false;
        return pair<list<FFEvent>*, TemporalConstraints*>(0,0);
    }

    cout << "\nResorting to best-first search\n";

    WAStar = true;
    visitedStates.clear();
    statesKept = auto_ptr<StatesToDelete>(new StatesToDelete());

    {

        if (FF::biasD) {
            initialHeuristic.qbreak = 1;
        } else if (FF::biasG) {
            initialHeuristic.qbreak = initialHeuristic.heuristicValue;
        } else {
            initialHeuristic.qbreak = 0;
        }


        SearchQueueItem * const initialSQI = new SearchQueueItem(&initialState, false);
        initialSQI->heuristicValue = initialHeuristic;
        bestHeuristic = initialHeuristic;
        searchQueue.insert(initialSQI, 1);
        //visitedStates.find(initialSQI->state)->second.second = true;

        {
            ExtendedMinimalState * const toHash = new ExtendedMinimalState(initialState);
            toHash->timeStamp = 0.0;
            StateHash::InsertIterator itr = visitedStates.insertState(toHash);
            itr.setTimestampOfThisState(toHash);
            statesKept->alsoCleanUp(toHash);
        }
    }

    while (!searchQueue.empty()) {

        auto_ptr<SearchQueueItem> currSQI(searchQueue.pop_front());

        if (Globals::globalVerbosity & 2) {
            cout << "\n--\n";
            cout << "Now visiting state with heuristic value of " << currSQI->heuristicValue.heuristicValue << " | " << currSQI->heuristicValue.makespan << "\n";
            printState(*(currSQI->state()));
            currSQI->printPlan();
            cout << "\n + \n";
        }

        bool foundBetter = false;

        //currSQI->printPlan();

        list<ActionSegment > applicableActions;

        if (!foundBetter) RPGBuilder::getHeuristic()->findApplicableActions(currSQI->state()->getInnerState(), currSQI->state()->timeStamp, applicableActions);

        reorderStartsBeforeEnds(applicableActions);

        reorderHelpfulFirst(applicableActions, currSQI->helpfulActions);

        if (nonDeletorsFirst) {
            reorderNonDeletorsFirst(applicableActions);
        }
        
        if (Globals::globalVerbosity & 2) {
            cout << "Applicable actions are:\n";
            list<ActionSegment >::iterator helpfulActsItr = applicableActions.begin();
            const list<ActionSegment >::iterator helpfulActsEnd = applicableActions.end();
            for (; helpfulActsItr != helpfulActsEnd; ++helpfulActsItr) {
                if (helpfulActsItr->second == VAL::E_AT) {
                    cout << "\t Timed initial literal " << helpfulActsItr->divisionID << "\n";
                } else {
                    cout << "\t " << *(helpfulActsItr->first) << ", ";
                    if (helpfulActsItr->second == VAL::E_AT_START) {
                        cout << "start\n";
                    } else if (helpfulActsItr->second == VAL::E_OVER_ALL) {
                        cout << "pt " << helpfulActsItr->divisionID << "\n";
                    } else {
                        cout << "end\n";
                    }
                }
            }
            cout << "\nHeuristic values are:\n";

        }

        FFheader_upToDate = false;
        FFonly_one_successor = (applicableActions.size() == 1);

        list<ActionSegment >::iterator helpfulActsItr = applicableActions.begin();
        const list<ActionSegment >::iterator helpfulActsEnd = applicableActions.end();

        const auto_ptr<ParentData> incrementalData(LPScheduler::prime(currSQI->plan, currSQI->state()->getInnerState().temporalConstraints,
                currSQI->state()->startEventQueue));

        for (; helpfulActsItr != helpfulActsEnd; ++helpfulActsItr) {
            auto_ptr<SearchQueueItem> succ;

            bool tsSound = false;
            const int oldTIL = currSQI->state()->getInnerState().nextTIL;

            list<ActionSegment> nowList;

            if (helpfulActsItr->second == VAL::E_AT) {

                ActionSegment tempSeg(0, VAL::E_AT, oldTIL, RPGHeuristic::emptyIntList);
                succ = auto_ptr<SearchQueueItem>(new SearchQueueItem(applyActionToState(tempSeg, *(currSQI->state())), true));

                if (!succ->state()) {
                    tsSound = false;
                } else {
                    tsSound = checkTemporalSoundness(*(succ->state()), tempSeg, oldTIL);
                }
            } else {

                //registerFinished(*(succ->state), helpfulActsItr->needToFinish);
                succ = auto_ptr<SearchQueueItem>(new SearchQueueItem(applyActionToState(*helpfulActsItr, *(currSQI->state())), true));

                if (!succ->state()) {
                    tsSound = false;
                } else {
                    tsSound = checkTemporalSoundness(*(succ->state()), *helpfulActsItr);
                }
            }

            /*cout << "Before:\n";
            printState(currSQI->state()->getInnerState());
            cout << "After:\n";
            if (succ->state()) {
                printState(succ->state()->getInnerState());
            } else {
                cout << "duration was invalid\n";
            }*/

            if (!tsSound) {
                if (Globals::globalVerbosity & 2) {
                    cout << "\tTemporally invalid choice\n";
                } else if (Globals::globalVerbosity & 1) {
                    cout << "t"; cout.flush();
                }
            } else {
                assert(succ->state());
                if (Globals::globalVerbosity & 2) {
                    if (helpfulActsItr->first) {
                        cout << "\tAfter applying " << *(helpfulActsItr->first);
                    } else {
                        cout << "\tAfter applying TIL";
                    }
                    cout << " state is:\n";
                    printState(*(succ->state()));
                }

                auto_ptr<SearchQueueItem> TILparentAutoDelete(0);
                SearchQueueItem * TILparent = 0;
                bool TILfailure = false;
                bool incrementalIsDead = false;

                if (helpfulActsItr->second == VAL::E_AT) {

                    int visitTheState = 0;

                    const StateHash::FindIterator lookup = visitedStates.findState(succ->state());

                    if (lookup.outerFind == lookup.outerEnd) {
                        visitTheState = 1;
                    } else {
                        if (lookup.innerFind == lookup.innerEnd) {
                            visitTheState = 2;
                        } else {
                            const double & previousTS = lookup.innerFind->second;
                            if (fabs(previousTS - succ->state()->timeStamp) > 0.0005 && (previousTS > succ->state()->timeStamp)) {
                                visitTheState = 2;
                            }
                        }
                    }

                    if (visitTheState) {

                        TILparent = currSQI.get();


                        for (int tn = oldTIL + 1; tn <= helpfulActsItr->divisionID; ++tn) {
                            ActionSegment tempSeg(0, VAL::E_AT, tn - 1, RPGHeuristic::emptyIntList);
                            FFcache_upToDate = false;

                            evaluateStateAndUpdatePlan(succ, *(succ->state()), TILparent->state(), goals, numericGoals,
                                                       (incrementalIsDead ? (ParentData*) 0 : incrementalData.get()),
                                                       succ->helpfulActions, tempSeg, TILparent->plan);

                            if (succ->heuristicValue.heuristicValue == -1.0) {
                                TILfailure = true;
                                break;
                            }
                            if (TILparent != currSQI.get()) {
                                delete TILparent;
                                TILparentAutoDelete.release();
                            }

                            TILparent = succ.release();
                            TILparentAutoDelete = auto_ptr<SearchQueueItem>(TILparent);

                            tempSeg = ActionSegment(0, VAL::E_AT, tn, RPGHeuristic::emptyIntList);
                            succ = auto_ptr<SearchQueueItem>(new SearchQueueItem(applyActionToState(tempSeg, *(TILparent->state())), true));
                            if (!succ->state() || !checkTemporalSoundness(*(succ->state()), tempSeg, tn - 1)) {
                                succ->heuristicValue.heuristicValue = -1.0;
                                TILfailure = true;
                                break;
                            }
                            incrementalIsDead = true;
                        }
                        FFcache_upToDate = false;
                    }
                }

                int visitTheState = 0;

                StateHash::InsertIterator insResult;

                if (!TILfailure) {

                    insResult = visitedStates.insertState(succ.get(), statesKept.get());

                    if (insResult.outerInsertion.second) {
                        visitTheState = 1;
                    } else {
                        if (insResult.innerInsertion.second) {
                            visitTheState = 2;
                        } else {
                            const double & previousTS = insResult.innerInsertion.first->second;
                            if (fabs(previousTS - succ->state()->timeStamp) > 0.0005 && (previousTS > succ->state()->timeStamp)) {
                                visitTheState = 2;
                            }
                        }
                    }

#ifdef STATEHASHDEBUG
                    if (insResult.outerInsertion.second || insResult.innerInsertion.second) {
                        succ->mustNotDeleteState = true;
                    }
#endif

                    if (visitTheState == 0) {
                        if (Globals::globalVerbosity & 2) {
                            cout << "\tState visited before\n";
                        } else if (Globals::globalVerbosity & 1) {
                            cout << "s";
                        }
                    }
                }

                if (visitTheState) {

                    if (helpfulActsItr->second == VAL::E_AT) {
                        evaluateStateAndUpdatePlan(succ, *(succ->state()), TILparent->state(), goals, numericGoals,
                                                   (incrementalIsDead ? (ParentData*) 0 : incrementalData.get()),
                                                   succ->helpfulActions, *helpfulActsItr, TILparent->plan);

                    } else {

                        evaluateStateAndUpdatePlan(succ, *(succ->state()), currSQI->state(), goals, numericGoals,
                                                   incrementalData.get(), succ->helpfulActions, *helpfulActsItr, currSQI->plan);
                    }

                    //succ->printPlan();
                    if (succ->heuristicValue.heuristicValue != -1.0) {


                        if (succ->heuristicValue.heuristicValue == 0.0) {
                            cout << "g"; cout.flush();
                            reachedGoal = true;
                            return make_pair(new list<FFEvent>(succ->plan), new TemporalConstraints(*(succ->state()->getInnerState().temporalConstraints)));
                        }
                        insResult.setTimestampOfThisState(succ->state());

                        if (succ->heuristicValue.heuristicValue < bestHeuristic.heuristicValue || (FF::makespanTieBreak && (succ->heuristicValue.heuristicValue == bestHeuristic.heuristicValue && succ->heuristicValue.makespan < bestHeuristic.makespan))) {

                            bestHeuristic = succ->heuristicValue;
                            if (Globals::globalVerbosity & 2) {
                                cout << "\t" << bestHeuristic.heuristicValue << " | " << bestHeuristic.makespan << ", category " << visitTheState << " - a new best heuristic value, with plan:\n";
                                succ->printPlan();
                            } else {
                                cout << "b (" << bestHeuristic.heuristicValue << " | " << bestHeuristic.makespan << ")" ; cout.flush();
                            }

                            searchQueue.insert(succ.release(), visitTheState);
                        } else {
                            if (Globals::globalVerbosity & 2) {
                                cout << "\t" << succ->heuristicValue.heuristicValue << " | " << succ->heuristicValue.makespan << ", category " << visitTheState << "\n";
                            } else if (Globals::globalVerbosity & 1) {
                                cout << "."; cout.flush();
                            }
                            searchQueue.insert(succ.release(), visitTheState);
                        }

                    } else {
                        if (Globals::globalVerbosity & 1) {
                            cout << "d"; cout.flush();
                        }
#ifndef NDEBUG
                        if (Globals::globalVerbosity & 2) {
                            cout << succ->heuristicValue.diagnosis << " ";
                            cout.flush();
                        }
#endif
                    }
                } else {
                    if (Globals::globalVerbosity & 1 && !(Globals::globalVerbosity & 2)) {
                        cout << "p"; cout.flush();
                    }
                }
            }
        }

    }

    visitedStates.clear();

    cout << "\nProblem Unsolvable\n";

    reachedGoal = false;
    return pair<list<FFEvent>*, TemporalConstraints*>(0,0);

};

list<FFEvent> * FF::reprocessPlan(list<FFEvent> * oldSoln, TemporalConstraints * cons)
{
    static bool initCSBase = false;

    if (!initCSBase) {
        initCSBase = true;
        const vector<NumericAnalysis::dominance_constraint> & dcs = NumericAnalysis::getDominanceConstraints();
        const int pneCount = dcs.size();
        CSBase::ignorableFluents = vector<bool>(pneCount);
        for (int i = 0; i < pneCount; ++i) {
            CSBase::ignorableFluents[i] = (dcs[i] == NumericAnalysis::E_METRICTRACKING);
        }
    }

    const bool ffDebug = (Globals::globalVerbosity & 2);

    FFheader_upToDate = false;
    FFonly_one_successor = false;
    WAStar = false;
    set<int> goals;
    set<int> numericGoals;
    ExtendedMinimalState initialState;

    {
        LiteralSet tinitialState;
        vector<double> tinitialFluents;

        RPGBuilder::getNonStaticInitialState(tinitialState, tinitialFluents);

        initialState.getEditableInnerState().setFacts(tinitialState);
        initialState.getEditableInnerState().setFacts(tinitialFluents);

        if (ffDebug) {
            cout << "Initial state has " << initialState.getInnerState().first.size() << " propositional facts\n";
        }
    }


    {
        list<Literal*>::iterator gsItr = RPGBuilder::getLiteralGoals().begin();
        const list<Literal*>::iterator gsEnd = RPGBuilder::getLiteralGoals().end();

        for (; gsItr != gsEnd; ++gsItr) {
            pair<bool, bool> & currStatic = RPGBuilder::isStatic(*gsItr);
            if (currStatic.first) {
                assert(currStatic.second);
            } else {
                goals.insert((*gsItr)->getID());
            }

        }

    }
    {
        list<pair<int, int> >::iterator gsItr = RPGBuilder::getNumericRPGGoals().begin();
        const list<pair<int, int> >::iterator gsEnd = RPGBuilder::getNumericRPGGoals().end();

        for (; gsItr != gsEnd; ++gsItr) {
            if (gsItr->first != -1) {
                numericGoals.insert(gsItr->first);
            }
            if (gsItr->second != -1) {
                numericGoals.insert(gsItr->second);
            }
        }
    }

    list<FFEvent*> sortedSoln;

    if (cons == 0) {

        list<FFEvent>::iterator osItr = oldSoln->begin();
        const list<FFEvent>::iterator osEnd = oldSoln->end();
        
        for (; osItr != osEnd; ++osItr) {
            if ((osItr->time_spec == VAL::E_AT_END) && RPGBuilder::canSkipToEnd(osItr->action->getID())) {
                continue;
            }
                    
            list<FFEvent*>::iterator sortedItr = sortedSoln.begin();
            const list<FFEvent*>::iterator sortedEnd = sortedSoln.end();
            
            for (; sortedItr != sortedEnd; ++sortedItr) {
                if (osItr->lpTimestamp < (*sortedItr)->lpTimestamp) {
                    sortedSoln.insert(sortedItr, &(*osItr));
                    break;
                }
            }
            if (sortedItr == sortedEnd) {
                sortedSoln.push_back(&(*osItr));
            }
        }
    } else {
        const int evCount = oldSoln->size();
        vector<set<int> > fanOut(evCount);
        vector<int> fanIn(evCount,0);
        vector<FFEvent*> eventForStep(evCount);
        
        list<int> openList;
        
        {
            list<FFEvent>::iterator osItr = oldSoln->begin();
            const list<FFEvent>::iterator osEnd = oldSoln->end();
            
            for (int ev = 0; osItr != osEnd; ++osItr, ++ev) {
                
                eventForStep[ev] = &(*osItr);
                
                const map<int,bool> * stepsBeforeThisOne = cons->stepsBefore(ev);
                
                if (stepsBeforeThisOne) {
                    map<int,bool>::const_iterator itr = stepsBeforeThisOne->begin();
                    const map<int,bool>::const_iterator itrEnd = stepsBeforeThisOne->end();
                    
                    for (; itr != itrEnd; ++itr) {
                        if (fanOut[itr->first].insert(ev).second) {
                            ++fanIn[ev];
                        }
                    }
                }

                if (osItr->time_spec == VAL::E_AT) {
                    list<FFEvent>::iterator ostItr = oldSoln->begin();
                    const list<FFEvent>::iterator ostEnd = oldSoln->end();
                    
                    for (int ost = 0; ostItr != ostEnd; ++ostItr, ++ost) {
                        if (osItr == ostItr) continue;
                        
                        if (ostItr->lpTimestamp < osItr->lpTimestamp - 0.0001) {
                            if (fanOut[ost].insert(ev).second) {
                                ++fanIn[ev];
                            }
                        } else if (osItr->lpTimestamp + 0.0001 < ostItr->lpTimestamp) {
                            if (fanOut[ev].insert(ost).second) {
                                ++fanIn[ost];
                            }
                        }
                    }
                }
            }
        }
        for (int ev = 0; ev < evCount; ++ev) {
            if (!(fanIn[ev])) {
                openList.push_back(ev);
            }
        }
        
        for (; !openList.empty(); openList.pop_front()) {
            
            const int & currStep = openList.front();         
            
            if ((eventForStep[currStep]->time_spec != VAL::E_AT_END) || !RPGBuilder::canSkipToEnd(eventForStep[currStep]->action->getID())) {            
                sortedSoln.push_back(eventForStep[currStep]);
            }
            
            set<int>::const_iterator foItr = fanOut[currStep].begin();
            const set<int>::const_iterator foEnd = fanOut[currStep].end();
            
            for (; foItr != foEnd; ++foItr) {
                if (!(--(fanIn[*foItr]))) {
                    openList.push_back(*foItr);
                }
            }
            
        }
    }
    
    if (Globals::globalVerbosity & 2) {
        list<FFEvent*>::const_iterator oldSolnItr = sortedSoln.begin();
        const list<FFEvent*>::const_iterator oldSolnEnd = sortedSoln.end();
        
        for (int stepID = 0; oldSolnItr != oldSolnEnd; ++oldSolnItr, ++stepID) {

            cout << "Sorted plan step " << stepID << ": ";
            if ((*oldSolnItr)->time_spec == VAL::E_AT) {
                cout << "TIL " << (*oldSolnItr)->divisionID << endl;
            } else {
                if ((*oldSolnItr)->time_spec == VAL::E_AT_START) {
                    cout << "start of ";
                } else {
                    cout << "end of ";
                }
                cout << *((*oldSolnItr)->action) << ", was at " << (*oldSolnItr)->lpTimestamp << endl;
            }
        }
            
    }
    
    StateHash visitedStates;    

    SearchQueueItem * currSQI = new SearchQueueItem(&initialState, false);
    {
        list<FFEvent> tEvent;
        FFheader_upToDate = false;
        FFonly_one_successor = true;
        calculateHeuristicAndSchedule(initialState, 0, goals, numericGoals, (ParentData*) 0, currSQI->helpfulActions, currSQI->plan, tEvent, -1);        
    }

    auto_ptr<StatesToDelete> statesKept(new StatesToDelete());

    list<FFEvent*>::const_iterator oldSolnItr = sortedSoln.begin();
    const list<FFEvent*>::const_iterator oldSolnEnd = sortedSoln.end();
    
    const int lastStep = sortedSoln.size() - 1;
    
    for (int stepID = 0; oldSolnItr != oldSolnEnd; ++oldSolnItr, ++stepID) {

        ActionSegment nextSeg((*oldSolnItr)->action, (*oldSolnItr)->time_spec, (*oldSolnItr)->divisionID, RPGHeuristic::emptyIntList);
                
        if (Globals::globalVerbosity & 2) {
            cout << "[" << stepID << "] ";
            cout.flush();
        }
        
        if (stepID == lastStep) {
            scheduleToMetric = true;
        }
        
        const auto_ptr<ParentData> incrementalData(LPScheduler::prime(currSQI->plan, currSQI->state()->getInnerState().temporalConstraints,
                currSQI->state()->startEventQueue));

                
        auto_ptr<SearchQueueItem> succ = auto_ptr<SearchQueueItem>(new SearchQueueItem(applyActionToState(nextSeg, *(currSQI->state())), true));
        
        evaluateStateAndUpdatePlan(succ,  *(succ->state()), currSQI->state(), goals, numericGoals, incrementalData.get(), succ->helpfulActions, nextSeg, currSQI->plan);

        delete currSQI;
        currSQI = succ.release();

    }
    list<FFEvent> * const toReturn = new list<FFEvent>(currSQI->plan);
    
    delete currSQI;
    return toReturn;
}


};
