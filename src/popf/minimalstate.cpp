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

#include "minimalstate.h"
#include "temporalconstraints.h"

#include "algorithm"

using std::transform;

namespace Planner
{

StateTransformer* MinimalState::globalTransformer = 0;

MinimalState::MinimalState(const map<int, PropositionAnnotation> & f, const vector<double> & sMin, const vector<double> & sMax, const map<int, set<int> > & sa, const int nt, const int unsigned pl, const unsigned int ae)
        : first(f), secondMin(sMin), secondMax(sMax), startedActions(sa),
        planLength(pl), actionsExecuting(ae), nextTIL(nt), temporalConstraints(globalTransformer->emptyTemporalConstraints())
{
}

MinimalState::MinimalState(const set<int> & f, const vector<double> & sMin, const vector<double> & sMax, const map<int, set<int> > & sa, const int nt, const unsigned int pl, const unsigned int ae)
        : secondMin(sMin), secondMax(sMax), startedActions(sa),
        planLength(pl), actionsExecuting(ae), nextTIL(nt), temporalConstraints(globalTransformer->emptyTemporalConstraints())
{
    setFacts(f);
}

void MinimalState::setFacts(const set<int> & f)
{
    insertIntFacts(f.begin(), f.end(), StepAndBeforeOrAfter());
}

void MinimalState::setFacts(const LiteralSet & f)
{
    insertFacts(f.begin(), f.end(), StepAndBeforeOrAfter());
}


void MinimalState::setFacts(const vector<double> & f)
{
    secondMin = f;
    secondMax = f;
}


MinimalState::MinimalState(const MinimalState & other, const int extendBy)
        : first(other.first), retired(other.retired), secondMin(other.secondMin), secondMax(other.secondMax), startedActions(other.startedActions),
        planLength(other.planLength), actionsExecuting(other.actionsExecuting), nextTIL(other.nextTIL), temporalConstraints(globalTransformer->cloneTemporalConstraints(other.temporalConstraints, extendBy))
{
}

MinimalState::MinimalState()
        : planLength(0), actionsExecuting(0), nextTIL(0), temporalConstraints(globalTransformer->emptyTemporalConstraints())
{
}

MinimalState::~MinimalState()
{
    delete temporalConstraints;
}

MinimalState & MinimalState::operator =(const MinimalState & other)
{
    first = other.first;
    retired = other.retired;
    secondMin = other.secondMin;
    secondMax = other.secondMax;
    startedActions = other.startedActions;
    planLength = other.planLength;
    actionsExecuting = other.actionsExecuting;
    nextTIL = other.nextTIL;
    delete temporalConstraints;
    temporalConstraints = globalTransformer->cloneTemporalConstraints(other.temporalConstraints);
    return *this;
}

bool StrongStateEquality::operator()(const MinimalState & a, const MinimalState & b)
{
    return (a.first == b.first && a.secondMin == b.secondMin && a.secondMax == b.secondMax && a.startedActions == b.startedActions && a.nextTIL == b.nextTIL);
}

bool WeakStateEquality::operator()(const MinimalState & a, const MinimalState & b)
{
    return (a.first == b.first && a.secondMin == b.secondMin && a.secondMax == b.secondMax && a.startedActions == b.startedActions && a.nextTIL == b.nextTIL);
}

void MinimalState::printState(ostream & cout) const
{

    cout << "Literals:";
    {
        map<int, PropositionAnnotation>::const_iterator itr = first.begin();
        const map<int, PropositionAnnotation>::const_iterator itrEnd = first.end();

        for (; itr != itrEnd; ++itr) {
            cout << " " << itr->first;
        }
    }

    cout << "\nStarted actions:";
    {
        map<int, set<int> >::const_iterator itr = startedActions.begin();
        const map<int, set<int> >::const_iterator itrEnd = startedActions.end();

        for (; itr != itrEnd; ++itr) {
            cout << " " << itr->first << " with ends recorded at steps:";
            set<int>::const_iterator iItr = itr->second.begin();
            const set<int>::const_iterator iEnd = itr->second.end();

            for (; iItr != iEnd; ++iItr) {
                cout << " " << *iItr;
            }
            cout << "\n";
        }
    }

    cout << "\nNext TIL: " << nextTIL;

    cout << "\n";

}

ostream & operator <<(ostream & o, const MinimalState & s)
{
    s.printState(o);
    return o;
}

ostream & operator <<(ostream & o, const StepAndBeforeOrAfter & s)
{
    s.write(o);
    return o;
}

};
