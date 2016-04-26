/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef TEMPORALCONSTRAINTS_H
#define TEMPORALCONSTRAINTS_H

#include <vector>
#include <set>
#include <map>
#include <cassert>

using std::vector;
using std::set;
using std::map;
using std::pair;

#ifdef POPF3ANALYSIS
#define EXTENDEDFLUENTANNOTATION
#endif

namespace Planner
{

#ifndef NDEBUG

extern pair<map<int, int>::iterator, bool> INVARIANTINSERT(map<int, int> & dest, const pair<int, int> & toInsert, const int & bound);
extern void INVARIANTERASE(map<int, int> & dest, const int & toErase, const int & bound);

#else

#define INVARIANTINSERT(x,y,z) x.insert(y)
#define INVARIANTERASE(x,y,z) x.erase(y)

#endif


/** @brief A record of the most recent and current interactions between a specific numeric variable, and the steps in the plan.
 */
struct FluentInteraction {

    /** @brief The plan step index of the last action to have a discrete numeric effect on this variable.*/
    int lastInstantaneousEffect;

    /** @brief Indices of plan steps with a precondition on this variable since the last effect. */
    set<int> stepsWithPreconditionsSinceTheLastEffect;
    
    /** @brief Ends of actions on the start event queue with an effect/precondition on this variable. */
    set<int> futureEndInteractions;
    
    /** @brief Compression-safe end effects on numeric variables.
     *
     *  When end numeric effects are compression safe, we queue them here so if a precondition can
     *  be satisfied by coming after just some of them, we do so.  This avoids blowing up the plan
     *  makespans by forcing all interaction to be sequential.
     * 
     *  The keys of the map denote timestamps.  The associated map pairs step IDs with the associated
     *  constant-valued increase/decrease effect gained by that step.
     */    
    map<double, map<int,double> > compressionSafeEndEffectOfStepsAtTime;
    
    /** @brief A set of plan steps, each the start of an action with a continuous numeric effect on this variable.*/
    set<int> activeCTSEffects;

    /** @brief A record of which steps have invariants acting on this variable.  The keys of the map denote the end steps of actions; the values the corresponding start step. */
    map<int, int> activeInvariants;    // end step maps to start step

    /** @brief Default constructor.  <code>lastInstantaneousEffect</code> is set to -1, i.e. no step has yet affected this variable.*/
    FluentInteraction() : lastInstantaneousEffect(-1) {
    }
};

class TemporalConstraints
{

protected:
    /**
     *  The vector of temporal constraints, for each step in the plan.  Each entry <code>i</code> in the vector
     *  consists of a map, the keys of which denote the steps that must precede <code>i</code>, and
     *  the corresponding keys, if <code>true</code> denote that separation of at least epsilon is needed.
     */
    vector<map<int, bool> * > stepsComeBeforeThisOne;

    /**
     *  The index of the step most recently added to the plan.  Used by TotalOrderTransformer
     *  to impose a total ordering.
     *  @see TotalOrderTransformer
     */
    int mostRecentStep;

public:
    #ifndef TOTALORDERSTATES
    /**
     *   A vector, with one entry per task numeric variable, recording the recent and current
     *  interactions between the steps in the plan and that variable.  @see FluentInteraction
     */
    vector<FluentInteraction> lastStepToTouchPNE;
    #endif
    
    /** Default constructor, initialising the temporal constraints to empty, setting
     *  <code>mostRecentStep</code> to -1, and invoking the default constructor
     *  for each entry in <code>lastStepToTouchPNE</code> to record that no step
     *  in the (currently empty) plan has yet touched a variable.
     */
    TemporalConstraints();

    /**
     *  Copy constructor, optionally extending the size of the constraints vector to reserve
     *  space for constraints on new steps.
     *
     *  @param c         The temporal constraints to copy.
     *  @param extendBy  The number of additional steps to allow space for in the constraints vector.
     */
    TemporalConstraints(const TemporalConstraints & c, const int extendBy = 0);

    virtual ~TemporalConstraints();

    /**
     *  Add an ordering between two snap-actions, optionally including an epsilon separation.
     *  @param comesFirst  Index of the step that must come before <code>comesSecond</code>
     *  @param comesSecond Index of the step that must come after <code>comesFirst</code>
     *  @param epsilon     If <code>true</code>, comesSecond must be strictly after comesFirst
     *                     separated by a gap of 'epsilon' (a small number).
     */
    virtual void addOrdering(const unsigned int & comesFirst, const unsigned int & comesSecond, const bool & epsilon);

    /**
     *  Extend the vector of temporal constraints to include entries for additional plan steps.
     *  @param extendBy  The number of new (initially empty) entries to allow space for.
     */
    virtual void extend(const int & extendBy);


    /**
     *  Accessor function to return the steps that must come before a given step.
     *
     *  @param i   The step for which the predecessors are desired
     *  @return    A map, the keys of which denote the steps which must precede <code>i</code>, and
     *             the corresponding keys, if <code>true</code>, denote that separation of at least epsilon is needed.
     */
    const map<int, bool> * stepsBefore(const int & i) const {
        return stepsComeBeforeThisOne[i];
    }

    /**  Returns the current size of the vector of temporal constraints. */
    const unsigned int size() const {
        return stepsComeBeforeThisOne.size();
    }

    /**
     * Returns the value of <code>mostRecentStep</code>, the index of the step most recently added to the plan.
     *
     * @return  The value of <code>mostRecentStep</code>
     */
    const int & getMostRecentStep() const {
        return mostRecentStep;
    }

    /** Updates the value of <code>mostRecentStep</code>, the index of the step most recently added to the plan.
     *
     *  @param last  The new value to give <code>mostRecentStep</code>
     */
    void setMostRecentStep(const int & last) {
        mostRecentStep = last;
    }
};

};

#endif // TEMPORALCONSTRAINTS_H
