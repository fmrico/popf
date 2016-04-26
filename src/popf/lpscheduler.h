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

#ifndef LPSCHEDULER
#define LPSCHEDULER

#include "FFSolver.h"

#include <map>
#include <list>
#include <set>
#include <vector>
#include <cstring>

using std::map;
using std::list;
using std::set;
using std::vector;

class MILPSolver;

namespace Planner
{
    
class ParentData;
class ChildData;

class LPScheduler
{

public:
    static bool hybridBFLP;
    static bool optimiseOrdering;
    
protected:
    int tsVarCount;
    ChildData * cd;

    struct ConstraintPtrLT;

    class Constraint
    {

    public:
        vector<double> weights;
        vector<int> variables;

        double lower;
        double upper;

        Constraint() : lower(0.0), upper(0.0) {
        };

        static const Constraint * requestConstraint(const Constraint & c) {
            const pair<set<Constraint>::iterator, bool> insResult = constraintStore.insert(c);
            if (insResult.second) {
                insResult.first->ID = constraintCount++;
            }
            return &(*(insResult.first));
        }

        bool operator <(const Constraint & c) const {
            const unsigned int thisVC = weights.size();
            const unsigned int otherVC = c.weights.size();

            if (thisVC < otherVC) return true;
            if (thisVC > otherVC) return false;

            if (fabs(lower - c.lower) > 0.0000001) {
                if (lower < c.lower) return true;
                if (lower > c.lower) return false;
            }

            if (fabs(upper - c.upper) > 0.0000001) {
                if (upper < c.upper) return true;
                if (upper > c.upper) return false;
            }


            for (unsigned int i = 0; i < thisVC; ++i) {
                if (variables[i] < c.variables[i]) return true;
                if (variables[i] > c.variables[i]) return false;

                if (fabs(weights[i] - c.weights[i]) > 0.0000001) {
                    if (weights[i] < c.weights[i]) return true;
                    if (weights[i] > c.weights[i]) return false;
                }
            }

            return false;
        }

    protected:

        static set<Constraint> constraintStore;
        static int constraintCount;
        mutable int ID;
        friend class ConstraintPtrLT;
    };

    struct ConstraintPtrLT {
        bool operator()(const Constraint * const a, const Constraint * const b) const {
            return (a->ID < b->ID);
        }
    };

    typedef set<const Constraint*, ConstraintPtrLT> ConstraintSet;

    class CountedConstraintSet : public map<const Constraint*, unsigned int, ConstraintPtrLT>
    {

    public:

        typedef map<const Constraint*, unsigned int, ConstraintPtrLT> __super;
        typedef __super::iterator iterator;

        void insert(const Constraint* const c);

        void erase(const Constraint* const c);

        void insert(const ConstraintSet & c);

        void erase(const ConstraintSet & c);



        iterator begin() {
            return __super::begin();
        }

        iterator end() {
            return __super::end();
        }


        template<class _InputIterator>
        void insert(_InputIterator itr, const _InputIterator & itrEnd) {
            bool firstTime = true;
            iterator thisItr;
            for (; itr != itrEnd; ++itr) {
                if (firstTime) {
                    thisItr = __super::insert(make_pair(*itr, 0)).first;
                    firstTime = false;
                } else {
                    thisItr = __super::insert(thisItr, make_pair(*itr, 0));
                }
                ++thisItr->second;
            }
        };

        template<class _InputIterator>
        void erase(_InputIterator itr, const _InputIterator & itrEnd) {
            for (; itr != itrEnd; ++itr) {
                const iterator thisItr = __super::find(*itr);
                if (thisItr != __super::end()) {
                    if (!(--(thisItr->second))) {
                        __super::erase(thisItr);
                    }
                }
            }
        };

    };

    /**
     *  A class to track the changes on a task numeric variable
     *  as the LP is built for the plan so far.
     */
    class FluentTracking
    {

    public:
        /**
         *  An enum defining three states a numeric variable can be in when building the LP:
         *  - <code>FS_NORMAL</code> denotes that the variable should be treated normally, with
         *    effects and preconditions on the variable being included in the LP.
         *  - <code>FS_IGNORE</code> denotes that the variable is a metric tracking variable, to
         *    be ignored (i.e. effects on it should be omitted from the LP)
         *  - <code>FS_ORDER_INDEPENDENT</code> denotes that the variable is an order-independent
         *    metric-tracking variable, and effects on it are to update the variables
         *    <code>FluentTracking::orderIndependentValueTerms</code> and <code>FluentTracking::orderIndependentValueConstant</code>, for
         *    eventual inclusion into the objective function.
         */
        enum fluent_status { FS_NORMAL = 0, FS_IGNORE = 1, FS_ORDER_INDEPENDENT = 2};
        
        /**
         *  The status of this fluent.  For metric-tracking variables, this is set to either <code>FS_IGNORE</code>
         *  or <code>FS_ORDER_INDEPENDENT</code> in the <code>LPScheduler</code> constructor, depending on whether
         *  the plan is to be scheduled to the metric.  For all other variables, this is set to <code>FS_NORMAL</code>.
         */
        fluent_status statusOfThisFluent;
        
        /**
         *  The value of the variable following the last effect.  This is only defined
         *  if <code>FluentTracking::lastEffectValueVariable = -1</code>. */
        double postLastEffectValue;
        
        /**
         *  The LP variable (column) containing the value of the variable following
         *  the last effect to act upon it.  If this takes the value <code>-1</code>,
         *  then the value is a constant, not an LP variable, stored in <code>FluentTracking::postLastEffectValue</code>.
         */
        int lastEffectValueVariable;
        
        /**
         *  The timestamp variable of the last action with an effect upon this variable.
         *  If this takes the value <code>-1</code>, then no such action exists (i.e. there
         *  have been no effects on the variable so far).
         */
        int lastEffectTimestampVariable;
        
        /**
         *  The gradient of the active continuous numeric change acting upon the variable.
         */
        double activeGradient;
        
        /**
         *  How many actions are having a continuous numeric effect upon the variable.  This
         *  is used to prevent rounding errors: if this value takes the value 0, then
         *  <code>FluentTracking::activeGradient</code> is assigned the value <code>0</code>.
         */
        int activeGradientCount;

        /**
         *  For metric-traking variables with order-independent effects, this map
         *  contains column&ndash;weight pairs that contribute to its value.
         */
        map<int,double> orderIndependentValueTerms;
        
        /**
         *  For metric-tracking variables with order-independent effects, the sum of
         *  this constant value and the weighted sum of the LP columns given in
         *  <code>FluentTracking::orderIndependentValueTerms</code> give the value of the variable
         *  at the end of the plan.
         */
        double orderIndependentValueConstant;
        
        /**
         *  Initial constructor: the variable takes the given value (that from the initial
         *  state), its status is set to <code>FS_NORMAL</code>, and no continuous numeric change is active.
         *
         *  @param initial The value of the fluent in the initial state.
         */
        FluentTracking(const double & initial)
            : statusOfThisFluent(FS_NORMAL), postLastEffectValue(initial), lastEffectValueVariable(-1), lastEffectTimestampVariable(-1), activeGradient(0.0), activeGradientCount(0), orderIndependentValueConstant(0.0) {
        }

        /**
         *  Default constructor (unfortunately needed to be able to create a vector of
         *  <code>FluentTracking</code> objects).
         */
        FluentTracking() { // unfortunately needed for vector<FluentTracking>
        }
    };

    class InterestingMap : map<int, bool>
    {

    public:

        typedef map<int, bool> __super;
        typedef __super::iterator iterator;
        typedef __super::const_iterator const_iterator;

        iterator begin() {
            return __super::begin();
        }

        const_iterator begin() const {
            return __super::begin();
        }

        iterator end() {
            return __super::end();
        }

        const_iterator end() const {
            return __super::end();
        }

        const_iterator find(const int & i) const {
            return __super::find(i);
        }

        iterator find(const int & i) {
            return __super::find(i);
        }

        void erase(const int & i) {
            __super::erase(i);
        }

        void erase(const iterator & i) {
            __super::erase(i);
        }

        InterestingMap() {
        }

        InterestingMap(const InterestingMap & other) : __super(other) {
        }

        InterestingMap(const map<int, bool> & in) : __super(in) {
        }

        InterestingMap & operator =(const __super & in) {
            __super::operator=(in);
            return *this;
        }

        template<class _InputIterator>
        void insertKeepingTrues(_InputIterator srcItr, const _InputIterator & srcEnd) {
            __super::iterator lastPos;
            bool firstTime = true;

            for (; srcItr != srcEnd; ++srcItr) {

                if (firstTime) {
                    lastPos = __super::insert(*srcItr).first;
                    firstTime = false;
                } else {
                    lastPos = __super::insert(lastPos, *srcItr);
                }

                if (srcItr->second) lastPos->second = true;

            }

        };

        virtual void insertKeepingTrues(const pair<int, bool> & toInsert);

        virtual void insertEffect(const int & i) {
            __super::insert(make_pair(i, true)).first->second = true;
        }

        virtual void insertPrecondition(const int & i) {
            __super::insert(make_pair(i, false));
        }

        template<class _InputIterator>
        void insertPreconditions(_InputIterator itr, const _InputIterator & itrEnd) {
            bool firstTime = true;
            __super::iterator thisItr;
            for (; itr != itrEnd; ++itr) {
                if (firstTime) {
                    thisItr = __super::insert(make_pair(*itr, false)).first;
                    firstTime = false;
                } else {
                    thisItr = __super::insert(thisItr, make_pair(*itr, false));
                }
            }
        };

        template<class _InputIterator>
        void insertEffects(_InputIterator itr, const _InputIterator & itrEnd) {
            bool firstTime = true;
            __super::iterator thisItr;
            for (; itr != itrEnd; ++itr) {
                if (firstTime) {
                    thisItr = __super::insert(make_pair(*itr, true)).first;
                    firstTime = false;
                } else {
                    thisItr = __super::insert(thisItr, make_pair(*itr, true));
                }
                thisItr->second = true;
            }
        };

    };

    static double* weightScratch;
    static int* varScratch;
    static bool scratchInit;

    struct ConstraintAdder;
    struct DurationAdder;

    friend struct ConstraintAdder;
    friend struct DurationAdder;

    MILPSolver * lp;
        
    /** The LP variable denoting the time of the most recent step in the plan. */
    int timestampToUpdateVar;
    
    /** The index, within the plan, of the most recent step. */
    int timestampToUpdateStep;
    
    /** The <code>FFEvent</code> representing the most recent step in the plan. */
    FFEvent * timestampToUpdate;
        
    /** The LP variable denoting the start/end action paired with the most recent step in the plan.
     *
     *  If no such action exists (i.e. the most recent step is for a non-temporal action or a timed
     *  initial literal), this variable takes the value -1.
     */
    int timestampToUpdatePartnerVar;
    
    /** The index, within the plan, of the step paired with the most recent step.
    *
    *  If no such action exists (i.e. the most recent step is for a non-temporal action or a timed
    *  initial literal), this variable takes the value -1.
    */    
    int timestampToUpdatePartnerStep;
    
    /** The <code>FFEvent</code> representing the plan step paired with the most recent step.
    *
    *  If no such action exists (i.e. the most recent step is for a non-temporal action or a timed
    *  initial literal), this variable takes the value 0.
    */    
    FFEvent * timestampToUpdatePartner;
    
    /** The variable last used as the objective of the LP.
     *
     *  This variable is used to facilitate changing the objective function of the LP.  As
     *  the objective only features a single variable, the prodedure for changing objectives
     *  is as follows:
     *
     *  - Set the objective coefficient of this variable to 0
     *  - Set this variable to the new variable to optimise
     *  - Set the objective coefficient of this variable to 1
     *
     *  If no objective has yet been given to the LP, this variable takes the value -1.
     */
    int previousObjectiveVar;

    //list<int> mutexCols;
    //list<int> mutexRows;
    
    /** The LP variables containing the timestamps of the corresponding plan steps. */
    vector<int> timestampVars;
    
    /** Information about how the task numeric variables are represented in the LP.
     *
     *  Each entry in the vector corresponds to a variable in <code>RPGBuilder::pnes</code>.
     *  As the LP is constructed, iterating through the plan steps, the entries in this
     *  vector are modified according to the effects of the actions.
     *
     *  @see LPScheduler::FluentTracking
     */
    vector<FluentTracking> finalNumericVars;
    
    /** LP variables for the timestamps of actions with no successors.
     *
     *  This list contains the variables in the LP representing the plan steps with
     *  no successors.  When minimising the makespan of the solution, the objective
     *  is set to minimise a variable whose value must exceed each of these.
     */
    list<int> endsOfThreads;
    
    /** Lower-bound on the makespan of the plan.
     *
     *  This is determined during LP construction, as the maximum timestamp
     *  lower bound across all steps in the plan.
     */
    double makespanVarMinimum;
    
    /** A record of whether a variable has only ever been subject to non-time-dependent effects
     *
     *  Each entry in the vector corresponds to a variable in <code>RPGBuilder::pnes</code>. If
     *  the value corresponding to a given variable is <code>false</code>, then the value of the
     *  variable is not stable.  This happens if either:
     *  - it has been subjected to continuous/duration-dependent change; or,
     *  - it has been subjected to discrete change, the value of which depended on a non-stable
     *    variable.
     *
     *  The distinction is made for efficiency: the bounds on variables' values only needs
     *  to be computed for non-stable variables, as no other variables are time-dependent
     *  (and hence need the LP).
     *
     *  @see LPScheduler::updateStateFluents
     */
    vector<bool> stableVariable;
    
    /** Whether the LP representing the plan could be solved. */
    bool solved;
    
    /** Whether LP elements should be named.
     *
     * This is an internal flag for debugging.  If set to <code>true</code>, i.e. if
     * <code>LPScheduler::lpDebug</code> is non-zero, the rows and columns of the LP
     * are given meaningful names, rather than the defaults.  As naming the elements
     * carries a small overhead, and serves no purpose other than making the resulting
     * LP easier for humans to comprehend, the variable takes a value of <code>false</code>
     * unless LP debugging output is enabled.
     */
    bool nameLPElements;
    
    /** Whether to include metric tracking variables in the LP.
     *
     *  By default, this is set to <code>false</code>, i.e. metric tracking variables are excluded
     *  from the LP, leading to smaller models.  However, if the plan is to be post-hoc
     *  scheduled to the metric, this is set to <code>true</code>, as the values of such variables
     *  need to be computed for subsequent optimisation.
     */
    bool includeMetricTrackingVariables;
    
    /** If true, a number of expensive assertions are enabled. */
    bool assertions;

    /** The timestamps of each timed initial literal, in order. */
    static vector<double> TILtimestamps;

    struct EndDetails {
        list<StartEvent>::iterator first;
        int imaginaryMin;
        int imaginaryMax;
        int lastToMin;

        EndDetails(const int & min, const int & max, const int & cons) : imaginaryMin(min), imaginaryMax(max), lastToMin(cons) {};
        EndDetails() {};
    };

    map<int, list<EndDetails> > openDurationConstraints;

    /**
     *  The gradient effects for each action.  At present, the vector contains a single entry, defining
     *  the linear continuous effects that occur across the execution of the action.  This may change
     *  if piecewise linear effects are introduced.
     */
    static vector<vector<list<pair<int, RPGBuilder::LinearEffects::EffectExpression> > > > gradientEffects;

    /**
     *  The instantaneous numeric effects of each action.  Each entry is itself a vector:
     *  - <code>instantEffects[a][0]</code> contains the instantaneous numeric effects for the start of action <code>a</code>
     *  - <code>instantEffects[a][1]</code> contains the instantaneous numeric effects for the end of action <code>a</code>     
     */
    static vector<vector<list<RPGBuilder::RPGNumericEffect* > > > instantEffects;


    /**
     *  The preconditions of the each action.  Each entry is itself a vector:
     *  - <code>constraints[a][0]</code> contains the conditions that must hold at the start of action <code>a</code>
     *  - <code>constraints[a][1]</code> contains the conditions that must hold throughout the execution of action <code>a</code>     
     *  - <code>constraints[a][2]</code> contains the conditions that must hold at the end of action <code>a</code>     
     */
    static vector<vector<list<const Constraint*> > > constraints;

    /**
     *  The numeric variables relevant to each point of each action.  Each entry is a vector of size 3, where:
     *  - <code>interesting[a][0]</code> defines the variables relevant to the start of action <code>a</code>
     *  - <code>interesting[a][1]</code> defines the variables relevant to the invariants of action <code>a</code>     
     *  - <code>interesting[a][2]</code> defines the variables relevant to the end of action <code>a</code>     
     *
     *  In all cases, if the boolean value paired with a variable is <code>true</code>, the action will have
     *  an effect on the variable.  Otherwise, it only requires to inspect its value (for a precondition).
     */
    static vector<vector<InterestingMap> > interesting;

    
    /**
     *  If an action has a conditional effect on a metric-tracking variable, where that condition depends on a
     *  fact controlled by optimisation TILs (TILs relevant only to such conditional effects), then 
     *  the mutexes between the action's start/end and the TILs must be respected.  Thus, for each conditional
     *  effect of an action a:
     *  - if it has a condition <code>(at (start (fact)))</code> then the start point of a cannot coincide
     *    with any TIL deleting <code>(fact)</code>
     *  - if it has a condition <code>(at (end (fact)))</code> then the end point of a cannot coincide
     *    with any TIL deleting <code>(fact)</code>
     *
     *  This vector contains the list of incompatible time points for each action, according to the optimisation
     *  TILs.  Each entry is a vector of size 2, where:
     *  - <code>pointsThatWouldBeMutexWithOptimisationTILs[a][0]</code> contains the time points mutex with the start of action <code>a</code>
     *  - <code>pointsThatWouldBeMutexWithOptimisationTILs[a][1]</code> contains the time points mutex with the end of action <code>a</code>   
     */
    static vector<vector<vector<double> > > pointsThatWouldBeMutexWithOptimisationTILs;
    
    /**
     *  Whether the effects of an action are 'boring', i.e. it does not introduce
     *  duration-dependent or continuous effects.  Each entry is a vector:
     *  - <code>boringAct[a][0]</code> is a pair:
     *    - <code>boringAct[a][0].first</code> is <code>false</code> if the start of the action introduces
     *      time-dependent change, in the case where metric tracking variables are ignored.
     *    - <code>boringAct[a][0].second</code> is <code>false</code> if the start of the action introduces
     *      time-dependent change, in the case where metric tracking variables are included.
     *  - <code>boringAct[a][1]</code> is a pair, defined similarly, for the end of the action.
     */
    static vector<vector<pair<bool,bool> > > boringAct;

    /** The values of each variable in the initial state */
    static vector<double> initialValues;
    
    /** The constraints that must hold in the goal state */
    static list<const Constraint*> goalConstraints;

    static const Constraint* buildConstraint(RPGBuilder::RPGNumericPrecondition & d);
    static int numVars;
    static bool initialised;

    int generateEndDetails(const VAL::time_spec & currTS, const int & actID, const int & stepID, FFEvent & currEvent,
                           const vector<FFEvent*> & planAsAVector, int & nextImaginaryEndVar, vector<EndDetails> & imaginaryMinMax);

    void addConstraintsToGetValuesOfVariablesNow(InterestingMap & currInterest, const int & stepID, const int & currVar, map<int, int> & beforeStep);

    static void collateRelevantVariablesAndInvariants(InterestingMap & currInterest, CountedConstraintSet & activeInvariants,
            const int & stepID, const VAL::time_spec & currTS, const int & actID,
            vector<set<int> > & activeAncestorsOfStep,
            vector<map<int, ConstraintSet > > & invariantsThisStepStartsOnVariableI);

            
    /** Record dependencies between numeric variables and the invariants given.
     *
     *  When building the LP, we need to know which active invariants depend on a given variable so that,
     *  if the value of the variable changes, the relevant invariants can be imposed upon the new value.
     *
     *  @param invariants             The invariants started at a given step
     *  @param invariantsOnVariableI  Where the output from the function: for each variable index, the constraints
     *                                that depend on it.
     */
    static void recordVariablesInvolvedInThisStepsInvariants(const list<const Constraint*> & invariants,
                                                             map<int, ConstraintSet> & invariantsOnVariableI);
            
    /**
     *  Add constraints to the LP to enforce that the given variable cannot take a value equal to
     *  any of the specified timestamps (excluding those that are known to be outwith the variables
     *  permissible bounds)
     *
     *  @param timestampVar  The timestamp variable in the LP to constrain
     *  @param mutexTimestamps  A vector of times whose values the given variable cannot take
     */
    void addConstraintsForTILMutexes(const int & timestampVar, const vector<double> & mutexTimestamps);
            
    /** Add constraints to the LP to determine whether a list of conditional effect conditions are satisfied.
     *
     *  @param numericConditions  The numeric conditions to satisfy, each a pair:
     *    - the first element is an index into <code>RPGBuilder::getNumericPreTable()</code>
     *    - the second element, a <code>VAL::time_spec</code>, dictates when it has to be satisfied:
     *      <code>VAL::E_AT_START</code>, <code>VAL::E_OVER_ALL</code>, or <code>VAL::E_AT_END</code>.
     *  @param actStartAt     The LP variable denoting the start of the action from which the conditions are taken
     *  @param actEndAt       The LP variable denoting the end of the action from which the conditions are taken
     *                        (or <code>-1</code> if the action is non-temporal)
     *  @param conditionVars  The LP columns created (one for each entry in <code>propositionConditions</code>)
     *                        are added to this list.
     *
     *  @return  <code>true</code> if the conditions can possibly be satisfied, <code>false</code> otherwise.
     */
    bool addAnyNumericConstraints(const list<pair<int, VAL::time_spec> > & numericConditions,
                                  const int & actStartAt, const int & actEndAt, list<int> & conditionVars);
            
    /** Add constraints to the LP to determine whether a list of conditional effect conditions are satisfied by the TILs.
     *
     *  @param propositionalConditions  The propositional conditions to satisfy, each a pair:
     *    - the first element, a <code>Literal*</code>, dictates which proposition to satisfy
     *    - the second element, a <code>VAL::time_spec</code>, dictates when it has to be satisfied:
     *      <code>VAL::E_AT_START</code>, <code>VAL::E_OVER_ALL</code>, or <code>VAL::E_AT_END</code>.
     *  @param actStartAt     The LP variable denoting the start of the action from which the conditions are taken
     *  @param actEndAt       The LP variable denoting the end of the action from which the conditions are taken
     *                        (or <code>-1</code> if the action is non-temporal)
     *  @param conditionVars  The LP columns created (one for each entry in <code>propositionConditions</code>)
     *                        are added to this list.
     *
     *  @return  <code>true</code> if the conditions can possibly be satisfied, <code>false</code> otherwise.
     */
    bool addAnyTimeWindowConstraints(const list<pair<Literal*, VAL::time_spec> > & propositionalConditions,
                                     const int & actStartAt, const int & actEndAt, list<int> & conditionVars);
                                     
     /** Set the objective function of the LP to minimise the metric stated in the problem file.
      * 
      * @return <code>false</code> if the new objective was quadratic and led to an unsolvable problem
      */
    bool scheduleToMetric();
       
    /** Set the lower bound on the most recent timestamp in the plan to its current timestamp.
     *
     * When a step is added to the plan, the LP is solved, minimising the timestamp of that step.
     * Thus, the timestamp obtained is a lower bound on the timestamp it can have in any state
     * reached from there onwards.  Furthermore, if the step is the start/end of a durative action
     * we can (potentially) refine the lower bound on the timestamp of the corresponding start/end.
     * 
     * @see timestampToUpdate
     */
    void pushTimestampToMin();
    
    vector<FFEvent*> planAsAVector;
    
public:
    LPScheduler(const MinimalState & s,
                list<FFEvent> & header,
                list<FFEvent> & now,
                const int & justAppliedStep,
                list<StartEvent> & seq,
                ParentData * parentData,
                map<int, list<list<StartEvent>::iterator > > & compulsaryEnds,
                const vector<double> * prevStateMin,
                const vector<double> * prevStateMax,
                list<int> * tilComesBefore,
                const bool & setObjectiveToMetric);

    LPScheduler(const MinimalState & s, list<FFEvent> & plan);
    ~LPScheduler();

    /**
     *  Ascertain whether a given snap-action introduces time-dependent effects or preconditions.
     *  
     *  @param a  The index of the action
     *  @param s  <code>0</code> for the start of the action, <code>1</code> for the end
     *  @param includeMetric If <code>true</code>, then effects on metric-tracking variables
     *         are considered cause for a snap-action not to be boring.
     *
     *  @return <code>true</code> if the specified snap-action introduces time-dependent effects
     *          or preconditions.
     */
    static inline bool isBoring(const int & a, const int & s, const bool & includeMetric) {
        if (includeMetric) {
            return boringAct[a][s].second;
        } else {
            return boringAct[a][s].first;
        }
    };

    const bool & isSolved() const {
        return solved;
    };

    /** Update the bounds on the task variables in the state reached by the plan so far.
     *
     *  Assuming the plan passed to the constructor led to an LP that could be solved
     *  (i.e. <code>isSolved() = true</code>), then this function can be used to calculate
     *  the upper and lower bounds of the task variables in a state after the actions in
     *  the plan so far.  Note that only the bounds of time-dependent variables are
     *  actually updated, so it is important that for all other variables, the bounds
     *  passed as input to the function are correct.
     *
     *  @param  min  The vector containing the lower bounds to be updated
     *  @param  max  The vector containing the upper bounds to be updated
     */
    void updateStateFluents(vector<double> & min, vector<double> & max);

    bool addRelaxedPlan(list<FFEvent> & header, list<FFEvent> & now, list<pair<double, list<ActionSegment> > > & relaxedPlan);

    bool isSolution(const MinimalState & state, list<FFEvent> & header, list<FFEvent> & now);
    
    /**
     *  Function to support the incremental Bellman Ford algorithm.  It creates an object representing the
     *  the temporal constraints (both action sequencing constraints, and duration constraints) relevant
     *  to the current plan header, and the list of actions that have started but not yet finished.
     *  Functions on this object can then be used to incrementally check the consistency of these temporal
     *  constraints when a new action is applied.
     *  @see ParentData, ChildData
     *
     *  @param header  The plan header for which to create the temporal constraint data.
     *  @param cons    The temporal separation constraints between the steps in the plan.
     *  @param open    The list of actions that have started, but not yet finished, following the supplied plan.
     *  @param includeMetric  If <code>true</code>, effects on metric-tracking variables are considered
     *                        when determining whether the steps in <code>header</code> necessitate the
     *                        the use of an LP.
     *
     *  @return  An object capturing all the temporal constraints on the plan header.
     */
    static ParentData* prime(list<FFEvent> & header, const TemporalConstraints * const cons, list<StartEvent> & open, const bool includeMetric=false);

    static void initialise();
    static int lpDebug;
    static const double & getTILTimestamp(const int & i) {
        return TILtimestamps[i];
    };
};



class LPQueueSet
{

private:

    const int arrSize;
    list<int> Q;
    bool * qSet;

public:

    bool * UB;
    bool * LB;
    bool * UBP;
    bool * LBP;
    int * NEW;

    LPQueueSet(const int & i) : arrSize(i + 1), qSet(new bool[arrSize]),
            UB(new bool[arrSize]), LB(new bool[arrSize]),
            UBP(new bool[arrSize]), LBP(new bool[arrSize]), NEW(new int[arrSize]) {
        ++qSet; ++UB; ++LB; ++UBP; ++LBP; ++NEW;
        memset(&qSet[-1], 0, arrSize * sizeof(bool));
        memset(&UB[-1], 0, arrSize * sizeof(bool));
        memset(&LB[-1], 0, arrSize * sizeof(bool));
        memset(&UBP[-1], 0, arrSize * sizeof(bool));
        memset(&LBP[-1], 0, arrSize * sizeof(bool));
        for (int j = -1; j < i; ++j) NEW[j] = -2;
    };

    ~LPQueueSet() {
        delete [](--qSet);
        delete [](--UB);
        delete [](--LB);
        delete [](--UBP);
        delete [](--LBP);
        delete [](--NEW);
    };

    void push_back(const int & u) {
        if (!qSet[u]) {
            Q.push_back(u);
            qSet[u] = true;
        }
    }

    bool empty() const {
        return (Q.empty());
    }

    int pop_front() {
        int toReturn = Q.front();
        qSet[toReturn] = false;
        Q.pop_front();
        return toReturn;
    }

    void cleanup(const int & from, const int & to) {
        reset(from, to);
        memset(&qSet[-1], 0, arrSize * sizeof(bool));
        Q.clear();
    }

    void reset(const int & from, const int & to) {
        memset(&UB[-1], 0, arrSize * sizeof(bool));
        memset(&LB[-1], 0, arrSize * sizeof(bool));
        memset(&UBP[-1], 0, arrSize * sizeof(bool));
        memset(&LBP[-1], 0, arrSize * sizeof(bool));
        NEW[from] = -2;
        NEW[to] = -2;
    }

    void visit(const int & from, const int & to) {

        push_back(from);
        push_back(to);

        LB[from] = true;
        UB[from] = true;
        NEW[from] = to;

        LB[to] = true;
        UB[to] = true;
        NEW[to] = from;
    }

};

class IncomingAndOutgoing
{

protected:
    map<int, bool> mustFollowThisD;
    map<int, bool> mustPrecedeThisD;

public:

    const map<int, bool> & mustFollowThis() const {
        return mustFollowThisD;
    }

    const map<int, bool> & mustPrecedeThis() const {
        return mustPrecedeThisD;
    }

    void addFollower(const int & a, const bool & b) {
        if (Globals::globalVerbosity & 4096) {
            if (b) {
                cout << "Insisting that " << a << " is at least epsilon after this step\n";
            } else {
                cout << "Insisting that " << a << " is at least 0 after this step\n";
            }
        }
        bool & orWith = mustFollowThisD.insert(make_pair(a, b)).first->second;
        orWith = (orWith || b);

    }

    void initialisePredecessors(const map<int, bool> & p) {
        assert(mustPrecedeThisD.empty());
        mustPrecedeThisD = p;
    }

    void addPredecessor(const int & a, const bool & b) {
        if (Globals::globalVerbosity & 4096) {
            if (b) {
                cout << "Insisting that " << a << " is at least epsilon before this step\n";
            } else {
                cout << "Insisting that " << a << " is at least 0 before this step\n";
            }
        }
        bool & orWith = mustPrecedeThisD.insert(make_pair(a, b)).first->second;
        orWith = (orWith || b);
    }
};

class ParentData
{

public:
    LPQueueSet Q;
private:
    int qs;
    int startGap;
    int endGap;
    vector<double> distFromZero;
    vector<double> distToZero;
    vector<int> pairWith;
    vector<FFEvent*> eventsWithFakes;
    list<FFEvent> * parentPlan;

    /**
     *   Temporal separation constraints for the  given step of the plan.  <code>temporaryEdges[i]</code> holds a pair of sets:
     *   - the first defines steps that must follow step i
     *   - the second defines steps that must precede step i
     */
    map<int, IncomingAndOutgoing > temporaryEdges;
    bool needsLP;
    int nextTIL;
public:
    ParentData(const int & qSize, list<FFEvent> * h, const int & nt)
            : Q(qSize), qs(qSize), startGap(-1), endGap(-1),
            distFromZero(qSize, DBL_MAX), distToZero(qSize, 0.0),
            pairWith(qSize, -1), eventsWithFakes(qSize, (FFEvent*)0),
            parentPlan(h), nextTIL(nt) {};

    ~ParentData() {
        for (int i = 0; i < qs; ++i) {
            delete eventsWithFakes[i];
        }
    }
    void setWhetherNeedsLP(const bool & b) {
        needsLP = b;
    };

    const vector<double> & getDistFromZero() const {
        return distFromZero;
    };
    const vector<double> & getDistToZero() const {
        return distToZero;
    };
    const vector<FFEvent*> & getEventsWithFakes() const {
        return eventsWithFakes;
    };
    const vector<int> & getPairWith() const {
        return pairWith;
    };
    const map<int, IncomingAndOutgoing > & getTemporaryEdges() const {
        return temporaryEdges;
    };


    inline void setRawDistToFromZero(const int & i, const double & t, const double & f) {
        distToZero[i] = t;
        distFromZero[i] = f;
    };

    inline void setPairWith(const int & i, const int & w) {
        pairWith[i] = w; pairWith[w] = i;
    };
    inline void setTIL(const int & i) {
        pairWith[i] = -2;
    };
    inline void setNonTemporal(const int & i) {
        pairWith[i] = -3;
    };
    inline void supplyFake(const int & i, FFEvent * const f) {
        eventsWithFakes[i] = f;
    };

    /**
      * Return the temporal separation constraints for the specified step.
      *
      * @param i The step for which to obtain the constraints
      *
      * @return A pair of sets: the first defines steps that must follow step i,
      *                         the second defines steps that must precede step i.
      */
    inline IncomingAndOutgoing & makeEdgeListFor(const int & i) {
        return temporaryEdges[i];
    };

    inline void startGapIsStep(const int & i) {
        startGap = i;
    };
    inline void endGapIsStep(const int & i) {
        endGap = i;
    };
    const int & whereIsStartGap() const {
        return startGap;
    };
    const int & whereIsEndGap() const {
        return endGap;
    };

    /**
     *  Helper function for the incremental Bellman Ford implementation.  From the current object,
     *  reached by the plan steps <code>header</code>, derive a <code>ChildData</code> object
     *  corresponding to either the application of the actions in <code>succ</code>, or
     *  by activating the end snap-action at index <code>stepID</code>.
     *
     *  @param seq     The list of actions that have not yet finished in the child state
     *  @param header  The actions to reach the parent state
     *  @param succ    Any new actions added to reach the child state
     *  @param includeMetric  Whether effects on metric tracking variables should be included -
     *                        affects whether or not the LP is considered necessary.
     *  @param cons    The temporal constraints on the steps in the child's plan
     *  @param stepID  The index into the child's plan of the new step added.
     *
     *  @return An object capturing the temporal constraints in place on the child's plan.
     */
    ChildData * spawnChildData(list<StartEvent> & seq,
                               list<FFEvent> & header, list<FFEvent> & succ,
                               const bool & includeMetric,
                               const TemporalConstraints * const cons,
                               const int & stepID);

    void sanityCheck() {
        const int loopLim = distToZero.size();
        list<FFEvent>::iterator hItr = parentPlan->begin();
        for (int i = 0; i < loopLim; ++i) {
            if (hItr != parentPlan->end()) {
                if (hItr->time_spec == VAL::E_AT && pairWith[i] != -2) {
                    cout << "Header event " << i << " is a TIL, but is not paired with -2\n";
                    assert(pairWith[i] == -2);
                }
                ++hItr;
            } else {
                if (eventsWithFakes[i] && eventsWithFakes[i]->time_spec == VAL::E_AT && pairWith[i] != -2) {
                    cout << "Event " << i << " is a TIL, but is not paired with -2\n";
                    assert(pairWith[i] == -2);
                }
            }
        }
    }


};

};

#endif
