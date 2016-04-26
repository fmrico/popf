#ifndef __MINIMALSTATE
#define __MINIMALSTATE

#include <set>
#include <map>
#include <vector>
#include <iostream>

#include <cassert>

#include <values.h>

using std::set;
using std::map;
using std::vector;
using std::pair;
using std::ostream;

#include "globals.h"

namespace Planner
{

class TemporalConstraints;

struct StepAndBeforeOrAfter {

    enum BeforeOrAfter {BEFORE = 0, AFTER = 1};

    static const unsigned int NEVER = 0x7FFFFFFF;
    unsigned int beforeOrAfter: 1;
    unsigned int stepID: 31;

    StepAndBeforeOrAfter()
            : beforeOrAfter(BEFORE), stepID(0) {
    }

    StepAndBeforeOrAfter(const BeforeOrAfter & bOrA, const unsigned int & stepIDin)
            : beforeOrAfter(bOrA), stepID(stepIDin) {
    }


    bool operator <(const StepAndBeforeOrAfter & o) const {
        if (stepID < o.stepID) return true;
        if (stepID > o.stepID) return false;

        if (beforeOrAfter == BEFORE && o.beforeOrAfter == AFTER) return true;
        return false;

    }

    bool operator >(const StepAndBeforeOrAfter & o) const {
        if (stepID > o.stepID) return true;
        if (stepID < o.stepID) return false;

        if (beforeOrAfter == AFTER && o.beforeOrAfter == BEFORE) return true;
        return false;

    }

    bool operator ==(const StepAndBeforeOrAfter & other) const {
        return (stepID == other.stepID && beforeOrAfter == other.beforeOrAfter);
    }

    void write(ostream & o) const {
        if (beforeOrAfter == BEFORE) {
            o << "before step " << stepID;
        } else {
            o << "after step " << stepID;
        }
    }

    void setToNever() {
        stepID = NEVER;
        beforeOrAfter = AFTER;
    }

    bool never() const {
        return (stepID == NEVER);
    }
};

ostream & operator <<(ostream & o, const StepAndBeforeOrAfter & s);

struct PropositionAnnotation {

#define SAFETOSKIP true
#define UNSAFETOSKIP false

    StepAndBeforeOrAfter negativeAvailableFrom;
    StepAndBeforeOrAfter availableFrom;
    map<StepAndBeforeOrAfter, bool> deletableFrom;
    map<StepAndBeforeOrAfter, bool> addableFrom;
    set<int> promisedDelete;
    set<int> promisedAdd;

    PropositionAnnotation(const bool polarity = true) {
        if (polarity) {
            negativeAvailableFrom.setToNever();
        } else {
            availableFrom.setToNever();
        }
    };

    /** Full Constructor for a Proposition Annotation
     *
     *  @param propositionIsAvailableFrom   Which step the proposition (or its negation) are available from
     *  @param propositionCanBeToggledFrom  <code>PropositionAnnotation::AFTER</code> if available from epsilon after that onwards, <code>PropositionAnnotation::BEFORE</code> otherwise.
     *  @param polarity                     If <code>true</code>, the proposition is available at the time given; if <code>false</code>, its negation is available at the time given.
     */
    PropositionAnnotation(const StepAndBeforeOrAfter &propositionIsAvailableFrom,
                          const map<StepAndBeforeOrAfter, bool> &propositionCanBeToggledFrom,
                          const bool & polarity) {
        if (polarity) {
            availableFrom = propositionIsAvailableFrom;
            negativeAvailableFrom.setToNever();
            deletableFrom = propositionCanBeToggledFrom;
        } else {
            availableFrom.setToNever();
            negativeAvailableFrom = propositionIsAvailableFrom;
            addableFrom = propositionCanBeToggledFrom;
        }
    };

    /** Lazy Constructor for a Proposition Annotation.  Use when a proposition has just been added.
     *  
     *  The resulting Proposition Annotation denotes that it is available from after the end of the
     *  specified step ID onwards, and can be deleted from the end of the specified stepID onwards.
     *
     *  @param stepID  An <code>unsigned int</code> denoting the step at which the proposition is added.
     */
    PropositionAnnotation(const unsigned int & stepID)
            : availableFrom(StepAndBeforeOrAfter(StepAndBeforeOrAfter::AFTER, stepID)) {
        deletableFrom.insert(make_pair(availableFrom, SAFETOSKIP));
        negativeAvailableFrom.setToNever();
    };

    /** Lazy Constructor for a Proposition Annotation.  Use for when a proposition has just been added.
    *  
    *  The resulting Proposition Annotation denotes that it is available from after the end of the
    *  specified step ID onwards, and can be deleted from the end of the specified stepID onwards.
    *
    *  @param sba  A <code>StepAndBeforeOrAfter</code> object denoting the point at which the proposition
    *              can be used.
    */
    PropositionAnnotation(const StepAndBeforeOrAfter & sba)
            : availableFrom(sba) {
        if (sba.stepID != 0 || sba.beforeOrAfter == StepAndBeforeOrAfter::AFTER) {
            deletableFrom.insert(make_pair(availableFrom, SAFETOSKIP));
        }
        negativeAvailableFrom.setToNever();
    };

    void markAsDeleted(const StepAndBeforeOrAfter & step) {
        if (!availableFrom.never()) {
            deletableFrom.insert(make_pair(availableFrom, true));
            availableFrom.setToNever();
        }
        addableFrom.clear();
        promisedDelete.clear();
        negativeAvailableFrom = step;
    }

    void markAsAdded(const StepAndBeforeOrAfter & step) {
        if (!negativeAvailableFrom.never()) {
            addableFrom.insert(make_pair(negativeAvailableFrom, true));
            negativeAvailableFrom.setToNever();
        }
        deletableFrom.clear();
        promisedAdd.clear();
        availableFrom = step;
    }


    bool operator ==(const PropositionAnnotation & other) const {
        return (availableFrom == other.availableFrom && deletableFrom == other.deletableFrom
                && negativeAvailableFrom == other.negativeAvailableFrom && addableFrom == other.addableFrom);
    }

};



class ActionSegment;

class TemporalConstraints;
class MinimalState;


/**
 *   Virtual class to wrap up the functionality of applying an action to a state and updating the
 *   temporal constraints.  @see PartialOrderTransformer, TotalOrderTransformer
 */
class StateTransformer
{

protected:
    StateTransformer() {};

public:
    virtual ~StateTransformer() {};
    virtual TemporalConstraints * cloneTemporalConstraints(const TemporalConstraints * const, const int extendBy = 0) = 0;
    virtual TemporalConstraints * emptyTemporalConstraints() = 0;


    /**
     *  Return the index of the step in the plan that must precede the ends of all actions that have not yet
     *  finished.  Default implementation is to return -1: i.e. no such step exists.
     *  @see TotalOrderTransformer
     */
    virtual int stepThatMustPrecedeUnfinishedActions(const TemporalConstraints * const) const
    {
        return -1;
    }

    /**
     *  Return the upper bound on the time-stamp of the next snap-action applied, after the
     *  specified number of TILs have been applied.  Default implementation is to return
     *  <code>DBL_MAX</code>, i.e. unbounded.
     *
     *  @param tilIndexApplied  How many TILs have been applied
     *  @return  The upper bound of the time-stamp of the next action applied
     *
     *  @see TotalOrderTransformer, RPGBuilder::FakeTILAction
     */
    virtual double latestTimePointOfActionsStartedHere(const int & tilIndexApplied) const
    {
        return DBL_MAX;
    }

    /**
     *  Apply a given action in the given state.
     *
     *  @param s          The state in which to apply the action a
     *  @param a          The action to apply in the state s
     *  @param inPlace    If <code>false</code>, a copy of <code>s</code> is made, and <code>a</code> is applied to this.
     *                    Otherwise, <code>a</code> is applied directly to <code>s</code>.
     *  @param minDur     An optimistic minimum duration of <code>a</code>
     *  @param maxDur     An optimistic maximum duration of <code>a</code>
     *
     *  @return           The state reached following the application of <code>a</code>.
     *                    If <code>inPlace=true</code>, this refers to the same state as <code>s</code>.
     */
    virtual MinimalState * applyAction(MinimalState & s, const ActionSegment & a, const bool & inPlace,
                                       const double & minDur, const double & maxDur) = 0;
                                       
   
    #ifdef POPF3ANALYSIS
    #ifndef TOTALORDERSTATES
    /** @brief Update the record of when the end of an action falls.
     *
     * When a compression-safe action is applied, this action is used to update the record
     * of its queued compression-safe numeric end effects.
     */
    virtual void updateWhenEndOfActionIs(MinimalState & s, const int & actID, const int & stepID, const double & newTS) = 0;
    #endif
    #endif
};

#ifdef TOTALORDERSTATES
typedef set<int> StateFacts;
typedef map<int, pair<set<int>, set<int> > > StateBFacts; // first: actions that add the fact; second: actions that require the fact as an invariant
#define FACTA(x) (*x)
#define FACTB(x) (x->first)
#else
typedef map<int, PropositionAnnotation> StateFacts;
#define FACTA(x) (x->first)
#endif

class MinimalState
{

protected:
    bool operator ==(MinimalState &) {
        assert(false);
        return false;
    }

    /** Pointer to the StateTransformer object used to apply objects to states.
     *  This is set in the main function of POPF, prior to search.
     *  @see StateTransformer
     */
    static StateTransformer* globalTransformer;

public:

    MinimalState & operator =(const MinimalState & s);

    #ifdef TOTALORDERSTATES
    StateFacts first;
    map<int, int> invariants; // a map from fact IDs to how many non-compression-safe actions have an invariant on that fact
    StateBFacts firstAnnotations;
    #else
    StateFacts first;
    StateFacts retired;
    #endif
    vector<double> secondMin;
    vector<double> secondMax;
    map<int, set<int> > startedActions;

    unsigned int planLength;
    unsigned int actionsExecuting;
    int nextTIL;

    TemporalConstraints * temporalConstraints;

    #ifdef STOCHASTICDURATIONS
    
    int* stepFromWhichLiteralGoalsHold;
    int** stepsFromWhichNumericGoalsHold;
    
    void deleteGoalStepRecords();
    void copyGoalStepRecords(const int * const literalIn, int** const numericIn);
    
    void literalGoalHoldsFromStep(const int & gID, const int & stepID);
    void numericGoalHoldsFromSteps(const int & gID, const list<int> & stepID);
    
    inline void numericGoalHoldsFromStep(const int & gID, const int & stepID) {
        list<int> tmp;
        tmp.push_back(stepID);
        numericGoalHoldsFromSteps(gID,tmp);        
    }        
    #endif
    
    #ifndef TOTALORDERSTATES    
    MinimalState(const set<int> & f,
                 const vector<double> & sMin, const vector<double> & sMax,
                 const map<int, set<int> > & sa,
                 const int nt = 0, const unsigned int pl = 0, const unsigned int ae = 0
                 #ifdef STOCHASTICDURATIONS
                 ,const int * const literalGoalStepsIn = 0, int** const numericGoalStepsIn = 0
                 #endif     
                 );
    #endif
    
    MinimalState(const StateFacts & f,
                 const vector<double> & sMin, const vector<double> & sMax,
                 const map<int, set<int> > & sa,
                 const int nt=0, const unsigned int pl=0, const unsigned int ae=0
                 #ifdef STOCHASTICDURATIONS
                 ,const int * const literalGoalStepsIn = 0, int** const numericGoalStepsIn = 0
                 #endif     
                );

    #ifdef TOTALORDERSTATES
    template<typename _InputIterator>
    void insertFacts(_InputIterator begin, const _InputIterator & end, const StepAndBeforeOrAfter &) {
        StateFacts::iterator insItr = first.end();
        for (; begin != end; ++begin) {
            insItr = first.insert(insItr, (*begin)->getStateID());
        }
        
    };
    
    template<typename _InputIterator>
    void insertIntFacts(_InputIterator begin, const _InputIterator & end, const StepAndBeforeOrAfter &) {

        StateFacts::iterator insItr = first.end();
        for (; begin != end; ++begin) {
            insItr = first.insert(insItr, *begin);
        }
        
        

    };
    #else
    
    template<typename _InputIterator>
    void insertFacts(_InputIterator begin, const _InputIterator & end, const StepAndBeforeOrAfter & from) {
        StateFacts::iterator insItr = first.end();
        for (; begin != end; ++begin) {
            insItr = first.insert(insItr, make_pair((*begin)->getStateID(), PropositionAnnotation(from)));
            insItr->second.availableFrom = from;
        }

    };

    template<typename _InputIterator>
    void insertIntFacts(_InputIterator begin, const _InputIterator & end, const StepAndBeforeOrAfter & from) {
        StateFacts::iterator insItr = first.end();
        for (; begin != end; ++begin) {
            insItr = first.insert(insItr, make_pair(*begin, PropositionAnnotation(from)));
            insItr->second.availableFrom = from;
        }

    };
    #endif

    MinimalState(const MinimalState & other, const int extra = 0);
    MinimalState();
    virtual ~MinimalState();

    /**
     *  Specify a <code>StateTransformer</code> object to use to handle applying actions to states, and updating
     *  the recorded temporal constraints.
     *
     *  @param s  The <code>StateTransformer</code> to use.
     *
     *  @see StateTransformer,TotalOrderTransformer,PartialOrderTransformer
     */
    static void setTransformer(StateTransformer * const s) {
        globalTransformer = s;
    }

    /**
     *  Return a pointer to the <code>StateTransformer</code> object to use to handle applying actions to states,
     *  and updating the recorded temporal constraints.
     *
     *  @return The <code>StateTransformer</code> object currently in use.
     *
     *  @see StateTransformer,TotalOrderTransformer,PartialOrderTransformer
     */
    static StateTransformer * getTransformer() {
        return globalTransformer;
    }

    MinimalState * applyAction(const ActionSegment & a, double minDur = 0.0, double maxDur = 0.0) const {
        return globalTransformer->applyAction(*const_cast<MinimalState*>(this), a, false, minDur, maxDur);
    }

    void applyActionLocally(const ActionSegment & a, double minDur = 0.0, double maxDur = 0.0) {
        globalTransformer->applyAction(*this, a, true, minDur, maxDur);
    }

    #ifdef POPF3ANALYSIS
    #ifndef TOTALORDERSTATES
    /** @brief Update the record of when the end of an action falls.
     *
     * When a compression-safe action is applied, this action is used to update the record
     * of its queued compression-safe numeric end effects.
     */
    void updateWhenEndOfActionIs(const int & actID, const int & stepID, const double & newTS) {
        globalTransformer->updateWhenEndOfActionIs(*this, actID, stepID, newTS);
    }
    #endif
    #endif
    
    void printState(ostream & o) const;

    void setFacts(const set<int> & s);
    void setFacts(const LiteralSet & s);
    void setFacts(const vector<double> & f);
};



struct StrongStateEquality {

    bool operator()(const MinimalState & a, const MinimalState & b);
};

struct StrongStateLessThan {

    bool operator()(const MinimalState & a, const MinimalState & b);
};


struct WeakStateEquality {

    bool operator()(const MinimalState & a, const MinimalState & b);
};

struct WeakStateLessThan {

    bool operator()(const MinimalState & a, const MinimalState & b);
};


ostream & operator <<(ostream & o, const MinimalState & s);

};

#endif
