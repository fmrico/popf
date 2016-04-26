#include "RPGBuilder.h"
#include "globals.h"
#include "temporalanalysis.h"
#include "numericanalysis.h"
#include "FFSolver.h"

#include <cmath>
#include <vector>
#include <iomanip>
#include <sstream>
#include <fstream>

#include "typecheck.h"
#include "TIM.h"
#include "FuncAnalysis.h"
#include "temporalconstraints.h"

#include "lpscheduler.h"

#ifdef STOCHASTICDURATIONS
#include "StochasticDurations.h"
#endif

#include "colours.h"

using namespace TIM;
using namespace Inst;
using namespace VAL;

using std::cerr;
using std::endl;
using std::vector;
using std::ostringstream;
using std::ofstream;

namespace Planner
{

    
/** @brief Determine the earliest point at which a precondition would be satisifed in the partial order.
 *
 * This is a helper function for the heuristic evaluation.  It takes a numeric precondition,
 * and a vector containing the earliest point at which each task variable can be referred to,
 * and from these deduces the next earliest point at which the precondition could conceivably
 * be considered to be satisfied (i.e. after all the most recent effects).
 *
 * @param p  The numeric precondition
 * @param earliestNumericPOTimes  The earliest point at which each variable can be referred to,
 *                                according to the partial order so far.
 *
 * @return  The earliest time point that the precondition can be considered to be true, i.e.
 *          the earliest point at which each action with this precondition would be true, were it
 *          added to the partial order.
 */
double earliestPointForNumericPrecondition(const RPGBuilder::RPGNumericPrecondition & p,
                                           const vector<double> * earliestNumericPOTimes) {
    static const int varCount = RPGBuilder::getPNECount();
    double TS = 0.0;

    for (int pass = 0; pass < 2; ++pass) {
        int var = (pass ? p.RHSVariable : p.LHSVariable);
        if (var == -1) continue;
        if (var >= 2 * varCount) {
            const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(var);

            for (int i = 0; i < currAV.size; ++i) {
                int varTwo = currAV.fluents[i];
                if (varTwo >= varCount) varTwo -= varCount;
                if ((*earliestNumericPOTimes)[varTwo] > TS) TS = (*earliestNumericPOTimes)[varTwo];
            }
        } else {
            if (var >= varCount) var -= varCount;
            if ((*earliestNumericPOTimes)[var] > TS) TS = (*earliestNumericPOTimes)[var];
        }
    }

    return TS;
}
    

/** @brief A struct representing the intermediate goals to achieve at a layer during RPG solution extraction.
 *
 *  TRPG solution extraction introduces four kinds of intermediate goals:
 *  - Propositions to be achieved
 *  - Numeric values to be achieved (v >= c);
 *  - Action starts that must be added (as their ends were chosen later)
 *  - Action ends that must be added (the ends of actions that have been started in the plan, but not yet finished)
 *
 *  Each of these is weighted by which TIL-induced deadline it is relevant to, to support the possibility
 *  of reordering the successors so that actions relevant to sooner deadlines are considered first.
 */
struct RPGRegress {

    /** The propositional subgoals to achieve in this layer */
    map<int, double> propositionalGoals;
    
    /** The numeric subgoals to achieve in this layer
     *
     *  Each numeric variable maps to a pair of doubles:
     *  - the first denotes the value it must be greater than or equal to
     *  - the second denotes the TIL deadline weight of this
     */
    map<int, pair<double,double> > numericValueGreaterThan;
    
    /** Request a numeric precondition in this layer.
     *
     *  If there is already a request for the specified variable to take a given variable, the maximum
     *  threshold out of that given and that already recorded is kept, and the earliest TIL deadline out
     *  of that given and that recorded is kept.
     *
     *  @param variable   The variable on which to request a precondition
     *  @param threshold  The value this variable must (at least) take
     *  @param weight     The TIL deadline weight of this request
     */
    void requestNumericThreshold(const int & variable, const double & threshold, const double & weight) {
        const pair<map<int, pair<double,double> >::iterator, bool> insPair = numericValueGreaterThan.insert(make_pair(variable, make_pair(threshold, weight)));
        
        if (!insPair.second) {
            if (insPair.first->second.first < threshold) {
                insPair.first->second.first = threshold;
            }
            if (insPair.first->second.second < weight) {
                insPair.first->second.second = weight;
            }
        }
        
    }
    

    //map<int, double> numericGoals;
    
    /** Action starts that must be included in this layer.
     *
     *  Each action index maps to a pair:
     *  - the first entry denotes how many times the action must be started in this layer;
     *  - the second denotes the TIL deadline weight.
     */
    map<int, pair<int, double> > actionStarts;
    
    /** Action starts that must be included in this layer.
     *
     *  Each action index maps to a pair:
     *  - the first entry denotes how many times the action must be ended in this layer;
     *  - the second denotes the TIL deadline weight.
     */    
    map<int, pair<int, double> > actionEnds;

};

#ifdef POPF3ANALYSIS
struct CostedAchieverDetails {
    
    double layer;
    pair<int, VAL::time_spec> achiever;
    vector<double> costs;
    bool costFree;
    
    CostedAchieverDetails() : layer(-1.0), costFree(true) {
    }
    
    CostedAchieverDetails(const vector<double> & costsIn, const bool & costFreeIn)
        : layer(0.0), costs(costsIn), costFree(costFreeIn) {
            
        achiever.first = -1;
    }
    
    CostedAchieverDetails(const double & layerTime,
                          const pair<int, VAL::time_spec> & achieverDetails,
                          vector<double> & costDetails,
                          const bool & costFreeIn)                           
        : layer(layerTime), achiever(achieverDetails), costs(costDetails), costFree(costFreeIn) {
            
    }
    
    
};
#endif

#ifdef NDEBUG
typedef map<double, RPGRegress, EpsilonComp> RPGRegressionMap;
#else

class RPGRegressionMap {
    
protected:
    map<double, RPGRegress, EpsilonComp> internal;
    double forbidLaterThan;
public:
    
    RPGRegressionMap()
    : forbidLaterThan(DBL_MAX) {        
    }
    
    typedef map<double, RPGRegress, EpsilonComp>::iterator iterator;
    typedef map<double, RPGRegress, EpsilonComp>::const_iterator const_iterator;
    
    typedef map<double, RPGRegress, EpsilonComp>::reverse_iterator reverse_iterator;
    typedef map<double, RPGRegress, EpsilonComp>::const_reverse_iterator const_reverse_iterator;
    
    RPGRegress & operator[](const double & t) {
        #ifndef NDEBUG
        if (t >= forbidLaterThan) {
            cerr << "Fatal internal error: in solution extraction, satisfying a numeric precondition at layer " << t << ", after previously satisfied one at layer " << forbidLaterThan << " - need to work strictly backwards\n";
            assert(t < forbidLaterThan);
        }
        #endif
        return internal[t];
    }
    
    bool empty() const {
        return internal.empty();
    }
    
    reverse_iterator rbegin() {
        return internal.rbegin();
    }
    
    reverse_iterator rend() {
        return internal.rend();
    }
    
    iterator end() {
        return internal.end();
    }
    
    void erase(const double & t) {
        forbidLaterThan = t;
        internal.erase(t);
    }
};

#endif

/** @brief A struct recording how many times an action is to be applied in a given layer.
 *
 *  This is used during solution extraction, when adding actions to support numeric
 *  preconditions: <code>FluentLayers::satisfyNumericPreconditionsAtLayer()</code> returns
 *  a list of these, indicating the actions it chose.  These then must be added to the relaxed
 *  plan and, in the case of end actions, the corresponding starts noted for insertion at an
 *  earlier layer.
 */
struct SupportingAction {
  
    int actID;
    VAL::time_spec ts;
    int howManyTimes;
    double tilR;
    
   
    SupportingAction(const int & actionID, const VAL::time_spec & timeSpecifier,
                     const int & applyThisManyTimes, const double & tilRelevance)
        : actID(actionID), ts(timeSpecifier),
          howManyTimes(applyThisManyTimes), tilR(tilRelevance) {
    }
};

void rpprintState(MinimalState & e)
{

    e.printState(cout);

};

/** @brief A struct used to record details about the actions capable of having each effect.
 * 
 *  This is used to update the variables' values when the values of the variables referred to by
 *  an effect have changed.
 */
struct ActionAndHowManyTimes {
    /** @brief The ID of the action. */
    int actID;
    
    /** @brief Whether the effect occurs at the start of the action or the end. */
    VAL::time_spec ts;
    
    /** @brief At most how many times the action can be applied (<code>INT_MAX</code> for infinite). */
    int howManyTimes;    

    /** @brief The minimum duration of the action. */
    double minDur;
    
    /** @brief The maximum duration of the action. */
    double maxDur;
    
    /** @brief The layer at which its gradients effect finish. */
    double gradientsFinishAt;
    
    /** @brief If this action starts its gradients. 
     *
     *  This is set to <code>true</code> in one of two cases:
     *  - The first time this action has appeared in the RPG, in which case all its gradient
     *    effects need to be started.
     *  - When the action reappears in the RPG (after an assignment has been made to a variable
     *    upon which it has a gradient effect), and then iff the actions gradient had expired
     *    earlier in the RPG (and hence needs re-starting again).
     */    
    bool gradientNeedsStarting;
    
    /** @brief If this action (a start action) is already in the plan. */
    bool alreadyInThePlan;
    
    ActionAndHowManyTimes(const int & act, const VAL::time_spec & timeSpec, const int & lim, const double & dmin, const double & dmax, const bool alreadyIn=false)
     : actID(act), ts(timeSpec), howManyTimes(lim), minDur(dmin), maxDur(dmax), gradientsFinishAt(DBL_MAX), gradientNeedsStarting(true), alreadyInThePlan(alreadyIn)
    {
        assert(howManyTimes > 0);
    }
    
    #ifndef NDEBUG
    ActionAndHowManyTimes(const ActionAndHowManyTimes & other)
        : actID(other.actID), ts(other.ts), howManyTimes(other.howManyTimes),
          minDur(other.minDur), maxDur(other.maxDur),
          gradientsFinishAt(other.gradientsFinishAt), gradientNeedsStarting(other.gradientNeedsStarting),
          alreadyInThePlan(other.alreadyInThePlan) {
        assert(howManyTimes);
    }
    
    ActionAndHowManyTimes & operator=(const ActionAndHowManyTimes & other) {
        actID = other.actID;
        ts = other.ts;
        howManyTimes = other.howManyTimes;
        minDur = other.minDur;
        maxDur = other.maxDur;
        gradientsFinishAt = other.gradientsFinishAt;
        gradientNeedsStarting = other.gradientNeedsStarting;
        alreadyInThePlan = other.alreadyInThePlan;
        assert(howManyTimes);
        return *this;
    }
    #endif
    
    bool operator<(const ActionAndHowManyTimes & other) const {
        
        if (!alreadyInThePlan && other.alreadyInThePlan) return true;
        if (alreadyInThePlan && !other.alreadyInThePlan) return false;
        
        if (actID < other.actID) return true;
        if (actID > other.actID) return false;
        
        if (ts < other.ts) return true;
        if (ts > other.ts) return false;
        
        return false;
    }
};

/** @brief A gradient effect of an action already in the plan */
struct DelayedGradientDescriptor : public ActionAndHowManyTimes {
   
    /** @brief Variable--Value pair for a gradient effect */
    pair<int,double> gradientEffect;
    
    DelayedGradientDescriptor(const int & act, const VAL::time_spec & timeSpec, const double & dmax, const pair<int,double> & effect)
        : ActionAndHowManyTimes(act, timeSpec, 1, dmax, dmax, true), gradientEffect(effect) {
    }
    
};


/** @brief Struct to represent a fact layer, during RPG expansion. */
struct FactLayerEntry {
    
    pair<set<int>, set<int> > * endOfJustApplied;
    
    /** @brief Propositional facts, new to this fact layer.
     *
     *  Each <code>int</code> corresponds to the literal with the same index
     *  in <code>RPGBuilder::literals</code>.
     */
    list<int> first;
   
    /** @brief Propositional facts, with lower cost in this fact layer.
    *
    *  Each <code>int</code> corresponds to the literal with the same index
    *  in <code>RPGBuilder::literals</code>.
    */
    set<int> firstRepeated;
    
    /** @brief Numeric preconditions, new to this fact layer.
     *
     *  Each <code>int</code> is an index into <code>RPGBuilder::getNumericPreTable()</code>.
     */
    list<int> second;
    
    /** @brief Facts added by Timed Initial Literals.
     *
     *   Each entry is a pair where:
     *   - the first <code>int</code> is the index of the TIL that adds it
     *   - the second <code>int</code> is the fact added, an index into <code>RPGBuilder::literals</code>.
     */
    list<pair<int, int> > TILs;
    
    /** @brief  Facts deleted by Timed Initial Literals.
     *
     *  Each <code>int</code> corresponds to the literal with the same index
     *  in <code>RPGBuilder::literals</code>.
     */
    list<int> negativeTILs;

    /** @brief Gradient effects that finish at this layer.
     *
     *  If an action is k-shot, then any gradient effects it has finish
     *  after it has been executed k-times in sequence.  This map records
     *  the decrease in gradient effects that occur at this point in the
     *  TRPG.  Each entry is a pair, where:
     *  - the first entry, an <code>int</code>, records the variable upon which the gradient is to change;
     *  - the second entry, a <code>double</code>, records the (cumulative) effect upon that variable that
     *    ceases at this layer.
     */
    map<int,double> gradientFinishes;
    
#ifdef POPF3ANALYSIS
    
    list<int> literalGoalsWeMustHaveByNow;
    list<int> numericGoalsWeMustHaveByNow;
    
#endif
    
    /** @brief  Default constructor - all the member variables are initialised to empty. */
    FactLayerEntry() : endOfJustApplied(0) {};
};
    
    
/** @brief A class representing the fluent layers in the TRPG */
class FluentLayers {

public:
    
    /** @brief A record of the assignment effect giving the maximum value of a variable in a layer.
     *
     *  During graph expansion, if the maximum value of a variable was attained through 
     *  an assignment effect, the details of that assignment are stored in an object of
     *  this type to allow them to be recalled and used, if necessary, during solution
     *  extraction.
     */
    struct RecordedAssignment {
        
        /** @brief The snap-action with the assignment effect. */
        const ActionAndHowManyTimes * act;
        
        /** @brief The effect itself, an index into <code>RPGBuilder::getNumericEff()</code>. */
        int eff;
        
        /** @brief Whether the right-hand side of the effect should be maximised or minimised.
         *
         *  If this variable is set to <code>true</code>, the maximum fluent values from the preceding
         *  fact layer should be used when evaluating the value this effect assigns to
         *  the variable it effects.  If set to <code>false</code>, the minimum fluent values
         *  should be used.
         */
        bool maximiseEffect;
        
        /** @brief Default constructor - sets <code>eff=-1</code>. */
        RecordedAssignment()
            : act(0), eff(-1), maximiseEffect(true) {
        }
        
        /** @brief Construct an object to represent an assignment effect.
         *
         *  @param a  The action with the effect
         *  @param e  The effect itself, an index into <code>RPGBuilder::getNumericEff()</code>.
         *  @param m  Whether the effect should be maximised, or minimised.  @see maximiseEffect
         */
        RecordedAssignment(const ActionAndHowManyTimes * const a, const int & e, const bool & m)
            : act(a), eff(e), maximiseEffect(m) {
        }
    };
    
    /** @brief A single fluent layer in the TRPG. @see FluentLayers */
    class FluentLayerEntry {

    protected:
        
        /** @brief The number of PNEs defined in the problem, from <code>RPGBuilder::getPNECount()</code>. */
        static int pneCount;
        friend class FluentLayers;
        
        /** @brief The layer should only be used to dictate input values to effects.
         *
         * When building the TRPG, the fluent layer epsilon before a new action layer needs
         * to be defined, purely to be able to give inputs to the RHS of the effect expressions
         * of those actions.  In this case, we can skip initialising certain data structures,
         * as no new effects appear in this layer.
         *
         * - If this variable is set to <code>true</code>, then <code>assignmentEffectThatGaveThisValue</code>
         *   and <code>effectsThatIncreasedThisVariable</code> are not initialised.
         * - Otherwise, they are resized to contain one entry per variable.
         */
        bool onlyUseLayerForEffectMagnitudes;
        
        /** @brief The maximum fluent values in the current layer.
         *
         *  - Entries <code>[0..pneCount]</code> store the maximum value of each PNE
         *  - Entries <code>[pneCount..(pneCount-1)]</code> store the negation of the minimum value of each PNE
         *  - Entries from <code>2 * pneCount</code> onwards store the maximum values of artificial variables.
         *
         *  @see RPGBuilder::ArtificialVariable
         */
        vector<double> internalValues;
        
        /** @brief The maximum gradients acting on each fluent in the current layer. */
        vector<double> gradients;
        
        /** @brief The variables with non-zero entries in <code>gradients</code>. 
         *
         *  The keys of the maps denote variables with non-zero entries.  For artificial
         *  variables, the corresponding values denote how many of the variables upon
         *  which the AV depends have non-zero entries.
         */
        map<int,int> nonZeroGradients;
        
        /** @brief If the recorded value of a variable was due to an assignment effect, note this here.
          *
          * This vector is used during solution extraction: if a precondition became true based
          * on the variable values in this layer, then if it was by assignment, we only want to
          * apply one assignment effect, rather than one or more increase effects.  An entry
          * of -1 denotes that the indexed variable wasn't assigned to; otherwise, an entry
          * <code>i</code> denotes that <code>RPGBuilder::getNumericEff()[i]</code> was used.
          */         
        vector<RecordedAssignment*> assignmentEffectThatGaveThisValue;
        
        /** @brief The effects immediately prior to this layer that increased the value of each variable. */
        vector<set<int>*> effectsThatIncreasedThisVariable;
        
        list<set<int>* > setGC;
        list<RecordedAssignment*> assignmentGC;
        
        /** @brief The instantaneous effects available directly before this layer.
         *
         *  - The keys are indices into <code>RPGBuilder::getNumericEff()</code>
         *  - The value corresponding to each key lists the available actions with this effect.
         */
        map<int, list<const ActionAndHowManyTimes*> > instantaneousEffectsDirectlyBeforeThisLayer;
        
        /** @brief The integrated effects available directly before this layer.
         *
         *  The keys in the map are the variables upon which the integrated effects act, and are indices
         *  into <code>RPGBuilder::pnes</code>.  The corresponding maps consist of pairs, where:
         *  - the first entry, a <code>double</code>, corresponds to the magnitude of the effect
         *  - the second entry lists the actions with this effect.
         */
        map<int, map<double, list<const ActionAndHowManyTimes*> > > integratedEffectsDirectlyBeforeThisLayer;
        
        /** @brief The gradient effects available directly before this layer.
         *
         *  The keys in the map are the variables upon which the gradient effects act, and are indices
         *  into <code>RPGBuilder::pnes</code>.  The corresponding maps consist of pairs, where:
         *  - the first entry, a <code>double</code>, corresponds to the magnitude of the effect
         *  - the second entry lists the actions with this effect.
         */
        map<int, map<double, list<const ActionAndHowManyTimes*> > > gradientEffectsDirectlyBeforeThisLayer;

        /** @brief The instantaneous effects to revisit in the action layer prior to this fact layer.
         *         
         *  If an effect depends on a variable's value, then if that variable's value changes,
         *  the effect needs to be reconsidered, as new preconditions may becomes satisfied.
         *  Or, if an assignment effect changes the value of a variable, increase/decrease
         *  effects upon it then need to be re-applied, to give the new upper/lower bounds
         *  after that point.
         *
         *  The entries in this set record such effects that need to be reconsidered.  Each
         *  <code>int</code> is an index into <code>RPGBuilder::getNumericEff()</code>.
         */
        set<int> instaneousEffectsToRevisitAfterThisLayer;
        
        /** The variables upon which the integrated/gradient effects need to be reconsidered
         *  in the action layer prior to this layer.
         *
         *  If an assignment effect changes the value of a variable, increase/decrease
         *  effects upon it then need to be re-applied, to give the new upper/lower bounds
         *  after that point.  The entries in this set record variables on which assignment
         *  effects have added, leading to the variable bounds needing to be recalculated.         
         */
        set<int> integratedOrGradientEffectsToRevisitAfterThisLayer;
        
        /** @brief Protected constructor - only used for testing if a layer exists. */
        FluentLayerEntry() {  
            onlyUseLayerForEffectMagnitudes = true;
        }
        
        /** @brief Supply the details of a fluent layer.
         *
         *  If a new fluent layer needs to be created, this function is used to define
         *  the values within it, based on the previous fluent layer and the time
         *  that has elapsed since then.
         *
         *  @param f                 The previous fluent layer
         *  @param timeDifference    The amount of time elapsed since then
         *  @param doApplyGradients  If <code>true</code>, apply the consequences of the gradients active
         *                           between fact layer <code>f</code> and the one constructed.
         */
        void supplyDetails(const FluentLayerEntry & f, const double & timeDifference, const bool & doApplyGradients, const bool & ignorableLayer) {
            internalValues = f.internalValues;
            gradients = f.gradients;
            
            if (ignorableLayer) {
                onlyUseLayerForEffectMagnitudes = true;
            } else {
                onlyUseLayerForEffectMagnitudes = false;
                assignmentEffectThatGaveThisValue.resize(internalValues.size(),0);
                effectsThatIncreasedThisVariable.resize(internalValues.size(),0);
            }
            
            if (timeDifference < 0.000001) return;
            
            if (doApplyGradients) {
                applyGradients(timeDifference);
            }
        }
        
        /** @brief Called when the sum gradient acting on a (non-artificial) variable becomes zero.
         *
         *  If the gradient on a non-artificial variable becomes zero, the gradients of the AVs
         *  depending on it need to be checked to see if they now have zero gradients too, i.e.
         *  have no remaining non-zero components.
         *
         *  @param  var  The variable whose gradient has become zero
         */
        void gradientBecomesZeroOn(const int & var) {
            map<int,int>::iterator delItr = nonZeroGradients.find(var);
            assert(delItr != nonZeroGradients.end());
            nonZeroGradients.erase(delItr);
            
            const list<int> & deps = RPGBuilder::getVariableDependencies(var);
            
            list<int>::const_iterator depItr = deps.begin();
            const list<int>::const_iterator depEnd = deps.end();
            
            for (; depItr != depEnd; ++depItr) {
                delItr = nonZeroGradients.find(*depItr);
                
                if (!(--(delItr->second))) {
                    nonZeroGradients.erase(delItr);
                }
            }            
        }
        
        /** @brief Called when the sum gradient acting on a (non-artificial) variable becomes non-zero.
         *
         *  If the gradient on a non-artificial variable becomes non-zero, the gradients of the AVs
         *  depending on it need to be checked to see if they now have non-zero gradients too, i.e.
         *  if this is the first non-zero components.
         *
         *  @param  var  The variable whose gradient has become non-zero
         */                
        void gradientBecomesNonZeroOn(const int & var) {
            static pair<int,int> pairWithZero(0,0);
            pair<map<int,int>::iterator,bool> insPair = nonZeroGradients.insert(make_pair(var,1));
            
            assert(insPair.second);
            
            map<int,int>::iterator insItr = nonZeroGradients.end();
            
            const list<int> & deps = RPGBuilder::getVariableDependencies(var);
            
            list<int>::const_iterator depItr = deps.begin();
            const list<int>::const_iterator depEnd = deps.end();
            
            for (; depItr != depEnd; ++depItr) {
                pairWithZero.first = *depItr;
                insItr = nonZeroGradients.insert(insItr, pairWithZero);
                ++(insItr->second);
            }            
        }
        
                
        /** @brief Start a gradient effect on the given variable.
         *
         *  - If <code>val > 0</code>, then the gradient on the upper bound on <code>var</code> is increased.
         *  - If <code>val < 0</code>, then the gradient on the negative lower bound on <code>var</code> is increased.
         *
         *  In both cases, the index of the affected variable is added to the set <code>varChanged</code>, and
         *  the value of the affected variable is increased by epsilon times the gradient.
         */
        void startGradient(const int & var, const double & val, set<int> & varChanged) {
            if (val > 0) {
                double & alter = gradients[var];
                const bool previouslyZero = (fabs(alter) < 0.0000001);
                alter += val;
                varChanged.insert(var);
                if (fabs(alter) < 0.0000001) {
                    if (!previouslyZero) gradientBecomesZeroOn(var);
                } else {
                    if (previouslyZero) gradientBecomesNonZeroOn(var);
                }
            } else if (val < 0) {
                double & alter = gradients[var + pneCount];
                const bool previouslyZero = (fabs(alter) < 0.0000001);
                alter -= val;
                varChanged.insert(var + pneCount);
                if (fabs(alter) < 0.0000001) {
                    if (!previouslyZero) gradientBecomesZeroOn(var + pneCount);
                } else {
                    if (previouslyZero) gradientBecomesNonZeroOn(var + pneCount);
                }
            }
        }
        
    public:

        /** @brief Create a fluent layer, initialised with the given variable bounds. */
        FluentLayerEntry(const vector<double> & v)
            : internalValues(v), gradients(v.size(), 0.0) {        
            pneCount = RPGBuilder::getPNECount();
            onlyUseLayerForEffectMagnitudes = false;
        }
        
        /** @brief Create a fluent layer, offset with the given time from the fluent layer given. */
        FluentLayerEntry(const FluentLayerEntry & f, const double & timeDifference, const bool & doApplyGradients, const bool & ignorableLayer) {
            supplyDetails(f,timeDifference, doApplyGradients, ignorableLayer);            
        }
        
        /** @brief Destructor - garbage-collects internal data. */
        ~FluentLayerEntry() {
            
            if (onlyUseLayerForEffectMagnitudes) {
                return;
            }
            
            list<set<int>* >::const_iterator sgcItr = setGC.begin();
            const list<set<int>* >::const_iterator sgcEnd = setGC.end();
            
            for (; sgcItr != sgcEnd; ++sgcItr) {
                delete *sgcItr;
            }
            
            
            list<RecordedAssignment*>::const_iterator agcItr = assignmentGC.begin();
            const list<RecordedAssignment*>::const_iterator agcEnd = assignmentGC.end();
            
            for (; agcItr != agcEnd; ++agcItr) {
                delete *agcItr;
            }
        }
        
        /** @brief Return a const reference to the variable values in this layer. */
        const vector<double> & values() const {
            return internalValues;        
        }
        
        /** @brief Return a const reference to the variable gradients in this layer. */
        const vector<double> & getGradients() const {
            return gradients;        
        }
        
        
        /** @brief Return a non-const reference to the variable values in this layer. */
        vector<double> & writeableValues() {
            return internalValues;
        }

        /** @brief Return <code>true</code> if this is not a dummy intermediate layer. */
        bool initialised() const {
            return (effectsThatIncreasedThisVariable.size() == internalValues.size());
        }

        /** @brief Apply the gradients recorded in this fluent layer to the variables.
         *
         *  After recording the snap-actions in the layer preceding a new fact
         *  layer, this function is used to update the variable values to reflect
         *  the gradient-induced change between the previous fact layer, and this
         *  one.
         *
         *  @param timeDifference  The length of time elapsed between the previous
         *                         fluent layer and this one.
         */
        void applyGradients(const double & timeDifference) {
            const int size = internalValues.size();
            
            for (int v = 0; v < size; ++v) {
                internalValues[v] += gradients[v] * timeDifference;            
            }            
        }
        
        void keepAssignmentIfBetter(const ActionAndHowManyTimes* act, const int & effID, const bool & maxEffect, const int & var, const double & val, set<int> & varChanged) {
            if (val > internalValues[var]) {
                internalValues[var] = val;
                varChanged.insert(var);
                
                RecordedAssignment *& ra = assignmentEffectThatGaveThisValue[var];
                if (ra) {
                    ra->act = act;
                    ra->eff = effID;
                    ra->maximiseEffect = maxEffect;
                } else {                   
                    ra = new RecordedAssignment(act, effID, maxEffect);
                    assignmentGC.push_back(ra);
                }
                set<int> * const sa = effectsThatIncreasedThisVariable[var];
                if (sa) {
                    sa->clear();
                }
            }
            
            if (-val > internalValues[var + pneCount]) {
                internalValues[var + pneCount] = val;
                varChanged.insert(var + pneCount);
                
                RecordedAssignment *& ra = assignmentEffectThatGaveThisValue[var + pneCount];
                if (ra) {
                    ra->act = act;
                    ra->eff = effID;
                    ra->maximiseEffect = maxEffect;
                } else {
                    ra = new RecordedAssignment(act, effID, maxEffect);
                    assignmentGC.push_back(ra);
                }
                set<int> * const sa = effectsThatIncreasedThisVariable[var + pneCount];
                if (sa) {
                    sa->clear();
                }
            }
        }
        
        void applyIncrease(const int & effID, const int & var, const double & minVal, const double & maxVal, const int & howManyTimes, set<int> & varChanged) {            
            if (minVal > 0 || maxVal > 0) {
                double val = maxVal;
                if (minVal > val) {
                    val = minVal;
                }
                if (internalValues[var] != DBL_MAX) {
                    if (val == DBL_MAX || howManyTimes == INT_MAX) {
                        internalValues[var] = DBL_MAX;
                    } else {
                        internalValues[var] += val * howManyTimes;
                    }
                    varChanged.insert(var);
                    
                    if (effID != -1) {
                        assert(var >= 0 && var < effectsThatIncreasedThisVariable.size());
                        set<int> *& sa = effectsThatIncreasedThisVariable[var];
                        if (!sa) {
                            setGC.push_back(sa = new set<int>());
                        }
                        sa->insert(effID);
                    }
                }
            }
            
            if (minVal < 0 || maxVal < 0) {
                double val = minVal;
                if (maxVal < val) {
                    val = maxVal;
                }
                if (internalValues[var + pneCount] != DBL_MAX) {
                    if (val == -DBL_MAX || howManyTimes == INT_MAX) {
                        internalValues[var + pneCount] = DBL_MAX;
                    } else {
                        internalValues[var + pneCount] -= val * howManyTimes;
                    }
                    varChanged.insert(var + pneCount);
                    
                    if (effID != -1) {
                        set<int> *& sa = effectsThatIncreasedThisVariable[var + pneCount];
                        if (!sa) {
                            setGC.push_back(sa = new set<int>());
                        }
                        sa->insert(effID);
                    }
                }
            }
        }


        /** @brief Stop a gradient effect on the given variable.
         *
         *  The gradient on <code>var</code> is decreased by <code>val</code>.
         *
         *  @param var  The variable for which the gradient changes (either a positive or negative variable)
         *  @param val  The amount by which to decrease the active gradient.
         *
         */
        void stopGradient(const int & var, const double & val) {
            gradients[var] -= val;

        }
                
        /** @brief Recalculate the values of the artificial variables. */
        template <typename T>
        void recalculateAVs(T & itr, const T & itrEnd) {
            for (; itr != itrEnd; ++itr) {
                internalValues[*itr] = RPGBuilder::getArtificialVariable(*itr).evaluate(internalValues);
            }
        }

        /** @brief Recalculate the gradients  of the artificial variables. */
        template <typename T>
        void recalculateAVGradients(T & itr, const T & itrEnd) {
            for (; itr != itrEnd; ++itr) {
                gradients[*itr] = RPGBuilder::getArtificialVariable(*itr).evaluateGradient(gradients);
            }
        }

        
        void markEffectsToRevisit(const set<int> & toRevisit, const vector<double> & maxNeeded) {
            set<int>::const_iterator effItr = toRevisit.begin();
            const set<int>::const_iterator effEnd = toRevisit.end();
            
            for (; effItr != effEnd; ++effItr) {
                if (instaneousEffectsToRevisitAfterThisLayer.find(*effItr) != instaneousEffectsToRevisitAfterThisLayer.end()) continue;
                
                const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*effItr];
                if (maxNeeded[currEff.fluentIndex] > internalValues[currEff.fluentIndex]) {
                    instaneousEffectsToRevisitAfterThisLayer.insert(*effItr);
                    continue;
                }
                
                list<int> & dependencies = RPGBuilder::getVariableDependencies(currEff.fluentIndex);
                
                list<int>::const_iterator depItr = dependencies.begin();
                const list<int>::const_iterator depEnd = dependencies.end();
                
                for (; depItr != depEnd; ++depItr) {
                    if (maxNeeded[*depItr] > internalValues[*depItr]) {
                        instaneousEffectsToRevisitAfterThisLayer.insert(*effItr);
                        break;
                    }
                }
            }
        }
        
        void markIntegratedOrGradientEffectVariableToRevisit(const int & toRevisit, const vector<double> & maxNeeded) {
            if (integratedOrGradientEffectsToRevisitAfterThisLayer.find(toRevisit) != integratedOrGradientEffectsToRevisitAfterThisLayer.end()) return;
            
            if (maxNeeded[toRevisit] > internalValues[toRevisit]) {
                integratedOrGradientEffectsToRevisitAfterThisLayer.insert(toRevisit);
                return;
            }
            
            list<int> & dependencies = RPGBuilder::getVariableDependencies(toRevisit);
                
            list<int>::const_iterator depItr = dependencies.begin();
            const list<int>::const_iterator depEnd = dependencies.end();
            
            for (; depItr != depEnd; ++depItr) {
                if (maxNeeded[*depItr] > internalValues[*depItr]) {
                    integratedOrGradientEffectsToRevisitAfterThisLayer.insert(toRevisit);
                    return;
                }
            }
            
        }
        
        map<int, list<const ActionAndHowManyTimes*> > & getInstantaneousEffectsDirectlyBeforeThisLayer() {
            return instantaneousEffectsDirectlyBeforeThisLayer;
        }
        
        map<int, map<double, list<const ActionAndHowManyTimes*> > > & getIntegratedEffectsDirectlyBeforeThisLayer() {
            return integratedEffectsDirectlyBeforeThisLayer;
        }
        
        map<int, map<double, list<const ActionAndHowManyTimes*> > > & getGradientEffectsDirectlyBeforeThisLayer() {
            return gradientEffectsDirectlyBeforeThisLayer;
        }
                        
        
        const set<int> & getInstantaneousEffectsToRevisitAfterThisLayer() const {
            return instaneousEffectsToRevisitAfterThisLayer;
        }
        
        const set<int> & getIntegratedOrGradientEffectsToRevisitAfterThisLayer() const {
            return integratedOrGradientEffectsToRevisitAfterThisLayer;
        }
        
        
        
        /** @brief Get the assignment effect used to give this value of v. */
        const RecordedAssignment * assignmentAppliedToVariable(const int & v) const {
            if (onlyUseLayerForEffectMagnitudes) {
                return 0;
            }
            return assignmentEffectThatGaveThisValue[v];
        }
        
        const set<int> * getInstantaneousEffectsThatIncreasedVariable(const int & v) const {
            if (onlyUseLayerForEffectMagnitudes) {
                return 0;
            }
            return effectsThatIncreasedThisVariable[v];
        }
        
        double willBecomeSatisfiedAfterDelay(const RPGBuilder::RPGNumericPrecondition & currPre) const {
            
            static const bool debug = false;
            
            const double currV = internalValues[currPre.LHSVariable];
            const double currG = gradients[currPre.LHSVariable];
            
            if (currPre.op == VAL::E_GREATEQ) {
                if (currV >= currPre.RHSConstant) {
                    // is true right now
                    if (debug) {
                        cout << "Is true right now\n";
                    }
                    return 0.0;
                }
                if (currG == 0) {
                    if (debug) {
                        cout << "No gradient, cannot become true yet\n";
                    } 
                    // no gradient, cannot become true
                    return DBL_MAX;
                }
                if (debug) {
                    cout << "Current LHS value is " << currV << ", RHS is " << currPre.RHSConstant << ", so with a gradient of " << currG << " that will mean a delay of " << ((currPre.RHSConstant - currV) / currG) << endl;
                }
                return ((currPre.RHSConstant - currV) / currG);
            } else {
                if (currV > currPre.RHSConstant) {
                    if (debug) {
                        cout << "Is true right now\n";
                    }                    
                    // is true right now
                    return 0.0;
                }
                if (currG == 0) {
                    if (debug) {
                        cout << "No gradient, cannot become true yet\n";
                    }                                        
                    // no gradient, cannot become true
                    return DBL_MAX;
                }
                if (debug) {
                    cout << "Current LHS value is " << currV << ", RHS is " << currPre.RHSConstant << ", so with a gradient of " << currG << " that will mean a delay of " << ((currPre.RHSConstant - currV) / currG) + EPSILON << endl;
                }                                        
                                                    
                // as it's v > c, allow epsilon extra to make sure it definitely exceeds c
                return ((currPre.RHSConstant - currV) / currG) + EPSILON;
            }
        }
    };
    
protected:

    /** @brief Fluent layers, to be garbage collected upon destruction. */
    list<FluentLayerEntry*> layerGC;
        
    /** @brief The type of the map used to hold the variable and gradient values at each point in time across the RPG. */
    typedef map<double, FluentLayerEntry*, EpsilonComp> LayerMap;
    
    /** @brief The values of the numeric variables, and the active gradients, at each point in time across the RPG. */
    LayerMap layers;
    
    inline FluentLayerEntry * newFluentLayer() {
        static FluentLayerEntry * newLayer;
        newLayer = new FluentLayerEntry();
        layerGC.push_back(newLayer);
        return newLayer;
    }
    
    inline FluentLayerEntry * newFluentLayer(const vector<double> & values) {
        static FluentLayerEntry * newLayer;
        newLayer = new FluentLayerEntry(values);
        layerGC.push_back(newLayer);
        return newLayer;
    }
    
    inline FluentLayerEntry * newFluentLayerEntry(const FluentLayerEntry * const previousFL, const double & timeDifference, const bool & applyGradients, const bool & ignorableLayer) {
        static FluentLayerEntry * newLayer;
        newLayer = new FluentLayerEntry(*previousFL, timeDifference, applyGradients, ignorableLayer);
        layerGC.push_back(newLayer);
        return newLayer;
    }
    
    /** @brief Return the fluent layer the desired time after the seed layer given.
     *
     *  If this is the first request for such a layer, the details of the supplied 
     *  layer are copied across, updating for any gradient effects.  Otherwise,
     *  the previous layer created for that time is used.
     *
     *  @param startAt         An iterator to <code>FluentLayers::layers</code>, to seed layer creation
     *  @param timeDifference  The temporal separation between the seed layer and the new one
     *
     *  @return An iterator to <code>FluentLayers::layers</code>, pointing to the new layer.
     */
    LayerMap::iterator addLayer(LayerMap::const_iterator & startAt, const double & timeDifference) {
        
        const pair<LayerMap::iterator,bool> insPair = layers.insert(make_pair(startAt->first + timeDifference, newFluentLayer()));
        
        if (insPair.second) {
            insPair.first->second->supplyDetails(*(startAt->second), timeDifference, false, false);
        }
        
        if (debug) {
            if (insPair.first->second->initialised()) {
                cout << COLOUR_yellow << "Layer at " << startAt->first + timeDifference << " is initialised\n";
            } else {
                cout << COLOUR_light_magenta << "Layer at " << startAt->first + timeDifference << " is not initialised\n";
            }
        }
        
        return insPair.first;
    }
    
    /** @brief Record the effects of an action's execution.
     *
     *  When adding an action to the relaxed plan, it may have numeric effects other than those intended.
     *  This function records all an action's effects, so that if it turns out a suitable effect has
     *  already been applied when considering whether another precondition is satisfied, we don't
     *  add redundant actions to the relaxed plan.
     *
     *  @param  toUse              The action to use;
     *  @param  howManyTimes       How many times it was added to the relaxed plan;
     *  @param  effectInputValues  The fluent layer immediately prior to the action being applied,
     *                             giving the variable values to take as the inputs to its effects.
     */
    void recordSideEffects(const ActionAndHowManyTimes * const toUse, const int & howManyTimes,
                           const vector<double> & effectInputValues) {
        
        /*const list<int> & numericEffs = (toUse->ts == VAL::E_AT_START ? RPGBuilder::getStartEffNumerics()[toUse->actID]
                                                                      : RPGBuilder::getEndEffNumerics()[toUse->actID]);
        list<int>::const_iterator effItr = numericEffs.begin();
        const list<int>::const_iterator effEnd = numericEffs.end();
        
        for (; effItr != effEnd; ++effItr) {
            const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*effItr];
            
            const double maxRHS = currEff.evaluate(effectInputValues, toUse->minDur, toUse->maxDur);
            const double minRHS = currEff.evaluateMin(effectInputValues, toUse->minDur, toUse->maxDur);
                        
            for (int pass = 0; pass < 2; ++pass) {
                const int v = currEff.fluentIndex + (pass ? FluentLayerEntry::pneCount : 0);
                
                for (int rhsC = 0; rhsC < 2; ++rhsC) {
                    
                    double rhs = (rhsC ? maxRHS : minRHS);                                             
                
                    if (pass) {
                        if (rhs >= 0) continue;
                        rhs = -rhs;
                    } else {
                        if (rhs <= 0) continue;
                    }
                    
                    const pair<map<int,double>::iterator,bool> insPair = alreadyAchieved.insert(make_pair(v,rhs));
                    
                    if (!insPair.second) {
                        if (currEff.isAssignment) {
                            if (rhs > insPair.first->second) {
                                insPair.first->second = rhs;
                            }
                        } else {
                            if (insPair.first->second != DBL_MAX) {
                                if (rhs == DBL_MAX || howManyTimes == INT_MAX) {
                                    insPair.first->second = DBL_MAX;
                                } else {
                                    insPair.first->second += rhs * howManyTimes;
                                }
                            }
                        }
                    }
                }
            }            
        }*/
    }
    
    /**
     *  If an effect depends on a variable's value, we need to revisit that effect
     *  if that value changes.  Thus, for each variable, we store a set of which 
     *  effect IDs depend upon it.
     */
    vector<set<int> > revisitInstantaneousEffectIfVariableValueChanges;
    
    /**
    *  If a variable is assigned a new, better, value, we need to then reapply all suitable
    *  increase effects.  Thus, for each variable, we store a set of which
    *  increase effect IDs act upon it.
    */
    vector<set<int> > revisitInstantaneousEffectIfVariableIsAssignedTo;
    
    /** For garbage collection */
    list<ActionAndHowManyTimes> gc;
    
    /** @brief For each effect ID, the actions in the planning graph that have ever had that effect.
     */
    vector<list<ActionAndHowManyTimes*> > actionsThatHaveHadThisEffect;
    
    /** @brief For each variable ID, the actions in the planning graph that have ever had an integrated effect of the magnitude given.
     */
    vector<map<double, list<ActionAndHowManyTimes*> > > actionsThatHaveHadThisIntegratedEffect;
    
    /** @brief For each variable ID, the actions in the planning graph that have ever had a gradient effect of the magnitude given.
    */
    vector<map<double, list<ActionAndHowManyTimes*> > > actionsThatHaveHadThisGradientEffect;
    
    /** @brief The set of facts true at the current layer, due to gradients having been started in the past.
     *
     *  Each entry is an index into <code>RPGBuilder::getNumericPreTable()</code>.
     */
    set<int> gradientsLeadToTheseFactsNowBeingTrue;
    
    LayerMap::const_iterator effectInputs;
    LayerMap::iterator effectsAffectThisLayer;
    LayerMap::iterator layerWithEffectsToBeRevisited;
    /** @brief For convenience - for each effect ID, the actions with that effect in the most recent action layer. */
    map<int, list<const ActionAndHowManyTimes*> > * mostRecentInstantaneousEffects;
    map<int, map<double, list<const ActionAndHowManyTimes*> > > * mostRecentIntegratedEffects;
    map<int, map<double, list<const ActionAndHowManyTimes*> > > * mostRecentGradientEffects;
    
    /** @brief The time at which preconditions become true, due to active gradients.
     *
     *  - the keys of the map are time-stamps
     *  - the set of associated values contains the preconditions that become true at that time
     *    (each being an index into <code>RPGBuilder::getNumericPreTable()</code>).
     */
    map<double, set<int> > preconditionsBecomingTrueAtTime;
    
    
    /** @brief Typedef used to record precondition satisfaction forecasts.
    *
    *  For each precondition in <code>RPGBuilder::getNumericPreTable()</code>, objects of this type
    *  are used to record details about the layer at which it is currently set to become true due to active
    *  gradients.  Each entry is a pair:
    *   - an iterator to the relevant layer in <code>preconditionsBecomingTrueAtTime</code>, or
    *     <code>preconditionsBecomingTrueAtTime.end()</code> if it is already true, or not
    *     yet forecast to become true;
    *   - an iterator into the set of preconditions within that iterator (undefined if
    *     the precondition is not yet forecast to become true).
    */
    typedef pair<map<double, set<int> >::iterator, set<int>::iterator> PreconditionToSatisfactionLayerPair;
    
    /** @brief The current layer at which a precondition has been forecast to become true.*/        
    vector<PreconditionToSatisfactionLayerPair> layerAtWhichPreconditionBecomesTrue;
    
    bool nowExtracting;
    LayerMap::iterator currentSubgoalLayer;

    // /** @brief Record effects of actions as they are added to the plan. */
    // map<int, double> alreadyAchieved;
    
    /** @brief The earliest point at which an effect can act upon/a precondition can refer to a given variable.
     *
     *  This is a pointer to <code>RPGHeuristic::Private::earliestNumericPOTimes</code>.
     */
    const vector<double> * earliestNumericPOTimes;
    
    const bool debug;
    
    /** @brief Common initialisation code when recording effects.
     *
     *  This function ensures that:
     *  - a fluent layer has been created for the specified time
     *  - the effects recorded at the last layer were applied
     *  - the class member variables point to the data structures inside the new layer
     *
     *  @param effectAppearsInLayer  The timestamp of the new fluent layer
     */
    void aboutToRecordAnEffect(const double & effectAppearsInLayer) {
        if (effectsAffectThisLayer == layers.end()) {
            
            effectInputs = layers.find(effectAppearsInLayer - EPSILON);
            #ifndef NDEBUG
            if (effectInputs == layers.end()) {
                cout << std::setprecision(8) << std::fixed;
                cout << "Internal error: current effect layer is " << effectAppearsInLayer << ", but no fluent values are defined for " << effectAppearsInLayer - EPSILON << endl;
                exit(1);
            }
            #endif
            
            effectsAffectThisLayer = addLayer(effectInputs, EPSILON);
            mostRecentInstantaneousEffects = &(effectsAffectThisLayer->second->instantaneousEffectsDirectlyBeforeThisLayer);
            mostRecentIntegratedEffects = &(effectsAffectThisLayer->second->integratedEffectsDirectlyBeforeThisLayer);
            mostRecentGradientEffects = &(effectsAffectThisLayer->second->gradientEffectsDirectlyBeforeThisLayer);
            
            if (debug) {
                cout << "New effect layer " << effectAppearsInLayer << " created\n";
            }
        } else {
            #ifndef NDEBUG
            if (fabs(effectAppearsInLayer - effectsAffectThisLayer->first) > 0.0005) {
                cout << std::setprecision(8) << std::fixed;
                cout << "Internal error: current effect layer is " << effectsAffectThisLayer->first << ", but asked for an effect to go in " << effectAppearsInLayer << endl;
                cout << "Absolute difference calculated to be " << fabs(effectAppearsInLayer - effectsAffectThisLayer->first) << endl;
                exit(1);
            }
            #endif
        }
        
    }
    
public:

    /** @brief Placeholder constructor.  The real initialisation is done in <code>setFactLayerZero()</code>.
     */
    FluentLayers() : debug(Globals::globalVerbosity & 64 || Globals::globalVerbosity & 128) {
        FluentLayerEntry::pneCount = RPGBuilder::getPNECount();
        
        nowExtracting = false;
    }
    
    ~FluentLayers() {
        list<FluentLayerEntry*>::const_iterator delItr = layerGC.begin();
        const list<FluentLayerEntry*>::const_iterator delEnd = layerGC.end();
        for (; delItr != delEnd; ++delItr) {
            delete *delItr;
        }
    }
    
    /** @brief Specify the values of the variables in fact layer zero.
     * 
     * Do not call this unless the object has not yet had any layers defined.
     *
     * @param values  The values of each variable (including negative and artificial variables)
     */
    void setFactLayerZero(const vector<double> & values, const vector<double> & cannotReferToVariableUntilTime) {
        assert(layers.empty());
        earliestNumericPOTimes = &(cannotReferToVariableUntilTime);
        layers.insert(make_pair(0.0, newFluentLayer(values)));
        revisitInstantaneousEffectIfVariableValueChanges.resize(values.size());
        revisitInstantaneousEffectIfVariableIsAssignedTo.resize(values.size());
        actionsThatHaveHadThisEffect.resize(RPGBuilder::getNumericEff().size());
        actionsThatHaveHadThisIntegratedEffect.resize(FluentLayerEntry::pneCount);
        actionsThatHaveHadThisGradientEffect.resize(FluentLayerEntry::pneCount);
        effectsAffectThisLayer = layers.end();
        layerWithEffectsToBeRevisited = layers.end();
        
        const int lptSize = RPGBuilder::getNumericPreTable().size();
        layerAtWhichPreconditionBecomesTrue.resize(lptSize);
        
        for (int p = 0; p < lptSize; ++p) {
            layerAtWhichPreconditionBecomesTrue[p].first = preconditionsBecomingTrueAtTime.end();
            // for now, the second entry of each pair is undefined - on the plus side
            // this means valgrind will scream if we try to use them
        }
            
    }
    
    /** @brief Function to access a writeable version of the fluents in fact layer zero.
     * 
     *  This is used to update for the CTS effects of currently executing actions, and find which facts are
     *  true in the state being evaluated.  Otherwise, it probably shouldn't be used.
     *
     *  @return A reference to the vector of fluent values at fact layer zero
     */
    vector<double> & borrowFactLayerZeroValues() {
        return layers.begin()->second->writeableValues();
    }
    
    /** @brief Return the next layer that is worth visiting purely on the merits of active effects.
     * 
     * This can be due to either continuous effects, or effects that need revisiting
     * as the values of the variables upon which they depend has changed.
     *
     * @param isToRevisitEffects  If set to <code>true</code>, then revisit the effects in the next layer
     * @param isDueToGradients    If set to <code>true</code>, then the gradients will cause facts to become
     *                            true EPSILON after the time returned.
     * @return  A pair, where:
     *          -  the first value is the timestamp of a layer to visit (or <code>DBL_MAX</code> if none is needed)
     *           - the secound value is <code>true</code> if this is epsilon after the previous layer (this is
     *             used to prevent rounding errors)
     */
    pair<double,bool> nextLayerWouldBeAt(bool & isToRevisitEffects, bool & isDueToGradients) {
        
        const map<double,set<int> >::const_iterator ptItr = preconditionsBecomingTrueAtTime.begin();
        
        // if we need to revisit effects whose input values have changed, do that right away
        if (layerWithEffectsToBeRevisited != layers.end()) {
            isToRevisitEffects = true;
            if (ptItr != preconditionsBecomingTrueAtTime.end()) {
                isDueToGradients = (fabs(ptItr->first - 2 * EPSILON) < 0.00001);
            }
            return make_pair(layerWithEffectsToBeRevisited->first, true);
        }
        
        // otherwise, consider the next point at which a precondition will become satisfied
        
        if (ptItr == preconditionsBecomingTrueAtTime.end()) {
            return make_pair(DBL_MAX,false);
        }

        isDueToGradients = true;

        // set 'is epsilon later' flag (the second entry of the pair) to true if
        // it is near as makes no odds to epsilon in the future
        return make_pair(ptItr->first - EPSILON, ((ptItr->first - 2 * EPSILON) < 0.000001));
        
    }

    /** @brief Create a dummy fluent layer at the time given.
     * 
     *  When advancing by more than epsilon, we need to create a dummy reference layer to provide input
     *  fluent values for the actions in the subsequent action layer in the RPG.
     *
     *  @param ts  The timestamp of the fact layer preceding the new actions to be added
     */
    void createThenIgnore(const double & ts) {
        assert(layerWithEffectsToBeRevisited == layers.end());
        assert(effectsAffectThisLayer == layers.end());
        
        LayerMap::iterator lastEntry = layers.end();
        --lastEntry;
        
        const double timeDifference = ts - lastEntry->first;
        
        if (debug) {
            cout << "Advance beyond previous fluent layer is " << timeDifference << endl;
        }
        if (timeDifference > (EPSILON / 2)) {        
            if (debug) {
                cout << "Made sure that fluent layer " << ts << " exists\n";
            }
            layers.insert(lastEntry, make_pair(ts, newFluentLayerEntry(lastEntry->second, timeDifference, true, true)));        
        } else {
            if (debug) {
                cout << "A fluent layer at " << ts << " already exists\n";
            }
        }
    }

    void recordEffectsThatAreNowToBeRevisited(map<double, FactLayerEntry, EpsilonComp > & factLayers) {
        if (layerWithEffectsToBeRevisited == layers.end()) return;
        
        assert(effectsAffectThisLayer == layers.end());

        effectInputs = layerWithEffectsToBeRevisited;        
        effectsAffectThisLayer = addLayer(effectInputs, EPSILON);
        
        mostRecentInstantaneousEffects = &(effectsAffectThisLayer->second->getInstantaneousEffectsDirectlyBeforeThisLayer());
        mostRecentIntegratedEffects = &(effectsAffectThisLayer->second->getIntegratedEffectsDirectlyBeforeThisLayer());
        mostRecentGradientEffects = &(effectsAffectThisLayer->second->getGradientEffectsDirectlyBeforeThisLayer());
        
        const set<int> & revisit = layerWithEffectsToBeRevisited->second->getInstantaneousEffectsToRevisitAfterThisLayer();

        
        {
            set<int>::const_iterator rvItr = revisit.begin();
            const set<int>::const_iterator rvEnd = revisit.end();
            
            for (; rvItr != rvEnd; ++rvItr) {
                list<const ActionAndHowManyTimes*> & dest = (*mostRecentInstantaneousEffects)[*rvItr];
                dest.insert(dest.end(), actionsThatHaveHadThisEffect[*rvItr].begin(), actionsThatHaveHadThisEffect[*rvItr].end());
            }
        }
        
        const set<int> & revisit2 = layerWithEffectsToBeRevisited->second->getIntegratedOrGradientEffectsToRevisitAfterThisLayer();
        
        {
            for (int pass = 0; pass < 2; ++pass) {
                
                set<int>::const_iterator rv2Itr = revisit2.begin();
                const set<int>::const_iterator rv2End = revisit2.end();
                
                for (; rv2Itr != rv2End; ++rv2Itr) {
                    map<double, list<const ActionAndHowManyTimes*> > & outerdest
                        = (pass ? (*mostRecentGradientEffects)[*rv2Itr] : (*mostRecentIntegratedEffects)[*rv2Itr]);

                    map<double, list<ActionAndHowManyTimes*> > & src
                        = (pass ? actionsThatHaveHadThisGradientEffect[*rv2Itr] : actionsThatHaveHadThisIntegratedEffect[*rv2Itr]);
                                        
                        
                    map<double, list<ActionAndHowManyTimes*> >::iterator effItr = src.begin();
                    const map<double, list<ActionAndHowManyTimes*> >::iterator effEnd = src.end();
                    
                    for (; effItr != effEnd; ++effItr) {
                        
                        list<const ActionAndHowManyTimes*> & dest = outerdest[effItr->first];
                        
                        list<ActionAndHowManyTimes*>::iterator actItr = effItr->second.begin();
                        const list<ActionAndHowManyTimes*>::iterator actEnd = effItr->second.end();                                                
                        
                        for (; actItr != actEnd; ++actItr) {
                            if (!pass) {
                                
                                // for instantaneous effects, copy them in
                                dest.push_back(*actItr);
                            } else {
                                
                                                                
                                
                                if ((*actItr)->gradientsFinishAt == DBL_MAX) {                                    
                                    if ((*actItr)->gradientNeedsStarting) {
                                        gc.push_back(*(*actItr));
                                        
                                        ActionAndHowManyTimes & newEntry = gc.back();                                        
                                        newEntry.gradientNeedsStarting = false;
                                        
                                        *actItr = &newEntry; // from now on, use this version which doesn't trigger an increase of the perpetual gradient
                                    }
                                    
                                    dest.push_back(*actItr);
                                    
                                } else {
                                    gc.push_back(*(*actItr));
                                                                        
                                    ActionAndHowManyTimes & newEntry = gc.back();
                                    
                                    newEntry.gradientNeedsStarting = false;
                                    
                                    const double endAt = ((effectsAffectThisLayer->first - EPSILON)           // the time of the action layer in which the effect began
                                                       + (*actItr)->howManyTimes * ((*actItr)->maxDur));      // plus the maximum sequential time of applications of this action
                                    
                                    if (factLayers.empty() || factLayers.begin()->first > (*actItr)->gradientsFinishAt) {
                                        // if the effect has already lapsed, start it again
                                                                            
                                        newEntry.gradientNeedsStarting = true;
                                        
                                        if (effItr->first > 0) {        
                                            factLayers[endAt].gradientFinishes.insert(make_pair(*rv2Itr,0.0)).first->second += effItr->first;
                                        } else {
                                            factLayers[endAt].gradientFinishes.insert(make_pair(*rv2Itr + FluentLayerEntry::pneCount,0.0)).first->second -= effItr->first;
                                        }
                                        
                                        newEntry.gradientsFinishAt = endAt;
                                        
                                    } else {
                                        
                                        // otherwise, don't start it again; but do delay its end
                                        
                                        if (effItr->first > 0) {        
                                            factLayers[(*actItr)->gradientsFinishAt].gradientFinishes.insert(make_pair(*rv2Itr,0.0)).first->second -= effItr->first;
                                            factLayers[endAt].gradientFinishes.insert(make_pair(*rv2Itr,0.0)).first->second += effItr->first;
                                        } else {
                                            factLayers[(*actItr)->gradientsFinishAt].gradientFinishes.insert(make_pair(*rv2Itr + FluentLayerEntry::pneCount,0.0)).first->second += effItr->first;
                                            factLayers[endAt].gradientFinishes.insert(make_pair(*rv2Itr + FluentLayerEntry::pneCount,0.0)).first->second -= effItr->first;
                                        }
                                        
                                        newEntry.gradientsFinishAt = endAt;
                                    }
                                    
                                                                                                            
                                    *actItr = &newEntry; // from now on, use the version with the most recent endAt record
                                    dest.push_back(*actItr);
                                }
                            }
                        }
                        
                    }                
                }
            }    
        }
                        
    }

    /** @brief Record consequences of active gradients, i.e. new facts becoming true at this time.
     *
     *  Only call this function if it is certain that the time given is the next time
     *  noted for preconditions becoming true.
     *
     *  @param newFactsAppearsInLayer  The next layer in which new facts are to appear,
     *                                 due to active gradients.
     */
    void recordConsequencesOfActiveGradients(const double & newFactsAppearsInLayer) {
        
        assert(!preconditionsBecomingTrueAtTime.empty());
        
        const map<double, set<int> >::iterator nextFacts = preconditionsBecomingTrueAtTime.begin();
        
        assert(fabs(nextFacts->first - newFactsAppearsInLayer) < 0.00001);
        
        
        assert(gradientsLeadToTheseFactsNowBeingTrue.empty());
        
        gradientsLeadToTheseFactsNowBeingTrue.swap(nextFacts->second);
        
        
        preconditionsBecomingTrueAtTime.erase(nextFacts);
        
        set<int>::const_iterator fItr = gradientsLeadToTheseFactsNowBeingTrue.begin();
        const set<int>::const_iterator fEnd = gradientsLeadToTheseFactsNowBeingTrue.end();
        
        for (; fItr != fEnd; ++fItr) {
            layerAtWhichPreconditionBecomesTrue[*fItr].first = preconditionsBecomingTrueAtTime.end();
        }
        
        aboutToRecordAnEffect(newFactsAppearsInLayer);
    }

    /** @brief Record that a given integrated continuous numeric effect can be applied a number of times, on behalf of the specified action.
     *
     * @param action                Details on the action, including how many times it can be applied
     * @param effectAppearsInLayer  The layer in which the consequences of the effect appear
     * @param effID                 The ID of the effect (an index into <code>RPGBuilder::getNumericEff()</code>)
     */
    void recordIntegratedNumericEffect(const ActionAndHowManyTimes & action, const double & effectAppearsInLayer, const pair<int,double> & effID) {
                
        if (debug) {
            cout << "Recording integrated effect " << effID.first;
            if (effID.second > 0) {
                cout << " += " << effID.second;
            } else {
                cout << " -= " << -effID.second;
            }
            if (action.howManyTimes == 1) {
                cout << ", once, ";
            } else {
                cout << ", " << action.howManyTimes << " times, ";
            }
            cout << "affecting fact layer " << effectAppearsInLayer << endl;
        }
        
        aboutToRecordAnEffect(effectAppearsInLayer);
        
        gc.push_back(action);
        
        actionsThatHaveHadThisIntegratedEffect[effID.first][effID.second].push_back(&(gc.back()));
        
        (*mostRecentIntegratedEffects)[effID.first][effID.second].push_back(&(gc.back()));
        
        if (debug) {
            cout << COLOUR_light_green << "Added integrated effect: " << *(RPGBuilder::getPNE(effID.first));
            if (effID.second > 0) {
                cout << " += " << effID.second << COLOUR_default << endl;
            } else {
                cout << " -= " << -effID.second << COLOUR_default << endl;
            }
        }
        
        assert(gc.back().howManyTimes > 0);
    }

    /** @brief Record that a given gradient continuous numeric effect can be applied a number of times, on behalf of the specified action.
     * 
     * If the action can only be applied finitely often, the fact that the effect has to finish is recorded in
     * <code>factLayers</code>.
     *
     * @param action                Details on the action, including how many times it can be applied
     * @param effectAppearsInLayer  The layer in which the consequences of the effect appear
     * @param effID                 The ID of the effect (an index into <code>RPGBuilder::getNumericEff()</code>)
     * @param factLayers            Updated to record the point (if any) at which the effect expires
     */
    void recordGradientNumericEffect(const ActionAndHowManyTimes & action, const double & effectAppearsInLayer,
                                     const pair<int,double> & effID, map<double, FactLayerEntry, EpsilonComp > & factLayers) {
                
        if (debug) {
            cout << "Recording gradient effect d" << effID.first;
            if (effID.second > 0) {
                cout << "/dt += " << effID.second;
            } else {
                cout << "/dt -= " << -effID.second;
            }
            if (action.howManyTimes == 1) {
                cout << ", once, ";
            } else {
                cout << ", " << action.howManyTimes << " times, ";
            }
            cout << "starting immediately before fact layer " << effectAppearsInLayer << endl;
        }

        aboutToRecordAnEffect(effectAppearsInLayer);
        
        gc.push_back(action);
        
        actionsThatHaveHadThisGradientEffect[effID.first][effID.second].push_back(&(gc.back()));

        (*mostRecentGradientEffects)[effID.first][effID.second].push_back(&(gc.back()));
        
        if (debug) {
            cout << COLOUR_light_green << "Added gradient effect: d" << *(RPGBuilder::getPNE(effID.first));
            if (effID.second > 0) {
                cout << "/dt += " << effID.second << COLOUR_default << endl;
            } else {
                cout << "/dt -= " << -effID.second << COLOUR_default << endl;
            }
        }
        
        assert(gc.back().howManyTimes > 0);

        gc.back().gradientNeedsStarting = true;
        
        if (action.howManyTimes == INT_MAX) return;
        if (action.maxDur == DBL_MAX) return;
        
        const double endAt = ((effectAppearsInLayer - EPSILON)           // the time of the action layer in which the effect began
                               + action.howManyTimes * (action.maxDur)); // plus the maximum sequential time of applications of this action
        
        gc.back().gradientsFinishAt = endAt;
        
        if (effID.second > 0) {        
            factLayers[endAt].gradientFinishes.insert(make_pair(effID.first,0.0)).first->second += effID.second;
        } else {
            factLayers[endAt].gradientFinishes.insert(make_pair(effID.first + FluentLayerEntry::pneCount,0.0)).first->second -= effID.second;
        }
        
    }
        
    
    /** @brief Record that a given numeric effect can be applied a number of times, on behalf of the specified action.
     *
     * @param action                Details on the action, including how many times it can be applied
     * @param effectAppearsInLayer  The layer in which the the consequences of the effect appear
     * @param effID                 The ID of the effect (an index into <code>RPGBuilder::getNumericEff()</code>)
     */
    void recordInstantaneousNumericEffect(const ActionAndHowManyTimes & action, const double & effectAppearsInLayer, const int & effID) {
        
        
        if (debug) {
            cout << "Recording effect " << effID << " affecting fact layer " << effectAppearsInLayer << endl;
        }
            
            
        aboutToRecordAnEffect(effectAppearsInLayer);
                        
        
        if (actionsThatHaveHadThisEffect[effID].empty()) {
            RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[effID];
            for (int s = 0; s < currEff.size; ++s) {
                if (currEff.variables[s] > 0) {
                    revisitInstantaneousEffectIfVariableValueChanges[currEff.variables[s]].insert(effID);
                }
            }
        }
        
        if (debug) {
            cout << COLOUR_light_blue << "Added effect: " << RPGBuilder::getNumericEff()[effID] << COLOUR_default << endl;
        }
        gc.push_back(action);
        
        actionsThatHaveHadThisEffect[effID].push_back(&(gc.back()));
        (*mostRecentInstantaneousEffects)[effID].push_back(&(gc.back()));        
    }
                                                           
   /** @brief Apply any effects recorded in the most recent action layer, supplied via calls to <code>recordInstantaneousNumericEffect()</code>.
    *
    * If no such effects exist, this function does nothing.
    *
    *  @param achievedInLayer          Layers at which the numeric preconditions were achieved
    *  @param newNumericPreconditions  If new numeric preconditions have become true, their indices are pushed onto this list
    *  @param maxNeeded                The maximum amount of each variable needed to satisfy the preconditions
    */
    void applyRecentlyRecordedEffects(vector<double> & achievedInLayer, list<int> & newNumericPreconditions,
                                      const vector<double> & maxNeeded) {
        if (effectsAffectThisLayer == layers.end()) return;
        
        assert(effectsAffectThisLayer->second->initialised());
        
        if (debug) {
            cout << "Applying the numeric effects noted in this layer\n";
        }
        
        set<int> variableChanged;        
        set<int> variableAssignedTo;

        {
            
            map<int, map<double, list<const ActionAndHowManyTimes*> > >::const_iterator effItr = mostRecentGradientEffects->begin();
            const map<int, map<double, list<const ActionAndHowManyTimes*> > >::const_iterator effEnd = mostRecentGradientEffects->end();

            for (; effItr != effEnd; ++effItr) {
                if (debug) {
                    cout << "Gradient effects on " << *(RPGBuilder::getPNE(effItr->first)) << ":";
                }
                map<double, list<const ActionAndHowManyTimes*> >::const_iterator magItr = effItr->second.begin();
                const map<double, list<const ActionAndHowManyTimes*> >::const_iterator magEnd = effItr->second.end();
                
                for (; magItr != magEnd; ++magItr) {
                    if (debug) {
                        cout << " " << magItr->first;
                    }
                    
                    list<const ActionAndHowManyTimes*>::const_iterator actItr = magItr->second.begin();
                    const list<const ActionAndHowManyTimes*>::const_iterator actEnd = magItr->second.end();
                    
                    for (; actItr != actEnd; ++actItr) {
                        if ((*actItr)->gradientNeedsStarting) {
                            assert((*actItr)->howManyTimes);
                            effectsAffectThisLayer->second->startGradient(effItr->first, magItr->first, variableChanged);
                        }
                    }
                }
                if (debug) {
                    cout << ", gradients on bounds are now [";
                    const double & l = effectsAffectThisLayer->second->getGradients()[effItr->first + FluentLayerEntry::pneCount];
                    if (l == 0) {
                        cout << "0.0";
                    } else {
                        cout << -l;
                    }
                    cout << "," << effectsAffectThisLayer->second->getGradients()[effItr->first] << "]" << endl;
                }
            }

            set<int> avsToRecalculate;
        
            set<int>::const_iterator vcItr = variableChanged.begin();
            const set<int>::const_iterator vcEnd = variableChanged.end();
            
            for (; vcItr != vcEnd; ++vcItr) {
                avsToRecalculate.insert(RPGBuilder::getVariableDependencies(*vcItr).begin(), RPGBuilder::getVariableDependencies(*vcItr).end());
            }

            {
                set<int>::const_iterator avItr = avsToRecalculate.begin();
                const set<int>::const_iterator avEnd = avsToRecalculate.end();            
                
                effectsAffectThisLayer->second->recalculateAVGradients(avItr, avEnd);
            }
            
            if (debug) {
                
                set<int>::const_iterator avItr = avsToRecalculate.begin();
                const set<int>::const_iterator avEnd = avsToRecalculate.end(); 
                
                for (; avItr != avEnd; ++avItr) {
                    cout << "Gradients of AV " << RPGBuilder::getArtificialVariable(*avItr) << " is now " << effectsAffectThisLayer->second->getGradients()[*avItr] << endl;
                }
            }
            
            
            // Now give us epsilon's worth of the gradient effects
            
            LayerMap::iterator previousLayer = effectsAffectThisLayer;
            --previousLayer;
            
            assert(fabs(effectsAffectThisLayer->first - previousLayer->first - EPSILON) < 0.0000001);
            effectsAffectThisLayer->second->applyGradients(EPSILON);
        }
        
        {
            // integrated CTS effects
            
            map<int, map<double, list<const ActionAndHowManyTimes*> > >::const_iterator effItr = mostRecentIntegratedEffects->begin();
            const map<int, map<double, list<const ActionAndHowManyTimes*> > >::const_iterator effEnd = mostRecentIntegratedEffects->end();
            
            for (; effItr != effEnd; ++effItr) {
                if (debug) {
                    cout << "Integrated effects on " << *(RPGBuilder::getPNE(effItr->first)) << ":";
                }
                map<double, list<const ActionAndHowManyTimes*> >::const_iterator magItr = effItr->second.begin();
                const map<double, list<const ActionAndHowManyTimes*> >::const_iterator magEnd = effItr->second.end();
                
                for (; magItr != magEnd; ++magItr) {
                    if (debug) {
                        cout << " " << magItr->first << " (";
                    }
                    list<const ActionAndHowManyTimes*>::const_iterator actItr = magItr->second.begin();
                    const list<const ActionAndHowManyTimes*>::const_iterator actEnd = magItr->second.end();
                    
                    for (; actItr != actEnd; ++actItr) {
                        if (debug) {
                            cout << " " << (*actItr)->howManyTimes;
                        }
                        assert((*actItr)->howManyTimes);
                        effectsAffectThisLayer->second->applyIncrease(-1, effItr->first, magItr->first, magItr->first, (*actItr)->howManyTimes, variableChanged);
                    }
                    if (debug) {
                        cout << " ) ";
                    }
                }
                if (debug) {
                    cout << endl;
                    const int affectedVar = effItr->first;
                    cout << "Bounds are now [";
                    if (effectsAffectThisLayer->second->values()[affectedVar + FluentLayerEntry::pneCount] == DBL_MAX) {
                        cout << "-inf,";
                    } else {
                        cout << -effectsAffectThisLayer->second->values()[affectedVar + FluentLayerEntry::pneCount] << ",";
                    }
                    
                    if (effectsAffectThisLayer->second->values()[affectedVar] == DBL_MAX) {
                        cout << "inf]\n";
                    } else {
                        cout << effectsAffectThisLayer->second->values()[affectedVar] << "]\n";
                    }
                }
            }
        }
        
        for (int pass = 0; pass < 2; ++pass) {
            
            // pass 0 - increase/decrease effects
            // pass 1 - assignments
            
            map<int, list<const ActionAndHowManyTimes*> >::const_iterator effItr = mostRecentInstantaneousEffects->begin();
            const map<int, list<const ActionAndHowManyTimes*> >::const_iterator effEnd = mostRecentInstantaneousEffects->end();
            
            for (; effItr != effEnd; ++effItr) {
                RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[effItr->first];
                if (currEff.isAssignment != (pass == 1)) continue;
                const int affectedVar = currEff.fluentIndex;
                
                list<const ActionAndHowManyTimes*>::const_iterator actItr = effItr->second.begin();
                const list<const ActionAndHowManyTimes*>::const_iterator actEnd = effItr->second.end();
                
                for (; actItr != actEnd; ++actItr) {
                    const double maxVal = currEff.evaluate(effectInputs->second->values(), (*actItr)->minDur, (*actItr)->maxDur);
                    const double minVal = currEff.evaluateMin(effectInputs->second->values(), (*actItr)->minDur, (*actItr)->maxDur);
                    if (currEff.isAssignment) {                    
                        effectsAffectThisLayer->second->keepAssignmentIfBetter(*actItr, effItr->first, true, affectedVar, maxVal, variableAssignedTo);
                        effectsAffectThisLayer->second->keepAssignmentIfBetter(*actItr, effItr->first, false, affectedVar, minVal, variableAssignedTo);                    
                    } else {
                        if (debug) {
                            cout << currEff << " gives us " << minVal << "," << maxVal << " of " << *(RPGBuilder::getPNE(currEff.fluentIndex)) << endl;
                            
                        }
                        effectsAffectThisLayer->second->applyIncrease(effItr->first, affectedVar, minVal, maxVal, (*actItr)->howManyTimes, variableChanged);
                        //effectsAffectThisLayer->second->applyIncrease(effItr->first, affectedVar, minVal, (*actItr)->howManyTimes, variableChanged);                    

                        if (debug) {
                            cout << "Bounds are now [";
                            if (effectsAffectThisLayer->second->values()[affectedVar + FluentLayerEntry::pneCount] == DBL_MAX) {
                                cout << "-inf,";
                            } else {
                                cout << -effectsAffectThisLayer->second->values()[affectedVar + FluentLayerEntry::pneCount] << ",";
                            }
                            if (effectsAffectThisLayer->second->values()[affectedVar] == DBL_MAX) {
                                cout << "inf]\n";
                            } else {
                                cout << effectsAffectThisLayer->second->values()[affectedVar] << "]\n";
                            }
                            
                        }
                    }
                }
            }
            
        }
        
        // mark assigned-to variables as having changed, too
        variableChanged.insert(variableAssignedTo.begin(), variableAssignedTo.end());
        
        set<int> avsToRecalculate;
            
        {
            
            set<int>::const_iterator vcItr = variableChanged.begin();
            const set<int>::const_iterator vcEnd = variableChanged.end();
            
            for (; vcItr != vcEnd; ++vcItr) {
                avsToRecalculate.insert(RPGBuilder::getVariableDependencies(*vcItr).begin(), RPGBuilder::getVariableDependencies(*vcItr).end());
            }
            
            {
                set<int>::const_iterator avItr = avsToRecalculate.begin();
                const set<int>::const_iterator avEnd = avsToRecalculate.end();
                
                effectsAffectThisLayer->second->recalculateAVs(avItr, avEnd);
            }
            
            if (debug) {
                set<int>::const_iterator avItr = avsToRecalculate.begin();
                const set<int>::const_iterator avEnd = avsToRecalculate.end();
                
                for (; avItr != avEnd; ++avItr) {
                    cout << "Lower bound of " << RPGBuilder::getArtificialVariable(*avItr) << " is now ";
                    if (effectsAffectThisLayer->second->values()[*avItr] == DBL_MAX) {
                        cout << "inf\n";
                    } else { 
                        cout << effectsAffectThisLayer->second->values()[*avItr] << endl;
                    }

                }
            }
        }

        set<int> presToRecalculate;
        
        // pull in preconditions that gradients have satisfied, but double-check this is definitely the case
        presToRecalculate.swap(gradientsLeadToTheseFactsNowBeingTrue);
        
        for (int pass = 0; pass < 2; ++pass) {
            const set<int> & variableSet = (pass ? avsToRecalculate : variableChanged);
            
            set<int>::const_iterator varItr = variableSet.begin();
            const set<int>::const_iterator varEnd = variableSet.end();
            
            for (; varItr != varEnd; ++varItr) {
                const list<int> & recalc = RPGBuilder::affectsRPGNumericPreconditions(*varItr);
                presToRecalculate.insert(recalc.begin(), recalc.end());
            }
        }
        
        {            
            set<int>::const_iterator preItr = presToRecalculate.begin();
            const set<int>::const_iterator preEnd = presToRecalculate.end();
            
            double satisfactionDelay;
            
            for (; preItr != preEnd; ++preItr) {
                if (achievedInLayer[*preItr] != -1.0) continue;
                
                PreconditionToSatisfactionLayerPair & forecastPair = layerAtWhichPreconditionBecomesTrue[*preItr];
                
                const RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[*preItr];
                
                satisfactionDelay = effectsAffectThisLayer->second->willBecomeSatisfiedAfterDelay(currPre);
                
                if (satisfactionDelay == 0.0) {
                    achievedInLayer[*preItr] = effectsAffectThisLayer->first;
                    newNumericPreconditions.push_back(*preItr);
                    if (debug) {
                        cout << COLOUR_yellow << "Precondition " << *preItr << ", " << currPre << ", becomes true in layer " << effectsAffectThisLayer->first << COLOUR_default << endl;
                    }
                    
                    // Now it's satisfied, note that it isn't forecast to become true at a future point
                    if (forecastPair.first != preconditionsBecomingTrueAtTime.end()) {
                        forecastPair.first->second.erase(forecastPair.second);
                        if (forecastPair.first->second.empty()) {
                            preconditionsBecomingTrueAtTime.erase(forecastPair.first);
                        }
                                                            
                        forecastPair.first = preconditionsBecomingTrueAtTime.end();
                    }
                    
                } else if (satisfactionDelay < DBL_MAX) {
                    
                    double futureLayer = ceil((effectsAffectThisLayer->first + satisfactionDelay) / EPSILON) * EPSILON;
                    const double earliestPOPoint = earliestPointForNumericPrecondition(currPre, earliestNumericPOTimes);
                    
                    if (futureLayer < earliestPOPoint) {
                        futureLayer = earliestPOPoint;
                    }
                    
                    if (forecastPair.first == preconditionsBecomingTrueAtTime.end()) {   
                        // never been forecast before
                        forecastPair.first = preconditionsBecomingTrueAtTime.insert(make_pair(futureLayer, set<int>())).first;
                        forecastPair.second = forecastPair.first->second.insert(*preItr).first;
                    } else {                    
                        map<double, set<int> >::iterator newForecastItr = preconditionsBecomingTrueAtTime.find(futureLayer);
                        if (newForecastItr != forecastPair.first) {
                            // forecast was previously at a different time
                            
                            forecastPair.first->second.erase(forecastPair.second);
                            if (forecastPair.first->second.empty()) {
                                preconditionsBecomingTrueAtTime.erase(forecastPair.first);
                            }
                            
                            forecastPair.first = preconditionsBecomingTrueAtTime.insert(make_pair(futureLayer, set<int>())).first;
                            forecastPair.second = forecastPair.first->second.insert(*preItr).first;
                        }
                    }
                    
                    
                } else if (forecastPair.first != preconditionsBecomingTrueAtTime.end()) {
                    // was forecast to become true, but isn't any longer - a gradient in its favour has expired
                    
                    forecastPair.first->second.erase(forecastPair.second);
                    if (forecastPair.first->second.empty()) {
                        preconditionsBecomingTrueAtTime.erase(forecastPair.first);
                    }
                    
                    forecastPair.first = preconditionsBecomingTrueAtTime.end();
                }
            }
                
        }


        {
            set<int>::const_iterator vcItr = variableChanged.begin();
            const set<int>::const_iterator vcEnd = variableChanged.end();
            
            for (; vcItr != vcEnd; ++vcItr) {
                effectsAffectThisLayer->second->markEffectsToRevisit(revisitInstantaneousEffectIfVariableValueChanges[*vcItr], maxNeeded);
            }
        }
        
        {
            set<int>::const_iterator atItr = variableAssignedTo.begin();
            const set<int>::const_iterator atEnd = variableAssignedTo.end();
            
            for (; atItr != atEnd; ++atItr) {

                effectsAffectThisLayer->second->markEffectsToRevisit(revisitInstantaneousEffectIfVariableIsAssignedTo[*atItr], maxNeeded);
                
                if (*atItr >= RPGBuilder::getPNECount()) {
                    if (!actionsThatHaveHadThisIntegratedEffect[*atItr - RPGBuilder::getPNECount()].empty()) {
                        effectsAffectThisLayer->second->markIntegratedOrGradientEffectVariableToRevisit(*atItr - RPGBuilder::getPNECount(), maxNeeded);
                    }
                    
                } else {                
                    if (!actionsThatHaveHadThisIntegratedEffect[*atItr].empty()) {
                        effectsAffectThisLayer->second->markIntegratedOrGradientEffectVariableToRevisit(*atItr, maxNeeded);
                    }
                }
            }
        }
        
        // As all effects have now been applied, we do two things:
        
        // i) If there are effects to revisit, we note that this is the case
        if (   !effectsAffectThisLayer->second->getInstantaneousEffectsToRevisitAfterThisLayer().empty()
            || !effectsAffectThisLayer->second->getIntegratedOrGradientEffectsToRevisitAfterThisLayer().empty()    ) {
            
            layerWithEffectsToBeRevisited = effectsAffectThisLayer;
        
        }
        
        // ii) We clear these variables, so that adding a new fluent layer is not blocked
        effectsAffectThisLayer = layers.end();
        mostRecentInstantaneousEffects = 0;
        
        if (debug) {
            cout << "Returning, having allowed the possibility of adding a new fluent layer\n";
        }
    }
    
    /** @brief Signal that the RPG has been built, and a plan is now to be extracted. */
    void prepareForExtraction() {
        assert(!nowExtracting);
        
        nowExtracting = true;
        currentSubgoalLayer = layers.end();
        --currentSubgoalLayer;
        
        if (debug) {
            cout << COLOUR_light_red << "Now extracting\n" << COLOUR_default << endl;
            cout << "Current subgoal layer is at time " << currentSubgoalLayer->first << endl;
        }
    }
    
    /** @brief Request a precondition to support an action, during solution extraction.
     *
     *  This function takes the desired numeric precondition, and ensures it is
     *  met prior to the point at which the action is applied.
     *
     * @param pre             The numeric precondition (an index into <code>RPGBuilder::getNumericPreTable()</code>
     * @param achievedAt      The fact layer at which this precondition was satisfied
     * @param forActionLayer  The action layer of the action this precondition needs to support
     * @param goalsAtLayer    The record of goals kept during solution extraction, to be updated
     *                        by this function.
     * @param tilR            The timestamp of the earliest deadline which the action this precondition
     *                        supports is relevant to.
     */
    void requestNumericPrecondition(const int & pre, const double & achievedAt,
                                    const double & forActionLayer, RPGRegressionMap & goalsAtLayer,
                                    const double & tilR) {
        
        if (debug) {
            if ((-currentSubgoalLayer->first + (forActionLayer - EPSILON)) > 0.000001) {
                cout << "It appears there is a cycle in the RPG solution extraction\n";
                cout << "Previously, we were asking for the preconditions of actions at " << currentSubgoalLayer->first << endl;
                cout << "However, the action for which preconditions are being requested now is at " << forActionLayer << endl;
            }
        }
        // check we are definitely going backwards through the RPG
        assert((-currentSubgoalLayer->first + (forActionLayer - EPSILON)) < 0.000001);
        
        while ((currentSubgoalLayer->first - forActionLayer) > 0.000001) {
            --currentSubgoalLayer;
            if (debug) {
                cout << "Current subgoal layer is now at time " << currentSubgoalLayer->first << endl;
            }
        }
        
        LayerMap::iterator appearedIn = layers.find(achievedAt);
        
        if (debug) {
            if (appearedIn == layers.end()) {
                cout << "Asking for a precondition, " << pre << ", that became true at time " << achievedAt << ", but there is no fluent layer recorded for that time\n";
            }
        }
        assert(appearedIn != layers.end());
        
        const RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[pre];
        
        const double threshold = (currPre.op == VAL::E_GREATER ? currPre.RHSConstant + 0.00001 : currPre.RHSConstant);
        
        RPGRegress & addTo = goalsAtLayer[achievedAt];
        
        addTo.requestNumericThreshold(currPre.LHSVariable, threshold, tilR);
        
        if (debug) {
            cout << "\t\t\t - Translated to a threshold " << currPre.LHSVariable << " >= " << threshold << endl;
        }
    }

    /** @brief Request that a numeric goal be achieved, during solution extraction.
     *
     *  This function takes the desired numeric goal, and ensures that actions
     *  are added to the plan to support it.
     *
     * @param pre             The numeric goal (an index into <code>RPGBuilder::getNumericPreTable()</code>
     * @param achievedAt      The fact layer at which this precondition was satisfied
     * @param goalsAtLayer    The record of goals kept during solution extraction, to be updated
     *                        by this function.
     */
    void requestGoal(const int & pre, const double & achievedAt,
                     RPGRegressionMap & goalsAtLayer) {
        
        LayerMap::iterator appearedIn = layers.find(achievedAt);
        assert(appearedIn != layers.end());
        
        const RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[pre];
        
        const double threshold = (currPre.op == VAL::E_GREATER ? currPre.RHSConstant + 0.001 : currPre.RHSConstant);
        
        RPGRegress & addTo = goalsAtLayer[achievedAt];
        
        if (debug) {
            cout << " - Requesting " << currPre.LHSVariable << " >= " << threshold << " at time " << achievedAt << endl;
        }
        addTo.requestNumericThreshold(currPre.LHSVariable, threshold, DBL_MAX);
        
    }


    /** @brief Record the numeric side-effects of an action being added to the plan.
     *
     *  If we add an action to the plan, it might have numeric effects on other variables,
     *  as well as an effect upon the intended fact.  This function records these side-effects,
     *  in case they are beneficial.
     *
     * 
     */
    void recordSideEffects(const int & act, const VAL::time_spec & ts, const double & actLayer) {
        
        LayerMap::const_iterator rollback = layers.end();
        --rollback;
        
        while ((rollback->first - actLayer) > 0.00001) {
            --rollback;
        }
        
        ActionAndHowManyTimes tmp(act, ts, 1, RPGBuilder::getOpMinDuration(act, 0), RPGBuilder::getOpMaxDuration(act, 0));
        
        recordSideEffects(&tmp, 1, rollback->second->values());
    }
    
    
    /** @brief Satisfy all the numeric preconditions at the given layer, by adding supporting actions.
     *
     *  This action takes the numeric preconditions to be achieved in the given fact layer (recorded
     *  as variable&ndash;value thresholds), and applies actions from those available immediately
     *  prior to this layer until the residual precondition is sufficiently small as to be
     *  satisfiable in an earlier fluent layer.
     *
     *  @param currentLayer  The layer containing the numeric preconditions to be satisfied. 
     *  @param goalsAtLayer  The record of goals kept during solution extraction, to be updated
     *                       by this function to contain those to be achieved at an earlier layer.
     *  @param actionsUsed   Each action used in this function is added to this list, along with its
     *                       time-stamp, to then allow it to be (elsewhere) added to the relaxed plan.
     */
    void satisfyNumericPreconditionsAtLayer(RPGRegressionMap::const_iterator & currentLayer,
                                            RPGRegressionMap & goalsAtLayer,
                                            list<pair<double, SupportingAction> > & actionsUsed) {
        
        if (currentLayer->second.numericValueGreaterThan.empty()) return;
        if (currentLayer->first < 0.00001) return;
        
        if (debug) {
            cout << COLOUR_light_blue << "Satisfying numeric preconditions in layer " << currentLayer->first << COLOUR_default << endl;
        }
        
        const LayerMap::iterator preconditionsIn = layers.find(currentLayer->first);
        
        LayerMap::const_iterator previousFluentLayer = preconditionsIn;
        --previousFluentLayer;
        
        

        map<int, pair<double,double> >::const_iterator thresholdItr = currentLayer->second.numericValueGreaterThan.begin();
        const map<int, pair<double,double> >::const_iterator thresholdEnd = currentLayer->second.numericValueGreaterThan.end();
        
        for (; thresholdItr != thresholdEnd; ++thresholdItr) {
            double needToAchieve = thresholdItr->second.first;
            
            if (debug) {
                cout << ": variable " << thresholdItr->first << " >= " << needToAchieve << endl;
            }
            
            /*{
                map<int,double>::const_iterator aaItr = alreadyAchieved.find(thresholdItr->first);
                if (aaItr != alreadyAchieved.end()) {
                    needToAchieve -= aaItr->second;
                    if (debug) {
                        cout << " * becomes variable " << thresholdItr->first << " >= " << needToAchieve << endl;
                    }
                }
            }*/
            
            if (needToAchieve < borrowFactLayerZeroValues()[thresholdItr->first]) {
                // have already satisfied this when satisfying something else
                if (debug) {
                    cout << "- Already have enough " << thresholdItr->first << endl;
                }
                continue;
            }

            const double valueInPreviousLayer = previousFluentLayer->second->values()[thresholdItr->first];
                        
            // if we can push it back to an earlier layer, then do so
            if (needToAchieve <= valueInPreviousLayer) {
                if (debug) {
                    cout << "- Could have " << needToAchieve << " of " << thresholdItr->first << " in the previous layer, " << previousFluentLayer->first << endl;
                }
                
                LayerMap::const_iterator rollback = previousFluentLayer;
                while (rollback->second->values()[thresholdItr->first] > needToAchieve) {
                    --rollback;
                }
                ++rollback;
                
                if (debug) {
                    cout << "  * Earliest possible point with sufficient is layer " << rollback->first << endl;
                }
                
                if (rollback->first > 0.00001) {
                    goalsAtLayer[rollback->first].requestNumericThreshold(thresholdItr->first, needToAchieve, thresholdItr->second.second);
                }
                continue;
            }
            
            // otherwise, we need to put some effort in
            
            assert(preconditionsIn->second->values()[thresholdItr->first] >= thresholdItr->second.first);
            
            const RecordedAssignment * const assignmentUsed = preconditionsIn->second->assignmentAppliedToVariable(thresholdItr->first);
            
            // if it was done with assignment...
            if (assignmentUsed && assignmentUsed->act) {
                
                const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[assignmentUsed->eff];
                
                const ActionAndHowManyTimes * const toUse = assignmentUsed->act;
                
                actionsUsed.push_back(make_pair(currentLayer->first,SupportingAction(toUse->actID, toUse->ts, 1, thresholdItr->second.second)));
                    
                recordSideEffects(toUse, 1, previousFluentLayer->second->values());
                
                
                double contributionToIncreaseFromDur = 0.0;
                if (assignmentUsed->maximiseEffect) {
                    for (int s = 0; s < currEff.size; ++s) {
                        if (currEff.variables[s] == -3) {
                            contributionToIncreaseFromDur += (currEff.weights[s] * toUse->minDur);
                        } else if (currEff.variables[s] == -19) {
                            contributionToIncreaseFromDur += (currEff.weights[s] * toUse->maxDur);
                        }
                    }
                } else {
                    for (int s = 0; s < currEff.size; ++s) {
                        if (currEff.variables[s] == -3) {
                            contributionToIncreaseFromDur += (currEff.weights[s] * toUse->maxDur);
                        } else if (currEff.variables[s] == -19) {
                            contributionToIncreaseFromDur += (currEff.weights[s] * toUse->minDur);
                        }
                    }
                }
                
                if (currEff.size) {
                    // if assigning a value derived from other variables, we need to make sure
                    // they hold values sufficient for the desired result
                    list<pair<int,double> > subTerms;
                    breakApart(currEff.variables, currEff.weights, currEff.size, currEff.constant,
                               assignmentUsed->maximiseEffect, thresholdItr->second.first - contributionToIncreaseFromDur,
                               previousFluentLayer, subTerms);

                    if (!subTerms.empty()) {// duration dependent variables don't induce subterms
                        RPGRegress & prev = goalsAtLayer[previousFluentLayer->first];
                        
                        list<pair<int,double> >::const_iterator tItr = subTerms.begin();
                        const list<pair<int,double> >::const_iterator tEnd = subTerms.end();
                        
                        for (; tItr != tEnd; ++tItr) {
                            prev.requestNumericThreshold(tItr->first, tItr->second, thresholdItr->second.second);
                        }
                    }
                }
            } else {
                
                list<pair<int,double> > subTerms;
                
                if (thresholdItr->first < 2 * FluentLayerEntry::pneCount) {
                    subTerms.push_back(make_pair(thresholdItr->first, thresholdItr->second.first));
                } else {
                    const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(thresholdItr->first);
                    double avRemaining = thresholdItr->second.first;
                    
                    /*for (int s = 0; s < currAV.size; ++s) {
                        const map<int,double>::const_iterator aaItr = alreadyAchieved.find(currAV.fluents[s]);
                        if (aaItr != alreadyAchieved.end()) {
                            avRemaining -= aaItr->second * currAV.weights[s];
                        }
                    }*/
                    
                    breakApart(currAV.fluents, currAV.weights, currAV.size, currAV.constant,
                               true, avRemaining,
                               preconditionsIn, subTerms);
                }
                
                list<pair<int,double> >::const_iterator tItr = subTerms.begin();
                const list<pair<int,double> >::const_iterator tEnd = subTerms.end();
                
                for (; tItr != tEnd; ++tItr) {
                    
                    double residual = tItr->second;
                    const double acceptableMinimum = previousFluentLayer->second->values()[tItr->first];

                    if (debug) {
                        cout << "- Considering ways of ";
                        if (tItr->first < FluentLayerEntry::pneCount) {
                            cout << "increasing " << *(RPGBuilder::getPNE(tItr->first)) << ": must have " << residual << " by now, given " << acceptableMinimum << " previously\n";
                        } else {
                            cout << "decreasing " << *(RPGBuilder::getPNE(tItr->first - FluentLayerEntry::pneCount))  << ": must have " << -residual << " by now, given " << -acceptableMinimum << " previously\n";
                        }                                                
                    }
                                        
                                        
                    
                    const set<int> * const instantaneousEffs = preconditionsIn->second->getInstantaneousEffectsThatIncreasedVariable(tItr->first);
                    
                    if (debug) {
                        if (instantaneousEffs && !instantaneousEffs->empty()) {
                            cout << "  * Considering instantaneous effects\n";
                        } else {
                            cout << "  * No suitable instantaneous effects\n";
                        }
                    }
                    
                    
                    if (instantaneousEffs) {
                        set<int>::const_iterator effItr = instantaneousEffs->begin();
                        const set<int>::const_iterator effEnd = instantaneousEffs->end();
                        
                        for (; effItr != effEnd && residual > acceptableMinimum; ++effItr) {
                            
                            const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*effItr];
                            
                            if (debug) {
                                cout << "      - One potential effect: " << currEff << endl;
                            }
                            
                            const list<const ActionAndHowManyTimes*> & actions = preconditionsIn->second->getInstantaneousEffectsDirectlyBeforeThisLayer()[*effItr];
                            
                            bool minUsed = false;
                            double minimumEffectMagnitudeNeeded;
                            bool maxUsed = false;
                            double maximumEffectMagnitudeNeeded;
                            
                            list<const ActionAndHowManyTimes*>::const_iterator aItr = actions.begin();
                            const list<const ActionAndHowManyTimes*>::const_iterator aEnd = actions.end();
                            
                            for (; aItr != aEnd && residual > acceptableMinimum; ++aItr) {
                                
                                //for (int pass = 0; pass < 2 && residual > acceptableMinimum; ++pass) {
                                
                                {
                                    
                                    const double minEff = currEff.evaluateMin(previousFluentLayer->second->values(), (*aItr)->minDur, (*aItr)->maxDur);
                                    const double maxEff = currEff.evaluate(previousFluentLayer->second->values(), (*aItr)->minDur, (*aItr)->maxDur);
                                    
                                    if (tItr->first >= FluentLayerEntry::pneCount ? (minEff < 0 || maxEff < 0) : (minEff > 0 || maxEff > 0)) {

                                        double localEff;
                                        bool * used = 0;
                                        double * magnitudeUpdate = 0;
                                        bool minRatherThanMax = false;
                                        
                                        if (tItr->first >= FluentLayerEntry::pneCount) {
                                            if (minEff < maxEff) {
                                                localEff = minEff;
                                                magnitudeUpdate = &(minimumEffectMagnitudeNeeded);
                                                used = &(minUsed);
                                                minRatherThanMax = true;
                                            } else {
                                                localEff = maxEff;
                                                magnitudeUpdate = &(maximumEffectMagnitudeNeeded);
                                                used = &(maxUsed);
                                            }
                                        } else {
                                            if (minEff > maxEff) {
                                                localEff = minEff;
                                                magnitudeUpdate = &(minimumEffectMagnitudeNeeded);
                                                used = &(minUsed);
                                                minRatherThanMax = true;
                                            } else {
                                                localEff = maxEff;
                                                magnitudeUpdate = &(maximumEffectMagnitudeNeeded);
                                                used = &(maxUsed);
                                            }
                                        }
                                        
                                        double contributionToIncreaseFromDur = 0.0;
                                        
                                        if (minRatherThanMax) {
                                            for (int s = 0; s < currEff.size; ++s) {
                                                if (currEff.variables[s] == -3) {
                                                    contributionToIncreaseFromDur += (currEff.weights[s] * (*aItr)->minDur);
                                                } else if (currEff.variables[s] == -19) {
                                                    contributionToIncreaseFromDur += (currEff.weights[s] * (*aItr)->maxDur);
                                                }
                                            }
                                        } else {
                                            for (int s = 0; s < currEff.size; ++s) {
                                                if (currEff.variables[s] == -3) {
                                                    contributionToIncreaseFromDur += (currEff.weights[s] * (*aItr)->maxDur);
                                                } else if (currEff.variables[s] == -19) {
                                                    contributionToIncreaseFromDur += (currEff.weights[s] * (*aItr)->minDur);
                                                }
                                            }
                                        }
                                        
                                        
                                        int applyTimes = (*aItr)->howManyTimes;
                                        
                                        
                                        if (debug) {
                                            cout << "          + Effect can be obtained " << applyTimes << " time(s) using ";
                                            if ((*aItr)->ts == VAL::E_AT_START) {
                                                cout << "start of ";
                                            } else {
                                                cout << "end of ";
                                            }
                                            cout << *(RPGBuilder::getInstantiatedOp((*aItr)->actID)) << endl;
                                        }
                                        if (localEff == DBL_MAX) {
                                            applyTimes = 1;
                                            residual = -DBL_MAX;                                        
                                        } else if (applyTimes == INT_MAX) {
                                            const double at = (residual - acceptableMinimum) / localEff;
                                            applyTimes = at;
                                            if (at > applyTimes) { // round up the number of times we need apply the action
                                                ++applyTimes;
                                            }
                                            residual -= applyTimes * localEff;
                                        } else if (applyTimes * localEff >= (residual - acceptableMinimum)) {
                                            const double at = (residual - acceptableMinimum) / localEff;
                                            applyTimes = at;
                                            if (at > applyTimes) { // round up the number of times we need apply the action
                                                ++applyTimes;
                                            }
                                            residual -= applyTimes * localEff;
                                        } else {
                                            residual -= applyTimes * localEff;
                                        }

                                        if (localEff == DBL_MAX) {
                                            if (*used) {
                                                if (minRatherThanMax) {
                                                    if (*magnitudeUpdate > residual - acceptableMinimum - contributionToIncreaseFromDur) {
                                                        *magnitudeUpdate = residual - acceptableMinimum - contributionToIncreaseFromDur;
                                                    }
                                                } else {
                                                    if (*magnitudeUpdate < residual - acceptableMinimum - contributionToIncreaseFromDur) {
                                                        *magnitudeUpdate = residual - acceptableMinimum - contributionToIncreaseFromDur;
                                                    }
                                                }
                                            } else {
                                                *magnitudeUpdate = residual - acceptableMinimum - contributionToIncreaseFromDur;
                                            }
                                        } else {
                                            if (*used) {
                                                if (minRatherThanMax) {
                                                    if (*magnitudeUpdate < localEff - contributionToIncreaseFromDur) {
                                                        *magnitudeUpdate = localEff - contributionToIncreaseFromDur;
                                                    }
                                                } else {
                                                    if (*magnitudeUpdate > localEff - contributionToIncreaseFromDur) {
                                                        *magnitudeUpdate = localEff - contributionToIncreaseFromDur;
                                                    }
                                                }
                                            } else {
                                                *magnitudeUpdate = localEff - contributionToIncreaseFromDur;
                                            }
                                        }
                                        
                                        
                                        actionsUsed.push_back(make_pair(currentLayer->first,SupportingAction((*aItr)->actID, (*aItr)->ts, applyTimes, thresholdItr->second.second)));
                                        
                                        recordSideEffects(*aItr, applyTimes, previousFluentLayer->second->values());
                                        
                                        *used = true;
                                    }                                
                                }
                            }
                            
                            if (maxUsed) {
                                
                                list<pair<int,double> > inputSubTerms;
                                breakApart(currEff.variables, currEff.weights, currEff.size, currEff.constant,
                                           true, maximumEffectMagnitudeNeeded,
                                           previousFluentLayer, inputSubTerms);
                                           
                               if (!inputSubTerms.empty()) {// duration dependent variables don't induce subterms
                                   RPGRegress & prev = goalsAtLayer[previousFluentLayer->first];
                                   
                                   list<pair<int,double> >::const_iterator t2Itr = inputSubTerms.begin();
                                   const list<pair<int,double> >::const_iterator t2End = inputSubTerms.end();
                                   
                                   for (; t2Itr != t2End; ++t2Itr) {
                                       prev.requestNumericThreshold(t2Itr->first, t2Itr->second, thresholdItr->second.second);
                                   }
                                }
                            }
                            
                                
                            if (minUsed) {
                                list<pair<int,double> > inputSubTerms;
                                breakApart(currEff.variables, currEff.weights, currEff.size, currEff.constant,
                                           false, -minimumEffectMagnitudeNeeded,
                                           previousFluentLayer, inputSubTerms);
                                           
                               if (!inputSubTerms.empty()) {// duration dependent variables don't induce subterms
                                   RPGRegress & prev = goalsAtLayer[previousFluentLayer->first];
                                   
                                   list<pair<int,double> >::const_iterator t2Itr = inputSubTerms.begin();
                                   const list<pair<int,double> >::const_iterator t2End = inputSubTerms.end();
                                   
                                   for (; t2Itr != t2End; ++t2Itr) {
                                       prev.requestNumericThreshold(t2Itr->first, t2Itr->second, thresholdItr->second.second);
                                   }
                                }
                            }
                        }
                        
                    }
                    
                    if (residual > acceptableMinimum) {
                        
                        // must need some integrated effects, too
                        
                        map<int, map<double, list<const ActionAndHowManyTimes*> > > & ieffs = preconditionsIn->second->getIntegratedEffectsDirectlyBeforeThisLayer();
                        
                        map<int, map<double, list<const ActionAndHowManyTimes*> > >::const_iterator ieItr;
                        
                        const bool keepPositive = tItr->first < FluentLayerEntry::pneCount;
                        
                        if (keepPositive) {
                            ieItr = ieffs.find(tItr->first);
                        } else {
                            ieItr = ieffs.find(tItr->first - FluentLayerEntry::pneCount);
                        }
                        
                        
                                    
                        if (ieItr == ieffs.end()) {
                            if (debug) {
                                cout << "  * No suitable integrated effects\n";
                            }                        
                        } else {
                            if (debug) {
                                cout << "  * Considering integrated effects\n";
                            }
                            
                            map<double, list<const ActionAndHowManyTimes*> >::const_iterator effItr = ieItr->second.begin();
                            const map<double, list<const ActionAndHowManyTimes*> >::const_iterator effEnd = ieItr->second.end();
                            
                            for (; effItr != effEnd && residual > acceptableMinimum; ++effItr) {
                                if (keepPositive) {
                                    if (effItr->first < 0) continue;
                                } else {
                                    if (effItr->first > 0) continue;
                                }
                                
                                const double eff = fabs(effItr->first);
                                
                                list<const ActionAndHowManyTimes*>::const_iterator aItr = effItr->second.begin();
                                const list<const ActionAndHowManyTimes*>::const_iterator aEnd = effItr->second.end();
                                
                                for (; aItr != aEnd && residual > acceptableMinimum; ++aItr) {
                                    double needed = (residual - acceptableMinimum) / eff;
                                    
                                    int nInt = needed;
                                    if (needed > nInt) { // round up
                                        ++nInt;
                                    }
                                    
                                    if (nInt > (*aItr)->howManyTimes) {
                                        residual -= (*aItr)->howManyTimes * eff;
                                        actionsUsed.push_back(make_pair(currentLayer->first,SupportingAction((*aItr)->actID, (*aItr)->ts, (*aItr)->howManyTimes, thresholdItr->second.second)));
                                        recordSideEffects(*aItr, (*aItr)->howManyTimes, previousFluentLayer->second->values());
                                    } else {
                                        residual -= nInt * eff;
                                        actionsUsed.push_back(make_pair(currentLayer->first,SupportingAction((*aItr)->actID, (*aItr)->ts, nInt, thresholdItr->second.second)));
                                        recordSideEffects(*aItr, nInt, previousFluentLayer->second->values());
                                    }
                                }
                            }
                            
                        }
                    }
                    
                    if (residual > acceptableMinimum) {
                        
                        if (debug) {
                            cout << "  * Needs gradient effects\n";
                        }
                        // must need some gradient effects too
                        LayerMap::iterator layerWithGradientOnThisVar = preconditionsIn;
                        
                        const bool keepPositive = (tItr->first < FluentLayerEntry::pneCount);
                        
                        map<int, map<double, list<const ActionAndHowManyTimes*> > >::const_iterator geItr;
                        
                        while (layerWithGradientOnThisVar != layers.begin()) {
                            map<int, map<double, list<const ActionAndHowManyTimes*> > > & geffs = layerWithGradientOnThisVar->second->getGradientEffectsDirectlyBeforeThisLayer();
                            if (keepPositive) {
                                geItr = geffs.find(tItr->first);
                            } else {
                                geItr = geffs.find(tItr->first - FluentLayerEntry::pneCount);
                            }
                            bool rightWay = false;
                            if (geItr != geffs.end()) {
                                map<double, list<const ActionAndHowManyTimes*> >::const_iterator effItr = geItr->second.begin();
                                const map<double, list<const ActionAndHowManyTimes*> >::const_iterator effEnd = geItr->second.end();
                                
                                for (; effItr != effEnd && residual > acceptableMinimum; ++effItr) {
                                    if (keepPositive) {
                                        if (effItr->first > 0) {
                                            rightWay = true;
                                            break;
                                        }
                                    } else {
                                        if (effItr->first < 0) {
                                            rightWay = true;
                                            break;
                                        }
                                    }
                                }
                            }
                            
                            if (rightWay) {
                                if (debug) {
                                    cout << "    + Suitable gradient effects in layer " << layerWithGradientOnThisVar->first << endl;
                                }                                                               
                                break;
                            } else {
                                if (debug) {
                                    cout << "    + No suitable gradient effects in layer " << layerWithGradientOnThisVar->first << endl;
                                }
                                
                                --layerWithGradientOnThisVar;
                            }
                        }
                        
                        // there are no active gradients in fact layer 0.0, so if this happens,
                        // there's an unexplained shortfall
                        assert(layerWithGradientOnThisVar != layers.begin());
                        
                        LayerMap::iterator layerBeforeGradientOnThisVar = layerWithGradientOnThisVar;
                        --layerBeforeGradientOnThisVar;
                        
                        map<double, list<const ActionAndHowManyTimes*> >::const_iterator effItr = geItr->second.begin();
                        const map<double, list<const ActionAndHowManyTimes*> >::const_iterator effEnd = geItr->second.end();
                        
                        for (; effItr != effEnd && residual > acceptableMinimum; ++effItr) {
                            if (keepPositive) {
                                if (effItr->first < 0) {
                                    if (debug) {
                                        cout << "    * Ignoring decrease with gradient " << effItr->first << endl;
                                    } else {
                                        cout << "    * Keeping increase with gradient " << effItr->first << endl;
                                    } 
                                    continue;
                                }
                            } else {
                                if (effItr->first > 0) {
                                    if (debug) {
                                        cout << "    * Ignoring increase with gradient " << effItr->first << endl;
                                    } else {
                                        cout << "    * Keeping decrease with gradient " << effItr->first << endl;
                                    } 
                                    
                                    continue;
                                }
                            }
                                                            
                            const double eff = fabs(effItr->first);
                        
                            
                            list<const ActionAndHowManyTimes*>::const_iterator aItr = effItr->second.begin();
                            const list<const ActionAndHowManyTimes*>::const_iterator aEnd = effItr->second.end();
                            
                            for (; aItr != aEnd && residual > acceptableMinimum; ++aItr) {
                                
                                const double durationOfEffect = ((*aItr)->gradientsFinishAt < preconditionsIn->first ? (*aItr)->gradientsFinishAt : preconditionsIn->first) - layerWithGradientOnThisVar->first;
                                
                                const double overThatTime = durationOfEffect * eff;
                                
                                const double needed = ceil(durationOfEffect / ((*aItr)->maxDur));
                                
                                if (debug) {
                                    cout << "       - Using " << needed << " of " << *(RPGBuilder::getInstantiatedOp((*aItr)->actID)) << endl;
                                }
                                                                    
                                residual -= overThatTime;
                                
                                if (!(*aItr)->alreadyInThePlan) {
                                    actionsUsed.push_back(make_pair(layerWithGradientOnThisVar->first,SupportingAction((*aItr)->actID, (*aItr)->ts, needed, thresholdItr->second.second)));
                                    recordSideEffects(*aItr, needed, layerBeforeGradientOnThisVar->second->values());
                                }
                            }
                        }
                        
                    
                    }
                    
                    if (residual > borrowFactLayerZeroValues()[tItr->first]) {
                        LayerMap::const_iterator rollback = previousFluentLayer;
                        while (rollback->second->values()[tItr->first] > residual) {
                            --rollback;
                        }
                        ++rollback;
                        goalsAtLayer[rollback->first].requestNumericThreshold(tItr->first, residual, thresholdItr->second.second);                                        
                    }
                }
            }
        }        
    }
    
    void breakApart(const vector<int> & variables, const vector<double> & weights, const int & size, const double & constant,
                    const bool & useMaxVariables, double threshold,
                    const LayerMap::const_iterator & reachableValues,
                    list<pair<int,double> > & subTerms) {
        
        const vector<double> & values = reachableValues->second->values();
        
        threshold -= constant;
        
        for (int s = 0; s < size && threshold > 0.0; ++s) {
            
            int v = variables[s];
            double w = weights[s];
            
            if (w < 0.0) {
                w = -w;
                v += FluentLayerEntry::pneCount;
            }
            
            if (!useMaxVariables) {
                if (v < FluentLayerEntry::pneCount) {
                    v += FluentLayerEntry::pneCount;
                } else {
                    v -= FluentLayerEntry::pneCount;
                }
            }
            
            if (values[v] * w >= threshold) {
                subTerms.push_back(make_pair(v, threshold / w));
                return;
            } else {
                subTerms.push_back(make_pair(v, values[v]));
                threshold -= w * values[v];
            }
        }
        
    }
                                       
};


int FluentLayers::FluentLayerEntry::pneCount;

struct NextFactLayer {
    
    pair<double,bool> timestamp;
    
    map<double, list<int>, EpsilonComp >::iterator endActionsAppearingAtThisTime;
    map<double, FactLayerEntry, EpsilonComp >::iterator newFactsAtThisTime;    
    bool revisitInstantaneousNumericEffects;
    bool gradientsCauseFactsToBecomeTrue;
    
    NextFactLayer() {
        reset();
    } 
                
    bool empty() const {
        return (timestamp.first == DBL_MAX);
    }
    
    void reset() {
        timestamp.first = DBL_MAX;
        timestamp.second = false;
        revisitInstantaneousNumericEffects = false;
        gradientsCauseFactsToBecomeTrue = false;
    }
};

struct DotDetails {
    
    struct ClusterDef {
        map<string,string> nodes;
    };
    map<double, ClusterDef > factLayerNodes;
    map<double, ClusterDef > actionLayerNodes;
    
    list<string> endActionsToStarts;
    list<string> factsToActions;
    map<int, string> achieverForFact;
    
    set<int> haveAnOpenEnd;
    
    set<int> highlightedEnd;
    set<int> highlightedStart;
    
    bool rawHighlight(const string & s, const bool failureOkay=false) {
        bool found = false;
        map<double, ClusterDef >::iterator alItr = actionLayerNodes.begin();
        const map<double, ClusterDef >::iterator alEnd = actionLayerNodes.end();
        
        for (; alItr != alEnd; ++alItr) {
            map<string,string>::iterator nItr = alItr->second.nodes.find(s);
            if (nItr != alItr->second.nodes.end()) {
                nItr->second += ", color=green";
                //cout << nItr->second << endl;
                found = true;
            }
        }
        if (!failureOkay) {
            assert(found);
        }
        return found;
    }
    
    void highlightAction(const int & act, const VAL::time_spec & ts) {
        switch (ts) {
            case VAL::E_AT_START: {
                if (!highlightedStart.insert(act).second) {
                    return;
                }
                ostringstream c1;
                c1 << "act" << act << "s";
                rawHighlight(c1.str());
                break;
            }
            case VAL::E_AT_END: {
                if (!highlightedEnd.insert(act).second) {
                    return;
                }
                bool found = false;
                {
                    ostringstream c1;
                    c1 << "act" << act << "e";
                    found = rawHighlight(c1.str(),true);                    
                }
                {
                    ostringstream c1;
                    c1 << "act" << act << "ne";
                    rawHighlight(c1.str(),found);                    
                }
                break;
            }
            default: {
            }
        }
        
    }
    
    void actionMeetsFactInRP(const int & act, const VAL::time_spec & ts, const int & fact) {
        switch (ts) {
            case VAL::E_AT_START: {
                ostringstream c1;
                c1 << "\tact" << act << "s -> fact" << fact << " [color=green];\n";
                factsToActions.push_back(c1.str());
                break;
            }
            case VAL::E_AT_END: {
                if (haveAnOpenEnd.find(act) != haveAnOpenEnd.end()) {
                    ostringstream c1;
                    c1 << "\tact" << act << "ne -> fact" << fact << " [color=green];\n";
                    factsToActions.push_back(c1.str());
                    break;
                } else {
                    ostringstream c1;
                    c1 << "\tact" << act << "e -> fact" << fact << " [color=green];\n";
                    factsToActions.push_back(c1.str());
                    break;
                }
            }
            default: {
            }
        }
    };
    
    void addFactNode(const double & t, const int & fact, const bool inState=false) {
        if (RPGBuilder::isStatic(RPGBuilder::getLiteral(fact)).first) {
            return;
        }
        ostringstream c1;
        ostringstream c2;
        c1 << "fact" << fact;
        if (inState) {
            c2 << "color=red, ";
        }
        c2 << "label=\"" << *(RPGBuilder::getLiteral(fact)) << "\"";
        
        const string nn = c1.str();
        
        factLayerNodes[t].nodes[nn] = c2.str();
    }

    void addNumericFactNode(const double & t, const int & pre, const bool inState=false) {
        ostringstream c1;
        c1 << "numfact" << pre;
        const string nn = c1.str();
        
        ostringstream c;
        if (inState) {
            c << "color=red, ";
        }
        
        c << "label=\"";
        const RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[pre];
        const int pneCount = RPGBuilder::getPNECount();
        if (currPre.LHSVariable < pneCount) {
            c << *(RPGBuilder::getPNE(currPre.LHSVariable));
            if (currPre.op == VAL::E_GREATER) {
                c << " > ";
            } else {
                c << " >= ";
            }
            c << currPre.RHSConstant;
        } else if (currPre.LHSVariable < 2 * pneCount) {
            c << *(RPGBuilder::getPNE(currPre.LHSVariable - pneCount));
            if (currPre.op == VAL::E_GREATER) {
                c << " < ";
            } else {
                c << " <= ";
            }
            if (currPre.RHSConstant == 0.0) {
                c << "0.0";
            } else {
                c << -currPre.RHSConstant;
            }
        } else {
            
            const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(currPre.LHSVariable);
            
            double localRHS = currPre.RHSConstant - currAV.constant;
            
            if (currAV.size == 1 && (currAV.fluents[0] >= pneCount) && (currAV.fluents[0] < (2 * pneCount))) {
                c << *(RPGBuilder::getPNE(currAV.fluents[0] - pneCount));
                if (currPre.op == VAL::E_GREATER) {
                    c << " < ";
                } else {
                    c << " <= ";
                }
                if (localRHS == 0.0) {
                    c << "0.0";
                } else {
                    c << -localRHS;
                }
            } else {
                
                for (int s = 0; s < currAV.size; ++s) {
                    if (currAV.fluents[s] < pneCount) {
                        if (currAV.weights[s] != 1.0) {
                            if (currAV.weights[s] > 0.0) {
                                c << " + " << currAV.weights[s] << "*";
                            } else {
                                c << " - " << -currAV.weights[s] << "*";
                            }                            
                        } else {
                            if (s) {
                                c << " + ";
                            }                                                                                                       
                        }
                        c << *(RPGBuilder::getPNE(currAV.fluents[s]));
                    } else {
                        if (currAV.weights[s] != 1.0) {
                            if (currAV.weights[s] > 0.0) {
                                c << " - " << currAV.weights[s] << "*";
                            } else {
                                c << " + " << -currAV.weights[s] << "*";
                            }                            
                        } else {
                            if (s) {
                                c << " - ";
                            }                                                                                                       
                        }
                        c << *(RPGBuilder::getPNE(currAV.fluents[s] - pneCount));
                    }
                }
                
                if (currPre.op == VAL::E_GREATER) {
                    c << " > ";
                } else {
                    c << " >= ";
                }
                c << localRHS;
            }
            
        }
        
        c << "\"";
        factLayerNodes[t].nodes[nn] = c.str();
    }
        
    void addActionNode(const double & t, const int & act, const VAL::time_spec & ts) {
        {
            ostringstream c1;
            ostringstream c2;
            switch (ts) {
                case VAL::E_AT: {
                    c1 << "TIL" << act;
                    break;
                }
                case VAL::E_AT_END: {
                    if (RPGBuilder::getRPGDEs(act).empty()) {
                        return;
                    }
                    c1 << "act" << act << "e";
                    c2 << "label=\"" << *(RPGBuilder::getInstantiatedOp(act)) << " end\"";
                    break;
                }
                case VAL::E_AT_START: {
                    c1 << "act" << act << "s";
                    c2 << "label=\"" << *(RPGBuilder::getInstantiatedOp(act));
                    if (!RPGBuilder::getRPGDEs(act).empty()) {
                        c2 << " start";
                    }
                    c2 << "\"";
                    break;
                }
                default: {
                    cerr << "Internal error: unexpected time specifier for action node\n";
                    exit(1);
                }
            }
            
            const string nn = c1.str();
            actionLayerNodes[t].nodes[nn] = c2.str();
        }
        
        if (ts == VAL::E_AT_END) {
            ostringstream c2;
            c2 << "act" << act << "s -> act" << act << "e;\n";
            endActionsToStarts.push_back(c2.str());
        }
        
        if (ts == VAL::E_AT) {
            return;
        }
        
        {
            const list<Literal*> & pres = (ts == VAL::E_AT_START ? RPGBuilder::getProcessedStartPropositionalPreconditions()[act] : RPGBuilder::getEndPropositionalPreconditions()[act]);
        
            list<Literal*>::const_iterator pItr = pres.begin();
            const list<Literal*>::const_iterator pEnd = pres.end();
            
            for (; pItr != pEnd; ++pItr) {
                if (RPGBuilder::isStatic(*pItr).first) {
                    continue;
                }
                addFactToActionEdge((*pItr)->getStateID(), act, ts);
            }
        }
        
        {
            const list<int> & numPres = (ts == VAL::E_AT_START ? RPGBuilder::getProcessedStartRPGNumericPreconditions()[act] : RPGBuilder::getEndRPGNumericPreconditions()[act]);
            
            list<int>::const_iterator pItr = numPres.begin();
            const list<int>::const_iterator pEnd = numPres.end();
            
            for (; pItr != pEnd; ++pItr) {
                addNumFactToActionEdge(*pItr, act, ts);
            }
        }
    }
    
    void addNeededEnd(const double & t, const int & act) {
        haveAnOpenEnd.insert(act);
        {
            ostringstream c1;
            ostringstream c2;
            c1 << "act" << act << "ne";
            c2 << "label=\"" << *(RPGBuilder::getInstantiatedOp(act)) << " n-end\"";
            
            const string nn = c1.str();
            actionLayerNodes[t].nodes[nn] = c2.str();
        }
        
        const list<Literal*> & pres = RPGBuilder::getEndPropositionalPreconditions()[act];
        
        list<Literal*>::const_iterator pItr = pres.begin();
        const list<Literal*>::const_iterator pEnd = pres.end();
        
        for (; pItr != pEnd; ++pItr) {
            if (RPGBuilder::isStatic(*pItr).first) {
                continue;
            }
            
            ostringstream c;
            c << "fact" << (*pItr)->getStateID() << " -> act" << act << "ne;\n";
            factsToActions.push_back(c.str());
        }
    }

    void addFactToActionEdge(const int & fact, const int & act, const VAL::time_spec & ts) {
        ostringstream c;
        c << "fact" << fact << " -> act" << act;
        if (ts == VAL::E_AT_END) {
            c << "e;\n";
        } else {
            c << "s;\n";
        }
        factsToActions.push_back(c.str());
    }
    
    void addNumFactToActionEdge(const int & num, const int & act, const VAL::time_spec & ts) {
        ostringstream c;
        c << "numfact" << num << " -> act" << act;
        if (ts == VAL::E_AT_END) {
            c << "e;\n";
        } else {
            c << "s;\n";
        }
        factsToActions.push_back(c.str());
    }
    
    void addActionToFactEdge(const int & act, const VAL::time_spec & ts, const int & fact) {
        ostringstream c;
        switch (ts) {
            case VAL::E_AT: {
                c << "TIL" << act;
                break;
            }
            case VAL::E_AT_END: {      
                if (haveAnOpenEnd.find(act) != haveAnOpenEnd.end()) {
                    c << "act" << act << "ne";
                } else {
                    c << "act" << act << "e";
                }
                break;
            }
            case VAL::E_AT_START: {                
                c << "act" << act << "s";
                break;
            }
            default: {
                cerr << "Internal error: unexpected time specifier for action node\n";
                exit(1);
            }
        }
        c << " -> fact" << fact << ";\n";
        achieverForFact[fact] = c.str();
    }

    
    void printLayerDef(ostream & o, const bool & factLayer, const double & atTime, const int & idx, const ClusterDef & def) const {
        o << "\tsubgraph cluster_" << idx << " {\n";
        if (factLayer) {
            o << "\t\tstyle=filled;\n";
            o << "\t\tcolor=lightgrey;\n";
            o << "\t\tnode [style=filled,color=white];\n";
            o << "\t\tlabel = \"\";\n";
            o << "\t\tc" << idx << " [label = \"fl(" << atTime << ")\"];\n";
        } else {
            o << "\t\tnode [style=filled];\n";
            o << "\t\tcolor=blue;\n";            
            o << "\t\tlabel = \"\";\n";
            o << "\t\tc" << idx << " [label = \"al(" << atTime << ")\"];\n";
        }
        
        map<string,string>::const_iterator nItr = def.nodes.begin();
        const map<string,string>::const_iterator nEnd = def.nodes.end();
        
        for (; nItr != nEnd; ++nItr) {
            o << "\t" << nItr->first << " [" << nItr->second << "];\n";
        }
        
        o << "\t}\n\n";
    };
        
};
    
    
ostream & operator <<(ostream & o, const DotDetails & d) {
    o << "digraph RPG {\n";
    o << "\trankdir = LR;\n";
    int cluster = 0;
    
    map<double, DotDetails::ClusterDef >::const_iterator flItr = d.factLayerNodes.begin();
    const map<double, DotDetails::ClusterDef >::const_iterator flEnd = d.factLayerNodes.end();
    
    map<double, DotDetails::ClusterDef >::const_iterator alItr = d.actionLayerNodes.begin();
    const map<double, DotDetails::ClusterDef >::const_iterator alEnd = d.actionLayerNodes.end();
    
    while (flItr != flEnd && alItr != alEnd) {
        if (alItr->first - flItr->first > 0.0005) {
            d.printLayerDef(o, true, flItr->first, cluster, flItr->second);
            if (cluster) {
                o << "\tc" << cluster - 1 << " -> c" << cluster << " [ltail=cluster_" << cluster-1 << ", lhead=cluster_" << cluster << "];\n";
            }
            ++flItr;
        } else if (flItr->first - alItr->first > 0.0005) {
            d.printLayerDef(o, false, alItr->first, cluster, alItr->second);
            if (cluster) {
                o << "\tc" << cluster - 1 << " -> c" << cluster << " [ltail=cluster_" << cluster-1 << ", lhead=cluster_" << cluster << "];\n";
            }                        
            ++alItr;
        } else {
            d.printLayerDef(o, true, flItr->first, cluster, flItr->second);
            ++flItr;
            if (cluster) {
                o << "\tc" << cluster - 1 << " -> c" << cluster << " [ltail=cluster_" << cluster-1 << ", lhead=cluster_" << cluster << "];\n";
            }
            ++cluster;
            d.printLayerDef(o, false, alItr->first, cluster, alItr->second);
            ++alItr;                
            if (cluster) {
                o << "\tc" << cluster - 1 << " -> c" << cluster << " [ltail=cluster_" << cluster-1 << ", lhead=cluster_" << cluster << "];\n";
            }
        }
        
        ++cluster;
    }
    
    for (; flItr != flEnd; ++flItr, ++cluster) {
        d.printLayerDef(o, true, flItr->first, cluster, flItr->second);
        if (cluster) {
            o << "\tc" << cluster - 1 << " -> c" << cluster << " [ltail=cluster_" << cluster-1 << ", lhead=cluster_" << cluster << "];\n";
        }
        
    }
    
    for (; alItr != alEnd; ++alItr, ++cluster) {
        d.printLayerDef(o, false, alItr->first, cluster, alItr->second);
        if (cluster) {
            o << "\tc" << cluster - 1 << " -> c" << cluster << " [ltail=cluster_" << cluster-1 << ", lhead=cluster_" << cluster << "];\n";
        }
        
    };
   
    {
        list<string>::const_iterator nItr = d.endActionsToStarts.begin();
        const list<string>::const_iterator nEnd = d.endActionsToStarts.end();
        
        for (; nItr != nEnd; ++nItr) {
            o << "\t" << *nItr;
        }
        
        o << "\n";
    }
    
    {
        list<string>::const_iterator nItr = d.factsToActions.begin();
        const list<string>::const_iterator nEnd = d.factsToActions.end();
        
        for (; nItr != nEnd; ++nItr) {
            o << "\t" << *nItr;
        }

        o << "\n";
    }
    
    map<int, string>::const_iterator aItr = d.achieverForFact.begin();        
    const map<int, string>::const_iterator aEnd = d.achieverForFact.end();
    
    for (; aItr != aEnd; ++aItr) {
        o << "\t" << aItr->second;
    }
            
    o << "}\n\n";
                    
    return o;
}


class RPGHeuristic::Private
{

public:
    
    Private(const bool & b,
            vector<list<Literal*> > * atse,
            vector<list<Literal*> > * atee,
            vector<list<pair<int, VAL::time_spec> > > * eta,
            vector<list<Literal*> > * atsne,
            vector<list<Literal*> > * atene,
            vector<list<pair<int, VAL::time_spec> > > * neta,
            vector<list<pair<int, VAL::time_spec> > > * pta,
            vector<list<Literal*> > * atsp,
            vector<list<Literal*> > * ati,
            vector<list<Literal*> > * atep,
            vector<list<RPGBuilder::NumericEffect> > * atnuse,
            vector<list<RPGBuilder::NumericEffect> > * atnuee,
            vector<list<int> > * atrnuse,
            vector<list<int> > * atrnuee,
            vector<list<int> > * atnusp,
            vector<list<int> > * atnui,
            vector<list<int> > * atnuep,
            vector<list<int> > * atpnuep,
            vector<int> * iusp,
            vector<int> * iuip,
            vector<int> * iuep,
            vector<double> * ail,
            vector<double> * ailr,
            vector<pair<int, VAL::time_spec> > * ab,
            vector<pair<int, VAL::time_spec> > * abr,
            vector<double> * nail,
            vector<double> * nailr,
            vector<ActionFluentModification*> * nab,
            vector<ActionFluentModification*> * nabr,
            vector<int> * iunsp,
            vector<int> * iuni,
            vector<int> * iunep,
            vector<RPGBuilder::RPGNumericPrecondition> * rnp,
            vector<RPGBuilder::RPGNumericEffect> * rne,
            vector<list<pair<int, VAL::time_spec> > > * ppta,
            vector<list<pair<int, VAL::time_spec> > > * nppta,
            vector<list<Literal*> > * atpsp,
            vector<int> * iupsp,
            vector<int> * iupsnp,
            list<pair<int, VAL::time_spec> > * pla,
            list<pair<int, VAL::time_spec> > * onpa)
            :   actionsToStartEffects(atse),
            actionsToEndEffects(atee),
            effectsToActions(eta),
            actionsToStartNegativeEffects(atsne),
            actionsToEndNegativeEffects(atene),
            negativeEffectsToActions(neta),
            preconditionsToActions(pta),
            actionsToProcessedStartPreconditions(atpsp),
            actionsToStartPreconditions(atsp),
            actionsToInvariants(ati),
            actionsToEndPreconditions(atep),
            actionsToNumericStartEffects(atnuse),
            actionsToNumericEndEffects(atnuee),
            actionsToRPGNumericStartEffects(atrnuse),
            actionsToRPGNumericEndEffects(atrnuee),
            actionsToNumericStartPreconditions(atnusp),
            actionsToNumericInvariants(atnui),
            actionsToNumericEndPreconditions(atnuep),
            actionsToProcessedStartNumericPreconditions(atpnuep),
            initialUnsatisfiedStartPreconditions(iusp),
            initialUnsatisfiedInvariants(iuip),
            initialUnsatisfiedEndPreconditions(iuep),
            #ifdef POPF3ANALYSIS
            achieverDetails(ail->size()),
            isAGoalFactWithIndependentAchivementCosts(ail->size()),
            #else
            achievedInLayer(ail),
            achievedInLayerReset(ailr),
            achievedBy(ab),
            achievedByReset(abr),
            #endif
            numericAchievedInLayer(nail),
            numericAchievedInLayerReset(nailr),
            numericAchievedBy(nab),
            numericAchievedByReset(nabr),
            initialUnsatisfiedNumericStartPreconditions(iunsp),
            initialUnsatisfiedNumericInvariants(iuni),
            initialUnsatisfiedNumericEndPreconditions(iunep),
            rpgNumericPreconditions(rnp),
            rpgNumericEffects(rne),
            processedPreconditionsToActions(ppta),
            processedNumericPreconditionsToActions(nppta),
            initialUnsatisfiedProcessedStartPreconditions(iupsp),
            initialUnsatisfiedProcessedStartNumericPreconditions(iupsnp),
            preconditionlessActions(pla),
            onlyNumericPreconditionActions(onpa),
            deleteArrays(b), expandFully(false), doneIntegration(false), evaluateDebug(false) {

            
        earliestPropositionPOTimes = vector<double>(ail->size());
        earliestNumericPOTimes = vector<double>(RPGBuilder::getPNECount());
//            earliestNumericPrePOTimes = vector<double>(numericAchievedInLayer->size());
        
        numericIsTrueInState.resize(numericAchievedInLayer->size(), false);

    }


    struct EndPrecRescale {
        int ID;
        int var;
        double offset;
        double totalchange;
        double duration;

        EndPrecRescale(const int & v, const double & off, const double & tc, const double & dur) : var(v), offset(off), totalchange(tc), duration(dur) {};

        EndPrecRescale() {};

        EndPrecRescale(const EndPrecRescale & e, const double & remaining) : var(e.var), offset(e.offset), totalchange(e.totalchange *(remaining / duration)), duration(remaining) {};

        bool operator <(const EndPrecRescale & r) const;

        bool isSatisfied(const vector<double> & tab) const {
            return (tab[var] >= totalchange + offset);
        }

    };

    
    class BuildingPayload
    {

    public:

        DotDetails dot;
        
        const MinimalState & startState;
        const list<StartEvent> * startEventQueue;
        
        vector<int> startPreconditionCounts;
        vector<int> endPreconditionCounts;
        vector<int> numericStartPreconditionCounts;
        vector<int> numericEndPreconditionCounts;
        
        #ifdef POPF3ANALYSIS
        /* The actions waiting for the costs of their preconditions to be reduced, before they're considered applicable. */
        set<pair<int,VAL::time_spec> > actionsWaitingForForLowerCostPreconditions;
        #endif

        /** @brief Garbage collection for <code>FactLayer::endOfJustApplied</code>. */
        list<pair<set<int>, set<int> > > setsToForbid;
        
        map<double, FactLayerEntry, EpsilonComp > factLayers;
        FluentLayers fluentLayers;
        map<double, map<int, list<ActionFluentModification> >, EpsilonComp> fluentModifications;
        map<double, list<int>, EpsilonComp > endActionsAtTime;

        /** @brief Find the timestamp of the next fact layer to visit.
         * 
         * This is based on the earliest interesting point out of:
         *  - new propositional facts/numeric preconditions just added by actions
         *  - the ends of actions which have been scheduled to appear
         *  - new facts due to TILs
         *  - the next point at which a numeric precondition becomes satisfied based on
         *    the available continuous effects
         *
         *  @param currTS  The current timestamp (that of the last fact last layer visited)
         *  @param next    A reference to a <code>NextFactLayer</code> object, which is updated
         *                 to contain the iterators and timestamp for the next layer to visit.
         */
        void nextThingToVisit(const double & currTS, NextFactLayer & next) {
           
            const bool debug = (Globals::globalVerbosity & 64);
            next.reset();
            next.endActionsAppearingAtThisTime = endActionsAtTime.end();
            next.newFactsAtThisTime = factLayers.end();
            
            const map<double, list<int>, EpsilonComp >::iterator eaItr = endActionsAtTime.begin();
            if (eaItr != endActionsAtTime.end()) {
                next.endActionsAppearingAtThisTime = eaItr;
                next.timestamp.first = eaItr->first;
                                                               
                if (debug) {
                    cout << "Next action layer could occur alongside actions applicable due to fact layer " << next.timestamp.first << endl;
                }
                if (fabs((next.timestamp.first - EPSILON) - currTS) < 0.00001) {
                    next.timestamp.second = true; // if it's epsilon after the fact layer just visited                    
                    if (debug) {
                        cout << " - i.e. epsilon ahead of now\n";
                    }
                }                
            }
            
            const map<double, FactLayerEntry, EpsilonComp >::iterator flItr = factLayers.begin();
            if (flItr != factLayers.end()) {
                const bool isEpsilonLater = (fabs(flItr->first - (currTS + EPSILON)) < 0.000001);
                if (isEpsilonLater) {
                    if (debug) {
                        cout << "Next fact layer would be " << flItr->first << ", i.e. epsilon after now - keeping\n";
                    }
                    next.newFactsAtThisTime = flItr;
                    if (!next.timestamp.second) {
                        // if the next thing to visit wasn't epsilon in the future
                        // then it's later than this - make sure the iterator is wiped                        
                        next.endActionsAppearingAtThisTime = endActionsAtTime.end();
                        next.timestamp.second = true;
                        next.timestamp.first = flItr->first;
                    }                    
                } else if (!next.timestamp.second) { // if the facts are greater than epsilon ahead, and so is the other option so far (if any)
                    if (fabs(next.timestamp.first - flItr->first) < 0.000001) {
                        // the same time point, allowing for rounding errors
                        next.newFactsAtThisTime = flItr;
                        if (debug) {
                            cout << "Next fact layer would be at " << flItr->first << ", i.e. the same time as the end actions\n";
                        }
                        
                    } else if (flItr->first < next.timestamp.first) {
                        // this is sooner than the point at which ends of actions appear
                        if (debug) {
                            cout << "Next fact layer would be at " << flItr->first << ", i.e. sooner than the end actions, so not visiting those for now\n";
                        }
                                                                        
                        next.endActionsAppearingAtThisTime = endActionsAtTime.end();
                        next.newFactsAtThisTime = flItr;
                        next.timestamp.first = flItr->first;
                    }
                } else {
                    if (debug) {
                        cout << "Next fact layer would be " << flItr->first << ", later than the end actions need\n";
                    }
                }
            }
            
            bool isToRecalculate = false;
            bool isDueToGradients = false;
            const pair<double,bool> fluentLayersWantToRevisit = fluentLayers.nextLayerWouldBeAt(isToRecalculate, isDueToGradients);
            
            if (fluentLayersWantToRevisit.first != DBL_MAX) {

                if (fluentLayersWantToRevisit.second) {
                    
                    next.revisitInstantaneousNumericEffects = isToRecalculate;
                    next.gradientsCauseFactsToBecomeTrue = isDueToGradients;
                    
                    if (next.timestamp.second) {
                        if (debug) {
                            if (next.revisitInstantaneousNumericEffects) {
                                cout << "Also revisiting effects at the time epsilon from now\n";
                            } else {
                                cout << "Also revisiting preconditions that have become true due to gradients, epsilon from now\n";
                            }
                        }
                    } else {
                        if (debug) {
                            if (next.revisitInstantaneousNumericEffects) {
                                cout << "Revisiting effects at the time epsilon from now, ignoring other later options\n";
                            } else {
                                cout << "Also revisiting preconditions that have become true due to gradients, epsilon from now; ignoring other later options\n";
                            }
                        }
                        next.endActionsAppearingAtThisTime = endActionsAtTime.end();
                        next.newFactsAtThisTime = factLayers.end();
                        next.timestamp = fluentLayersWantToRevisit;
                    }
                } else {
                    
                    if (next.timestamp.second) {
                        if (debug) {
                            cout << "Ignoring fluent-layer update at " << fluentLayersWantToRevisit.first << ", is later than the other options\n";
                        }
                    } else {
                        
                        if ((fluentLayersWantToRevisit.first - next.timestamp.first ) > 0.00001) {
                            if (debug) {
                                cout << "Ignoring fluent-layer update at " << fluentLayersWantToRevisit.first << ", is later than the other options\n";
                            }
                        } else if (fabs(fluentLayersWantToRevisit.first - next.timestamp.first) < 0.00001) {
                            next.revisitInstantaneousNumericEffects = isToRecalculate;
                            next.gradientsCauseFactsToBecomeTrue = isDueToGradients;
                            if (debug) {
                                cout << "Fluent-layer update at " << fluentLayersWantToRevisit.first << ", alongside the other options\n";
                            }                            
                        } else {
                            next.endActionsAppearingAtThisTime = endActionsAtTime.end();
                            next.newFactsAtThisTime = factLayers.end();
                            next.timestamp = fluentLayersWantToRevisit;
                            
                            next.revisitInstantaneousNumericEffects = isToRecalculate;
                            next.gradientsCauseFactsToBecomeTrue = isDueToGradients;
                            if (debug) {
                                cout << "Fluent-layer update at " << fluentLayersWantToRevisit.first << ", earlier than the other options\n";
                            }
                        }
                    }
                }
            }
            
            if (debug) {
                cout << "After that: timestamp of next layer is " << next.timestamp.first << endl;
            }
        }
        
        vector<double> startActionSchedule;
        vector<double> endActionSchedule;
        vector<double> openEndActionSchedule;

        const map<int, set<int> > & startedActions;
        int unsatisfiedGoals;
        int unappearedEnds;
        bool tooExpensive;
        map<int, set<int> > insistUponEnds;
        map<int, int> forbiddenStart;
        map<int, int> forbiddenEnd;
        const int vCount;
        const int avCount;

        const vector<double> & minTimestamps;

        list<ActionSegment> & helpfulActions;

        map<int, double> earliestStartOf;

        map<int, pair<double, double> > propositionMustBeDeletedAddedAfter;

        list<pair<set<int>, set<int> > > gc;

        BuildingPayload(const MinimalState & theState, const list<StartEvent> * seq,
                        vector<int> & spc, vector<int> & epc, vector<int> & nspc, vector<int> & nepc,
                        const int & easSize, const int goalCount,
                        const vector<double> & mtsIn, list<ActionSegment> & haIn)
                : startState(theState), startEventQueue(seq),
                startPreconditionCounts(spc), endPreconditionCounts(epc),
                numericStartPreconditionCounts(nspc), numericEndPreconditionCounts(nepc),
                startActionSchedule(easSize, -1.0), endActionSchedule(easSize, -1.0), openEndActionSchedule(easSize, -1.0),
                startedActions(theState.startedActions),
                unsatisfiedGoals(goalCount), unappearedEnds(0),
#ifdef POPF3ANALYSIS
                tooExpensive(!NumericAnalysis::getGoalNumericUsageLimits().empty()),
#endif
                vCount(theState.secondMin.size()), avCount(RPGBuilder::getAVCount()),
                minTimestamps(mtsIn), helpfulActions(haIn)

        {
            assert(vCount == instantiatedOp::howManyNonStaticPNEs());
        }

    };

#ifdef MDIDEBUG

#define MAKEENDMDI(dest,iv,ot,a) MaxDependentInfo dest(iv,ot,a)
#define MAKESTARTMDI(dest,a) MaxDependentInfo dest(a)


    class MaxDependentInfo
    {

    private:

        double offsetToEarlier;
        bool haveCalculated;
        double value;

        VAL::time_spec ts;
        int currAct;


        static BuildingPayload * referTo;

    public:

        static bool debug;

        static void updatePayload(BuildingPayload* const p) {
            referTo = p;
        }



        MaxDependentInfo(const double & initialValue, const double & offsetTE, const int & actID)
                : offsetToEarlier(offsetTE), haveCalculated(false), value(initialValue),
                ts(VAL::E_AT_END), currAct(actID) {


            if (debug) {
                cout << "Made MDI for " << *(RPGBuilder::getInstantiatedOp(currAct)) << " end\n";
            }

        }


        MaxDependentInfo(const int & actID)
                : offsetToEarlier(0.001), haveCalculated(false), value(0.0),
                ts(VAL::E_AT_START), currAct(actID) {

#ifdef MDIDEBUG
            if (debug) {
                cout << "Made MDI for " << *(RPGBuilder::getInstantiatedOp(currAct)) << " start\n";
            }
#endif
        }


        const double & get() {
            if (haveCalculated) return value;
            if (debug) {
                cout << "MDI for " << *currAct;
                if (ts == VAL::E_AT_START) {
                    cout << " start";
                } else {
                    cout << " end";
                }

                cout << " = max[";
            }
            /*
                            for (int pass = 0; pass < 3; ++pass) {
                                double offset;
                                const list<Literal*> * loopOn;
                                const list<int> * numLoopOn;

                                switch(pass) {
                                    case 0:
                                    {
                                        offset = offsetToEarlier;
                                        loopOn = &(RPGBuilder::getStartPropositionalPreconditions()[currAct]);
                                        numLoopOn = &(RPGBuilder::getStartPreNumerics()[currAct]);
                                        break;
                                    }

                                    case 1:
                                    {
                                        offset = offsetToEarlier - 0.001;
                                        loopOn = &(RPGBuilder::getInvariantPropositionalPreconditions()[currAct]);
                                        numLoopOn = &(RPGBuilder::getInvariantNumerics()[currAct]);
                                        break;
                                    }

                                    case 2:
                                    {
                                        if (ts == VAL::E_AT_START) {
                                            loopOn = (list<Literal*>*)0;
                                            numLoopOn = (list<int>*)0;
                                        } else {
                                            offset = 0.001;
                                            loopOn = &(RPGBuilder::getEndPropositionalPreconditions()[currAct]);
                                            numLoopOn = &(RPGBuilder::getEndPreNumerics()[currAct]);
                                        }
                                        break;
                                    }

                                }

                                if (loopOn) {

                                    list<Literal*>::const_iterator preItr = loopOn->begin();
                                    const list<Literal*>::const_iterator preEnd = loopOn->end();

                                    for (; preItr != preEnd; ++preItr) {
                                        const int ID = (*preItr)->getID();
                                        const double poTS = RPGHeuristic::Private::earliestPropositionPOTimes[ID] + offset;
                                        if (debug) {
                                            if (pass == 0) {
                                                cout << " " << *(*preItr) << "s=" << poTS;
                                            } else if (pass == 1) {
                                                cout << " " << *(*preItr) << "i=" << poTS;
                                            } else {
                                                cout << " " << *(*preItr) << "e=" << poTS;
                                            }
                                        }
                                        if (poTS > value) value = poTS;
                                    }
                                }

                                if (numLoopOn) {
                                    list<int>::const_iterator preItr = numLoopOn->begin();
                                    const list<int>::const_iterator preEnd = numLoopOn->end();

                                    for (; preItr != preEnd; ++preItr) {
                                        RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[*preItr];
                                        const double poTS = RPGHeuristic::Private::earliestPointForNumericPrecondition(currPre) + offset;
                                        if (debug) {
                                            if (pass == 0) {
                                                cout << " num" << *preItr << "sn=" << poTS;
                                            } else if (pass == 1) {
                                                cout << " num" << *preItr << "in=" << poTS;
                                            } else {
                                                cout << " num" << *preItr << "en=" << poTS;
                                            }
                                        }

                                        if (poTS > value) value = poTS;
                                    }
                                }

                            }

                            const int passCount = (ts == VAL::E_AT_START ? 1 : 2);
                            for (int pass = 0; pass < passCount; ++pass) {
                                const double offset = (pass ? 0.001 : offsetToEarlier);
                                const list<Literal*> * const addLoop = (pass ? &(RPGBuilder::getEndPropositionAdds()[currAct]) : &(RPGBuilder::getStartPropositionAdds()[currAct]));
                                const list<Literal*> * const delLoop = (pass ? &(RPGBuilder::getEndPropositionDeletes()[currAct]) : &(RPGBuilder::getStartPropositionDeletes()[currAct]));
                                const list<int> * const numLoop = (pass ? &(RPGBuilder::getEndEffNumerics()[currAct]) : &(RPGBuilder::getStartEffNumerics()[currAct]));

                                {
                                    list<Literal*>::const_iterator preItr = addLoop->begin();
                                    const list<Literal*>::const_iterator preEnd = addLoop->end();

                                    for (; preItr != preEnd; ++preItr) {
                                        const int ID = (*preItr)->getID();
                                        const map<int, pair<double,double> >::iterator poRestrictItr = referTo->propositionMustBeDeletedAddedAfter.find(ID);
                                        if (poRestrictItr != referTo->propositionMustBeDeletedAddedAfter.end()) {
                                            const double poTS = poRestrictItr->second.second + offset;
                                            #ifdef MDIDEBUG
                                            if (debug) {
                                                if (pass == 0) {
                                                    cout << " " << *(*preItr) << "s+=" << poTS;
                                                } else {
                                                    cout << " " << *(*preItr) << "e-=" << poTS;
                                                }
                                            }
                                            #endif
                                            if (poTS > value) value = poTS;
                                        }
                                        #ifdef MDIDEBUG
                                        else {
                                            if (debug) {
                                                if (pass == 0) {
                                                    cout << " " << *(*preItr) << "s+=I";
                                                } else {
                                                    cout << " " << *(*preItr) << "e+=I";
                                                }
                                            }
                                        }
                                        #endif
                                    }
                                }

                                {
                                    list<Literal*>::const_iterator preItr = delLoop->begin();
                                    const list<Literal*>::const_iterator preEnd = delLoop->end();

                                    for (; preItr != preEnd; ++preItr) {
                                        const int ID = (*preItr)->getID();
                                        const map<int, pair<double,double> >::iterator poRestrictItr = referTo->propositionMustBeDeletedAddedAfter.find(ID);
                                        if (poRestrictItr != referTo->propositionMustBeDeletedAddedAfter.end()) {
                                            const double poTS = poRestrictItr->second.first + offset;
                                            #ifdef MDIDEBUG
                                            if (debug) {
                                                if (pass == 0) {
                                                    cout << " " << *(*preItr) << "s-=" << poTS;
                                                } else {
                                                    cout << " " << *(*preItr) << "e-=" << poTS;
                                                }
                                            }
                                            #endif
                                            if (poTS > value) value = poTS;
                                        }
                                        #ifdef MDIDEBUG
                                        else {
                                            if (debug) {
                                                if (pass == 0) {
                                                    cout << " " << *(*preItr) << "s-=I";
                                                } else {
                                                    cout << " " << *(*preItr) << "e-=I";
                                                }
                                            }
                                        }
                                        #endif

                                    }
                                }

                                {
                                    list<int>::const_iterator preItr = numLoop->begin();
                                    const list<int>::const_iterator preEnd = numLoop->end();

                                    for (; preItr != preEnd; ++preItr) {
                                        const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*preItr];
                                        const double poTS = RPGHeuristic::Private::earliestPointForNumericEffect(currEff) + offset;
                                        #ifdef MDIDEBUG
                                        if (debug) {
                                            if (pass == 0) {
                                                cout << " " << *preItr << "sne=" << poTS;
                                            } else {
                                                cout << " " << *preItr << "ene=" << poTS;
                                            }
                                        }
                                        #endif
                                        if (poTS > value) value = poTS;

                                    }
                                }

                            }

                            if (RPGBuilder::getRPGDEs(currAct).empty()) {
                                const double poTS = earliestPointForDuration(*(RPGBuilder::getRPGDEs(currAct)[0])) + (ts == VAL::E_AT_START ? 0.001 : offsetToEarlier);

                                #ifdef MDIDEBUG
                                if (debug) {
                                    cout << " dur=" << poTS;
                                }
                                #endif
                                if (poTS > value) value = poTS;
                            }
            */

            if (debug) {
                cout << "] = " << value << "\n";
            }

            haveCalculated = true;


            return value;
        }

    };

#else

    typedef int MaxDependentInfo;

#define MAKEENDMDI(dest,iv,ot,a) MaxDependentInfo dest = a
#define MAKESTARTMDI(dest,a) MaxDependentInfo dest = a
#endif

    set<int> goals;
    set<int>::iterator gsEnd;
    set<int> goalFluents;
    set<int>::iterator gfEnd;

    vector<Literal*> literalGoalVector;

    vector<double> maxNeeded;

    vector<list<Literal*> > * const actionsToStartEffects;
    vector<list<Literal*> > * const actionsToEndEffects;
    vector<list<pair<int, VAL::time_spec> > > * const effectsToActions;

    vector<list<Literal*> > * const actionsToStartNegativeEffects;
    vector<list<Literal*> > * const actionsToEndNegativeEffects;
    vector<list<pair<int, VAL::time_spec> > > * const negativeEffectsToActions;


    vector<list<pair<int, VAL::time_spec> > > * const preconditionsToActions;

    vector<list<Literal*> > * const actionsToProcessedStartPreconditions;
    vector<list<Literal*> > * const actionsToStartPreconditions;
    vector<list<Literal*> > * const actionsToInvariants;
    vector<list<Literal*> > * const actionsToEndPreconditions;

    vector<list<RPGBuilder::NumericEffect> > * const actionsToNumericStartEffects;
    vector<list<RPGBuilder::NumericEffect> > * const actionsToNumericEndEffects;

    vector<list<int> > integratedCTSEffectVar;
    vector<list<double> > integratedCTSEffectChange;

    vector<list<int> > gradientCTSEffectVar;
    vector<list<double> > gradientCTSEffectChange;
    
    vector<list<int> > * const actionsToRPGNumericStartEffects;
    vector<list<int> > * const actionsToRPGNumericEndEffects;

    vector<list<int> > * const actionsToNumericStartPreconditions;
    vector<list<int> > * const actionsToNumericInvariants;
    vector<list<int> > * const actionsToNumericEndPreconditions;
    vector<list<int> > * const actionsToProcessedStartNumericPreconditions;

    vector<int> * const initialUnsatisfiedStartPreconditions;
    vector<int> * const initialUnsatisfiedInvariants;
    vector<int> * const initialUnsatisfiedEndPreconditions;

    #ifdef POPF3ANALYSIS
    vector<list<CostedAchieverDetails> > achieverDetails;
    vector<bool> isAGoalFactWithIndependentAchivementCosts;
    vector<double> nullCosts;                  
    vector<double> currentCosts;
    #else 
    vector<double> * const achievedInLayer;
    vector<double> * const achievedInLayerReset;
    vector<pair<int, VAL::time_spec> > * const achievedBy;
    vector<pair<int, VAL::time_spec> > * const achievedByReset;
    #endif
    
    vector<double> * const numericAchievedInLayer;
    vector<double> * const numericAchievedInLayerReset;
    vector<bool> numericIsTrueInState;
    vector<ActionFluentModification*> * const numericAchievedBy;
    vector<ActionFluentModification*> * const numericAchievedByReset;

    vector<int> * const initialUnsatisfiedNumericStartPreconditions;
    vector<int> * const initialUnsatisfiedNumericInvariants;
    vector<int> * const initialUnsatisfiedNumericEndPreconditions;

    vector<RPGBuilder::RPGNumericPrecondition> * const rpgNumericPreconditions;
    vector<RPGBuilder::RPGNumericEffect> * const rpgNumericEffects;


    vector<list<pair<int, VAL::time_spec> > > * const processedPreconditionsToActions;
    vector<list<pair<int, VAL::time_spec> > > * const processedNumericPreconditionsToActions;

    vector<int> * const initialUnsatisfiedProcessedStartPreconditions;
    vector<int> * const initialUnsatisfiedProcessedStartNumericPreconditions;

    list<pair<int, VAL::time_spec> > * const preconditionlessActions;
    list<pair<int, VAL::time_spec> > * const onlyNumericPreconditionActions;
    list<pair<int, VAL::time_spec> > noLongerForbidden;


    static vector<double> earliestStartAllowed;
    static vector<double> earliestEndAllowed;
    static vector<double> latestStartAllowed;
    static vector<double> latestEndAllowed;
    static vector<double> deadlineAtTime;
    static vector<double> earliestDeadlineRelevancyStart;
    static vector<double> earliestDeadlineRelevancyEnd;

    static vector<list<int> > tilEffects;
    static vector<list<int> > tilNegativeEffects;
    static vector<list<int> > tilTemporaryNegativeEffects;
    static vector<double> tilTimes;
    static bool tilInitialised;
    static int tilCount;

    static vector<double> earliestPropositionPOTimes;
    static vector<double> earliestNumericPOTimes;
//    static vector<double> earliestNumericPrePOTimes;

    static vector<vector<set<int> > > actionsAffectedByFluent;

    #ifdef POPF3ANALYSIS
    static vector<vector<double> > startEffectsOnResourceLimits;
    static vector<vector<double> > endEffectsOnResourceLimits;
    static vector<bool> costsAreIndependentGoalCosts;
    
    /** @brief The maximum possible useful cost of a given literal.
     *
     *  If all the actions needing a fact have an effect on a limited resource, then the maximum
     *  possible cost of that fact is that such that the minimum resulting effect does not
     *  violate the resource bounds.
     */
    static vector<vector<double> > maxPermissibleCostOfAFact;
    #endif
    
    const bool deleteArrays;
    bool expandFully;
    bool doneIntegration;
    bool evaluateDebug;


    /** @brief Find which action achieved a given fact, and when.
     *
     * This function is used during solution extraction to find which action to use to achieve a given fact.
     *
     * @param  factID   The ID of the propositional fact
     * @param  layerTS  The action layer containing the action whose precondition needs to be supported
     * @param  returnAchievedIn   After execution, contains the layer in which to add the proposition as an intermediate goal
     * @param  returnAchiever     After execution, contains the achiever to use.
     */
    void getAchieverDetails(const int & factID, const double & layerTS,
                            double & returnAchievedIn, pair<int, VAL::time_spec> & returnAchiever) {
        
        #ifdef POPF3ANALYSIS
        list<CostedAchieverDetails>::const_reverse_iterator abItr = achieverDetails[factID].rbegin();
        assert(abItr != achieverDetails[factID].rend());
        while ((abItr->layer - layerTS) > (EPSILON/2)) {
            ++abItr;
            #ifndef NDEBUG
            if (abItr == achieverDetails[factID].rend()) {
                cout << std::setprecision(8) << std::fixed;
                cerr << "Internal error: asking for precondition " << *(RPGBuilder::getLiteral(factID)) << " at time " << layerTS << ", but the achievers were:\n";
                list<CostedAchieverDetails>::const_iterator abfItr = achieverDetails[factID].begin();
                const list<CostedAchieverDetails>::const_iterator abfEnd = achieverDetails[factID].end();
                
                for (; abfItr != abfEnd; ++abfItr) {
                    cout << "- at " << abfItr->layer << " - ";
                    if (abfItr->achiever.second == VAL::E_AT) {
                        cout << "TIL " << abfItr->achiever.first << endl;
                    } else if (abfItr->achiever.second == VAL::E_AT_END) {
                        cout << "end of " << *(RPGBuilder::getInstantiatedOp(abfItr->achiever.first)) << endl;
                    } else {
                        cout << "start of " << *(RPGBuilder::getInstantiatedOp(abfItr->achiever.first)) << endl;
                    }
                }
                exit(1);
            }
            #endif
        }
        //cout << "Using achiever that adds fact in layer " << abItr->layer << " to support a need in " << layerTS << endl;        
        returnAchievedIn = abItr->layer;
        returnAchiever = abItr->achiever;
        //++abItr;
        //if (abItr != achieverDetails[factID].rend()) {
        //    cout << "Previous achiever was in layer " << abItr->layer << endl;
        //}
        #else 
        returnAchievedIn = (*achievedInLayer)[factID];
        returnAchiever = (*achievedBy)[factID];
        #endif
        
    }
    
    int proposeNewAchiever(const int & factID,
                           const int & actID, const VAL::time_spec & ts,
                           const bool & openEnd,
                           const double & cTime) {
        #ifdef POPF3ANALYSIS
        if (!achieverDetails[factID].empty() && achieverDetails[factID].back().costFree) {
            // already have a cost-free achiever, can't do better
            return 0;
        }
        if (ts == VAL::E_AT || currentCosts.size() == 0) {
            // if it's a TIL, or there are no action costs
            achieverDetails[factID].push_back(CostedAchieverDetails(cTime, make_pair(actID, ts), currentCosts, true));
            return (achieverDetails[factID].size() > 1 ? 1 : 2);
        }
        const vector<NumericAnalysis::NumericLimitDescriptor> & limits = NumericAnalysis::getGoalNumericUsageLimits();
        
        const int ulCount = limits.size();
        
        vector<double> actCosts(currentCosts);
        
        bool costFree = true;
        
        for (int pPass = (openEnd ? 1 : 0); pPass < 2; ++pPass) {
            
            if (pPass == 1 && ts == VAL::E_AT_START) {
                break;
            }
            
            const list<Literal*> & pres = (pPass == 1 ? (*actionsToEndPreconditions)[actID] : (*actionsToProcessedStartPreconditions)[actID]);
            list<Literal*>::const_iterator pItr = pres.begin();
            const list<Literal*>::const_iterator pEnd = pres.end();
            
            for (; pItr != pEnd; ++pItr) {
                list<CostedAchieverDetails>::const_reverse_iterator cItr = achieverDetails[(*pItr)->getStateID()].rbegin();
                while (cTime - cItr->layer < (EPSILON / 2)) {
                    // find the achiever before now
                    ++cItr;
                    assert(cItr != achieverDetails[(*pItr)->getStateID()].rend());
                }
                for (int ul = 0; ul < ulCount; ++ul) {
                    if (cItr->costs[ul] > 0.0) {
                        if (actCosts[ul] < cItr->costs[ul]) {                        
                            actCosts[ul] = cItr->costs[ul];                        
                        }
                    } else if (cItr->costs[ul] < 0.0) {
                        if (actCosts[ul] > cItr->costs[ul]) {                        
                            actCosts[ul] = cItr->costs[ul];                        
                        }
                    }
                }
            }
            
            if (pPass == 0 && ts == VAL::E_AT_END) {
                // equivalent to an imaginary fact that the action has started: max of its preconditions, + the cost of the action itself
                
                const vector<double> & addOn = startEffectsOnResourceLimits[actID];
                
                for (int ul = 0; ul < ulCount; ++ul) {
                    actCosts[ul] += addOn[ul];           
                }
            }
        }
        
        {
            const vector<double> & addOn = (ts == VAL::E_AT_END ? endEffectsOnResourceLimits[actID] : startEffectsOnResourceLimits[actID]);
            
            for (int ul = 0; ul < ulCount; ++ul) {
                actCosts[ul] += addOn[ul];               
                if (fabs(actCosts[ul] - currentCosts[ul]) > 0.0000001)  {
                    costFree = false;
                }
            }
        }
        
        if (costFree) {
            // short-circuit the rest of the code: have a zero-cost achiever, which is great
            achieverDetails[factID].push_back(CostedAchieverDetails(cTime, make_pair(actID, ts), currentCosts, true));
            return (achieverDetails[factID].size() > 1 ? 1 : 2);
        }
                      
        for (int ul = 0; ul < ulCount; ++ul) {            
            switch (limits[ul].op) {
                case E_GREATER:
                {
                    if (actCosts[ul] <= maxPermissibleCostOfAFact[factID][ul]) {
                        // don't even keep it as an achiever if it could never be used
                        return false;
                    }
                    break;
                }
                case E_GREATEQ:
                {
                    if (actCosts[ul] < maxPermissibleCostOfAFact[factID][ul]) {
                        // don't even keep it as an achiever if it could never be used
                        return false;
                    }
                    break;
                }
                
                default:
                {
                    cerr << "Internal error: limits should be defined in terms of > or >=\n";
                    exit(1);
                }
            }
        }
        
        
        // if we get this far it's a plausible achiever
        
        if (achieverDetails[factID].empty()) {
            // first achiever - keep it
            achieverDetails[factID].push_back(CostedAchieverDetails(cTime, make_pair(actID, ts), actCosts, false));
            return 2;
        }
        
        // otherwise, do an admissible intersection approximation of the costs compared to the best recorded achiever
        
        bool anyImprovement = false;
        vector<double> admissibleCosts(achieverDetails[factID].back().costs);
        
        for (int ul = 0; ul < ulCount; ++ul) {
            if (admissibleCosts[ul] == currentCosts[ul]) {
                continue;
            }
            if (actCosts[ul] == currentCosts[ul]) {
                admissibleCosts[ul] = currentCosts[ul];
                anyImprovement = true;
                continue;
            }
            
            // now, both costs are non-zero
            if (actCosts[ul] > 0.0) {
                // for increases (towards upper bounds) keep the lowest increase
                if (admissibleCosts[ul] > actCosts[ul]) {
                    admissibleCosts[ul] = actCosts[ul];
                    anyImprovement = true;
                }
            } else if (actCosts[ul] < 0.0) {
                // for decreases (towards lower bounds) keep the smallest magnitude decrease
                if (admissibleCosts[ul] < actCosts[ul]) {
                    admissibleCosts[ul] = actCosts[ul];
                    anyImprovement = true;
                }
            }                
        }
        
        if (!anyImprovement) {
            // better off in all ways to use the previous achiever
            return 0;
        }
        
        costFree = true;
        
        for (int ul = 0; ul < ulCount; ++ul) {
            if (fabs(admissibleCosts[ul] - currentCosts[ul]) > 0.0000001) {
                costFree = false;
                break;
            }
        }
        
        achieverDetails[factID].push_back(CostedAchieverDetails(cTime, achieverDetails[factID].back().achiever, admissibleCosts, costFree));
        return 1;
        
        #else
        double & currAIL = (*(achievedInLayer))[factID];
        if (currAIL == -1.0) { // this is new
            currAIL = cTime;
            (*(achievedBy))[factID] = pair<int, VAL::time_spec>(actID, ts);
            return 2;
        }
        return 0;
        #endif
    }
    
    void setDebugFlag(const bool & b) {
        evaluateDebug = b;
    }

    void buildEmptyActionFluentLookupTable() {

        const int varCount = RPGBuilder::getPNECount();
        actionsAffectedByFluent = vector<vector<set<int> > >(varCount, vector<set<int> >(2));


    }

    void populateActionFluentLookupTable() {

        static bool populated = false;

        if (populated) return;

        populated = true;

        const int actCount = actionsToNumericStartPreconditions->size();
        const int varCount = RPGBuilder::getPNECount();
        
        const vector<RPGBuilder::RPGNumericPrecondition> & preTable = RPGBuilder::getNumericPreTable();
        const vector<RPGBuilder::RPGNumericEffect> & effTable = RPGBuilder::getNumericEff();

        for (int pass = 0; pass < 2; ++pass) {
            const vector<list<int> > * const nowOn = (pass ? actionsToNumericEndPreconditions : actionsToProcessedStartNumericPreconditions);

            for (int a = 0; a < actCount; ++a) {
                if (RPGBuilder::rogueActions[a]) continue;
                list<int>::const_iterator pItr = ((*nowOn)[a]).begin();
                const list<int>::const_iterator pEnd = ((*nowOn)[a]).end();

                for (; pItr != pEnd; ++pItr) {
                    const RPGBuilder::RPGNumericPrecondition & currPre = preTable[*pItr];
                    for (int side = 0; side < 2; ++side) {
                        int var = (side ? currPre.RHSVariable : currPre.LHSVariable);
                        if (var == -1) continue;

                        if (var < 2 * varCount) {
                            if (var >= varCount) var -= varCount;
                            assert(var < varCount);
                            assert(var >= 0);
                            assert(!RPGBuilder::rogueActions[a]);
                            actionsAffectedByFluent[var][pass].insert(a);
                        } else {
                            const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(var);

                            for (int i = 0; i < currAV.size; ++i) {
                                int lvar = currAV.fluents[i];
                                if (lvar >= varCount) lvar -= varCount;
                                assert(lvar < varCount);
                                assert(lvar >= 0);
                                assert(!RPGBuilder::rogueActions[a]);
                                actionsAffectedByFluent[lvar][pass].insert(a);
                            }
                        }
                    }
                }
            }
        }

        
         
        for (int pass = 0; pass < 2; ++pass) {
            const vector<list<int> > * const nowOn = (pass ? actionsToRPGNumericEndEffects : actionsToRPGNumericStartEffects);

            for (int a = 0; a < actCount; ++a) {
                if (RPGBuilder::rogueActions[a]) continue;



                list<int>::const_iterator eItr = ((*nowOn)[a]).begin();
                const list<int>::const_iterator eEnd = ((*nowOn)[a]).end();

                for (; eItr != eEnd; ++eItr) {
                    const RPGBuilder::RPGNumericEffect & currEff = effTable[*eItr];
                    assert(!RPGBuilder::rogueActions[a]);
                    
                    
                    
                    actionsAffectedByFluent[currEff.fluentIndex][pass].insert(a);

                    for (int i = 0; i < currEff.size; ++i) {
                        int var = currEff.variables[i];
                        if (var < 0) continue;
                        if (var >= varCount) var -= varCount;
                        actionsAffectedByFluent[var][pass].insert(a);
                    }
                }
                
                
            }
        }


        for (int a = 0; a < actCount; ++a) {
            if (RPGBuilder::rogueActions[a]) continue;
            const vector<RPGBuilder::RPGDuration*> & currDEs = RPGBuilder::getRPGDEs(a);
            if (currDEs.empty()) continue;

            for (int pass = 0; pass < 3; ++pass) {
                const list<RPGBuilder::DurationExpr*> & currList = (*(currDEs[0]))[pass];

                list<RPGBuilder::DurationExpr*>::const_iterator clItr = currList.begin();
                const list<RPGBuilder::DurationExpr*>::const_iterator clEnd = currList.end();

                for (; clItr != clEnd; ++clItr) {
                    const int loopLim = (*clItr)->variables.size();
                    for (int i = 0; i < loopLim; ++i) {
                        assert(!RPGBuilder::rogueActions[a]);
                        #ifdef STOCHASTICDURATIONS
                        int & vToUse = (*clItr)->variables[i].first;
                        #else
                        int & vToUse = (*clItr)->variables[i];
                        #endif
                        if (vToUse != -1) {
                            actionsAffectedByFluent[vToUse][0].insert(a);
                        }
                    }
                }
            }
        }



        vector<RPGBuilder::LinearEffects*> & LD = RPGBuilder::getLinearDiscretisation();

        if (!LD.empty()) {

            for (int a = 0; a < actCount; ++a) {
                if (RPGBuilder::rogueActions[a]) continue;
                const RPGBuilder::LinearEffects * const currLD = LD[a];

                if (!currLD) continue;
                const int effCount = currLD->vars.size();

                for (int eff = 0; eff < effCount; ++eff) {
                    assert(!RPGBuilder::rogueActions[a]);
                    actionsAffectedByFluent[currLD->vars[eff]][0].insert(a);
                }
            }

        }
        
        #ifdef POPF3ANALYSIS
        
        metricHasChanged();
                
        #endif
    }
    
    #ifdef POPF3ANALYSIS
    
    void metricHasChanged() {
        
        const int actCount = actionsToNumericStartPreconditions->size();
                        
        const vector<RPGBuilder::RPGNumericEffect> & effTable = RPGBuilder::getNumericEff();
         
        
        const vector<NumericAnalysis::NumericLimitDescriptor> & limits = NumericAnalysis::getGoalNumericUsageLimits();
        
        const int ulCount = limits.size();
        
        nullCosts.resize(ulCount);
        
        for (int l = 0; l < ulCount; ++l) {
            nullCosts[l] = 0.0;
        }
                                                                
                                                                        
        
        startEffectsOnResourceLimits.clear();
        startEffectsOnResourceLimits.resize(actCount, nullCosts);
        endEffectsOnResourceLimits.clear();
        endEffectsOnResourceLimits.resize(actCount, nullCosts);        
        costsAreIndependentGoalCosts.clear();
        costsAreIndependentGoalCosts.resize(actCount, false);
        
        {            
            
            vector<double> nanCosts(ulCount);
            for (int l = 0; l < ulCount; ++l) {
                nanCosts[l] = std::numeric_limits< double >::signaling_NaN();
            }
            maxPermissibleCostOfAFact.clear();
            maxPermissibleCostOfAFact.resize(achieverDetails.size(), nanCosts);                                
            
            //cout << "Created space for cost details of " << maxPermissibleCostOfAFact.size() << " facts, and " << nanCosts.size() << " costs\n";
            vector<double> hardLimits(ulCount);
            for (int l = 0; l < ulCount; ++l) {
                hardLimits[l] = limits[l].limit;
            }
            
            set<int>::const_iterator goalItr = goals.begin();
            const set<int>::const_iterator goalEnd = goals.end();
            for (; goalItr != goalEnd; ++goalItr) {
                maxPermissibleCostOfAFact[*goalItr] = hardLimits;
            }
        
        }
            
        
        // Make some maps to quickly determine which variables are relevant to which limits        
        map<int,set<pair<int,double> > > varToLowerResourceLimitVariable;
        
        {          
            for (int l = 0; l < ulCount; ++l) {
                switch (limits[l].op) {
                    case VAL::E_GREATEQ:
                    case VAL::E_GREATER:
                    {
                        map<int,double>::const_iterator vItr = limits[l].var.begin();
                        const map<int,double>::const_iterator vEnd = limits[l].var.end();                        
                        
                        for (; vItr != vEnd; ++vItr) {
                            varToLowerResourceLimitVariable[vItr->first].insert(make_pair(l, vItr->second));
                        }
                        break;
                    }
                    default:
                    {
                        cerr << "Internal error: should not have limits specified as anything other than >= or >\n";
                        exit(1);
                    }
                }
            }
        }
        
        {
            const vector<map<int, list<int> > > & igc = NumericAnalysis::getIndependentGoalCosts();
            
            const int igcSize = igc.size();
            
            for (int gID = 0; gID < igcSize; ++gID) {
                if (literalGoalVector[gID] && !igc[gID].empty()) {
                    
                    isAGoalFactWithIndependentAchivementCosts[literalGoalVector[gID]->getStateID()] = true;
                    
                    map<int, list<int> >::const_iterator lItr = igc[gID].begin();
                    const map<int, list<int> >::const_iterator lEnd = igc[gID].end();
                    
                    for (; lItr != lEnd; ++lItr) {
                        list<int>::const_iterator alItr = lItr->second.begin();
                        const list<int>::const_iterator alEnd = lItr->second.end();
                        
                        for (; alItr != alEnd; ++alItr) {
                            costsAreIndependentGoalCosts[*alItr] = true;
                        }
                    }
                }
            }
            
        }
        
        for (int pass = 0; pass < 2; ++pass) {
            const vector<list<int> > * const nowOn = (pass ? actionsToRPGNumericEndEffects : actionsToRPGNumericStartEffects);
            vector<vector<double> > & costsGoIn = (pass ? endEffectsOnResourceLimits : startEffectsOnResourceLimits);
                        
                        
            for (int a = 0; a < actCount; ++a) {
                if (RPGBuilder::rogueActions[a]) continue;

                set<int> costsChanged;

                list<int>::const_iterator eItr = ((*nowOn)[a]).begin();
                const list<int>::const_iterator eEnd = ((*nowOn)[a]).end();

                for (; eItr != eEnd; ++eItr) {
                    const RPGBuilder::RPGNumericEffect & currEff = effTable[*eItr];
                    assert(!RPGBuilder::rogueActions[a]);
                    
                    if (!currEff.size && !currEff.isAssignment) {
                        // resource limits are only detectable from constant-valued increase/decrease effects
                        // so check that is the case first
                        
                        {
                            // does it act on a lower bound?
                            map<int,set<pair<int,double> > >::const_iterator lsItr = varToLowerResourceLimitVariable.find(currEff.fluentIndex);
                            if (lsItr != varToLowerResourceLimitVariable.end()) {
                                set<pair<int,double> >::const_iterator lItr = lsItr->second.begin();
                                const set<pair<int,double> >::const_iterator lEnd = lsItr->second.end();
                                
                                for (; lItr != lEnd; ++lItr) {
                                    costsGoIn[a][lItr->first] += currEff.constant * lItr->second;
                                    costsChanged.insert(lItr->first);
                                }
                            }
                        }

                    }
                }
                
                set<int>::const_iterator ccItr = costsChanged.begin();
                const set<int>::const_iterator ccEnd = costsChanged.end();
                
                for (; ccItr != ccEnd; ++ccItr) {
                    for (int ipass = 0; ipass <= pass; ++ipass) {
                        const list<Literal*> & pres = (ipass ? RPGBuilder::getEndPropositionalPreconditions()[a]
                                                            : RPGBuilder::getProcessedStartPropositionalPreconditions()[a]);
                            
                                                            
                        double toUse = limits[*ccItr].limit;
                        
                        if (Globals::bestSolutionQuality > -DBL_MAX && limits[*ccItr].optimisationLimit) {
                            if (Globals::bestSolutionQuality + 0.001 > toUse) {
                                toUse = Globals::bestSolutionQuality + 0.001;
                            }
                        }
                        
                        if (toUse > -DBL_MAX) {
                            const double newPreLimit = toUse - costsGoIn[a][*ccItr];
                            {
                                list<Literal*>::const_iterator preItr = pres.begin();
                                const list<Literal*>::const_iterator preEnd = pres.end();
                                for (; preItr != preEnd; ++preItr) {
                                    vector<double> & toUpdate = maxPermissibleCostOfAFact[(*preItr)->getStateID()];
                                    if (toUpdate[*ccItr] == std::numeric_limits<double>::signaling_NaN()) {
                                        toUpdate[*ccItr] = newPreLimit;
                                    } else if (newPreLimit < toUpdate[*ccItr]) {
                                        toUpdate[*ccItr] = newPreLimit;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        const int factCount = achieverDetails.size();
        for (int l = 0; l < ulCount; ++l) {
            for (int f = 0; f < factCount; ++f) {
                if (maxPermissibleCostOfAFact[f][l] == std::numeric_limits<double>::signaling_NaN()) {
                    maxPermissibleCostOfAFact[f][l] = limits[l].limit;
                }
            }
        }
    }
    
    #endif

    void integrateContinuousEffects() {
        if (doneIntegration) return;

        if (evaluateDebug) {
            cout << "Integrating actions' effects\n";
        }
        maxNeeded = RPGBuilder::getMaxNeeded();

        vector<RPGBuilder::LinearEffects*> & LD = RPGBuilder::getLinearDiscretisation();

        const int loopLim = LD.size();

        integratedCTSEffectVar.resize(loopLim);
        integratedCTSEffectChange.resize(loopLim);
        gradientCTSEffectVar.resize(loopLim);
        gradientCTSEffectChange.resize(loopLim);
        

        set<int> cannotUseAsGradients;
        
        LiteralSet initialState;
        vector<double> initialFluents;
        
        RPGBuilder::getInitialState(initialState, initialFluents);
        
        MinimalState refState;
        refState.insertFacts(initialState.begin(), initialState.end(), StepAndBeforeOrAfter());
        
        refState.secondMin = initialFluents;
        refState.secondMax = initialFluents;
        
        
        
        if (!RPGHeuristic::makeCTSEffectsInstantaneous) {
            
            const vector<RPGBuilder::RPGNumericEffect> & numEffs = RPGBuilder::getNumericEff();
            
            const int lim = numEffs.size();
            
            int s;
            
            for (int e = 0; e < lim; ++e) {
                const RPGBuilder::RPGNumericEffect & currEff = numEffs[e];
                
                if ((s = currEff.size)) {
                    for (int v = 0; v < s; ++v) {
                        if (currEff.variables[v] >= 0) {
                            if (currEff.variables[v] >= RPGBuilder::getPNECount()) {
                                cannotUseAsGradients.insert(currEff.variables[v] - RPGBuilder::getPNECount());
                            } else {
                                cannotUseAsGradients.insert(currEff.variables[v]);
                            }
                        }
                    }
                }
            }
        }
        
        
        for (int lda = 0; lda < loopLim; ++lda) {
            if (RPGBuilder::rogueActions[lda]) continue;
            RPGBuilder::LinearEffects * const currLD = LD[lda];
            if (currLD) {
                
                bool mustIntegrate = RPGHeuristic::makeCTSEffectsInstantaneous;
                if (!mustIntegrate) {
                    mustIntegrate = (!RPGBuilder::isSelfMutex(lda) && RPGBuilder::howManyTimesOptimistic(lda, refState) > 1);
                }
                
                const double dur = RPGBuilder::getOpMaxDuration(lda, 0);
                const int effCount = currLD->vars.size();
                
                for (int eff = 0; eff < effCount; ++eff) {
                    if (mustIntegrate || cannotUseAsGradients.find(currLD->vars[eff]) != cannotUseAsGradients.end()) {
                        if (evaluateDebug) {
                            cout << "Integrating effect of ";
                            if (!RPGBuilder::isSelfMutex(lda)) {
                                cout << " non-self-mutex action ";
                            }
                            cout << *(RPGBuilder::getInstantiatedOp(lda)) << ":";
                        }
                        integratedCTSEffectVar[lda].push_back(currLD->vars[eff]);
                        integratedCTSEffectChange[lda].push_back(currLD->effects[0][eff].constant * dur);
                        if (evaluateDebug) {                            
                            cout << " " << integratedCTSEffectChange[lda].back() << " of " << *(RPGBuilder::getPNE(integratedCTSEffectVar[lda].back())) << endl;
                        } 
                    } else {
                        if (evaluateDebug) {
                            cout << "Not integrating effect of " << *(RPGBuilder::getInstantiatedOp(lda)) << ":";
                        }
                        gradientCTSEffectVar[lda].push_back(currLD->vars[eff]);
                        gradientCTSEffectChange[lda].push_back(currLD->effects[0][eff].constant);
                        if (evaluateDebug) {                            
                            cout << "d" << *(RPGBuilder::getPNE(gradientCTSEffectVar[lda].back())) << "/dt = " << gradientCTSEffectChange[lda].back() << endl;
                        } 
                    }
                }
                
                
            }
        }

        doneIntegration = true;
    }

    BuildingPayload * spawnNewPayload(const MinimalState & theState, const list<StartEvent> * seq, const vector<double> & minTimestamps, list<ActionSegment> & haIn) {

        static const int easSize = initialUnsatisfiedEndPreconditions->size();

        BuildingPayload * const toReturn =
            new BuildingPayload(theState, seq,
                                *(initialUnsatisfiedProcessedStartPreconditions),
                                *(initialUnsatisfiedEndPreconditions),
                                *(initialUnsatisfiedProcessedStartNumericPreconditions),
                                *(initialUnsatisfiedNumericEndPreconditions),
                                easSize, goals.size() + goalFluents.size(),
                                minTimestamps, haIn);

        
        vector<double> maxFluentTable(toReturn->vCount * 2 + toReturn->avCount);

        {
            if (evaluateDebug && toReturn->vCount) {
                cout << "Fluent table bounds for fluents in the range " << 0 << ".." << toReturn->vCount - 1 << ".  Number of non-static PNEs: " << instantiatedOp::howManyNonStaticPNEs() << endl;
            }
            for (int i = 0; i < toReturn->vCount; ++i) {
                if (evaluateDebug) {
                    cout << "Fluent " << i << " ";
                    cout.flush();
                    cout << *(RPGBuilder::getPNE(i)) << " bounds: [";
                }
                {
                    const double ov = theState.secondMax[i];
                    maxFluentTable[i] = ov;

                }
                {
                    const double ov = theState.secondMin[i];
                    if (ov != 0.0) {
                        maxFluentTable[i + toReturn->vCount] = 0.0 - ov;
                    } else {
                        maxFluentTable[i + toReturn->vCount] = 0.0;
                    }
                    if (evaluateDebug) cout << ov << ",";
                }
                if (evaluateDebug) {
                    cout << maxFluentTable[i] << "]\n";
                }
            }
        }
        {
            const int startLim = toReturn->vCount * 2;
            const int endLim = startLim + RPGBuilder::getAVCount();
            for (int i = startLim; i < endLim; ++i) {
                maxFluentTable[i] = RPGBuilder::getArtificialVariable(i).evaluate(maxFluentTable);
                if (evaluateDebug) cout << "AV " << i << " = " << maxFluentTable[i] << "\n";
            }
        }

        toReturn->fluentLayers.setFactLayerZero(maxFluentTable, earliestNumericPOTimes);



        {
            map<int, set<int> >::const_iterator saItr = theState.startedActions.begin();
            const map<int, set<int> >::const_iterator saEnd = theState.startedActions.end();

            for (; saItr != saEnd; ++saItr) {
                double earliest = DBL_MAX;

                set<int>::const_iterator instanceItr = saItr->second.begin();
                const set<int>::const_iterator instanceEnd = saItr->second.end();

                for (; instanceItr != instanceEnd; ++instanceItr) {
                    const double newTS = minTimestamps[*instanceItr];
                    if (newTS < earliest) earliest = newTS;
                }

                earliest += RPGBuilder::getOpMinDuration(saItr->first, 0);

                toReturn->earliestStartOf.insert(make_pair(saItr->first, earliest));
            }
        }

#ifdef MDIDEBUG
        MaxDependentInfo::updatePayload(toReturn);
#endif
        return toReturn;
    }

    /** @brief Prime the TRPG with the gradient effects of executing actions.
     *
     * @param payload                      The current TRPG data
     * @param timeAtWhichValueIsDefined    If the state numeric bound was immediately after the last relevant action, this is that time point.
     * @param gradientsToStart             Updated by this function to contain gradient effects to start corresponding to actions already in the plan that have not yet finished.
     */
    void giveUsTheEffectsOfExecutingActions(BuildingPayload * const payload,
                                            const vector<double> & timeAtWhichValueIsDefined,
                                            map<double, list<DelayedGradientDescriptor> > & gradientsToStart) {
        if (!payload->startEventQueue) {
            assert(payload->startState.startedActions.empty());
            return;
        }
        
        vector<double> * maxFluentTable = 0;
        vector<RPGBuilder::LinearEffects*> & LD = RPGBuilder::getLinearDiscretisation();
                
        list<StartEvent>::const_iterator evItr = payload->startEventQueue->begin();
        const list<StartEvent>::const_iterator evEnd = payload->startEventQueue->end();
        
        for (; evItr != evEnd; ++evItr) {

            const RPGBuilder::LinearEffects* currLD = LD[evItr->actID];
            if (!currLD) continue;
                       
            const double multiplier = evItr->maxDuration - evItr->elapsed;
            
            const vector<int> & varList = currLD->vars;
            const vector<RPGBuilder::LinearEffects::EffectExpression> & changeList = currLD->effects[0];

            const int effCount = varList.size();
            
            for (int e = 0; e < effCount; ++e) {
                if (timeAtWhichValueIsDefined.empty() || timeAtWhichValueIsDefined[varList[e]] < 0.0) {
                    const double change = changeList[e].constant * multiplier;
                    if (change > 0.0) {
                        if (!maxFluentTable) {
                            maxFluentTable = &(payload->fluentLayers.borrowFactLayerZeroValues());
                        }
                        (*maxFluentTable)[varList[e]] += change;
                    } else if (change < 0.0) {
                        if (!maxFluentTable) {
                            maxFluentTable = &(payload->fluentLayers.borrowFactLayerZeroValues());
                        }
                        (*maxFluentTable)[payload->vCount + varList[e]] += (0.0 - change);
                    }
                } else {
                    // if this is the case, we've been passed a time at which the bound on this
                    // variable was determined - which means we can fire off the 'effects of executing
                    // actions' as a proper gradient.
                    #ifdef DEBUG
                    if (RPGHeuristic::makeCTSEffectsInstantaneous) {
                        cerr << "Error: making continuous effects instantaneous (as in the the Colin IJCAI paper) is incompatible with the revised approach to state variable bounds for heuristic purposes.\n";
                        exit(1);
                    }
                    #endif      
                    
                    gradientsToStart[timeAtWhichValueIsDefined[varList[e]]].push_back(DelayedGradientDescriptor(evItr->actID, VAL::E_AT_START, multiplier, make_pair(varList[e], changeList[e].constant)));
                                                      
                }
            }

            //if (evaluateDebug || eeDebug) cout << "Clearing extra ends attached to " << *(RPGBuilder::getInstantiatedOp(saItr->first)) << "\n";
        }


    }

    /** @brief Add delayed gradient effects as slope functions rather than using integration.
     */
    void addDelayedGradientEffects(BuildingPayload * const payload, const map<double, list<DelayedGradientDescriptor> > & delayedGradients) {
        
        if (RPGBuilder::modifiedRPG) {
            // popf heuristic, so gradients might not start straight away
            cerr << "The -/ option is currently not implemented for the POPF heuristic - add -c or -T to the command line\n";
            exit(1);
        }
        
        static const bool localDebug = false;
        
        map<double, list<DelayedGradientDescriptor> >::const_iterator dgItr = delayedGradients.begin();
        const map<double, list<DelayedGradientDescriptor> >::const_iterator dgEnd = delayedGradients.end();
        
        for (; dgItr != dgEnd; ++dgItr) {
            // ignore the first entry of the pair, as we start everything straight away
            
            list<DelayedGradientDescriptor>::const_iterator gItr = dgItr->second.begin();
            const list<DelayedGradientDescriptor>::const_iterator gEnd = dgItr->second.end();
            
            for (; gItr != gEnd; ++gItr) {           
                if (localDebug) {
                    cout << "-- Adding gradient effect from executing action, d" << *(RPGBuilder::getPNE(gItr->gradientEffect.first)) << "/dt = " << gItr->gradientEffect.second << " until " << gItr->maxDur;
                    cout << ", from initial bounds [" << payload->startState.secondMin[gItr->gradientEffect.first] << "," << payload->startState.secondMax[gItr->gradientEffect.first] << "]\n";
                }
                payload->fluentLayers.recordGradientNumericEffect(*gItr, EPSILON, gItr->gradientEffect, payload->factLayers);
            }
        }
    }


    void resetAchievedBy() {
        #ifdef POPF3ANALYSIS
        const int fCount = achieverDetails.size();
        for (int f = 0; f < fCount; ++f) {
            achieverDetails[f].clear();
        }        
        #else
        *achievedBy = (*achievedByReset);
        *achievedInLayer = (*achievedInLayerReset);
        #endif
        
        *numericAchievedBy = (*numericAchievedByReset);
        *numericAchievedInLayer = (*numericAchievedInLayerReset);

    }

    static double localEarliestPointForNumericPrecondition(const RPGBuilder::RPGNumericPrecondition & p) {
        return earliestPointForNumericPrecondition(p, &earliestNumericPOTimes);
    }

    static double earliestPointForNumericEffect(const RPGBuilder::RPGNumericEffect & p) {

        static const int varCount = RPGBuilder::getPNECount();

        double TS = 0.0;

        if (earliestNumericPOTimes[p.fluentIndex] > TS) TS = earliestNumericPOTimes[p.fluentIndex];

        for (int s = 0; s < p.size; ++s) {
            int var = p.variables[s];
            if (var < 0) continue;
            if (var >= varCount) var -= varCount;
            if (earliestNumericPOTimes[var] > TS) TS = earliestNumericPOTimes[var];
        }

        return TS;
    }


    static double earliestPointForDuration(const RPGBuilder::RPGDuration & currDE) {
        double TS = 0.0;

        for (int pass = 0; pass < 3; ++pass) {
            const list<RPGBuilder::DurationExpr*> & currList = currDE[pass];
            list<RPGBuilder::DurationExpr*>::const_iterator lItr = currList.begin();
            const list<RPGBuilder::DurationExpr*>::const_iterator lEnd = currList.end();

            for (; lItr != lEnd; ++lItr) {
                const int vSize = ((*lItr)->variables.size());
                for (int i = 0; i < vSize; ++i) {
                    #ifdef STOCHASTICDURATIONS
                    const int var = (*lItr)->variables[i].first;
                    if (var == -1) continue;
                    #else
                    const int var = (*lItr)->variables[i];
                    #endif
                    if (earliestNumericPOTimes[var] > TS) TS = earliestNumericPOTimes[var];
                    
                }
            }
        }
        return TS;
    }

#ifdef TOTALORDERSTATES
    void recordFactLayerZero(BuildingPayload * const payload) {
        
        #ifdef POPF3ANALYSIS
        
        const vector<NumericAnalysis::NumericLimitDescriptor> & limits = NumericAnalysis::getGoalNumericUsageLimits();
        
        const int ulCount = limits.size();
        
        currentCosts = vector<double>(ulCount,0.0);
        
        for (int ul = 0; ul < ulCount; ++ul) {
            map<int,double>::const_iterator vItr = limits[ul].var.begin();
            const map<int,double>::const_iterator vEnd = limits[ul].var.end();
            
            for (; vItr != vEnd; ++vItr) {
                if (vItr->second >= 0.0) {
                    currentCosts[ul] += payload->startState.secondMax[vItr->first] * vItr->second;
                } else {
                    currentCosts[ul] += payload->startState.secondMin[vItr->first] * vItr->second;
                }
            }
        }
        
        #endif
    
        assert(!RPGBuilder::modifiedRPG);
        
        {
            StateFacts::const_iterator stateItr = payload->startState.first.begin();
            const StateFacts::const_iterator stateEnd = payload->startState.first.end();
            
            for (; stateItr != stateEnd; ++stateItr) {
                const int factID = FACTA(stateItr);
                #ifdef POPF3ANALYSIS
                achieverDetails[factID].push_back(CostedAchieverDetails(currentCosts,true));                
                #else
                (*achievedInLayer)[factID] = 0.0;
                #endif
            }
        }

        

        {
            const int loopLim = earliestNumericPOTimes.size();
            
            for (int i = 0; i < loopLim; ++i) {
                earliestNumericPOTimes[i] = 0.0;
            }
        }

        {
            const int loopLim = rpgNumericPreconditions->size();

            if (loopLim) {
                const vector<double> * const maxFluentTable = &(payload->fluentLayers.borrowFactLayerZeroValues());
                for (int i = 0; i < loopLim; ++i) {
                    if (ignoreNumbers || (*rpgNumericPreconditions)[i].isSatisfied(*maxFluentTable)) {
                        numericIsTrueInState[i] = true;
                        const double poTS = localEarliestPointForNumericPrecondition((*rpgNumericPreconditions)[i]);
                        //earliestNumericPrePOTimes[i] = poTS;
                        (*numericAchievedBy)[i] = 0;
                        (*numericAchievedInLayer)[i] = (RPGBuilder::modifiedRPG && poTS >= 0.001 ? (poTS - 0.001) : 0.0);
                        if (evaluateDebug && RPGBuilder::modifiedRPG && (*numericAchievedInLayer)[i] > 0.0) {
                            cout << "RPG modified: delaying numeric fact " << i << " to layer " << (*numericAchievedInLayer)[i] << "\n";
                        }
                        if (evaluateDebug && ignoreNumbers) {
                            cout << "Assuming numeric precondition " << i << " is satisfied\n";
                        }
                    } else {
                        numericIsTrueInState[i] = false;
                    }
                }
            }
        }

    }
#else

    void recordFactLayerZero(BuildingPayload * const payload) {
        
        #ifdef POPF3ANALYSIS
        
        const vector<NumericAnalysis::NumericLimitDescriptor> & limits = NumericAnalysis::getGoalNumericUsageLimits();
        
        const int ulCount = limits.size();
        
        currentCosts = vector<double>(ulCount,0.0);
        
        for (int ul = 0; ul < ulCount; ++ul) {
            map<int,double>::const_iterator vItr = limits[ul].var.begin();
            const map<int,double>::const_iterator vEnd = limits[ul].var.end();
            
            for (; vItr != vEnd; ++vItr) {
                if (vItr->second >= 0.0) {
                    currentCosts[ul] += payload->startState.secondMax[vItr->first] * vItr->second;
                } else {
                    currentCosts[ul] += payload->startState.secondMin[vItr->first] * vItr->second;
                }
            }
        }
        
        #endif
        
        if (RPGBuilder::modifiedRPG) {
            StateFacts::const_iterator stateItr = payload->startState.first.begin();
            const StateFacts::const_iterator stateEnd = payload->startState.first.end();

            for (; stateItr != stateEnd; ++stateItr) {
                const int factID = FACTA(stateItr);
                double * achievedTS;
                #ifdef POPF3ANALYSIS
                achieverDetails[factID].push_back(CostedAchieverDetails(currentCosts,true));
                achievedTS = &(achieverDetails[factID].back().layer);
                #else
                achievedTS = &((*achievedInLayer)[factID]);
                *achievedTS = 0.0;
                #endif
                pair<double, double> * toUpdate = 0;
                const StepAndBeforeOrAfter & from = stateItr->second.availableFrom;
                if (from.beforeOrAfter == StepAndBeforeOrAfter::BEFORE) { // special case: initial state is anything achieved /before/ step 0, rather than at step 0
                    earliestPropositionPOTimes[factID] = 0.0;
                    //if (!stateItr->second.deletableFrom.empty()) {
                    //    toUpdate = (&payload->propositionMustBeDeletedAddedAfter.insert(make_pair(factID,make_pair(0.0,0.0))).first->second);
                    //}
                    //if (!expandFully) cout << "Initial Fact " << *(RPGBuilder::getLiteral(factID)) << " is true in the initial state\n";
                } else {
                    const unsigned int dependsOnActionAtStep = from.stepID;
                    const double actTS = payload->minTimestamps[dependsOnActionAtStep];
                    earliestPropositionPOTimes[factID] = actTS;
                    //if (!expandFully) cout << "Initial Fact " << *(RPGBuilder::getLiteral(factID)) << " appears in PO no earlier than action " << dependsOnActionAtStep << ", i.e. t=" << earliestPropositionPOTimes[factID] << "\n";

                    toUpdate = &(payload->propositionMustBeDeletedAddedAfter.insert(make_pair(factID, make_pair(0.0, 0.0))).first->second);

                    if (toUpdate->first < actTS) toUpdate->first = actTS;
                    if (toUpdate->second < actTS) toUpdate->second = actTS;

                    if (RPGBuilder::modifiedRPG && actTS >= 0.001) {
                        if (evaluateDebug) cout << "RPG modified: delaying " << *(RPGBuilder::getLiteral(factID)) << " to layer " << (actTS - 0.001) << "\n";
                        *achievedTS = actTS - 0.001;
                    }

                }

                if (toUpdate) {
                    map<StepAndBeforeOrAfter, bool>::const_iterator invItr = stateItr->second.deletableFrom.begin();
                    const map<StepAndBeforeOrAfter, bool>::const_iterator invEnd = stateItr->second.deletableFrom.end();

                    for (; invItr != invEnd; ++invItr) {
                        const double actTS = payload->minTimestamps[invItr->first.stepID] - (invItr->first.beforeOrAfter == StepAndBeforeOrAfter::BEFORE ? 0.001 : 0.0);
                        if (actTS > 0.0) {
                            if (toUpdate->first < actTS) toUpdate->first = actTS;
                            if (evaluateDebug) cout << "RPG modified: cannot delete " << *(RPGBuilder::getLiteral(factID)) << " until layer " << actTS << "\n";
                        }
                    }
                }
            }

            //if (!expandFully) MaxDependentInfo::debug = true;
        } else {
            map<int, PropositionAnnotation>::const_iterator stateItr = payload->startState.first.begin();
            const map<int, PropositionAnnotation>::const_iterator stateEnd = payload->startState.first.end();
            
            for (; stateItr != stateEnd; ++stateItr) {
                const int factID = stateItr->first;
                #ifdef POPF3ANALYSIS
                achieverDetails[factID].push_back(CostedAchieverDetails(currentCosts,true));                
                #else
                (*achievedInLayer)[factID] = 0.0;
                #endif
            }
        }

        if (RPGBuilder::modifiedRPG) {
            map<int, PropositionAnnotation>::const_iterator stateItr = payload->startState.retired.begin();
            const map<int, PropositionAnnotation>::const_iterator stateEnd = payload->startState.retired.end();

            for (; stateItr != stateEnd; ++stateItr) {
                const int factID = stateItr->first;
                const StepAndBeforeOrAfter & from = stateItr->second.negativeAvailableFrom;
                const unsigned int dependsOnActionAtStep = from.stepID;
                const double actTS = payload->minTimestamps[dependsOnActionAtStep];
                pair<double, double> & toUpdate = payload->propositionMustBeDeletedAddedAfter.insert(make_pair(factID, make_pair(0.0, 0.0))).first->second;

                if (toUpdate.first < actTS) {
                    if (evaluateDebug) cout << "RPG modified: cannot delete " << *(RPGBuilder::getLiteral(factID)) << " until layer " << actTS << ", due to existing deletor\n";
                    toUpdate.first = actTS;
                }
                if (toUpdate.second < actTS) {
                    if (evaluateDebug) cout << "RPG modified: cannot add " << *(RPGBuilder::getLiteral(factID)) << " until layer " << actTS << ", due to existing deletor\n";
                    toUpdate.second = actTS;
                }

                map<StepAndBeforeOrAfter, bool>::const_iterator invItr = stateItr->second.deletableFrom.begin();
                const map<StepAndBeforeOrAfter, bool>::const_iterator invEnd = stateItr->second.deletableFrom.end();

                for (; invItr != invEnd; ++invItr) {
                    const double actTS = payload->minTimestamps[invItr->first.stepID] - (invItr->first.beforeOrAfter == StepAndBeforeOrAfter::BEFORE ? 0.001 : 0.0);
                    if (actTS > 0.0) {
                        if (toUpdate.first < actTS) {
                            if (evaluateDebug) cout << "RPG modified: cannot delete " << *(RPGBuilder::getLiteral(factID)) << " until layer " << actTS << ", due to previous additions\n";
                            toUpdate.first = actTS;
                        }
                    }
                }
            }

        }

        if (RPGBuilder::modifiedRPG) {
            const vector<FluentInteraction> & lastStepToTouchPNE = payload->startState.temporalConstraints->lastStepToTouchPNE;

            const int loopLim = lastStepToTouchPNE.size();

            for (int i = 0; i < loopLim; ++i) {
                const int stepID = lastStepToTouchPNE[i].lastInstantaneousEffect;
                const double actTS = (stepID == -1 ? -0.001 : payload->minTimestamps[stepID]);                
                earliestNumericPOTimes[i] = actTS;
            }
        } else {
            const int loopLim = earliestNumericPOTimes.size();
            
            for (int i = 0; i < loopLim; ++i) {
                earliestNumericPOTimes[i] = 0.0;
            }
        }

        {
            const int loopLim = rpgNumericPreconditions->size();

            if (loopLim) {
                const vector<double> * const maxFluentTable = &(payload->fluentLayers.borrowFactLayerZeroValues());
                for (int i = 0; i < loopLim; ++i) {
                    if (ignoreNumbers || (*rpgNumericPreconditions)[i].isSatisfied(*maxFluentTable)) {
                        numericIsTrueInState[i] = true;
                        const double poTS = localEarliestPointForNumericPrecondition((*rpgNumericPreconditions)[i]);
                        //earliestNumericPrePOTimes[i] = poTS;
                        (*numericAchievedBy)[i] = 0;
                        (*numericAchievedInLayer)[i] = (RPGBuilder::modifiedRPG && poTS >= 0.001 ? (poTS - 0.001) : 0.0);
                        if (evaluateDebug && RPGBuilder::modifiedRPG && (*numericAchievedInLayer)[i] > 0.0) {
                            cout << "RPG modified: delaying numeric fact " << i << " to layer " << (*numericAchievedInLayer)[i] << "\n";
                        }
                        if (evaluateDebug && ignoreNumbers) {
                            cout << "Assuming numeric precondition " << i << " is satisfied\n";
                        }
                    } else {
                        numericIsTrueInState[i] = false;
                    }
                }
            }
        }

    }
#endif

    void addRequirementToHaveSeenTheEndOfAllCurrentlyExecutingActions(BuildingPayload * const payload) {
        map<int, set<int> >::const_iterator saItr = payload->startedActions.begin();
        const map<int, set<int> >::const_iterator saEnd = payload->startedActions.end();

        for (; saItr != saEnd; ++saItr) {
            if (!TemporalAnalysis::canSkipToEnd(saItr->first)) {
                payload->insistUponEnds.insert(*saItr);
                if (evaluateDebug) cout << "Insisting on the end of " << saItr->first << " - is not skippable\n";
            } else {
                if (evaluateDebug) cout << "End of " << *(RPGBuilder::getInstantiatedOp(saItr->first)) << " is a skippable\n";
            }

        }

        payload->unappearedEnds = payload->insistUponEnds.size();
    }

    void seeWhatGoalsAreTrueToStartWith(BuildingPayload * const payload) {

        {
            set<int>::iterator gsItr = goals.begin();

            for (; gsItr != gsEnd; ++gsItr) {
                const StateFacts::const_iterator gfItr = payload->startState.first.find(*gsItr);
                if (gfItr != payload->startState.first.end()) {
                    if (evaluateDebug) {
                        cout << "\t" << *gsItr << " true in initial state\n";
                    }
                    --(payload->unsatisfiedGoals);
                }
            }
        }


        {

            set<int>::iterator gfItr = goalFluents.begin();

            if (gfItr == gfEnd) return;

            const vector<double> * const maxFluentTable = &(payload->fluentLayers.borrowFactLayerZeroValues());

            for (; gfItr != gfEnd; ++gfItr) {
                if (ignoreNumbers || (*rpgNumericPreconditions)[*gfItr].isSatisfied(*maxFluentTable)) {
                    if (evaluateDebug) {
                        cout << "\t" << (*rpgNumericPreconditions)[*gfItr] << " true in initial state\n";
                    }

                    --(payload->unsatisfiedGoals);
                } else if (evaluateDebug) {
                    cout << "\t" << (*rpgNumericPreconditions)[*gfItr] << " false in initial state\n";
                }
            }

        }


    }


    void delayOpenEndsUntilTheirPOPositions(BuildingPayload * const payload) {
        if (!RPGBuilder::modifiedRPG) {
            return;
        }
        
        if (!payload->startEventQueue) {
            return;
        }
        
        list<StartEvent>::const_iterator seqItr = payload->startEventQueue->begin();
        const list<StartEvent>::const_iterator seqEnd = payload->startEventQueue->end();
        
        for (; seqItr != seqEnd; ++seqItr) {
            double & eas = payload->openEndActionSchedule[seqItr->actID];
            const double & localTS = payload->minTimestamps[seqItr->stepID + 1];
            if (eas == -1.0 || localTS < eas) {
                eas = localTS;
            }
        }
    }

    void addTemporalConstraintsFromActiveActions(BuildingPayload * const payload) {
        if (!RPGBuilder::modifiedRPG) {
            return;
        }

        map<int, pair<double, double> >::iterator constrItr = payload->propositionMustBeDeletedAddedAfter.begin();
        const map<int, pair<double, double> >::iterator constrEnd = payload->propositionMustBeDeletedAddedAfter.end();

        for (; constrItr != constrEnd; ++constrItr) {

            const int litID = constrItr->first;

            for (int pass = 0; pass < 2; ++pass) {
                const list<pair<int, VAL::time_spec> > & effList = (pass ? (*negativeEffectsToActions)[litID] : (*effectsToActions)[litID]);
                const double layer = (pass ? constrItr->second.second : constrItr->second.first) - 0.001;

                if (layer <= 0.001) continue;

                assert(layer >= 0.000);
                FactLayerEntry & layerEntry = payload->factLayers[layer];

                if (!layerEntry.endOfJustApplied) {
                    payload->gc.push_back(make_pair(set<int>(), set<int>()));
                    layerEntry.endOfJustApplied = &(payload->gc.back());
                }

                pair<set<int>, set<int> > * const dest = layerEntry.endOfJustApplied;

                list<pair<int, VAL::time_spec> >::const_iterator effItr = effList.begin();
                const list<pair<int, VAL::time_spec> >::const_iterator effEnd = effList.end();

                for (; effItr != effEnd; ++effItr) {
                    if (!RPGBuilder::rogueActions[effItr->first]) {
                        if (effItr->second == VAL::E_AT_START) {
                            if (dest->first.insert(effItr->first).second) {
                                ++(payload->forbiddenStart.insert(pair<int, int>(effItr->first, 0)).first->second);
                            }
                        } else {
                            if (dest->second.insert(effItr->first).second) {
                                ++(payload->forbiddenEnd.insert(pair<int, int>(effItr->first, 0)).first->second);
                            }
                        }
                    }
                }

                if (layerEntry.endOfJustApplied->first.empty() && layerEntry.endOfJustApplied->second.empty()) {
                    layerEntry.endOfJustApplied = 0;
                    payload->gc.pop_back();
                }
            }

        }

        const int varCount = earliestNumericPOTimes.size();

        for (int v = 0; v < varCount; ++v) {
            const double layer = earliestNumericPOTimes[v] - 0.001;
            if (layer <= 0.001) continue;
            assert(layer >= 0.000);
            FactLayerEntry & layerEntry = payload->factLayers[layer];

            if (!layerEntry.endOfJustApplied) {
                payload->gc.push_back(make_pair(set<int>(), set<int>()));
                layerEntry.endOfJustApplied = &(payload->gc.back());
            }

            pair<set<int>, set<int> > * const dest = layerEntry.endOfJustApplied;

            for (int pass = 0; pass < 2; ++pass) {
                set<int> & destSet = (pass ? dest->second : dest->first);
                const set<int> & loopSet = actionsAffectedByFluent[v][pass];

                set<int>::const_iterator aItr = loopSet.begin();
                const set<int>::const_iterator aEnd = loopSet.end();

                for (; aItr != aEnd; ++aItr) {
                    if (!RPGBuilder::rogueActions[*aItr]) {
                        if (destSet.insert(*aItr).second) {
                            if (pass) {
                                ++(payload->forbiddenEnd.insert(pair<int, int>(*aItr, 0)).first->second);
                            } else {
                                ++(payload->forbiddenStart.insert(pair<int, int>(*aItr, 0)).first->second);
                            }
                        }
                    }
                }
            }


            if (layerEntry.endOfJustApplied->first.empty() && layerEntry.endOfJustApplied->second.empty()) {
                layerEntry.endOfJustApplied = 0;
                payload->gc.pop_back();
            }

        }

        //static vector<double> earliestNumericPOTimes;
    }


    bool addTemporalConstraintsFromActiveActions(BuildingPayload * const payload, map<double, list<pair<int, int> > > * const justApplied, const double & stateTS, const int & nextTIL, const double & tilFrom) {

        if (!justApplied) {
            if (evaluateDebug) {
                cout << "Not just applied a start, so no definitely active invariants\n";
            }

            return true;
        }

        double TILoffset = (nextTIL < tilCount ? tilTimes[nextTIL] - tilFrom : 0.0);

        list<pair<set<int>, set<int> > > & setsToForbid = payload->setsToForbid;        
        

        map<double, list<pair<int, int> > >::iterator jaItr = justApplied->begin();
        const map<double, list<pair<int, int> > >::iterator jaEnd = justApplied->end();
        for (; jaItr != jaEnd; ++jaItr) {
            const double insLayer = jaItr->first;
            if (insLayer > 0.0) {
                setsToForbid.push_back(pair<set<int>, set<int> >());
                set<int> & affectedStarts = setsToForbid.back().first;
                set<int> & affectedEnds = setsToForbid.back().second;

                list<pair<int, int> >::iterator liItr = jaItr->second.begin();
                const list<pair<int, int> >::iterator liEnd = jaItr->second.end();

                for (; liItr != liEnd; ++liItr) {
                    const int currActID = liItr->first;
                    {

                        list<Literal*> & invs = (*actionsToInvariants)[currActID];
                        list<Literal*>::iterator invItr = invs.begin();
                        const list<Literal*>::iterator invEnd = invs.end();

                        for (; invItr != invEnd; ++invItr) {
                            const int litID = (*invItr)->getStateID();
                            list<pair<int, VAL::time_spec> > & affected = (*negativeEffectsToActions)[litID];
                            list<pair<int, VAL::time_spec> >::iterator affItr = affected.begin();
                            const list<pair<int, VAL::time_spec> >::iterator affEnd = affected.end();
                            for (; affItr != affEnd; ++affItr) {
                                if (affItr->second == VAL::E_AT_START) {
                                    if (evaluateDebug) {
                                        cout << "Delaying start of " << affItr->first << " to " << insLayer << "\n";
                                    }
                                    affectedStarts.insert(affItr->first);
                                } else {
                                    affectedEnds.insert(affItr->first);
                                    if (evaluateDebug) {
                                        cout << "Delaying end of " << affItr->first << " to " << insLayer << "\n";
                                    }

                                }
                            }
                            if (nextTIL < tilCount) {

                                for (int i = nextTIL; i < tilCount; ++i) {
                                    const double earliestTILtime = tilTimes[i] - TILoffset;

                                    if (earliestTILtime < insLayer) {
                                        list<int>::iterator effItr = tilTemporaryNegativeEffects[i].begin();
                                        const list<int>::iterator effEnd = tilTemporaryNegativeEffects[i].end();

                                        for (; effItr != effEnd; ++effItr) {
                                            if (*effItr == litID) {
                                                TILoffset -= ((insLayer - earliestTILtime) + EPSILON);
                                                if (TILoffset < stateTS) {
                                                    if (evaluateDebug) cout << "Dead end found: TIL definitely going to delete invariant of open action\n";
                                                    return false;
                                                }
                                                break;
                                            }

                                        }
                                    }
                                }

                            }
                        }
                    }
                    if (nextTIL < tilCount) {
                        list<Literal*> & invs = (*actionsToEndPreconditions)[currActID];
                        list<Literal*>::iterator precItr = invs.begin();
                        const list<Literal*>::iterator precEnd = invs.end();

                        for (; precItr != precEnd; ++precItr) {
                            const int litID = (*precItr)->getStateID();
                            for (int i = nextTIL; i < tilCount; ++i) {
                                const double earliestTILtime = tilTimes[i] - TILoffset;

                                if (earliestTILtime < insLayer) {
                                    list<int>::iterator effItr = tilNegativeEffects[i].begin();
                                    const list<int>::iterator effEnd = tilNegativeEffects[i].end();

                                    for (; effItr != effEnd; ++effItr) {
                                        if (*effItr == litID) {
                                            TILoffset -= ((insLayer - earliestTILtime) + EPSILON);
                                            if (TILoffset < stateTS) {
                                                if (evaluateDebug) cout << "Dead end found: TIL definitely going to delete invariant of open action\n";
                                                return false;
                                            }
                                            break;
                                        }

                                    }
                                }
                            }
                        }

                    }
                }

                if (!affectedStarts.empty() || !affectedEnds.empty()) {
                    assert(insLayer >= 0.000);
                    payload->factLayers[insLayer].endOfJustApplied = &(setsToForbid.back());
                    {
                        set<int>::iterator sItr = affectedStarts.begin();
                        const set<int>::iterator sEnd = affectedStarts.end();
                        for (; sItr != sEnd; ++sItr) {
                            ++(payload->forbiddenStart.insert(pair<int, int>(*sItr, 0)).first->second);
                            if (evaluateDebug) {
                                cout << "Start of " << *sItr << " now delayed because of " << payload->forbiddenStart[*sItr] << " actions\n";
                            }
                        }
                    }
                    {
                        set<int>::iterator sItr = affectedEnds.begin();
                        const set<int>::iterator sEnd = affectedEnds.end();
                        for (; sItr != sEnd; ++sItr) {
                            ++(payload->forbiddenEnd.insert(pair<int, int>(*sItr, 0)).first->second);
                            if (evaluateDebug) {
                                cout << "End of " << *sItr << " now delayed because of " << payload->forbiddenEnd[*sItr] << " actions\n";
                            }
                        }
                    }

                }
                if (!affectedEnds.empty()) {
                    set<int>::iterator sItr = affectedEnds.begin();
                    const set<int>::iterator sEnd = affectedEnds.end();

                    for (; sItr != sEnd; ++sItr) {
                        const double actDur = RPGBuilder::getOpMaxDuration(*sItr, 0);
                        const double earlierIns = insLayer - actDur;
                        if (earlierIns > 0.0) {
                            assert(earlierIns >= 0.000);
                            FactLayerEntry & fle = payload->factLayers[earlierIns];
                            if (!fle.endOfJustApplied) {
                                setsToForbid.push_back(pair<set<int>, set<int> >());
                                fle.endOfJustApplied = &(setsToForbid.back());
                                fle.endOfJustApplied->first.insert(*sItr);
                                ++(payload->forbiddenStart.insert(pair<int, int>(*sItr, 0)).first->second);
                            } else if (fle.endOfJustApplied->first.insert(*sItr).second) {
                                ++(payload->forbiddenStart.insert(pair<int, int>(*sItr, 0)).first->second);
                            }

                        }

                    }


                }
            }
            {
                list<pair<int, int> >::iterator liItr = jaItr->second.begin();
                const list<pair<int, int> >::iterator liEnd = jaItr->second.end();

                for (; liItr != liEnd; ++liItr) {
                    const int currActID = liItr->first;
                    double & eas = payload->openEndActionSchedule[currActID];
                    if (eas == -1.0) {
                        eas = insLayer;
                        if (evaluateDebug) cout << "Delaying end of " << currActID << " until " << insLayer << "\n";
                    }

                }

            }
        }

        return true;
    }

    void performTILInitialisation() {

        if (tilInitialised) return;

        tilInitialised = true;
        list<RPGBuilder::FakeTILAction> & TILs = RPGBuilder::getTILs();
        tilCount = TILs.size();
        tilEffects = vector<list<int> >(tilCount);
        tilNegativeEffects = vector<list<int> >(tilCount);
        tilTemporaryNegativeEffects = vector<list<int> >(tilCount);
        tilTimes = vector<double>(tilCount);

        {
            const int loopLim = processedPreconditionsToActions->size();
            deadlineAtTime = vector<double>(loopLim);
            for (int i = 0; i < loopLim; ++i) deadlineAtTime[i] = DBL_MAX;

        }

        earliestDeadlineRelevancyStart = vector<double>(initialUnsatisfiedEndPreconditions->size());
        earliestDeadlineRelevancyEnd = vector<double>(initialUnsatisfiedEndPreconditions->size());


        list<RPGBuilder::FakeTILAction>::reverse_iterator tilItr = TILs.rbegin();
        const list<RPGBuilder::FakeTILAction>::reverse_iterator tilEnd = TILs.rend();

        set<int> addedLater;

        for (int i = tilCount - 1; tilItr != tilEnd; ++tilItr, --i) {

            tilTimes[i] = tilItr->duration;


            {
                list<Literal*>::iterator effItr = tilItr->addEffects.begin();
                const list<Literal*>::iterator effEnd = tilItr->addEffects.end();

                for (; effItr != effEnd; ++effItr) {
                    const int currEffID = (*effItr)->getStateID();
                    tilEffects[i].push_back(currEffID);
                    addedLater.insert(currEffID);
                }
            }

            {
                list<Literal*>::iterator effItr = tilItr->delEffects.begin();
                const list<Literal*>::iterator effEnd = tilItr->delEffects.end();

                for (; effItr != effEnd; ++effItr) {
                    const int currEffID = (*effItr)->getStateID();
                    tilTemporaryNegativeEffects[i].push_back(currEffID);
                    if ((*effectsToActions)[currEffID].empty() && addedLater.find(currEffID) == addedLater.end()) {
                        tilNegativeEffects[i].push_back(currEffID);
                        deadlineAtTime[currEffID] = tilTimes[i];
                    }
                }
            }


        }


    }

    void initialiseLatestArrays() {

        static bool initLatestArrays = false;


        static const int easSize = initialUnsatisfiedEndPreconditions->size();

        if (!initLatestArrays) {
            earliestStartAllowed = vector<double>(easSize);
            earliestEndAllowed = vector<double>(easSize);
            latestStartAllowed = vector<double>(easSize);
            latestEndAllowed = vector<double>(easSize);
            initLatestArrays = true;
        }



        for (int i = 0; i < easSize; ++i) latestStartAllowed[i] = DBL_MAX;
        for (int i = 0; i < easSize; ++i) latestEndAllowed[i] = DBL_MAX;

        if (expandFully) {
            for (int i = 0; i < easSize; ++i) earliestStartAllowed[i] = DBL_MAX;
            for (int i = 0; i < easSize; ++i) earliestEndAllowed[i] = DBL_MAX;
        }
    }

    void addTILsBeforeExpansion(const int & nextTIL, map<double, FactLayerEntry, EpsilonComp > & factLayers, const double & stateTS, const double & tilFrom) {
        if (nextTIL >= tilCount) return;

        static const int easSize = initialUnsatisfiedEndPreconditions->size();

        for (int i = 0; i < easSize; ++i) earliestDeadlineRelevancyStart[i] = DBL_MAX;
        for (int i = 0; i < easSize; ++i) earliestDeadlineRelevancyEnd[i] = DBL_MAX;


        set<int> earlier;

        const double TILoffset = (RPGBuilder::modifiedRPG ? 0.000 : (nextTIL < tilCount ? tilTimes[nextTIL] - tilFrom : 0.0));

        for (int i = nextTIL; i < tilCount; ++i) {
            const double thisTS = tilTimes[i] - TILoffset;

            list<int>::iterator effItr = tilEffects[i].begin();
            const list<int>::iterator effEnd = tilEffects[i].end();

            for (; effItr != effEnd; ++effItr) {

                const int currEff = (*effItr);
                #ifdef POPF3ANALYSIS
                if (achieverDetails[currEff].empty()) {
                    if (earlier.insert(currEff).second) {
                        assert(thisTS >= 0.000);
                        factLayers[thisTS].TILs.push_back(pair<int, int>(i, currEff));
                    }
                }
                #else
                double & currAIL = (*achievedInLayer)[currEff];                
                if (currAIL == -1.0 && (*achievedBy)[currEff].first == -1) { // not in initial state
                    if (earlier.insert(currEff).second) {
                        assert(thisTS >= 0.000);
                        factLayers[thisTS].TILs.push_back(pair<int, int>(i, currEff));
                    }
                }
                #endif

            }

        }

        for (int i = nextTIL; i < tilCount; ++i) {
            const double thisTS = tilTimes[i] - stateTS;

            list<int>::iterator effItr = tilNegativeEffects[i].begin();
            const list<int>::iterator effEnd = tilNegativeEffects[i].end();

            for (; effItr != effEnd; ++effItr) {

                list<pair<int, VAL::time_spec> > & currList = (*preconditionsToActions)[*effItr];

                list<pair<int, VAL::time_spec> >::iterator ioItr = currList.begin();
                const list<pair<int, VAL::time_spec> >::iterator ioEnd = currList.end();

                for (; ioItr != ioEnd; ++ioItr) {
                    const int actID = ioItr->first;
                    if (ioItr->second == VAL::E_AT_START) {
                        if (latestStartAllowed[actID] > thisTS) latestStartAllowed[actID] = thisTS;
                        const double minDur = RPGBuilder::getOpMinDuration(actID, 0);
                        if (latestEndAllowed[actID] > thisTS + minDur) latestEndAllowed[actID] = thisTS + minDur;
                    } else {
                        const double minDur = RPGBuilder::getOpMinDuration(actID, -1);
                        if (latestStartAllowed[actID] > thisTS - minDur) latestStartAllowed[actID] = thisTS - minDur;
                        if (latestEndAllowed[actID] > thisTS) latestEndAllowed[actID] = thisTS;
                    }
                }


            }

        }



    }

    void addBoundsFromTemporalAnalysis(const double & stateTS) {

        const int actCount = latestStartAllowed.size();

        for (int a = 0; a < actCount; ++a) {
            {
                double firmUpper = TemporalAnalysis::getActionTSBounds()[a][0].second;
                if (firmUpper != DBL_MAX) {
                    firmUpper -= stateTS;
                    if (firmUpper < latestStartAllowed[a]) latestStartAllowed[a] = firmUpper;
                }
                if (latestStartAllowed[a] < 0.000) latestEndAllowed[a] = -1.0;
            }
            {
                double firmUpper = TemporalAnalysis::getActionTSBounds()[a][1].second;
                if (firmUpper != DBL_MAX) {
                    firmUpper -= stateTS;
                    if (firmUpper < latestEndAllowed[a]) latestEndAllowed[a] = firmUpper;
                }
                if (latestEndAllowed[a] < 0.001) latestStartAllowed[a] = -1.0;
            }

        }

    }

    double earliestTILForAction(const unsigned int & i, const bool & isStart);
    void findApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions);
    void filterApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions);
    bool testApplicability(const MinimalState & theState, const double & stateTime, const ActionSegment & actID, const bool & fail, const bool & ignoreDeletes);

    instantiatedOp* getOp(const unsigned int & i) {

        assert(i >= 0);
        assert(i < actionsToEndEffects->size());
        return RPGBuilder::getInstantiatedOp(i);

    };


    bool sensiblePair(const BuildingPayload * const payload, const pair<int, VAL::time_spec> & achiever, const unsigned int & fact) {
        assert(fact >= 0);
        assert(fact < preconditionsToActions->size());

        if (achiever.second == VAL::E_AT) {
            assert(tilInitialised);
            assert(achiever.first >= 0);
            assert(achiever.first < tilCount);
        } else {
            if (achiever.first < 0) {
                cerr << "\t\tTrying to use a negative indexed action as an achiever for fact " << *(RPGBuilder::getLiteral(fact)) << "\n";
                if (payload->startState.first.find(fact) != payload->startState.first.end()) {
                    cerr << "\t\t\tWas true in the initial state\n";
                }

                assert(achiever.first >= 0);
            }
            assert((unsigned int) achiever.first < actionsToEndPreconditions->size());
        }

        return true;
    }

    void extractRP(BuildingPayload * const payload, int & h, list<pair<double, list<ActionSegment> > > & relaxedPlan, pair<int, VAL::time_spec> & earliestTIL, double & makespanEstimate) {

        makespanEstimate = 0.0;

        const MinimalState & theState = payload->startState;
        map<int, set<int> > & insistUponEnds = payload->insistUponEnds;
        vector<double> & startActionSchedule = payload->startActionSchedule;
        vector<double> & openEndActionSchedule = payload->openEndActionSchedule;

        payload->fluentLayers.prepareForExtraction();
        
        static const set<int> emptyIntSet;

        RPGRegressionMap goalsAtLayer;

        {
            set<int>::iterator gsItr = goals.begin();
            const set<int>::iterator gsEnd = goals.end();

            for (; gsItr != gsEnd; ++gsItr) {
                const int currGoal = *gsItr;
                #ifdef POPF3ANALYSIS
                const double insLayer = achieverDetails[currGoal].back().layer;
                if (insLayer > 0.0) {
                    const int achiever = achieverDetails[currGoal].back().achiever.first;
                #else
                const double insLayer = (*achievedInLayer)[currGoal];
                if (insLayer > 0.0) {
                    const int achiever = (*achievedBy)[currGoal].first;                    
                #endif
                    if (achiever != -1) {
                        goalsAtLayer[insLayer].propositionalGoals.insert(pair<int, double>(currGoal, DBL_MAX));
                        if (evaluateDebug) cout << "Goal " << *(RPGBuilder::getLiteral(*gsItr)) << " to be achieved in layer with TS " << insLayer << "\n";
                    } else {
                        assert(payload->startState.first.find(currGoal) != payload->startState.first.end());
                        if (evaluateDebug) cout << "Goal " << *(RPGBuilder::getLiteral(*gsItr)) << " achieved in initial state at layer " << insLayer << "\n";
                    }
                } else if (evaluateDebug) cout << "Goal " << *(RPGBuilder::getLiteral(*gsItr)) << " achieved in initial state, not adding to RPG regression\n";


                if (insLayer > makespanEstimate) {
                    makespanEstimate = insLayer;
                }
            }
        }

        if (!ignoreNumbers) {
            set<int>::iterator gfItr = goalFluents.begin();
            set<int>::iterator gfEnd = goalFluents.end();

            for (; gfItr != gfEnd; ++gfItr) {
                const int currGoal = *gfItr;
                const double insLayer = (*numericAchievedInLayer)[currGoal];
                
                if (insLayer > makespanEstimate) {
                    makespanEstimate = insLayer;
                }
                
                if (numericIsTrueInState[currGoal]) {
                    
                    if (evaluateDebug) cout << "Numeric goal achieved in state, not adding to RPG regression\n";
                    continue;
                }
                
                assert(insLayer > 0.0);
                    
                if (evaluateDebug) cout << "Numeric goal to be achieved in layer with TS " << insLayer << "\n";                    
                payload->fluentLayers.requestGoal(currGoal, insLayer, goalsAtLayer);

                

            }
        }


        if (!RPGBuilder::nonTemporalProblem()) {
            map<int, set<int> >::iterator ueItr = insistUponEnds.begin();
            const map<int, set<int> >::iterator ueEnd = insistUponEnds.end();

            for (; ueItr != ueEnd; ++ueItr) {
                const int currAct = ueItr->first;
                const double insLayer = openEndActionSchedule[currAct];
                if (evaluateDebug) cout << "End of " << *(RPGBuilder::getInstantiatedOp(currAct)) << " goes at " << insLayer << endl;
                const double tilR = earliestTILForAction(currAct, false);
                (goalsAtLayer[insLayer + EPSILON].actionEnds.insert(pair<int, pair<int, double> >(currAct, pair<int, double>(0, tilR))).first->second.first) += ueItr->second.size();
                // HACK
            }
        }


        while (!goalsAtLayer.empty()) {

            const double currTS = goalsAtLayer.rbegin()->first;
            map<int, double> & currGAL = goalsAtLayer.rbegin()->second.propositionalGoals;
            
            if (evaluateDebug) cout << COLOUR_light_green << currTS << ":\n" <<     COLOUR_default;
            map<int, pair<int, double> > & actionStarts = goalsAtLayer.rbegin()->second.actionStarts;
            map<int, pair<int, double> > & actionEnds   = goalsAtLayer.rbegin()->second.actionEnds;
            
            bool alreadyPushed = false;

            {


                map<int, pair<int, double> >::iterator asItr = actionStarts.begin();
                const map<int, pair<int, double> >::iterator asEnd = actionStarts.end();

                if (evaluateDebug && asItr != asEnd) cout << "Adding starts at TS " << currTS << "\n";

                for (; asItr != asEnd; ++asItr) {

                    if (evaluateDebug) cout << "\t\tAdding start of " << asItr->first << "\n";

                    double tilR = earliestTILForAction(asItr->first, true);
                    if (tilR > asItr->second.second) tilR = asItr->second.second;

                    if (!alreadyPushed) {
                        relaxedPlan.push_front(pair<double, list<ActionSegment> >(currTS - EPSILON, list<ActionSegment>()));
                        alreadyPushed = true;
                    }

                    relaxedPlan.front().second.insert(relaxedPlan.front().second.end(), asItr->second.first, ActionSegment(getOp(asItr->first), VAL::E_AT_START, -1, emptyIntList));
                    ++h;

                    if (RPGHeuristic::printRPGAsDot) {
                        payload->dot.highlightAction(asItr->first, VAL::E_AT_START);
                    }
                    
    //              if (currTS == EPSILON) {
    //                  helpfulActions.push_back(pair<int, VAL::time_spec>(*asItr, VAL::E_AT_START));
    //                  if (evaluateDebug) cout << "\t\tIs a helpful action\n";
    //              }

                    {
                        list<Literal*> & actionEffectsList = (*actionsToStartEffects)[asItr->first];
                        list<Literal*>::iterator aelItr = actionEffectsList.begin();
                        const list<Literal*>::iterator aelEnd = actionEffectsList.end();

                        for (; aelItr != aelEnd; ++aelItr) {
                            map<int, double>::iterator cgItr = currGAL.find((*aelItr)->getStateID());
                            if (cgItr != currGAL.end()) {
                                if (tilR > cgItr->second) tilR = cgItr->second;
                                currGAL.erase(cgItr);
                            }
                        }
                    }
                    

                    bool isHelpful = true;

                    {
                        list<Literal*> & actionPreconditionlist = (*actionsToProcessedStartPreconditions)[asItr->first];
                        list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                        const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();

                        if (evaluateDebug) cout << "\t\tPreconditions:\n";

                        for (; aplItr != aplEnd; ++aplItr) {

                            const int currPrec = (*aplItr)->getStateID();
                            double acIn;
                            pair<int, VAL::time_spec> acBy;
                            
                            if (evaluateDebug) {
                                cout << "\t\tPrecondition " << currPrec << " " << *(RPGBuilder::getLiteral(currPrec)) << ": ";
                                cout.flush();
                            }
                            
                            getAchieverDetails(currPrec, currTS, acIn, acBy);
                                
                            if (acIn > 0.0) {
                                if (acBy.first != -1) {
                                    map<int, double>::iterator insItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                    if (insItr->second > tilR) insItr->second = tilR;
                                    if (evaluateDebug) cout << "required at time " << acIn << "\n";
                                    assert(acIn < currTS);
                                    isHelpful = false;
                                } else {
                                    if (evaluateDebug) cout << "in initial state at layer " << acIn << "\n";
                                }
                            } else {
                                if (evaluateDebug) cout << "in initial state\n";
                            }

                        }

                        if (evaluateDebug) cout << "\t\tPreconditions done.\n";
                    }

                    if (!ignoreNumbers) {
                        list<int> & actionPreconditionlist = (*actionsToProcessedStartNumericPreconditions)[asItr->first];
                        list<int>::iterator aplItr = actionPreconditionlist.begin();
                        const list<int>::iterator aplEnd = actionPreconditionlist.end();

                        if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                        for (; aplItr != aplEnd; ++aplItr) {

                            const int currPrec = (*aplItr);
                            const double acIn = (*numericAchievedInLayer)[currPrec];
                            if (acIn > 0.0 && !numericIsTrueInState[currPrec]) {
                                if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";
                                
                                payload->fluentLayers.requestNumericPrecondition(currPrec, acIn, currTS, goalsAtLayer, tilR);
                                isHelpful = false;
                            } else {
                                if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                            }

                        }

                        if (evaluateDebug) cout << "\t\tPreconditions done.\n";
                    }


                    if (isHelpful) {
                        payload->helpfulActions.push_front(ActionSegment(RPGBuilder::getInstantiatedOp(asItr->first), VAL::E_AT_START, -1, emptyIntSet));
                        if (evaluateDebug) cout << "\t\tIs a helpful action\n";
                    }

                    double & oldR = earliestDeadlineRelevancyStart[asItr->first];
                    if (oldR > tilR) oldR = tilR;

                }
            }

            {


                map<int, pair<int, double> >::iterator asItr = actionEnds.begin();
                const map<int, pair<int, double> >::iterator asEnd = actionEnds.end();

                if (evaluateDebug && asItr != asEnd) cout << "Adding ends at TS " << currTS << "\n";

                for (; asItr != asEnd; ++asItr) {

                    if (evaluateDebug) cout << "\t\tAdding end of " << asItr->first << "\n";

                    double tilR = earliestTILForAction(asItr->first, false);
                    if (tilR > asItr->second.second) tilR = asItr->second.second;

                    if (RPGHeuristic::printRPGAsDot) {
                        payload->dot.highlightAction(asItr->first, VAL::E_AT_END);
                    }
                    

                    if (!alreadyPushed) {
                        relaxedPlan.push_front(pair<double, list<ActionSegment> >(currTS - EPSILON, list<ActionSegment>()));
                        alreadyPushed = true;
                    }

                    if (!TemporalAnalysis::canSkipToEnd(asItr->first)) {
                        const int loopLim = asItr->second.first;
                        const ActionSegment theOp(getOp(asItr->first), VAL::E_AT_END, -1, emptyIntList);
                        for (int pb = 0; pb < loopLim; ++pb) {
                            relaxedPlan.front().second.push_back(theOp);
                        }
                        h += loopLim;
                    }


                    {
                        list<Literal*> & actionEffectsList = (*actionsToEndEffects)[asItr->first];
                        list<Literal*>::iterator aelItr = actionEffectsList.begin();
                        const list<Literal*>::iterator aelEnd = actionEffectsList.end();

                        for (; aelItr != aelEnd; ++aelItr) {
                            map<int, double>::iterator cgItr = currGAL.find((*aelItr)->getStateID());
                            if (cgItr != currGAL.end()) {
                                if (tilR > cgItr->second) tilR = cgItr->second;
                                currGAL.erase(cgItr);
                            }
                        }
                    }

                    
                    bool isHelpful = true;

                    {
                        list<Literal*> & actionPreconditionlist = (*actionsToEndPreconditions)[asItr->first];
                        list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                        const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();

                        for (; aplItr != aplEnd; ++aplItr) {

                            const int currPrec = (*aplItr)->getStateID();
                            double acIn;
                            pair<int, VAL::time_spec> acBy;
                            
                            getAchieverDetails(currPrec, currTS, acIn, acBy);
                            
                            if (acIn > 0.0) {
                                if (acBy.first != -1) {
                                    map<int, double>::iterator galItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                    if (galItr->second > tilR) galItr->second = tilR;
                                    isHelpful = false;
                                }
                            }

                        }


                    }

                    if (!ignoreNumbers) {
                        list<int> & actionPreconditionlist = (*actionsToNumericEndPreconditions)[asItr->first];
                        list<int>::iterator aplItr = actionPreconditionlist.begin();
                        const list<int>::iterator aplEnd = actionPreconditionlist.end();

                        if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                        for (; aplItr != aplEnd; ++aplItr) {

                            const int currPrec = (*aplItr);
                            const double acIn = (*numericAchievedInLayer)[currPrec];
                            if (acIn > 0.0 && !numericIsTrueInState[currPrec]) {
                                if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";
                                
                                payload->fluentLayers.requestNumericPrecondition(currPrec, acIn, currTS, goalsAtLayer, tilR);
                                
                                isHelpful = false;
                            } else {
                                if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                            }

                        }


                    }



                    if (evaluateDebug) cout << "\t\tPreconditions done.\n";


                    if (isHelpful) {
                        if (payload->startState.startedActions.find(asItr->first) != payload->startState.startedActions.end()) {
                            payload->helpfulActions.push_front(ActionSegment(RPGBuilder::getInstantiatedOp(asItr->first), VAL::E_AT_END, -1, emptyIntSet));
                            if (evaluateDebug) cout << "\t\tIs a helpful action\n";
                        }
                    }

                    double & oldR = earliestDeadlineRelevancyEnd[asItr->first];
                    if (oldR > tilR) oldR = tilR;

                }
            }


            
            if (!ignoreNumbers) {
                
                RPGRegressionMap::const_iterator thisLayer = goalsAtLayer.end();
                --thisLayer;
                
                list<pair<double, SupportingAction> > actionsUsed;
                payload->fluentLayers.satisfyNumericPreconditionsAtLayer(thisLayer, goalsAtLayer, actionsUsed);
                
                if (!actionsUsed.empty()) {
                    
                    if (!alreadyPushed) {
                        relaxedPlan.push_front(pair<double, list<ActionSegment> >(currTS - EPSILON, list<ActionSegment>()));
                        alreadyPushed = true;
                    }
                    
                    if (evaluateDebug) cout << "Adding achievers for numeric goals at TS " << currTS << "\n";

                    list<pair<double, SupportingAction> >::const_iterator sapItr = actionsUsed.begin();
                    const list<pair<double, SupportingAction> >::const_iterator sapEnd = actionsUsed.end();
            
                    for (; sapItr != sapEnd; ++sapItr) {
                        
                        const SupportingAction* const saItr = &(sapItr->second);
                        
                        if (sapItr->first < (currTS - 0.0001)) {
                            
                            if (saItr->ts == VAL::E_AT_START) {
                                map<int, pair<int, double> >::iterator galItr = goalsAtLayer[sapItr->first].actionStarts.insert(pair<int, pair<int, double> >(saItr->actID, make_pair(saItr->howManyTimes, saItr->tilR))).first;
                                if (galItr->second.second > saItr->tilR) galItr->second.second = saItr->tilR;
                                if (evaluateDebug) cout << "\t\tAdding requirement for start of action at time " << sapItr->first << "\n";
                                                                    
                            } else {
                                cerr << "Adding actions other than EPSILON earlier (in this case " << currTS << " - " << sapItr->first << ") should only occur for gradient actions\n";
                                exit(1);
                            }
                            
                            continue;
                        }
                        
                        pair<int, VAL::time_spec> currAchievedBy(saItr->actID, saItr->ts);
                        
                        
                        if (currAchievedBy.second == VAL::E_AT_START) {
                            if (evaluateDebug) {
                                cout << "\t\tUsing start of " << currAchievedBy.first << " - " << *(RPGBuilder::getInstantiatedOp(currAchievedBy.first));
                                if (saItr->howManyTimes > 1) {
                                    cout << ", " << saItr->howManyTimes << " times";
                                }
                                cout << endl;
                            }
                            if (evaluateDebug && currTS == EPSILON) cout << "\t\tIs a helpful action\n";
                            relaxedPlan.front().second.insert(relaxedPlan.front().second.end(), saItr->howManyTimes, ActionSegment(getOp(currAchievedBy.first), VAL::E_AT_START, -1, emptyIntList));
                            h += saItr->howManyTimes;

                            if (RPGHeuristic::printRPGAsDot) {
                                payload->dot.highlightAction(currAchievedBy.first, VAL::E_AT_START);
                            }

                            const double sTIL = earliestTILForAction(currAchievedBy.first, true);
                            double tilR = saItr->tilR;
                            
                            if (tilR > sTIL) tilR = sTIL;

                            {
                                list<Literal*> & actionEffectsList = (*actionsToStartEffects)[currAchievedBy.first];
                                list<Literal*>::iterator aelItr = actionEffectsList.begin();
                                const list<Literal*>::iterator aelEnd = actionEffectsList.end();
                                
                                for (; aelItr != aelEnd; ++aelItr) {
                                    map<int, double>::iterator cgItr = currGAL.find((*aelItr)->getStateID());
                                    if (cgItr != currGAL.end()) {
                                        if (tilR > cgItr->second) tilR = cgItr->second;
                                        currGAL.erase(cgItr);
                                    }
                                }
                            }
                                                        
                                                                                    
                                                                                                                
                            bool isHelpful = true;

                            {
                                list<Literal*> & actionPreconditionlist = (*actionsToProcessedStartPreconditions)[currAchievedBy.first];
                                list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                                const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();
                                if (evaluateDebug) cout << "\t\tPreconditions:\n";
                                for (; aplItr != aplEnd; ++aplItr) {

                                    const int currPrec = (*aplItr)->getStateID();
                                    
                                    double acIn;
                                    pair<int, VAL::time_spec> acBy;

                                    if (evaluateDebug) {
                                        cout << "\t\tPrecondition " << currPrec << " " << *(RPGBuilder::getLiteral(currPrec)) << ": ";
                                        cout.flush();
                                    }
                                    
                                    getAchieverDetails(currPrec, currTS - EPSILON, acIn, acBy);
                                    
                                    
                                    if (acIn > 0.0) {
                                        if (acBy.first != -1) {
                                            map<int, double>::iterator galItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                            if (galItr->second > tilR) galItr->second = tilR;
                                            if (evaluateDebug) cout << "required at time " << acIn << "\n";
                                            assert(acIn < currTS);
                                            isHelpful = false;
                                        } else {
                                            if (evaluateDebug) cout << "in initial state at time " << acIn << "\n";
                                        }
                                    } else {
                                        if (evaluateDebug) cout << "in initial state\n";
                                    }

                                }

                                if (evaluateDebug) cout << "\t\tPreconditions done\n";

                            }

                            if (!ignoreNumbers) {
                                list<int> & actionPreconditionlist = (*actionsToProcessedStartNumericPreconditions)[currAchievedBy.first];
                                list<int>::iterator aplItr = actionPreconditionlist.begin();
                                const list<int>::iterator aplEnd = actionPreconditionlist.end();

                                if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                                for (; aplItr != aplEnd; ++aplItr) {

                                    const int currPrec = (*aplItr);
                                    const double acIn = (*numericAchievedInLayer)[currPrec];
                                    if (acIn > 0.0 && !numericIsTrueInState[currPrec]) {
                                        
                                        if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";                                    
                                        payload->fluentLayers.requestNumericPrecondition(currPrec, acIn, currTS, goalsAtLayer, tilR);
                                        isHelpful = false;
                                    } else {
                                        if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                                    }

                                }

                                if (evaluateDebug) cout << "\t\tPreconditions done.\n";                                                        
                            }

                            if (isHelpful) {
                                payload->helpfulActions.push_front(ActionSegment(RPGBuilder::getInstantiatedOp(currAchievedBy.first), VAL::E_AT_START, -1, emptyIntSet));
                            }

                            double & oldR = earliestDeadlineRelevancyStart[currAchievedBy.first];
                            if (oldR > tilR) oldR = tilR;

                        } else {
                            if (evaluateDebug) cout << "\t\tUsing end of " << currAchievedBy.first << " - " << *(RPGBuilder::getInstantiatedOp(currAchievedBy.first)) << endl;
                            if (evaluateDebug && currTS == EPSILON) cout << "\t\tIs a helpful action\n";
                            relaxedPlan.front().second.push_back(ActionSegment(getOp(currAchievedBy.first), VAL::E_AT_END, -1, emptyIntList));
    //                  if (currTS == EPSILON) {
    //                      helpfulActions.push_back(pair<int, VAL::time_spec>(currAchievedBy->act, VAL::E_AT_END));
    //                  }
                            h += saItr->howManyTimes;

                            if (RPGHeuristic::printRPGAsDot) {
                                payload->dot.highlightAction(currAchievedBy.first, VAL::E_AT_END);
                            }

                            
                            const double sTIL = earliestTILForAction(currAchievedBy.first, false);
                            double tilR = saItr->tilR;
                            if (tilR > sTIL) tilR = sTIL;

                            {
                                list<Literal*> & actionEffectsList = (*actionsToEndEffects)[currAchievedBy.first];
                                list<Literal*>::iterator aelItr = actionEffectsList.begin();
                                const list<Literal*>::iterator aelEnd = actionEffectsList.end();
                                
                                for (; aelItr != aelEnd; ++aelItr) {
                                    map<int, double>::iterator cgItr = currGAL.find((*aelItr)->getStateID());
                                    if (cgItr != currGAL.end()) {
                                        if (tilR > cgItr->second) tilR = cgItr->second;
                                        currGAL.erase(cgItr);
                                    }
                                }
                            }
                            
                            bool isHelpful = true;

                            {
                                list<Literal*> & actionPreconditionlist = (*actionsToEndPreconditions)[currAchievedBy.first];
                                list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                                const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();

                                if (evaluateDebug) cout << "\t\tPreconditions:\n";

                                for (; aplItr != aplEnd; ++aplItr) {

                                    const int currPrec = (*aplItr)->getStateID();

                                    double acIn;
                                    pair<int, VAL::time_spec> acBy;
                                    
                                    getAchieverDetails(currPrec, currTS - EPSILON, acIn, acBy);
                                    
                                    if (acIn > 0.0) {
                                        if (acBy.first != -1) {
                                            map<int, double>::iterator galItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                            if (galItr->second > tilR) galItr->second = tilR;

                                            if (evaluateDebug) cout << "\t\tAdding requirement for " << currPrec << " at time " << acIn << "\n";
                                            assert(acIn < currTS);
                                            isHelpful = false;
                                        } else {
                                            if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state at time " << acIn << "\n";
                                        }
                                    } else {
                                        if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state\n";
                                    }

                                }

                                if (evaluateDebug) cout << "\t\tPreconditions done\n";

                                int addToThePast = saItr->howManyTimes;
                                
                                {
                                    map<int, set<int> >::const_iterator saItr = payload->startState.startedActions.find(currAchievedBy.first);
                                    if (saItr != payload->startState.startedActions.end()) {
                                        addToThePast -= saItr->second.size();
                                    }
                                }

                                if (addToThePast > 0) {
                                    map<int, pair<int, double> >::iterator galItr = goalsAtLayer[startActionSchedule[currAchievedBy.first] + EPSILON].actionStarts.insert(pair<int, pair<int, double> >(currAchievedBy.first, make_pair(addToThePast, tilR))).first;
                                    if (galItr->second.second > tilR) galItr->second.second = tilR;
                                    if (evaluateDebug) cout << "\t\tAdding requirement for start of action at time " << (startActionSchedule[currAchievedBy.first] + EPSILON) << "\n";
                                    isHelpful = false;
                                } else {
                                    if (evaluateDebug) cout << "\t\tAction has already been started, do not need to schedule start in RPG\n";
                                }
                            }

                            if (!ignoreNumbers) {
                                list<int> & actionPreconditionlist = (*actionsToNumericEndPreconditions)[currAchievedBy.first];
                                list<int>::iterator aplItr = actionPreconditionlist.begin();
                                const list<int>::iterator aplEnd = actionPreconditionlist.end();

                                if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                                for (; aplItr != aplEnd; ++aplItr) {

                                    const int currPrec = (*aplItr);
                                    const double acIn = (*numericAchievedInLayer)[currPrec];
                                    if (acIn > 0.0 && !numericIsTrueInState[currPrec]) {
                                        if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";
                                        payload->fluentLayers.requestNumericPrecondition(currPrec, acIn, currTS, goalsAtLayer, tilR);
                                        isHelpful = false;
                                        
                                    } else {
                                        if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                                    }

                                }


                            }


                            if (evaluateDebug) cout << "\t\tPreconditions done.\n";

                            if (isHelpful) {
                                payload->helpfulActions.push_front(ActionSegment(RPGBuilder::getInstantiatedOp(currAchievedBy.first), VAL::E_AT_END, -1, emptyIntSet));
                            }


                            double & oldR = earliestDeadlineRelevancyEnd[currAchievedBy.first];
                            if (oldR > tilR) oldR = tilR;

                        }

                    }
                    
                }

            }
            
            if (!currGAL.empty()) {


                 if (!alreadyPushed) {
                    relaxedPlan.push_front(pair<double, list<ActionSegment> >(currTS - EPSILON, list<ActionSegment>()));
                    alreadyPushed = true;
                }

                if (evaluateDebug && !currGAL.empty()) cout << "Finding achievers for goals at TS " << currTS << "\n";
                while (!currGAL.empty()) {
                    const map<int, double>::iterator nta = currGAL.begin();
                    const int nextToAchieve = nta->first;
                    double tilR = nta->second;
                    currGAL.erase(nta);
                    if (evaluateDebug) cout << "\tGoal " << nextToAchieve << "\n";
                    
                    double layerIgnore;
                    pair<int, VAL::time_spec> currAchievedBy;
                    
                    getAchieverDetails(nextToAchieve, currTS + EPSILON, layerIgnore, currAchievedBy);
                    
                    assert(sensiblePair(payload, currAchievedBy, nextToAchieve));

                    if (currAchievedBy.second == VAL::E_AT_START) {
                        if (evaluateDebug) cout << "\t\tUsing start of " << currAchievedBy.first << " - " << *(RPGBuilder::getInstantiatedOp(currAchievedBy.first)) << "\n";
                        if (evaluateDebug && currTS == EPSILON) cout << "\t\tIs a helpful action\n";
                        relaxedPlan.front().second.push_back(ActionSegment(getOp(currAchievedBy.first), VAL::E_AT_START, -1, emptyIntList));
                        ++h;

                        if (RPGHeuristic::printRPGAsDot) {
                            payload->dot.highlightAction(currAchievedBy.first, VAL::E_AT_START);
                            payload->dot.actionMeetsFactInRP(currAchievedBy.first, VAL::E_AT_START, nextToAchieve);
                        }
                        
                        const double sTIL = earliestTILForAction(currAchievedBy.first, true);
                        if (tilR > sTIL) tilR = sTIL;
    //                  if (currTS == EPSILON) {
    //                      helpfulActions.push_back(currAchievedBy);
                        //
    //                  }

                        {
                            list<Literal*> & actionEffectsList = (*actionsToStartEffects)[currAchievedBy.first];
                            list<Literal*>::iterator aelItr = actionEffectsList.begin();
                            const list<Literal*>::iterator aelEnd = actionEffectsList.end();

                            for (; aelItr != aelEnd; ++aelItr) {
                                map<int, double>::iterator cgItr = currGAL.find((*aelItr)->getStateID());
                                if (cgItr != currGAL.end()) {
                                    if (tilR > cgItr->second) tilR = cgItr->second;
                                    currGAL.erase(cgItr);
                                }
                            }
                        }

                        bool isHelpful = true;

                        {
                            list<Literal*> & actionPreconditionlist = (*actionsToProcessedStartPreconditions)[currAchievedBy.first];
                            list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                            const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();
                            if (evaluateDebug) cout << "\t\tPreconditions:\n";
                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr)->getStateID();
                                
                                double acIn;
                                pair<int, VAL::time_spec> acBy;
                                
                                getAchieverDetails(currPrec, currTS - EPSILON, acIn, acBy);
                                
                                if (acIn > 0.0) {
                                    if (acBy.first != -1) {
                                        map<int, double>::iterator galItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                        if (galItr->second > tilR) galItr->second = tilR;
                                        if (evaluateDebug) cout << "\t\tAdding requirement for " << currPrec << " at time " << acIn << "\n";
                                        assert(acIn < currTS);
                                        isHelpful = false;
                                    } else {
                                        if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state at time " << acIn << "\n";
                                    }
                                } else {
                                    if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state\n";
                                }

                            }

                            if (evaluateDebug) cout << "\t\tPreconditions done\n";

                        }

                        if (!ignoreNumbers) {
                            list<int> & actionPreconditionlist = (*actionsToProcessedStartNumericPreconditions)[currAchievedBy.first];
                            list<int>::iterator aplItr = actionPreconditionlist.begin();
                            const list<int>::iterator aplEnd = actionPreconditionlist.end();

                            if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr);
                                const double acIn = (*numericAchievedInLayer)[currPrec];
                                if (acIn > 0.0 && !numericIsTrueInState[currPrec]) {
                                    if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";
                                    
                                    payload->fluentLayers.requestNumericPrecondition(currPrec, acIn, currTS - EPSILON, goalsAtLayer, tilR);
                                    isHelpful = false;
                                } else {
                                    if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                                }

                            }

                            payload->fluentLayers.recordSideEffects(currAchievedBy.first, currAchievedBy.second, currTS - EPSILON);
                            
                        }
                                                                     
                                                                     

                        if (evaluateDebug) cout << "\t\tPreconditions done.\n";

                        if (isHelpful) {
                            payload->helpfulActions.push_front(ActionSegment(RPGBuilder::getInstantiatedOp(currAchievedBy.first), VAL::E_AT_START, -1, emptyIntSet));
                            if (evaluateDebug) cout << "\t\tIs a helpful action\n";
                        }

                        double & oldR = earliestDeadlineRelevancyStart[currAchievedBy.first];
                        if (oldR > tilR) oldR = tilR;


                    } else if (currAchievedBy.second == VAL::E_AT_END) {
                        if (evaluateDebug) cout << "\t\tUsing end of " << currAchievedBy.first << " - " << *(RPGBuilder::getInstantiatedOp(currAchievedBy.first)) << endl;
                        
                        if (RPGHeuristic::printRPGAsDot) {
                            payload->dot.highlightAction(currAchievedBy.first, VAL::E_AT_END);
                            payload->dot.actionMeetsFactInRP(currAchievedBy.first, VAL::E_AT_END, nextToAchieve);
                        }
                                                            
                        
                        if (evaluateDebug && currTS == EPSILON) cout << "\t\tIs a helpful action\n";
                        relaxedPlan.front().second.push_back(ActionSegment(getOp(currAchievedBy.first), VAL::E_AT_END, -1, emptyIntList));
    //                  if (currTS == EPSILON) {
    //                      helpfulActions.push_back(currAchievedBy);
    //                  }
                        if (!TemporalAnalysis::canSkipToEnd(currAchievedBy.first)) ++h;

                        const double sTIL = earliestTILForAction(currAchievedBy.first, false);
                        if (tilR > sTIL) tilR = sTIL;


                        {
                            list<Literal*> & actionEffectsList = (*actionsToEndEffects)[currAchievedBy.first];
                            list<Literal*>::iterator aelItr = actionEffectsList.begin();
                            const list<Literal*>::iterator aelEnd = actionEffectsList.end();

                            for (; aelItr != aelEnd; ++aelItr) {
                                map<int, double>::iterator cgItr = currGAL.find((*aelItr)->getStateID());
                                if (cgItr != currGAL.end()) {
                                    if (tilR > cgItr->second) tilR = cgItr->second;
                                    currGAL.erase(cgItr);
                                }
                            }
                        }

                        bool isHelpful = true;
                        {
                            list<Literal*> & actionPreconditionlist = (*actionsToEndPreconditions)[currAchievedBy.first];
                            list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                            const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();

                            if (evaluateDebug) cout << "\t\tPreconditions:\n";

                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr)->getStateID();
                                double acIn;
                                pair<int, VAL::time_spec> acBy;
                                
                                getAchieverDetails(currPrec, currTS - EPSILON, acIn, acBy);
                                
                                if (acIn > 0.0) {
                                    if (acBy.first != -1) {
                                        map<int, double>::iterator galItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                        if (galItr->second > tilR) galItr->second = tilR;
                                        if (evaluateDebug) cout << "\t\tAdding requirement for " << currPrec << " at time " << acIn << "\n";
                                        assert(acIn < currTS);
                                        isHelpful = false;
                                    } else {
                                        if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state at layer " << acIn << "\n";
                                    }
                                } else {
                                    if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state\n";
                                }

                            }

                            if (evaluateDebug) cout << "\t\tPreconditions done\n";

                            if (theState.startedActions.find(currAchievedBy.first) == theState.startedActions.end()) {
                                const double sas = startActionSchedule[currAchievedBy.first];
                                if (sas == -1.0) {
                                    assert(TemporalAnalysis::canSkipToEnd(currAchievedBy.first));
                                    if (!TemporalAnalysis::canSkipToEnd(currAchievedBy.first)) {
                                        cerr << "Critical error - trying to schedule goal to before start of plan\n";
                                        exit(1);
                                    }
                                } else {
                                    map<int, pair<int,double> >::iterator galItr = goalsAtLayer[sas + EPSILON].actionStarts.insert(pair<int, pair<int, double> >(currAchievedBy.first, pair<int,double>(1,tilR))).first;
                                    if (galItr->second.second > tilR) galItr->second.second = tilR;
                                    if (evaluateDebug) cout << "\t\tAdding requirement for start of action at time " << (startActionSchedule[currAchievedBy.first] + EPSILON) << "\n";
                                }

                                isHelpful = false;
                            } else {
                                if (evaluateDebug) cout << "\t\tAction has already been started, do not need to schedule start in RPG\n";
                            }
                        }

                        if (!ignoreNumbers) {
                            list<int> & actionPreconditionlist = (*actionsToNumericEndPreconditions)[currAchievedBy.first];
                            list<int>::iterator aplItr = actionPreconditionlist.begin();
                            const list<int>::iterator aplEnd = actionPreconditionlist.end();

                            if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr);
                                const double acIn = (*numericAchievedInLayer)[currPrec];
                                if (acIn > 0.0 && !numericIsTrueInState[currPrec]) {
                                    if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";
                                    
                                    payload->fluentLayers.requestNumericPrecondition(currPrec, acIn, currTS - EPSILON, goalsAtLayer, tilR);
                                } else {
                                    if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                                }

                            }

                            payload->fluentLayers.recordSideEffects(currAchievedBy.first,  currAchievedBy.second, currTS - EPSILON);                            

                        }


                        if (evaluateDebug) cout << "\t\tPreconditions done.\n";

                        if (isHelpful) {
                            payload->helpfulActions.push_front(ActionSegment(RPGBuilder::getInstantiatedOp(currAchievedBy.first), VAL::E_AT_END, -1, emptyIntSet));
                            if (evaluateDebug) cout << "\t\tIs a helpful action\n";
                        }


                        double & oldR = earliestDeadlineRelevancyEnd[currAchievedBy.first];
                        if (oldR > tilR) oldR = tilR;

                    } else { // aha, is a timed initial literal!

    //                  if (currTS == EPSILON) {
    //                      helpfulActions.push_back(currAchievedBy);
    //                  }
                        earliestTIL = currAchievedBy;

                        {
                            list<int> & tilEffectsList = tilEffects[currAchievedBy.first];
                            list<int>::iterator aelItr = tilEffectsList.begin();
                            const list<int>::iterator aelEnd = tilEffectsList.end();

                            for (; aelItr != aelEnd; ++aelItr) {
                                currGAL.erase(*aelItr);
                            }
                        }

                    }

                }

            }                        
            if (evaluateDebug) cout << "All goals at this TS now satisfied\n";
                                        
            goalsAtLayer.erase(currTS);
        }
    }
    
    /** @brief Add the numeric effects of an action to the RPG.
     * 
     * @param payload  The data describing the RPG currently being built
     * @param currAct  The action ID to apply     
     * @param currTS   The time specifier of the action: <code>VAL::E_AT_START</code> or <code>VAL::E_AT_END</code>
     * @param nlTime   The fact layer altered by the consequences of its effects
     * @param limitTo  A limit on how many times the action can be applied
     *
     * @return <code>true</code> if adding the action led to all goals being achieved; <code>false</code> otherwise.
     */                
    bool applyNumericEffects(Private::BuildingPayload * const payload, const int & currAct, const VAL::time_spec & currTS, const double & nlTime, const int & limitTo);
    
    
    /** @brief Add the propositional effects of an action to the RPG
     * 
     * @param payload  The data describing the RPG currently being built
     * @param currAct  The action ID to apply     
     * @param currTS   The time specifier of the action: <code>VAL::E_AT_START</code> or <code>VAL::E_AT_END</code>
     * @param nlTime   The fact layer altered by the consequences of its effects
     * @param POtime   Data to support use of the partial-order modified heuristic.
     *
     * @return <code>true</code> if adding the action led to all goals being achieved; <code>false</code> otherwise.
     */                
    bool applyPropositionalEffects(Private::BuildingPayload * const payload, const int & currAct, const VAL::time_spec & currTS, const bool & openEnd, const double & nlTime, MaxDependentInfo & POtime);

    bool checkPreconditionsAreSatisfied(const int & currAct, const VAL::time_spec & ts, const double & layer);

    bool updateActionsForNewLiteralFact(BuildingPayload * const payload, const int & toPropagate, const double & factLayerTime, const bool & isActuallyNew);
    bool updateActionsForNewNumericFact(BuildingPayload * const payload, const int & toPropagate, const double & factLayerTime);
    bool applyEndEffectNow(BuildingPayload * const payload, const int & currAct, const bool & openAct, const double & factLayerTime);

#ifdef POPF3ANALYSIS
    void calculateGoalCost(BuildingPayload * const payload);
    void updateAdditiveCosts(BuildingPayload * const payload, const int & ci, const double & deadline, const int & updateForGoal, double & costToUpdate);
#endif
    
};

#ifdef MDIDEBUG
RPGHeuristic::Private::BuildingPayload * RPGHeuristic::Private::MaxDependentInfo::referTo = 0;
bool RPGHeuristic::Private::MaxDependentInfo::debug = false;
#endif

vector<double> RPGHeuristic::Private::earliestStartAllowed;
vector<double> RPGHeuristic::Private::earliestEndAllowed;
vector<double> RPGHeuristic::Private::latestStartAllowed;
vector<double> RPGHeuristic::Private::latestEndAllowed;
vector<double> RPGHeuristic::Private::deadlineAtTime;
vector<double> RPGHeuristic::Private::earliestDeadlineRelevancyStart;
vector<double> RPGHeuristic::Private::earliestDeadlineRelevancyEnd;

vector<list<int> > RPGHeuristic::Private::tilEffects;
vector<list<int> > RPGHeuristic::Private::tilNegativeEffects;
vector<list<int> > RPGHeuristic::Private::tilTemporaryNegativeEffects;
vector<double> RPGHeuristic::Private::tilTimes;
bool RPGHeuristic::Private::tilInitialised = false;
int RPGHeuristic::Private::tilCount = 0;

vector<double> RPGHeuristic::Private::earliestPropositionPOTimes;
vector<double> RPGHeuristic::Private::earliestNumericPOTimes;
//vector<double> RPGHeuristic::Private::earliestNumericPrePOTimes;

vector<vector<set<int> > > RPGHeuristic::Private::actionsAffectedByFluent;

#ifdef POPF3ANALYSIS
vector<vector<double> > RPGHeuristic::Private::startEffectsOnResourceLimits;
vector<vector<double> > RPGHeuristic::Private::endEffectsOnResourceLimits;    
vector<bool> RPGHeuristic::Private::costsAreIndependentGoalCosts;
vector<vector<double> > RPGHeuristic::Private::maxPermissibleCostOfAFact;
#endif

vector<double> & RPGHeuristic::getEarliestForStarts()
{
    return Private::earliestStartAllowed;
};
vector<double> & RPGHeuristic::getEarliestForEnds()
{
    return Private::earliestEndAllowed;
};

double & RPGHeuristic::getDeadlineRelevancyStart(const int & i)
{
    return Private::earliestDeadlineRelevancyStart[i];
}

double & RPGHeuristic::getDeadlineRelevancyEnd(const int & i)
{
    return Private::earliestDeadlineRelevancyEnd[i];
}


void RPGHeuristic::doFullExpansion(MinimalState & refState)
{
    set<int> dummyGoals;
    set<int> dummyGoalFluents;
    list<ActionSegment> dummyHelpful;
    list<pair<double, list<ActionSegment> > > dummyRP;
    vector<double> minTimestamps(1, 0.0);
    double dummyEstimate;
    d->buildEmptyActionFluentLookupTable();
    
    const bool wasBlind = blindSearch;
    const bool wasNoNumbers = ignoreNumbers;
    
    d->expandFully = true;
    blindSearch = false;
    ignoreNumbers = false;
    
    vector<double> timeAtWhichValueIsDefined(refState.secondMin.size(),0.0);
    
    getRelaxedPlan(refState, 0, minTimestamps, 0.0, refState.secondMin, refState.secondMax, timeAtWhichValueIsDefined, dummyHelpful, dummyRP, dummyEstimate);
    
    d->expandFully = false;    
    blindSearch = wasBlind;
    ignoreNumbers = wasNoNumbers;
}

RPGHeuristic::RPGHeuristic(const bool & b,
                           vector<list<Literal*> > * atse,
                           vector<list<Literal*> > * atee,
                           vector<list<pair<int, VAL::time_spec> > > * eta,
                           vector<list<Literal*> > * atsne,
                           vector<list<Literal*> > * atene,
                           vector<list<pair<int, VAL::time_spec> > > * neta,
                           vector<list<pair<int, VAL::time_spec> > > * pta,
                           vector<list<Literal*> > * atsp,
                           vector<list<Literal*> > * ati,
                           vector<list<Literal*> > * atep,
                           vector<list<RPGBuilder::NumericEffect> > * atnuse,
                           vector<list<RPGBuilder::NumericEffect> > * atnuee,
                           vector<list<int> > * atrnuse,
                           vector<list<int> > * atrnuee,
                           vector<list<int> > * atnusp,
                           vector<list<int> > * atnui,
                           vector<list<int> > * atnuep,
                           vector<list<int> > * atpnuep,
                           vector<int> * iusp,
                           vector<int> * iuip,
                           vector<int> * iuep,
                           vector<double> * ail,
                           vector<double> * ailr,
                           vector<pair<int, VAL::time_spec> > * ab,
                           vector<pair<int, VAL::time_spec> > * abr,
                           vector<double> * nail,
                           vector<double> * nailr,
                           vector<ActionFluentModification*> * nab,
                           vector<ActionFluentModification*> * nabr,
                           vector<int> * iunsp,
                           vector<int> * iuni,
                           vector<int> * iunep,
                           vector<RPGBuilder::RPGNumericPrecondition> * rnp,
                           vector<RPGBuilder::RPGNumericEffect> * rne,
                           vector<list<pair<int, VAL::time_spec> > > * ppta,
                           vector<list<pair<int, VAL::time_spec> > > * nppta,
                           vector<list<Literal*> > * atpsp,
                           vector<int> * iupsp,
                           vector<int> * iupsnp,
                           list<pair<int, VAL::time_spec> > * pla,
                           list<pair<int, VAL::time_spec> > * onpa)
        :   d(new Private(b,
                          atse,
                          atee,
                          eta,
                          atsne,
                          atene,
                          neta,
                          pta,
                          atsp,
                          ati,
                          atep,
                          atnuse,
                          atnuee,
                          atrnuse,
                          atrnuee,
                          atnusp,
                          atnui,
                          atnuep,
                          atpnuep,
                          iusp,
                          iuip,
                          iuep,
                          ail,
                          ailr,
                          ab,
                          abr,
                          nail,
                          nailr,
                          nab,
                          nabr,
                          iunsp,
                          iuni,
                          iunep,
                          rnp,
                          rne,
                          ppta,
                          nppta,
                          atpsp,
                          iupsp,
                          iupsnp,
                          pla,
                          onpa))
{

    {
        
        d->literalGoalVector.resize(RPGBuilder::getLiteralGoals().size(), (Literal*) 0);
        
        list<Literal*>::iterator gsItr = RPGBuilder::getLiteralGoals().begin();
        const list<Literal*>::iterator gsEnd = RPGBuilder::getLiteralGoals().end();

        for (int gID = 0; gsItr != gsEnd; ++gsItr, ++gID) {
            if (!RPGBuilder::isStatic(*gsItr).first) {
                d->goals.insert((*gsItr)->getStateID());
                d->literalGoalVector[gID] = *gsItr;
            }
        }

        d->gsEnd = d->goals.end();
    }

    {
        list<pair<int, int> >::iterator gsItr = RPGBuilder::getNumericRPGGoals().begin();
        const list<pair<int, int> >::iterator gsEnd = RPGBuilder::getNumericRPGGoals().end();

        for (; gsItr != gsEnd; ++gsItr) {
            if (gsItr->first != -1) {
                d->goalFluents.insert(gsItr->first);
            }
            if (gsItr->second != -1) {
                d->goalFluents.insert(gsItr->second);
            }
        }

        d->gfEnd = d->goalFluents.end();
    }

};

RPGHeuristic::~RPGHeuristic()
{
    if (d->deleteArrays) {
        assert(false);
    }
}



RPGHeuristic* RPGBuilder::generateRPGHeuristic()
{

    // for now, don't prune the RPG

    return new RPGHeuristic(false,  // subproblem does not own the arrays
                            &actionsToStartEffects,
                            &actionsToEndEffects,
                            &effectsToActions,
                            &actionsToStartNegativeEffects,
                            &actionsToEndNegativeEffects,
                            &negativeEffectsToActions,
                            &preconditionsToActions,
                            &actionsToStartPreconditions,
                            &actionsToInvariants,
                            &actionsToEndPreconditions,
                            &actionsToStartNumericEffects,
                            &actionsToEndNumericEffects,
                            &actionsToRPGNumericStartEffects,
                            &actionsToRPGNumericEndEffects,
                            &actionsToRPGNumericStartPreconditions,
                            &actionsToRPGNumericInvariants,
                            &actionsToRPGNumericEndPreconditions,
                            &actionsToProcessedStartRPGNumericPreconditions,
                            &initialUnsatisfiedStartPreconditions,
                            &initialUnsatisfiedInvariants,
                            &initialUnsatisfiedEndPreconditions,
                            &achievedInLayer,
                            &achievedInLayerReset,
                            &achievedBy,
                            &achievedByReset,
                            &numericAchievedInLayer,
                            &numericAchievedInLayerReset,
                            &numericAchievedBy,
                            &numericAchievedByReset,
                            &initialUnsatisfiedNumericStartPreconditions,
                            &initialUnsatisfiedNumericInvariants,
                            &initialUnsatisfiedNumericEndPreconditions,
                            &rpgNumericPreconditions,
                            &rpgNumericEffects,
                            &processedPreconditionsToActions,
                            &processedRPGNumericPreconditionsToActions,
                            &actionsToProcessedStartPreconditions,
                            &initialUnsatisfiedProcessedStartPreconditions,
                            &initialUnsatisfiedProcessedStartNumericPreconditions,
                            &preconditionlessActions,
                            &onlyNumericPreconditionActions);

};

bool RPGHeuristic::Private::EndPrecRescale::operator <(const RPGHeuristic::Private::EndPrecRescale & r) const
{

    if (var < r.var) return true;
    if (r.var > var) return false;

    if (offset < r.offset) return true;
    if (offset > r.offset) return false;

    if (totalchange < r.totalchange) return true;
    if (totalchange > r.totalchange) return false;

    if (duration < r.duration) return true;
    if (duration > r.duration) return false;

    return false;


};


set<int> RPGHeuristic::emptyIntList;
unsigned int RPGHeuristic::statesEvaluated = 0;

#ifdef POPF3ANALYSIS
void RPGHeuristic::metricHasChanged()
{
    d->metricHasChanged();
}
#endif

int RPGHeuristic::getRelaxedPlan(const MinimalState & theState, const list<StartEvent> * startEventQueue,
                                 const vector<double> & minTimestamps, const double & stateTS,
                                 const vector<double> & extrapolatedMin, const vector<double> & extrapolatedMax, const vector<double> & timeAtWhichValueIsDefined,
                                 list<ActionSegment> & helpfulActions, list<pair<double, list<ActionSegment> > > & relaxedPlan,double & finalPlanMakespanEstimate,
                                 map<double, list<pair<int, int> > > * justApplied, double tilFrom)
{

    const bool evaluateDebug = Globals::globalVerbosity & 64;

    d->setDebugFlag(evaluateDebug);
    
    if (!d->expandFully) {
        ++statesEvaluated;
    }
        
//    const int vCount = theState.secondMin.size();
//    const int avCount = RPGBuilder::getAVCount();

    const int nextTIL = theState.nextTIL;

    finalPlanMakespanEstimate = 0.0;

    d->integrateContinuousEffects();

//  startedActionExtraEndPreconditions.clear();

    
    d->populateActionFluentLookupTable();

    if (evaluateDebug) cout << "Evaluating a state with " << theState.startedActions.size() << " sorts of actions on the go\n";

    //rpprintState(theState);

    auto_ptr<Private::BuildingPayload> payload(d->spawnNewPayload(theState, startEventQueue, minTimestamps, helpfulActions));

    map<double, list<DelayedGradientDescriptor> > delayedGradients;
    
    d->giveUsTheEffectsOfExecutingActions(payload.get(), timeAtWhichValueIsDefined, delayedGradients);

    /*if (evaluateDebug || eeDebug) {
    const int eeuSize = extraEndUnsatisfied.size();
    for (int i = 0; i < eeuSize; ++i) {
    if (extraEndUnsatisfied[i]) cout << "Still extra ends attached to " << *(RPGBuilder::getInstantiatedOp(i)) << "\n";
    }
    }*/
//  MILPRPG(theState.second);

    int heuristicOffset = 0;

    d->addRequirementToHaveSeenTheEndOfAllCurrentlyExecutingActions(payload.get());

    if (evaluateDebug) {
        cout << "Aiming for number of goals satisfied: " << payload->unsatisfiedGoals << "\n";
    }

    d->seeWhatGoalsAreTrueToStartWith(payload.get());

    if (evaluateDebug) {
        cout << "Goals unsatisfied in initial state: " << payload->unsatisfiedGoals << "\n";
        cout << "Ends not appeared so far: " << payload->unappearedEnds << "\n";
    }

    if (d->expandFully) payload->unsatisfiedGoals = INT_MAX;

    if (!payload->unsatisfiedGoals && !payload->unappearedEnds) return heuristicOffset;


    if (blindSearch) return 1;
    
    d->resetAchievedBy();
    d->recordFactLayerZero(payload.get());
    d->performTILInitialisation();

    d->initialiseLatestArrays();

    d->addTemporalConstraintsFromActiveActions(payload.get());
    
    if (RPGBuilder::modifiedRPG) {
        d->delayOpenEndsUntilTheirPOPositions(payload.get());
    }

    if (!d->addTemporalConstraintsFromActiveActions(payload.get(), justApplied, stateTS, nextTIL, tilFrom)) {

        // will return false in cases where there's a direct contradiction between a TIL
        // and an executing action's invariants

        return -1;
    }
    d->addTILsBeforeExpansion(nextTIL, payload->factLayers, stateTS, tilFrom);
    d->addBoundsFromTemporalAnalysis(stateTS);


    d->noLongerForbidden.clear();


    {
        if (evaluateDebug) cout << "Considering preconditionless actions\n";
        d->updateActionsForNewLiteralFact(payload.get(), -1, 0.0, true);
    }






    if (payload->unsatisfiedGoals || payload->unappearedEnds) {

        if (!RPGBuilder::modifiedRPG && RPGBuilder::sortedExpansion) {

            map<double, list<int> > expansionOrder;
                        
            StateFacts::const_iterator stateItr = theState.first.begin();
            const StateFacts::const_iterator stateEnd = theState.first.end();

            for (; stateItr != stateEnd; ++stateItr) {
                const int factID = FACTA(stateItr);
                const double poTS = d->earliestPropositionPOTimes[factID];
                expansionOrder[poTS].push_back(factID);
            }


            map<double, list<int> >::const_iterator eoItr = expansionOrder.begin();
            const map<double, list<int> >::const_iterator eoEnd = expansionOrder.end();

            for (; eoItr != eoEnd; ++eoItr) {
                list<int>::const_iterator fItr = eoItr->second.begin();
                const list<int>::const_iterator fEnd = eoItr->second.end();

                for (; fItr != fEnd; ++fItr) {
                    if (d->updateActionsForNewLiteralFact(payload.get(), *fItr, 0.0, true)) break;
                }
                if (fItr != fEnd) break;
            }


        } else {            
            if (evaluateDebug) cout << "Considering " << theState.first.size() << " initial propositional facts\n";
            {
                StateFacts::const_iterator stateItr = theState.first.begin();
                const StateFacts::const_iterator stateEnd = theState.first.end();

                for (; stateItr != stateEnd; ++stateItr) {
                    const int factID = FACTA(stateItr);
                    //const double poTS = (RPGBuilder::modifiedRPG ? 0.0 : d->earliestPropositionPOTimes[factID]);
                    #ifdef POPF3ANALYSIS
                    const double layer = d->achieverDetails[factID].back().layer;
                    #else
                    const double layer = (*(d->achievedInLayer))[factID];
                    #endif
                    if (layer == 0.0) {
                        if (RPGHeuristic::printRPGAsDot) {
                            payload->dot.addFactNode(0.0, factID, true);
                        }
                        if (evaluateDebug) cout << "Updating from fact " << factID << " " << *(RPGBuilder::getLiteral(factID)) << "\n";
                        if (d->updateActionsForNewLiteralFact(payload.get(), factID, 0.0, true)) break;
                    } else {
                        if (evaluateDebug) cout << "Modified RPG: Not updating from fact " << factID << " until " << layer << "\n";
                        payload->factLayers[layer].first.push_back(factID);
                        if (RPGHeuristic::printRPGAsDot) {
                            payload->dot.addFactNode(layer, factID, true);
                        }
                    }
                }
            }
            
        }

    }

    if (payload->unsatisfiedGoals || payload->unappearedEnds) {       
        const int loopLim = d->rpgNumericPreconditions->size();

        if (loopLim) {
            if (evaluateDebug) cout << "Considering initial numeric facts\n";
            
            const vector<double> * const maxFluentTable = &(payload->fluentLayers.borrowFactLayerZeroValues());
            for (int i = 0; i < loopLim; ++i) {
                RPGBuilder::RPGNumericPrecondition & currPre = (*(d->rpgNumericPreconditions))[i];
                if (ignoreNumbers || currPre.isSatisfied(*maxFluentTable)) {
                    const double layer = (*(d->numericAchievedInLayer))[i];
                    if (layer == 0.0) {
                        if (printRPGAsDot) {
                            payload->dot.addNumericFactNode(0.0, i, true);
                        }
                        if (evaluateDebug) {
                            cout << "Updating from numeric fact " << i << ":  " << (*(d->rpgNumericPreconditions))[i];
                            cout << "; was achieved in the initial state\n";                            
                        }
                        if (d->updateActionsForNewNumericFact(payload.get(), i, 0.0)) {
                            break;
                        }
                    } else {
                        if (evaluateDebug) cout << "Modified RPG: Not updating from numeric fact " << i << ":  " << (*(d->rpgNumericPreconditions))[i] << " until " << layer << "\n";
                        if (printRPGAsDot) {
                            payload->dot.addNumericFactNode(layer, i, true);
                        }
                        payload->factLayers[layer].second.push_back(i);
                    }
                }
            }
            
                        
        }
        
        if (!delayedGradients.empty()) {
            d->addDelayedGradientEffects(payload.get(), delayedGradients);
        }
                    
        
        list<int> newNumericPreconditions;
        
        payload->fluentLayers.applyRecentlyRecordedEffects(*(d->numericAchievedInLayer), newNumericPreconditions, d->maxNeeded);
        
        if (!newNumericPreconditions.empty()) {
            list<int>::const_iterator npItr = newNumericPreconditions.begin();
            const list<int>::const_iterator npEnd = newNumericPreconditions.end();
            
            for (; npItr != npEnd; ++npItr) {
                if (d->goalFluents.find(*npItr) != d->gfEnd) {
                    if (!(--(payload->unsatisfiedGoals))) break;
                }
            }
                            
            if (payload->unsatisfiedGoals) {
                if (RPGBuilder::modifiedRPG) {
                    
                    list<int>::const_iterator preID = newNumericPreconditions.begin();
                    const list<int>::const_iterator preIDEnd = newNumericPreconditions.end();
                    
                    for (; preID != preIDEnd; ++preID) {
                        const double earliestPOPoint = earliestPointForNumericPrecondition(RPGBuilder::getNumericPreTable()[*preID], &(d->earliestNumericPOTimes));
                        if (earliestPOPoint > EPSILON) {
                            payload->factLayers[earliestPOPoint].second.push_back(*preID);
                            if (printRPGAsDot) {
                                payload->dot.addNumericFactNode(earliestPOPoint, *preID);
                            }                            
                        } else {
                            payload->factLayers[EPSILON].second.push_back(*preID);
                            if (printRPGAsDot) {
                                payload->dot.addNumericFactNode(EPSILON, *preID);
                            }
                            
                        }
                    }
                    
                } else {
                    list<int> & dest = payload->factLayers[EPSILON].second;
                    dest.insert(dest.end(), newNumericPreconditions.begin(), newNumericPreconditions.end());
                }
            }
        }
    }


    if (evaluateDebug) {
        cout << "Unsatisfied goals: " << payload->unsatisfiedGoals << ", Unappeared ends: " << payload->unappearedEnds << "\n";
    }

#ifdef POPF3ANALYSIS
    const bool deadline2Debug = false;
#endif

    NextFactLayer nextHappening;
    
    if (payload->unsatisfiedGoals || payload->unappearedEnds) {
        
#ifdef POPF3ANALYSIS
        {
            
            const list<Literal*> & literalGoals = RPGBuilder::getLiteralGoals();
            const list<double> & literalGoalDeadlines = RPGBuilder::getLiteralGoalDeadlines();
            
            list<Literal*>::const_iterator gItr = literalGoals.begin();        
            const list<Literal*>::const_iterator gEnd = literalGoals.end();
            
            list<double>::const_iterator gdItr = literalGoalDeadlines.begin();
            
            for (int fID; gItr != gEnd; ++gItr, ++gdItr) {
                
                if (RPGBuilder::isStatic(*gItr).first) {
                    continue;
                }
                
                fID = (*gItr)->getStateID();
                          
                if (theState.first.find(fID) != theState.first.end()) continue;


                payload->factLayers[*gdItr].literalGoalsWeMustHaveByNow.push_back(fID);
                
                if (deadline2Debug) {
                    cout << "Must have " << *(*gItr) << " by " << *gdItr << endl;
                }
            }
            
        }
        
        
        {
         
            const list<pair<int,int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
            const list<double> & numericGoalDeadlines = RPGBuilder::getNumericRPGGoalDeadlines();
            
            const vector<double> * const maxFluentTable = &(payload->fluentLayers.borrowFactLayerZeroValues());
            
            list<pair<int,int> >::const_iterator gItr = numericGoals.begin();
            const list<pair<int,int> >::const_iterator gEnd = numericGoals.end();
            
            list<double>::const_iterator gdItr = numericGoalDeadlines.begin();
            
            for (int fID; gItr != gEnd; ++gItr, ++gdItr) {
                
                for (int ipass = 0; ipass < 2; ++ipass) {
                    fID = (ipass ? gItr->second : gItr->first);
                    
                    if (fID < 0) continue;
                    
                    const RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[fID];

                    if (!ignoreNumbers && !currPre.isSatisfied(*maxFluentTable)) {
                        payload->factLayers[*gdItr].numericGoalsWeMustHaveByNow.push_back(fID);
                        
                        if (deadline2Debug) {
                            cout << "Must have " << currPre << " by " << *gdItr << endl;
                        }
                        
                    }
                }
                
            }
        }

#endif
        
        payload->nextThingToVisit(0.0, nextHappening);
    } else {
        #ifdef POPF3ANALYSIS
        payload->tooExpensive = false;
        #endif
    }
    
    #ifdef POPF3ANALYSIS
    const bool breakOnGoalsAndEnds = NumericAnalysis::getGoalNumericUsageLimits().empty();
    #else
    const bool breakOnGoalsAndEnds = true;
    #endif
    
    #ifdef POPF3ANALYSIS
    bool failedToMeetDeadline = false;
    #endif

    while (!nextHappening.empty()) {

        if (evaluateDebug) {
            cout << "Unsatisfied goals: " << payload->unsatisfiedGoals << ", Unappeared ends: " << payload->unappearedEnds << "\n";
            cout << "Expanding RPG forwards\n";
        }

        #ifdef STOCHASTICDURATIONS        
        if (nextHappening.timestamp.first > solutionDeadlineTime + (EPSILON / 2)) {
            break;
        }        
        #endif

        {
            // make sure the fluent layer just before the actions that are about to appear exists            
            payload->fluentLayers.createThenIgnore(nextHappening.timestamp.first);
        }
        
        if (nextHappening.revisitInstantaneousNumericEffects) {
            payload->fluentLayers.recordEffectsThatAreNowToBeRevisited(payload->factLayers);
        }
        
        if (nextHappening.gradientsCauseFactsToBecomeTrue) {
            payload->fluentLayers.recordConsequencesOfActiveGradients(nextHappening.timestamp.first + EPSILON);
        }
        
        if (nextHappening.endActionsAppearingAtThisTime != payload->endActionsAtTime.end()) {

            if (evaluateDebug) cout << "ACTION LAYER AT TIME " << nextHappening.endActionsAppearingAtThisTime->first << "\n";

            const map<double, list<int>, EpsilonComp >::iterator & eaatItr = nextHappening.endActionsAppearingAtThisTime;

            list<int>::iterator eaItr = eaatItr->second.begin();
            const list<int>::iterator eaEnd = eaatItr->second.end();

            const double cTime = eaatItr->first;

            for (; eaItr != eaEnd; ++eaItr) {
                int actToPass = *eaItr;
                bool actIsOpen = false;
                if (actToPass < 0) {
                    actToPass = -actToPass - 1;
                    actIsOpen = true;
                }
                if (cTime > Private::latestEndAllowed[actToPass]) {
                    if (evaluateDebug) {
                        cout << "End of action has been cancelled: invariant or one-way end precondition deleted by TIL\n";
                    }
                } else {
                    if (evaluateDebug) {
                        cout << "Applying ";
                        if (actIsOpen) cout << "open ";
                        cout << "end effect of " << actToPass << " " << *(RPGBuilder::getInstantiatedOp(actToPass)) << "\n";
                    }
                    if (d->applyEndEffectNow(payload.get(), actToPass, actIsOpen, cTime) && breakOnGoalsAndEnds) break;
                }
            }
            if (evaluateDebug) cout << "Finished action layer\n";
            payload->endActionsAtTime.erase(eaatItr);

        }
        
        if (nextHappening.newFactsAtThisTime != payload->factLayers.end()) {
            
            const map<double, FactLayerEntry, EpsilonComp >::iterator currFactLayerItr = nextHappening.newFactsAtThisTime;
            if (evaluateDebug) cout << "FACT LAYER AT TIME " << currFactLayerItr->first << "\n";

            const double cTime = currFactLayerItr->first;


            {
                list<int>::iterator stateItr = currFactLayerItr->second.first.begin();
                const list<int>::iterator stateEnd = currFactLayerItr->second.first.end();

                for (; stateItr != stateEnd; ++stateItr) {
                    if (evaluateDebug) cout << "Updating from fact " << *stateItr << ", " << *(RPGBuilder::getLiteral(*stateItr)) << endl;
                    if (d->updateActionsForNewLiteralFact(payload.get(), *stateItr, cTime, true) && breakOnGoalsAndEnds) break;
                }

            }
            
            {
                set<int>::iterator stateItr = currFactLayerItr->second.firstRepeated.begin();
                const set<int>::iterator stateEnd = currFactLayerItr->second.firstRepeated.end();
                
                for (; stateItr != stateEnd; ++stateItr) {
                    if (evaluateDebug) cout << "Updating from repeated fact " << *stateItr << ", " << *(RPGBuilder::getLiteral(*stateItr)) << endl;
                    if (d->updateActionsForNewLiteralFact(payload.get(), *stateItr, cTime, false) && breakOnGoalsAndEnds) break;
                }
                
            }

            {
                list<int>::iterator stateItr = currFactLayerItr->second.second.begin();
                const list<int>::iterator stateEnd = currFactLayerItr->second.second.end();

                for (; stateItr != stateEnd; ++stateItr) {

                    if (evaluateDebug) {
                        cout << "Updating from numeric fact " << *stateItr << ":  " << (*(d->rpgNumericPreconditions))[*stateItr] << endl;
                    }
                    if (d->updateActionsForNewNumericFact(payload.get(), *stateItr, cTime) && breakOnGoalsAndEnds) {
                        break;
                    }
                }

            }


            {
                list<pair<int, int> >::iterator stateItr = currFactLayerItr->second.TILs.begin();
                const list<pair<int, int> >::iterator stateEnd = currFactLayerItr->second.TILs.end();

                for (; stateItr != stateEnd; ++stateItr) {
                    if (evaluateDebug) cout << "Updating from TIL " << stateItr->first << ", fact " << stateItr->second << "\n";

                    const int applyResult = d->proposeNewAchiever(stateItr->second, stateItr->first, VAL::E_AT, false, cTime);
                    if (applyResult) {
                        if (d->updateActionsForNewLiteralFact(payload.get(), stateItr->second, cTime, (applyResult == 2)) && breakOnGoalsAndEnds) break;
                    }


                }
            }

            if (currFactLayerItr->second.endOfJustApplied) {
                d->noLongerForbidden.clear();
                set<int> & startSet = currFactLayerItr->second.endOfJustApplied->first;
                set<int> & endSet = currFactLayerItr->second.endOfJustApplied->second;

                bool spawn = false;

                {
                    set<int>::iterator fItr = startSet.begin();
                    const set<int>::iterator fEnd = startSet.end();

                    for (; fItr != fEnd; ++fItr) {
                        map<int, int>::iterator nrItr = payload->forbiddenStart.find(*fItr);
                        if (!(--(nrItr->second))) {
                            payload->forbiddenStart.erase(nrItr);
                            if (!payload->startPreconditionCounts[*fItr]) {
                                d->noLongerForbidden.push_back(pair<int, VAL::time_spec>(*fItr, VAL::E_AT_START));
                                spawn = true;
                            }
                        }
                    }
                }
                {
                    set<int>::iterator fItr = endSet.begin();
                    const set<int>::iterator fEnd = endSet.end();

                    for (; fItr != fEnd; ++fItr) {
                        map<int, int>::iterator nrItr = payload->forbiddenEnd.find(*fItr);
                        if (!(--(nrItr->second))) {
                            payload->forbiddenEnd.erase(nrItr);
                            if (!payload->endPreconditionCounts[*fItr]) {
                                d->noLongerForbidden.push_back(pair<int, VAL::time_spec>(*fItr, VAL::E_AT_END));
                                spawn = true;
                            }
                        }
                    }
                }

                if (spawn) {
                    if (d->updateActionsForNewLiteralFact(payload.get(), -2, cTime, true) && breakOnGoalsAndEnds) break;
                }
            }

            {
                list<int>::iterator negItr = currFactLayerItr->second.negativeTILs.begin();
                const list<int>::iterator negEnd = currFactLayerItr->second.negativeTILs.end();

                for (; negItr != negEnd; ++negItr) {

                    list<pair<int, VAL::time_spec> > & currList = (*(d->preconditionsToActions))[*negItr];

                    list<pair<int, VAL::time_spec> >::iterator ioItr = currList.begin();
                    const list<pair<int, VAL::time_spec> >::iterator ioEnd = currList.end();

                    for (; ioItr != ioEnd; ++ioItr) {
                        if (ioItr->second == VAL::E_AT_START) {
                            {
                                int & checkAndIncr = payload->startPreconditionCounts[ioItr->first];
                                if (checkAndIncr) {
                                    ++checkAndIncr;
                                    ++(payload->endPreconditionCounts[ioItr->first]);
                                }
                            }
                        } else {
                            ++(payload->startPreconditionCounts[ioItr->first]);
                            ++(payload->endPreconditionCounts[ioItr->first]);
                        }
                    }

                }
            }

            #ifdef POPF3ANALYSIS
            {
                list<int>::const_iterator gItr = currFactLayerItr->second.literalGoalsWeMustHaveByNow.begin();
                const list<int>::const_iterator gEnd = currFactLayerItr->second.literalGoalsWeMustHaveByNow.end();
                
                for (; gItr != gEnd; ++gItr) {
                    #ifdef POPF3ANALYSIS
                    if (d->achieverDetails[*gItr].empty()) {
                    #else
                    if ((*(d->achievedInLayer))[*gItr] < 0.0) {
                    #endif
                        failedToMeetDeadline = true;
                        break;
                    }

                }
            }
            
            if (!failedToMeetDeadline) {
                
                list<int>::const_iterator gItr = currFactLayerItr->second.numericGoalsWeMustHaveByNow.begin();
                const list<int>::const_iterator gEnd = currFactLayerItr->second.numericGoalsWeMustHaveByNow.end();
                
                for (; gItr != gEnd; ++gItr) {
                    if ((*(d->numericAchievedInLayer))[(*gItr)] < 0.0) {
                        failedToMeetDeadline = true;
                        break;
                    }
                }
                
            }
            #endif

            if (evaluateDebug) cout << "Finished fact layer\n";


            payload->factLayers.erase(currFactLayerItr);

        }
        
        #ifdef POPF3ANALYSIS
        if (!failedToMeetDeadline) 
        #endif
        {
            // now we've applied all the effects we're going to get, see if new numeric preconditions are true
            // epsilon in the future
            
            list<int> newNumericPreconditions;            
            payload->fluentLayers.applyRecentlyRecordedEffects(*(d->numericAchievedInLayer), newNumericPreconditions, d->maxNeeded);
            
            if (!newNumericPreconditions.empty()) {
                                
                list<int>::const_iterator npItr = newNumericPreconditions.begin();
                const list<int>::const_iterator npEnd = newNumericPreconditions.end();
                
                for (; npItr != npEnd; ++npItr) {
                    if (d->goalFluents.find(*npItr) != d->gfEnd) {
                        --(payload->unsatisfiedGoals);
                    }
                }
                
                if (RPGBuilder::modifiedRPG) {
                    
                    list<int>::const_iterator preID = newNumericPreconditions.begin();
                    const list<int>::const_iterator preIDEnd = newNumericPreconditions.end();
                    
                    for (; preID != preIDEnd; ++preID) {
                        const double earliestPOPoint = earliestPointForNumericPrecondition(RPGBuilder::getNumericPreTable()[*preID], &(d->earliestNumericPOTimes));
                        if (earliestPOPoint > nextHappening.timestamp.first + EPSILON) {
                            payload->factLayers[earliestPOPoint].second.push_back(*preID);
                            if (printRPGAsDot) {
                                payload->dot.addNumericFactNode(earliestPOPoint, *preID);
                            }
                        } else {
                            payload->factLayers[nextHappening.timestamp.first + EPSILON].second.push_back(*preID);
                            if (printRPGAsDot) {
                                payload->dot.addNumericFactNode(nextHappening.timestamp.first + EPSILON, *preID);
                            }
                            
                        }
                    }
                                        
                } else {
                    FactLayerEntry & factsGoAt = payload->factLayers[nextHappening.timestamp.first + EPSILON];
                                        
                    factsGoAt.second.insert(factsGoAt.second.end(), newNumericPreconditions.begin(), newNumericPreconditions.end());
                }
            } else {
                if (evaluateDebug) {
                    cout << "No new numeric preconditions became true\n";
                }                
            }
        }
        
        #ifdef POPF3ANALYSIS
        if (failedToMeetDeadline) {
            break;
        }
        #endif
    
#ifdef POPF3ANALYSIS
        if (!payload->unsatisfiedGoals && !payload->unappearedEnds) {
            d->calculateGoalCost(payload.get());
        }
#endif

        if (payload->unsatisfiedGoals || payload->unappearedEnds
            #ifdef POPF3ANALYSIS
            || payload->tooExpensive
            #endif            
        ) {
            const double thisTS = nextHappening.timestamp.first;
            payload->nextThingToVisit(thisTS, nextHappening);
        } else {
            break;
        }
        
    }


    if (payload->unsatisfiedGoals || payload->unappearedEnds
#ifdef POPF3ANALYSIS
        || payload->tooExpensive
        || failedToMeetDeadline
#endif                
    ) {
        if (RPGHeuristic::printRPGAsDot) {
            const DotDetails & dot = payload->dot;
            ostringstream dotfn;
            dotfn << "rpg" << statesEvaluated << ".dot";
            const string fn = dotfn.str();
            ofstream dotfile(fn.c_str());
            if (dotfile.bad()) {
                cerr << "Warning: cannot write dot file to " << fn << endl;
            } else {
                dotfile << dot;
                dotfile.close();
            }
        }
        
        
        if (evaluateDebug) {
            cout << "Dead end found in RPG\n";
            
            cout << "unsatisfiedGoals = " << payload->unsatisfiedGoals << ", unappearedEnds = " << payload->unappearedEnds;
            #ifdef POPF3ANALYSIS
            if (payload->tooExpensive) {
                cout << ", too expensive";
            }
            if (failedToMeetDeadline) {
                cout << ", failed to meet deadline";
            }
            #endif            
            cout << endl;
            
            #ifdef ENABLE_DEBUGGING_HOOKS
            if (!Globals::remainingActionsInPlan.empty()) {
                list<ActionSegment>::const_iterator actItr = Globals::remainingActionsInPlan.begin();
                const list<ActionSegment>::const_iterator actEnd = Globals::remainingActionsInPlan.end();
                
                for (; actItr != actEnd; ++actItr) {
                    if (actItr->second == VAL::E_AT) {
                        // skip TILs
                        continue;
                    }
                
                    cout << "Seeing if preconditions of " << *(actItr->first);
                    if (actItr->second == VAL::E_AT_START) {
                        cout << ", start, are satisfied\n";
                        if (payload->startPreconditionCounts[actItr->first->getID()]) {
                            cout << "Noted number of unsatisfied start preconditions: " << payload->startPreconditionCounts[actItr->first->getID()] << endl;
                        }
                        map<int,int>::iterator fItr = payload->forbiddenStart.find(actItr->first->getID());
                        if (fItr != payload->forbiddenStart.end()) {
                            cout << "Noted as being forbidden: " << fItr->second << endl;
                        }
                    } else {
                        cout << ", end, are satisfied\n";
                        if (payload->endPreconditionCounts[actItr->first->getID()]) {
                            cout << "Noted number of unsatisfied end preconditions: " << payload->endPreconditionCounts[actItr->first->getID()] << endl;
                        }                        
                        map<int,int>::iterator fItr = payload->forbiddenEnd.find(actItr->first->getID());
                        if (fItr != payload->forbiddenEnd.end()) {
                            cout << "Noted as being forbidden: " << fItr->second << endl;
                        }                        
                    }
                    
                    {
                        const list<Literal*> pres = (actItr->second == VAL::E_AT_START
                                                        ? (*(d->actionsToProcessedStartPreconditions))[actItr->first->getID()]
                                                        : (*(d->actionsToEndPreconditions))[actItr->first->getID()]);
                        
                        list<Literal*>::const_iterator pItr = pres.begin();
                        const list<Literal*>::const_iterator pEnd = pres.end();
                        
                        for (; pItr != pEnd; ++pItr) {
                            double acIn;
                            pair<int, VAL::time_spec> acBy;
                            
                            d->getAchieverDetails((*pItr)->getStateID(), DBL_MAX, acIn, acBy);
                            
                            if (acIn < 0.0) {                                
                                cout << "\tPrecondition " << *(*pItr) << " was never seen. ";
                            } else {                                
                                if (theState.first.find((*pItr)->getStateID()) != theState.first.end()) {
                                    cout << "\tPrecondition " << *(*pItr) << " was achieved in layer " << acIn << endl;
                                } else {
                                    cout << "\tPrecondition " << *(*pItr) << " was true in the partial order at " << acIn << endl;
                                }
                            }
                        }
                    }
                    
                    {
                        const list<int> & pres = (actItr->second == VAL::E_AT_START
                                                    ? (*(d->actionsToProcessedStartNumericPreconditions))[actItr->first->getID()]
                                                    : (*(d->actionsToNumericEndPreconditions))[actItr->first->getID()]);

                        list<int>::const_iterator pItr = pres.begin();
                        const list<int>::const_iterator pEnd = pres.end();
                        
                        for (; pItr != pEnd; ++pItr) {
                            
                            if ((*(d->numericAchievedInLayer))[(*pItr)] < 0.0) {                                
                                cout << "\tNumeric precondition " << RPGBuilder::getNumericPreTable()[*pItr] << " was never seen. ";
                            } else {
                                if (d->numericIsTrueInState[*pItr]) {
                                    cout << "\tNumeric precondition " << RPGBuilder::getNumericPreTable()[*pItr] << " is true in state, PO time " << (*(d->numericAchievedInLayer))[(*pItr)] << endl;
                                } else {
                                    cout << "\tNumeric precondition " << RPGBuilder::getNumericPreTable()[*pItr] << " was achieved at time " << (*(d->numericAchievedInLayer))[(*pItr)] << endl;
                                } 
                            
                            }
                        }
                        
                    }
                }
            }
            #endif
            
            set<int>::const_iterator gItr = d->goals.begin();
            const set<int>::const_iterator gEnd = d->goals.end();

            for (; gItr != gEnd; ++gItr) {
                #ifdef POPF3ANALYSIS
                if (d->achieverDetails[*gItr].empty()) {
                #else
                if ((*(d->achievedInLayer))[*gItr] < 0.0) {
                #endif
                    cout << "Goal " << *gItr << ", " << *(RPGBuilder::getLiteral(*gItr)) << ", was never seen. ";
                    assert(!(RPGBuilder::isStatic(RPGBuilder::getLiteral(*gItr)).first));
                    cout << "Possible achievers: " << (*(d->effectsToActions))[*gItr].size() << endl;
                    list<pair<int, VAL::time_spec> >::const_iterator aItr = (*(d->effectsToActions))[*gItr].begin();
                    const list<pair<int, VAL::time_spec> >::const_iterator aEnd = (*(d->effectsToActions))[*gItr].end();

                    for (; aItr != aEnd; ++aItr) {
                        if (aItr->second == VAL::E_AT_START) {
                            if (payload->startActionSchedule[aItr->first] == -1.0) {
                                cout << "- Start of " << aItr->first << ", " << *(RPGBuilder::getInstantiatedOp(aItr->first)) << ", was never seen\n";
                            }
                            list<Literal*> & actPres = (*(d->actionsToProcessedStartPreconditions))[aItr->first];
                            list<Literal*>::const_iterator preItr = actPres.begin();
                            const list<Literal*>::const_iterator preEnd = actPres.end();
                            for (; preItr != preEnd; ++preItr) {
                                double acIn;
                                pair<int, VAL::time_spec> acBy;
                                
                                d->getAchieverDetails((*preItr)->getStateID(), DBL_MAX, acIn, acBy);
                                if (acIn < 0.0) {
                                    cout << "   * Start precondition " << *(*preItr) << " never appeared\n";
                                }
                            }
                        } else {
                            if (payload->endActionSchedule[aItr->first] == -1.0) {
                                if (payload->startActionSchedule[aItr->first] != -1.0) {
                                    cout << "- End of " << aItr->first << ", " << *(RPGBuilder::getInstantiatedOp(aItr->first)) << ", was never seen\n";
                                } else {
                                    cout << "- End of " << aItr->first << ", " << *(RPGBuilder::getInstantiatedOp(aItr->first)) << " was never seen (nor was its start)\n";

                                    list<Literal*> & actPres = (*(d->actionsToProcessedStartPreconditions))[aItr->first];
                                    list<Literal*>::const_iterator preItr = actPres.begin();
                                    const list<Literal*>::const_iterator preEnd = actPres.end();
                                    for (; preItr != preEnd; ++preItr) {
                                        double acIn;
                                        pair<int, VAL::time_spec> acBy;
                                        
                                        d->getAchieverDetails((*preItr)->getStateID(), DBL_MAX, acIn, acBy);
                                        if (acIn < 0.0) {
                                            cout << "   * Start precondition " << *(*preItr) << " never appeared\n";
                                        }
                                    }
                                }
                                list<Literal*> & actPres = (*(d->actionsToEndPreconditions))[aItr->first];
                                list<Literal*>::const_iterator preItr = actPres.begin();
                                const list<Literal*>::const_iterator preEnd = actPres.end();
                                for (; preItr != preEnd; ++preItr) {
                                    double acIn;
                                    pair<int, VAL::time_spec> acBy;
                                    
                                    d->getAchieverDetails((*preItr)->getStateID(), DBL_MAX, acIn, acBy);
                                    if (acIn < 0.0) {
                                        cout << "   * End precondition " << *(*preItr) << " never appeared\n";
                                    }
                                }

                            }
                        }
                    }
                }
            }

        }
        if (evaluateDebug) cout << "Leaving getRelaxedPlan()\n";
        return -1;
    } else {
        if (evaluateDebug) cout << "RPG found all goals and ends\n";
    }



    int h = heuristicOffset;

//  evaluateDebug = true;


    if (!ignoreNumbers && !timeAtWhichValueIsDefined.empty()) {
        const int loopLim = d->rpgNumericPreconditions->size();
        for (int i = 0; i < loopLim; ++i) {
            if (!d->numericIsTrueInState[i]) {
                if ((*(d->rpgNumericPreconditions))[i].isSatisfiedWCalculate(extrapolatedMin, extrapolatedMax)) {
                    d->numericIsTrueInState[i] = true;
                }
            }
        }        
    }

    pair<int, VAL::time_spec> earliestTIL(-1, VAL::E_AT);

    d->extractRP(payload.get(), h, relaxedPlan, earliestTIL, finalPlanMakespanEstimate);


    if (RPGHeuristic::printRPGAsDot) {
        const DotDetails & dot = payload->dot;
        ostringstream dotfn;
        dotfn << "rpg" << statesEvaluated << ".dot";
        const string fn = dotfn.str();
        ofstream dotfile(fn.c_str());
        if (dotfile.bad()) {
            cerr << "Warning: cannot write dot file to " << fn << endl;
        } else {
            dotfile << dot;
            dotfile.close();
        }
    }
    
    

    {
        // HACK

        map<int, int> started;
        map<int, int>::iterator insItr = started.end();
        
        {
            map<int, set<int> >::const_iterator saItr = theState.startedActions.begin();
            const map<int, set<int> >::const_iterator saEnd = theState.startedActions.end();

            for (; saItr != saEnd; ++saItr) {
                insItr = started.insert(insItr, make_pair(saItr->first, saItr->second.size()));                
            }

        }
        list<pair<double, list<ActionSegment> > >::iterator rpItr = relaxedPlan.begin();
        const list<pair<double, list<ActionSegment> > >::iterator rpEnd = relaxedPlan.end();

        for (; rpItr != rpEnd; ++rpItr) {

            list<ActionSegment>::iterator rlItr = rpItr->second.begin();
            const list<ActionSegment>::iterator rlEnd = rpItr->second.end();

            for (; rlItr != rlEnd; ++rlItr) {
                if (rlItr->second == VAL::E_AT) continue;
                if (rlItr->second == VAL::E_AT_END) {
                    if (!TemporalAnalysis::canSkipToEnd(rlItr->first->getID())) {
                        insItr = started.insert(insItr, make_pair(rlItr->first->getID(), 0));
                        --(insItr->second);
                    }
                } else {
                    if (!RPGBuilder::getRPGDEs(rlItr->first->getID()).empty()
                        && !TemporalAnalysis::canSkipToEnd(rlItr->first->getID())) {                        
                        insItr = started.insert(insItr, make_pair(rlItr->first->getID(), 0));
                        ++(insItr->second);
                    }
                }
            }

        }

        map<int, int>::iterator sItr = started.begin();
        const map<int, int>::iterator sEnd = started.end();

        for (; sItr != sEnd; ++sItr) {
            //assert(sItr->second >= 0);
            if (sItr->second > 0) {
                h += sItr->second;
            }
        }


    }

    list<double> haWeights;

    if (RPGBuilder::fullFFHelpfulActions) {

        set<int> startsIn;
        set<int> endsIn;

        {
            list<ActionSegment>::iterator haItr = helpfulActions.begin();
            const list<ActionSegment>::iterator haEnd = helpfulActions.end();

            for (; haItr != haEnd; ++haItr) {
                if (haItr->second == VAL::E_AT_START) {
                    startsIn.insert(haItr->first->getID());
                } else if (haItr->second == VAL::E_AT_END) {
                    endsIn.insert(haItr->first->getID());
                }
            }
        }

        {
            list<ActionSegment>::iterator haItr = helpfulActions.begin();
            const list<ActionSegment>::iterator haEnd = helpfulActions.end();

            for (; haItr != haEnd; ++haItr) {
                list<ActionSegment>::iterator haNext = haItr;
                ++haNext;
                const list<Literal*> * effs = 0;
                if (haItr->second == VAL::E_AT_START) {
                    effs = &((*(d->actionsToStartEffects))[haItr->first->getID()]);
                } else {
                    continue;
                }

                const double sas = payload->startActionSchedule[haItr->first->getID()];

                list<Literal*>::const_iterator eItr = effs->begin();
                const list<Literal*>::const_iterator eEnd = effs->end();

                for (; eItr != eEnd; ++eItr) {
                    const list<pair<int, VAL::time_spec> > & acBy = (*(d->effectsToActions))[(*eItr)->getStateID()];
                    list<pair<int, VAL::time_spec> >::const_iterator candItr = acBy.begin();
                    const list<pair<int, VAL::time_spec> >::const_iterator candEnd = acBy.end();

                    for (; candItr != candEnd; ++candItr) {
                        set<int> * checkIn = 0;
                        const list<Literal*> * deps = 0;
                        if (candItr->second == VAL::E_AT_START) {
                            const double thisSAS = payload->startActionSchedule[candItr->first];
                            if (thisSAS == -1.0) continue;
                            if (thisSAS > sas) continue;
                            checkIn = &startsIn;
                            deps = &((*(d->actionsToProcessedStartPreconditions))[candItr->first]);
                        } else {
                            continue;
                        }
                        if (checkIn->insert(candItr->first).second) {
                            /*                            // first time we've had to work out if this is helpful
                                                        list<Literal*>::const_iterator depItr = deps->begin();
                                                        const list<Literal*>::const_iterator depEnd = deps->end();

                                                        for (; depItr != depEnd; ++depItr) {
                                                            const int factID = (*depItr)->getID();
                                                            const double acIn = (*(d->achievedInLayer))[factID];
                                                            if (acIn == -1.0) break;
                                                            if (acIn > 0.0 && (*(d->achievedBy))[factID].first == -1) break;
                                                        }
                                                        if (depItr == depEnd) {*/
                            helpfulActions.insert(haNext, ActionSegment(RPGBuilder::getInstantiatedOp(candItr->first), candItr->second, -1, emptyIntList));
                            /*}*/
                        }
                    }
                }

                if (haItr->second == VAL::E_AT_START) {
                    effs = &((*(d->actionsToEndEffects))[haItr->first->getID()]);

                    const double endStamp = payload->endActionSchedule[haItr->first->getID()];
                    list<Literal*>::const_iterator eItr = effs->begin();
                    const list<Literal*>::const_iterator eEnd = effs->end();

                    for (; eItr != eEnd; ++eItr) {
                        const list<pair<int, VAL::time_spec> > & acBy = (*(d->effectsToActions))[(*eItr)->getStateID()];
                        list<pair<int, VAL::time_spec> >::const_iterator candItr = acBy.begin();
                        const list<pair<int, VAL::time_spec> >::const_iterator candEnd = acBy.end();
                        for (; candItr != candEnd; ++candItr) {
                            if (candItr->second != VAL::E_AT_END) continue;
                            if (fabs(payload->endActionSchedule[candItr->first] - endStamp) < 0.0001) {
                                if (endsIn.insert(candItr->first).second && startsIn.insert(candItr->first).second) {
                                    /*const list<Literal*> * deps = &((*(d->actionsToProcessedStartPreconditions))[candItr->first]);

                                    list<Literal*>::const_iterator depItr = deps->begin();
                                    const list<Literal*>::const_iterator depEnd = deps->end();

                                    for (; depItr != depEnd; ++depItr) {
                                        const int factID = (*depItr)->getID();
                                        const double acIn = (*(d->achievedInLayer))[factID];
                                        if (acIn == -1.0) break;
                                        if (acIn > 0.0 && (*(d->achievedBy))[factID].first == -1) break;
                                    }
                                    if (depItr == depEnd) {                                                                */
                                    helpfulActions.insert(haNext, ActionSegment(RPGBuilder::getInstantiatedOp(candItr->first), VAL::E_AT_START, -1, emptyIntList));
                                    /*}*/
                                }

                            }
                        }
                    }

                }

            }
        }
    }


    if (false && !relaxedPlan.empty()) {

        list<ActionSegment> unsortedList;
        unsortedList.swap(helpfulActions);

        if (h && earliestTIL.first != -1) {

            helpfulActions.push_back(ActionSegment(0, earliestTIL.second, earliestTIL.first, emptyIntList));
            haWeights.push_back(DBL_MAX);

        }


        list<ActionSegment>::iterator rlItr = unsortedList.begin();
        const list<ActionSegment>::iterator rlEnd = unsortedList.begin();

        for (; rlItr != rlEnd; ++rlItr) {
            //const int thisIOp = rlItr->first;
            if (nextTIL < Private::tilCount && rlItr->second != VAL::E_AT) {
// HACK
                const double w = (rlItr->second == VAL::E_AT_START ? Private::earliestDeadlineRelevancyStart[rlItr->first->getID()] : Private::earliestDeadlineRelevancyEnd[rlItr->first->getID()]);

                list<ActionSegment>::iterator haItr = helpfulActions.begin();
                const list<ActionSegment>::iterator haEnd = helpfulActions.end();
                list<double>::iterator wItr = haWeights.begin();

                for (; haItr != haEnd && (*wItr) < w; ++haItr, ++wItr);

                helpfulActions.insert(haItr, *rlItr);
                haWeights.insert(wItr, w);
                if (w == DBL_MAX) {
//                  cout << thisID << " " << "inf\n";
                } else {
//                  cout << thisID << " " << w << "\n";
                }


            } else {
                helpfulActions.push_back(*rlItr);
                haWeights.push_back(DBL_MAX);
            }
        }


    }

    if (false) {
        cout << "Relaxed plan for state:\n";
        list<pair<double, list<ActionSegment> > >::iterator rpItr = relaxedPlan.begin();
        const list<pair<double, list<ActionSegment> > >::iterator rpEnd = relaxedPlan.end();

        for (; rpItr != rpEnd; ++rpItr) {
            cout << "\t" << rpItr->first << ":\n";
            list<ActionSegment>::iterator rlItr = rpItr->second.begin();
            const list<ActionSegment>::iterator rlEnd = rpItr->second.end();

            for (; rlItr != rlEnd; ++rlItr) {
                cout << "\t\t" << *(rlItr->first);
                if (rlItr->second == VAL::E_AT_END) {
                    cout << ", end\n";
                } else {
                    cout << ", start\n";
                }
            }


        }
    }


    if (evaluateDebug || Globals::globalVerbosity & 1048576) {

        cout << "Helpful actions:\n";

        list<ActionSegment>::iterator haItr = helpfulActions.begin();
        const list<ActionSegment>::iterator haEnd = helpfulActions.end();

        for (; haItr != haEnd; ++haItr) {
            if (haItr->second == VAL::E_AT) {
                cout << "Timed initial literal action " << haItr->divisionID << "\n";
            } else {
                cout << *(haItr->first) << ", ";
                if (haItr->second == VAL::E_AT_START) cout << "start\n";
                if (haItr->second == VAL::E_AT_END) cout << "end\n";
                if (haItr->second == VAL::E_OVER_ALL) cout << "middle point " << haItr->divisionID << "\n";
            }
        }


    }

    return h;

};

double RPGHeuristic::Private::earliestTILForAction(const unsigned int & i, const bool & isStart)
{

    assert(i >= 0);
    assert(i < actionsToEndPreconditions->size());
    double toReturn = DBL_MAX;

    list<Literal*> & inspect = (isStart ? (*actionsToProcessedStartPreconditions)[i] : (*actionsToEndPreconditions)[i]);
    list<Literal*>::iterator insItr = inspect.begin();
    const list<Literal*>::iterator insEnd = inspect.end();

//  cout << "\t\tRequires precs:\n";
    for (; insItr != insEnd; ++insItr) {
//      cout << "\t\t\t" << *(*insItr);
        assert(*insItr);
        double & thisD = deadlineAtTime[(*insItr)->getStateID()];
        if (toReturn > thisD) toReturn = thisD;
        if (thisD == DBL_MAX) {
//          cout << ", which is never a deadline\n";
        } else {
//          cout << ", which is a deadline at " << thisD << "\n";
        }
    }

    return toReturn;

};

double RPGHeuristic::earliestTILForAction(const int & i, const bool & isStart)
{

    return d->earliestTILForAction(i, isStart);

};

bool RPGHeuristic::Private::applyPropositionalEffects(Private::BuildingPayload * const payload, const int & currAct, const VAL::time_spec & currTS, const bool & openEnd, const double & nlTime, MaxDependentInfo & POtime)
{
    static const bool updateDebug = Globals::globalVerbosity & 64;

    list<Literal*> & addEffects = (currTS == VAL::E_AT_START ? RPGBuilder::getStartPropositionAdds()[currAct] : RPGBuilder::getEndPropositionAdds()[currAct]);

    list<Literal*>::iterator addEffItr = addEffects.begin();
    const list<Literal*>::iterator addEffEnd = addEffects.end();

    for (; addEffItr != addEffEnd; ++addEffItr) {
        const int currEff = (*addEffItr)->getStateID();
        const int applyResult = proposeNewAchiever(currEff, currAct, currTS, openEnd, nlTime);
                       
        if (applyResult) {
            if (applyResult == 1) {
                // fact is repeated, now with lower cost
                if (RPGHeuristic::printRPGAsDot) {
                    payload->dot.addActionToFactEdge(currAct, currTS, currEff);
                }
                payload->factLayers[nlTime].firstRepeated.insert(currEff);
            } else {
                // fact is properly new
                if (RPGHeuristic::printRPGAsDot) {
                    payload->dot.addFactNode(nlTime, currEff);
                    payload->dot.addActionToFactEdge(currAct, currTS, currEff);
                }
                                                            
                                                            
                payload->factLayers[nlTime].first.push_back(currEff);
                //earliestPropositionPOTimes[currEff] = POtime.get();
                if (updateDebug) cout << "\t\tFact " << currEff << " is new\n";
                if (goals.find(currEff) != gsEnd) {
                    if (!(--(payload->unsatisfiedGoals)) && !(payload->unappearedEnds)) return true;
                }
            }
        } else {
            if (updateDebug) cout << "\t\tFact " << currEff << " was already achieved\n";
        }
    }

    return false;
}

bool RPGHeuristic::Private::checkPreconditionsAreSatisfied(const int & currAct, const VAL::time_spec & ts, const double & layer)
{

    {
        const list<Literal*> & precList = (ts == VAL::E_AT_START
                                           ? (*actionsToProcessedStartPreconditions)[currAct]
                                           : (*actionsToEndPreconditions)[currAct]
                                          );


        list<Literal*>::const_iterator pItr = precList.begin();
        const list<Literal*>::const_iterator pEnd = precList.end();


        for (; pItr != pEnd; ++pItr) {
            #ifdef POPF3ANALYSIS
            assert(!achieverDetails[(*pItr)->getStateID()].empty());
            assert(achieverDetails[(*pItr)->getStateID()].front().layer <= layer);
            assert(achieverDetails[(*pItr)->getStateID()].front().achiever != make_pair(currAct, ts));
            if (evaluateDebug) cout << "\t\t\t\tPrecondition " << *(*pItr) << " first became true at " << achieverDetails[(*pItr)->getStateID()].front().layer << "\n";
            #else
            assert((*achievedInLayer)[(*pItr)->getStateID()] != -1.0);
            assert((*achievedInLayer)[(*pItr)->getStateID()] <= layer);
            assert((*achievedBy)[(*pItr)->getStateID()] != make_pair(currAct, ts));
            if (evaluateDebug) cout << "\t\t\t\tPrecondition " << *(*pItr) << " became true at " << (*achievedInLayer)[(*pItr)->getStateID()] << "\n";
            #endif
        }
    }

    {
        const list<int> & precList = (ts == VAL::E_AT_START
                                      ? (*actionsToProcessedStartNumericPreconditions)[currAct]
                                      : (*actionsToNumericEndPreconditions)[currAct]
                                     );

        list<int>::const_iterator pItr = precList.begin();
        const list<int>::const_iterator pEnd = precList.end();


        for (; pItr != pEnd; ++pItr) {
            assert((*numericAchievedInLayer)[(*pItr)] != -1.0);
            assert((*numericAchievedInLayer)[(*pItr)] <= layer);
            ActionFluentModification * const ac = (*numericAchievedBy)[(*pItr)];
            if (ac) {
                assert(!(ac->act == currAct && ac->ts == ts));
            }
            if (evaluateDebug) cout << "\t\t\t\tNumeric precondition " << (*pItr) << " became true at " << (*numericAchievedInLayer)[(*pItr)] << "\n";
        }
    }

    return true;
}

bool RPGHeuristic::Private::applyNumericEffects(Private::BuildingPayload * const payload,
                                                const int & currAct, const VAL::time_spec & currTS, const double & nlTime,
                                                const int & limitTo)
{
    
    const int actualLimit = (limitTo != -1 ? limitTo : RPGBuilder::howManyTimesOptimistic(currAct, payload->startState));
    
    if (actualLimit == 0) {
        if (evaluateDebug) {
            cout << "\t\tNot applying numeric effects: are not interesting\n";
        }
        return false;
    }
    
    assert(actualLimit > 0);
    
    ActionAndHowManyTimes details(currAct, currTS,
                                  actualLimit,
                                  RPGBuilder::getOpMinDuration(currAct, 0), RPGBuilder::getOpMaxDuration(currAct, 0) );
    
    {
        // first, instantaneous numeric effects
        const list<int> & numericEffects = (currTS == VAL::E_AT_START ? (*actionsToRPGNumericStartEffects)[currAct]
                                                                      : (*actionsToRPGNumericEndEffects)[currAct]  );
        
        list<int>::const_iterator numEffItr = numericEffects.begin();
        const list<int>::const_iterator numEffEnd = numericEffects.end();
        
        for (; numEffItr != numEffEnd; ++numEffItr) {
            payload->fluentLayers.recordInstantaneousNumericEffect(details, nlTime, *numEffItr);
        }
    }

    if (currTS == VAL::E_AT_START) {
        // any continuous effects?
        
        // first, any that have been integrated
        if (!integratedCTSEffectChange[currAct].empty()) {            
            list<double>::const_iterator contChangeItr = integratedCTSEffectChange[currAct].begin();
            const list<double>::const_iterator contChangeEnd = integratedCTSEffectChange[currAct].end();
            
            list<int>::const_iterator contChangeVar = integratedCTSEffectVar[currAct].begin();
            
            for (; contChangeItr != contChangeEnd; ++contChangeItr, ++contChangeVar) {
                payload->fluentLayers.recordIntegratedNumericEffect(details, nlTime, make_pair(*contChangeVar, *contChangeItr));
            }
        }
        
        if (!gradientCTSEffectChange[currAct].empty()) {

            list<double>::const_iterator contChangeItr = gradientCTSEffectChange[currAct].begin();
            const list<double>::const_iterator contChangeEnd = gradientCTSEffectChange[currAct].end();
            
            list<int>::const_iterator contChangeVar = gradientCTSEffectVar[currAct].begin();
            
            for (; contChangeItr != contChangeEnd; ++contChangeItr, ++contChangeVar) {
                payload->fluentLayers.recordGradientNumericEffect(details, nlTime, make_pair(*contChangeVar, *contChangeItr), payload->factLayers);

            }
            
        }
    }
    
    return false;
}
   

bool RPGHeuristic::Private::updateActionsForNewLiteralFact(Private::BuildingPayload * const payload, const int & toPropagate, const double & factLayerTime, const bool & decrementRemainingUnsatisfiedPreconditionCounters)
{
    
    vector<int> & startPreconditionCounts = payload->startPreconditionCounts;
    vector<int> & endPreconditionCounts = payload->endPreconditionCounts;
    vector<int> & numericStartPreconditionCounts = payload->numericStartPreconditionCounts;
    vector<int> & numericEndPreconditionCounts = payload->numericEndPreconditionCounts;
    map<double, list<int>, EpsilonComp > & endActionsAtTime = payload->endActionsAtTime;
    vector<double> & startActionSchedule = payload->startActionSchedule;
    vector<double> & endActionSchedule = payload->endActionSchedule;
    vector<double> & openEndActionSchedule = payload->openEndActionSchedule;
    const map<int, set<int> > & startedActions = payload->startedActions;
    int & unsatisfiedGoals = payload->unsatisfiedGoals;
    int & unappearedEnds = payload->unappearedEnds;
    map<int, set<int> > & insistUponEnds = payload->insistUponEnds;
    map<int, int> & forbiddenStart = payload->forbiddenStart;
    map<int, int> & forbiddenEnd = payload->forbiddenEnd;


    const double nlTime = factLayerTime + EPSILON;
    const bool updateDebug = Globals::globalVerbosity & 64;
    const bool preconditionless = (toPropagate < 0);
    list<pair<int, VAL::time_spec> > & dependents = ((toPropagate == -1) ? (*preconditionlessActions) : ((toPropagate == -2) ? (noLongerForbidden) : (*processedPreconditionsToActions)[toPropagate]));

    if (evaluateDebug) {
        if (toPropagate == -1) {
            cout << "*Special case: preconditionless actions\n";
        } else if (toPropagate == -2) {
            cout << "*Special case: actions that are no longer forbidden\n";
        }
    }

    list<pair<int, VAL::time_spec> >::iterator depItr = dependents.begin();
    const list<pair<int, VAL::time_spec> >::iterator depEnd = dependents.end();

    if (updateDebug) cout << "\tAffects " << dependents.size() << " actions\n";

    for (; depItr != depEnd; ++depItr) {
        const int currAct = depItr->first;
        const bool startAct = (depItr->second == VAL::E_AT_START);
        assert(depItr->second != VAL::E_OVER_ALL);
        if (updateDebug) cout << "\tAffects " << currAct << ", " << (startAct ? "start" : "end") << " " << *(RPGBuilder::getInstantiatedOp(currAct)) << "\n";
        if (startAct) {
            int & spc = startPreconditionCounts[currAct];
            if (!preconditionless && decrementRemainingUnsatisfiedPreconditionCounters) --spc;
            if ((!spc && !numericStartPreconditionCounts[currAct]) && forbiddenStart.find(currAct) == forbiddenStart.end() && nlTime < latestStartAllowed[currAct]) {
                assert(!RPGBuilder::rogueActions[currAct]);
                if (updateDebug) {
                    cout << "\tStart of action " << currAct << " is now applicable: " << startPreconditionCounts[currAct] << "/" << numericStartPreconditionCounts[currAct];
                    if (preconditionless) cout << ", considered preconditionless";
                    cout << "\n";
                }

                if (RPGHeuristic::printRPGAsDot) {
                    payload->dot.addActionNode(factLayerTime, depItr->first, depItr->second);
                }

                //assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

                if (expandFully) earliestStartAllowed[currAct] = factLayerTime;

                if (decrementRemainingUnsatisfiedPreconditionCounters) {
                    startActionSchedule[currAct] = factLayerTime;
                }

                MAKESTARTMDI(POtime, currAct);

                if (applyPropositionalEffects(payload, currAct, VAL::E_AT_START, false, nlTime, POtime)) return true;

                if (applyNumericEffects(payload, currAct, VAL::E_AT_START, nlTime, -1)) return true;


                if (decrementRemainingUnsatisfiedPreconditionCounters) {
                    static pair<double, list<int> > defaultEntry;
                    
                    defaultEntry.first = factLayerTime + RPGBuilder::getOpMinDuration(currAct, 0);
                    
                    endActionSchedule[currAct] = defaultEntry.first;

                    if (!endPreconditionCounts[currAct] && !numericEndPreconditionCounts[currAct]) {
                        if (updateDebug) cout << "\t\tEnd of action can be put at fact layer sufficiently far in future (" << defaultEntry.first << "\n";

                        endActionsAtTime.insert(defaultEntry).first->second.push_back(currAct);
                    }
                }

                //assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

            } else {
                if (updateDebug) cout << "\tStart of action " << currAct << " now only has " << startPreconditionCounts[currAct] << " unsatisfied propositional preconditions and " << numericStartPreconditionCounts[currAct] << " numeric\n";
            }
        } else {
            int & epc = endPreconditionCounts[currAct];
            if (!preconditionless && decrementRemainingUnsatisfiedPreconditionCounters) --epc;
            if ((!epc && !numericEndPreconditionCounts[currAct]) && forbiddenEnd.find(currAct) == forbiddenEnd.end() && nlTime < latestEndAllowed[currAct]) {
                if (RPGBuilder::rogueActions[currAct]) {
                    cout << "Critical Error: Trying to apply end of action " << currAct << ", " << *(RPGBuilder::getInstantiatedOp(currAct)) << ", which is invalid or irrelevant\n";
                    if (&dependents == preconditionlessActions) {
                        cout << "\tFound it on the list of preconditionless actions\n";
                    } else {
                        cout << "\tFound it on the list of actions depending on " << RPGBuilder::getLiteral(toPropagate) << "\n";
                    }
                    assert(!RPGBuilder::rogueActions[currAct]);
                }
                if (updateDebug) cout << "\tEnd of action " << currAct << " is now applicable\n";


                //assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

                bool insistOnThisEnd = (insistUponEnds.find(currAct) != insistUponEnds.end());
                
                if (decrementRemainingUnsatisfiedPreconditionCounters) {
                
                    if (expandFully) earliestEndAllowed[currAct] = factLayerTime;

                    
                    if (insistOnThisEnd) {
                        if (updateDebug) cout << "\tEnd is insisted upon, had " << unappearedEnds << " remaining\n";
                        if (factLayerTime - openEndActionSchedule[currAct] > -0.0001) {
                            openEndActionSchedule[currAct] = factLayerTime;
                            if (!(--unappearedEnds) && !unsatisfiedGoals) return true;
                            if (updateDebug) cout << "\tNow down to " << unappearedEnds << "\n";
                        } else {
                            if (updateDebug) cout << "\tOpen end cannot appear quite yet\n";
                            insistOnThisEnd = false;
                            endActionsAtTime[openEndActionSchedule[currAct]].push_back(-1 - currAct);
                            if (!(--unappearedEnds) && !unsatisfiedGoals) return true;
                            if (updateDebug) cout << "\tNow down to " << unappearedEnds << "\n";
                        }
                    }
                }




                for (int epPass = 0; epPass < 2; ++epPass) {
                    bool doLoop = false;
                    bool limitToOnce = false;
                    if (!epPass) {
                        doLoop = insistOnThisEnd;
                        limitToOnce = true;
                    } else {
                        if (!startPreconditionCounts[currAct] && !numericStartPreconditionCounts[currAct] && startActionSchedule[currAct] > -1.0) {
                            const double enforcedEnd = endActionSchedule[currAct];
                            if (factLayerTime - enforcedEnd > -0.0001) {
                                if (decrementRemainingUnsatisfiedPreconditionCounters) {
                                    endActionSchedule[currAct] = factLayerTime;
                                }
                                doLoop = true;
                                if (updateDebug) {
                                    cout << "\t\tDoing the update loop because " << factLayerTime << " >= " << enforcedEnd << endl;
                                }
                                
                            } else {
                                endActionsAtTime[enforcedEnd].push_back(currAct);
                                if (updateDebug) {
                                    cout << "\t\tNot doing the update loop because " << factLayerTime << " < " << enforcedEnd << endl;
                                }
                                
                            }
                        }
                    }

                    if (doLoop) {
                        if (updateDebug) {
                            if (limitToOnce) {
                                cout << "\tStarted before RPG\n";
                            } else {
                                cout << "\tStart is sufficiently earlier\n";
                            }
                        }

                        if (RPGHeuristic::printRPGAsDot) {
                            if (epPass == 0) {
                                payload->dot.addNeededEnd(factLayerTime, depItr->first);
                            } else {
                                payload->dot.addActionNode(factLayerTime, depItr->first, depItr->second);
                            }
                        }

                        MAKEENDMDI(POtime, (limitToOnce ? payload->earliestStartOf[currAct] : 0.0), RPGBuilder::getOpMinDuration(currAct, 0), currAct);

                        if (applyPropositionalEffects(payload, currAct, VAL::E_AT_END, limitToOnce, nlTime, POtime)) return true;

                        if (limitToOnce) {
                            const map<int, set<int> >::const_iterator saItr = startedActions.find(currAct);
                            if (saItr != startedActions.end()) {
                                if (applyNumericEffects(payload, currAct, VAL::E_AT_END, nlTime, saItr->second.size())) return true;
                            }
                        } else {
                            if (applyNumericEffects(payload, currAct, VAL::E_AT_END, nlTime, -1)) return true;
                        }


                    }

                }

                //assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));


            } else {
                if (updateDebug) cout << "\tEnd of action " << currAct << " now only has " << endPreconditionCounts[currAct] << " unsatisfied preconditions\n";
            }
        }
    }

    return false;

};

bool RPGHeuristic::Private::updateActionsForNewNumericFact(Private::BuildingPayload * const payload, const int & toPropagate, const double & factLayerTime)
{

    vector<int> & startPreconditionCounts = payload->startPreconditionCounts;
    vector<int> & endPreconditionCounts = payload->endPreconditionCounts;
    vector<int> & numericStartPreconditionCounts = payload->numericStartPreconditionCounts;
    vector<int> & numericEndPreconditionCounts = payload->numericEndPreconditionCounts;
    map<double, list<int>, EpsilonComp > & endActionsAtTime = payload->endActionsAtTime;
    vector<double> & startActionSchedule = payload->startActionSchedule;
    vector<double> & endActionSchedule = payload->endActionSchedule;
    vector<double> & openEndActionSchedule = payload->openEndActionSchedule;
    const map<int, set<int> > & startedActions = payload->startedActions;

    int & unsatisfiedGoals = payload->unsatisfiedGoals;
    int & unappearedEnds = payload->unappearedEnds;
    map<int, set<int> > & insistUponEnds = payload->insistUponEnds;
    map<int, int> & forbiddenStart = payload->forbiddenStart;
    map<int, int> & forbiddenEnd = payload->forbiddenEnd;


    const double nlTime = factLayerTime + EPSILON;
    const bool updateDebug = Globals::globalVerbosity & 64;

    static bool initDummy = false;
    list<pair<int, VAL::time_spec> > dummyList;
    if (!initDummy) {
        dummyList.push_back(pair<int, VAL::time_spec>(-1, VAL::E_AT_END));
        initDummy = true;
    }

    list<pair<int, VAL::time_spec> > & dependents = ((toPropagate >= 0) ? (*processedNumericPreconditionsToActions)[toPropagate] : dummyList);

    list<pair<int, VAL::time_spec> >::iterator depItr = dependents.begin();
    const list<pair<int, VAL::time_spec> >::iterator depEnd = dependents.end();

    if (updateDebug && (toPropagate >= 0)) cout << "\tAffects " << dependents.size() << " actions\n";

    for (; depItr != depEnd; ++depItr) {
        const int currAct = ((toPropagate >= 0) ? depItr->first : (-1 - toPropagate));
        const bool startAct = ((toPropagate >= 0) ? (depItr->second == VAL::E_AT_START) : false);
        assert(depItr->second != VAL::E_OVER_ALL);
        if (updateDebug) cout << "\tAffects " << currAct << ", " << (startAct ? "start" : "end") << " " << *(RPGBuilder::getInstantiatedOp(currAct)) << "\n";
        if (startAct) {

            if (!(--numericStartPreconditionCounts[currAct]) && !startPreconditionCounts[currAct] && forbiddenStart.find(currAct) == forbiddenStart.end() && nlTime < latestStartAllowed[currAct]) {
                assert(!RPGBuilder::rogueActions[currAct]);
                if (updateDebug) cout << "\tStart of action " << currAct << " is now applicable\n";

                //assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

                if (RPGHeuristic::printRPGAsDot) {
                    payload->dot.addActionNode(factLayerTime, depItr->first, depItr->second);
                }
                
                
                if (expandFully) earliestStartAllowed[currAct] = factLayerTime;

                startActionSchedule[currAct] = factLayerTime;
                
                MAKESTARTMDI(POtime, currAct);

                if (applyPropositionalEffects(payload, currAct, VAL::E_AT_START, false, nlTime, POtime)) return true;

                if (applyNumericEffects(payload, currAct, VAL::E_AT_START, nlTime, -1)) return true;


                {
                    const double endTime = factLayerTime + RPGBuilder::getOpMinDuration(currAct, 0);

                    endActionSchedule[currAct] = endTime;

                    if (!endPreconditionCounts[currAct] && !numericEndPreconditionCounts[currAct]) {
                        if (updateDebug) cout << "\t\tEnd of action can be put at fact layer sufficiently far in future (" << endTime << ")\n";

                        endActionsAtTime[endTime].push_back(currAct);
                    }
                }

                //assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

            } else {
                if (updateDebug) cout << "\tStart of action " << currAct << " now only has " << startPreconditionCounts[currAct] << " unsatisfied propositional preconditions\n";
            }
        } else {
            if (toPropagate >= 0) --numericEndPreconditionCounts[currAct];
            if (!numericEndPreconditionCounts[currAct] && !endPreconditionCounts[currAct] && forbiddenEnd.find(currAct) == forbiddenEnd.end() && nlTime < latestEndAllowed[currAct]) {
                assert(!RPGBuilder::rogueActions[currAct]);
                if (updateDebug) cout << "\tEnd of action " << currAct << " is now applicable\n";

                //assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

                
                if (expandFully) earliestEndAllowed[currAct] = factLayerTime;

                bool insistOnThisEnd = (insistUponEnds.find(currAct) != insistUponEnds.end());
                if (insistOnThisEnd) {
                    if (factLayerTime - openEndActionSchedule[currAct] > -0.0001) {
                        openEndActionSchedule[currAct] = factLayerTime;
                        if (!(--unappearedEnds) && !unsatisfiedGoals) return true;
                        if (updateDebug) cout << "\tNow down to " << unappearedEnds << "\n";
                    } else {
                        if (updateDebug) cout << "\tOpen end cannot appear quite yet\n";
                        insistOnThisEnd = false;
                        endActionsAtTime[openEndActionSchedule[currAct]].push_back(-1 - currAct);
                        if (!(--unappearedEnds) && !unsatisfiedGoals) return true;
                        if (updateDebug) cout << "\tNow down to " << unappearedEnds << "\n";
                    }
                }

                for (int epPass = 0; epPass < 2; ++epPass) {
                    bool doLoop = false;
                    bool limitToOnce = false;
                    if (!epPass) {
                        doLoop = insistOnThisEnd;
                        limitToOnce = true;
                    } else {
                        if (!startPreconditionCounts[currAct] && !numericStartPreconditionCounts[currAct] && startActionSchedule[currAct] > -1.0) {
                            const double enforcedEnd = endActionSchedule[currAct];
                            if (factLayerTime - enforcedEnd > -0.0001) {
                                endActionSchedule[currAct] = factLayerTime;
                                doLoop = true;
                                if (updateDebug) {
                                    cout << "\t\tDoing the update loop because " << factLayerTime << " >= " << enforcedEnd << endl;
                                }
                            } else {
                                endActionsAtTime[enforcedEnd].push_back(currAct);
                                if (updateDebug) {
                                    cout << "\t\tNot doing the update loop because " << factLayerTime << " < " << enforcedEnd << endl;
                                }
                            }
                        }
                    }

                    if (doLoop) {

                        if (updateDebug) {
                            if (limitToOnce) {
                                cout << "\tStarted before RPG\n";
                            } else {
                                cout << "\tStart is sufficiently earlier\n";
                            }
                        }
                        
                        if (RPGHeuristic::printRPGAsDot) {
                            if (epPass == 0) {
                                payload->dot.addNeededEnd(factLayerTime, depItr->first);
                            } else {
                                payload->dot.addActionNode(factLayerTime, depItr->first, depItr->second);
                            }
                        }
                                                                
                                                                                
                        
                        MAKEENDMDI(POtime, (limitToOnce ? payload->earliestStartOf[currAct] : 0.0), RPGBuilder::getOpMinDuration(currAct, 0), currAct);
                        
                        if (applyPropositionalEffects(payload, currAct, VAL::E_AT_END, limitToOnce, nlTime, POtime)) return true;
                        
                        
                        if (limitToOnce) {
                            const map<int, set<int> >::const_iterator saItr = startedActions.find(currAct);
                            if (saItr != startedActions.end()) {
                                if (applyNumericEffects(payload, currAct, VAL::E_AT_END, nlTime, saItr->second.size())) return true;
                            }
                        } else {
                            if (applyNumericEffects(payload, currAct, VAL::E_AT_END, nlTime, -1)) return true;
                        }
                        


                    }

                }

                //assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

            } else {

                if (updateDebug) {
                    cout << "\tEnd of action " << currAct;
                    cout << " now only has " << endPreconditionCounts[currAct] << " unsatisfied preconditions\n";
                }
            }
        }
    }

    return false;

};

bool RPGHeuristic::Private::applyEndEffectNow(Private::BuildingPayload * const payload, const int & currAct, const bool & openEnd, const double & factLayerTime)
{


    const map<int, set<int> > & startedActions = payload->startedActions;


    const double nlTime = factLayerTime + EPSILON;
    const bool updateDebug = Globals::globalVerbosity & 64;


    if (updateDebug) cout << "\tEnd of action " << currAct << " is now applicable\n";

    //assert(checkPreconditionsAreSatisfied(currAct, VAL::E_AT_END, nlTime));

    if (RPGHeuristic::printRPGAsDot) {
        if (openEnd) {
            payload->dot.addNeededEnd(factLayerTime, currAct);
        } else {
            payload->dot.addActionNode(factLayerTime, currAct, VAL::E_AT_END);
        }
    }
    
    if (expandFully) earliestEndAllowed[currAct] = factLayerTime;

    MAKEENDMDI(POtime, (openEnd ? payload->earliestStartOf[currAct] : 0.0), RPGBuilder::getOpMinDuration(currAct, 0), currAct);

    if (applyPropositionalEffects(payload, currAct, VAL::E_AT_END, openEnd, nlTime, POtime)) return true;


    if (openEnd) {
        const map<int, set<int> >::const_iterator saItr = startedActions.find(currAct);
        if (saItr != startedActions.end()) {
            if (applyNumericEffects(payload, currAct, VAL::E_AT_END, nlTime, saItr->second.size())) return true;
        }
    } else {
        if (applyNumericEffects(payload, currAct, VAL::E_AT_END, nlTime, -1)) return true;
    }

    
    //assert(checkPreconditionsAreSatisfied(currAct, VAL::E_AT_END, nlTime));

    return false;

};

#ifdef POPF3ANALYSIS

void printVec(const vector<double> & costVec) {
    const int cSize = costVec.size();
    
    cout << "[";
    for (int i = 0; i < cSize; ++i) {
        if (i) cout << ", ";
        if (costVec[i] == DBL_MAX) {
            cout << "inf";
        } else if (costVec[i] == -DBL_MAX) {
            cout << "-inf";
        } else {
            cout << costVec[i];
        }
    }
    cout << "]";
}

void RPGHeuristic::Private::updateAdditiveCosts(BuildingPayload * const payload, const int & ci, const double & deadline, const int & updateForGoal, double & costToUpdate)
{

    if (updateForGoal == -1) {
        return;
    }

    const map<int, list<int> > & igc = NumericAnalysis::getIndependentGoalCosts()[updateForGoal];
    
    if (igc.empty()) {
        return;
    }

    const map<int, list<int> >::const_iterator igcItr = igc.find(ci);
    
    if (igcItr == igc.end()) {
        return;
    }

    static const int localDebug = 0;
    
    static const string UAC("\tupdateAdditiveCosts(): ");

    // if we get this far, we have independent goal costs - iterate over achievers,
    // finding which exist, and add the smallest of those to the additive cost
    
    double costUpdate = 0.0;
    bool achieverYet = false;
    
    list<int>::const_iterator achievedByItr = igcItr->second.begin();
    const list<int>::const_iterator achievedByEnd = igcItr->second.end();

    
    for (double localCost; achievedByItr != achievedByEnd; ++achievedByItr) {
        if (payload->startActionSchedule[*achievedByItr] == -1.0) {
            
            // don't use an achiever if its start hasn't appeared yet
            continue;
        }
        
        if (!RPGBuilder::getRPGDEs(*achievedByItr).empty()
            && payload->endActionSchedule[*achievedByItr] == -1.0
            && payload->openEndActionSchedule[*achievedByItr] == -1.0) {
            
            // for durative actions, don't use an achiever if its end hasn't appeared yet
            continue;
        }
        
        if (payload->startActionSchedule[*achievedByItr] > deadline) {
            // don't use achievers that start too late
            continue;
        }
        
        
        
        {
            const list<Literal*> & startAdds = RPGBuilder::getStartPropositionAdds()[*achievedByItr];
            list<Literal*>::const_iterator addItr = startAdds.begin();
            const  list<Literal*>::const_iterator addEnd = startAdds.end();
            for (; addItr != addEnd; ++addItr) {
                if (*addItr == literalGoalVector[updateForGoal]) {
                    break;
                }
            }
            
            if (addItr == addEnd) {
                // here, it means it wasn't added by the start of the action
                
                if (payload->endActionSchedule[*achievedByItr] <= deadline) {
                    
                } else {
                    
                    if (payload->startState.startedActions.find(*achievedByItr) == payload->startState.startedActions.end() 
                        || payload->openEndActionSchedule[*achievedByItr] > deadline) {
                        
                        // don't use achievers that end too late
                        
                        continue;
                    }
                }
                                        
            }
        }
        
        
        localCost = startEffectsOnResourceLimits[*achievedByItr][ci] + endEffectsOnResourceLimits[*achievedByItr][ci];
        
        if (localCost == 0.0) {
            // have a cost-free achiever - assume we'd use it
            return;
        }
        
        if (achieverYet) {
            if (localCost > 0.0) {
                if (localCost < costUpdate) {
                    costUpdate = localCost;
                }
            } else {
                if (localCost > costUpdate) {
                    costUpdate = localCost;
                }                                
            } 
        
        } else {
            costUpdate = localCost;
            achieverYet = true;
        } 
    
    }
    
    if (localDebug) {
        cout << COLOUR_yellow << UAC << "- Additive cost of goal " << *(literalGoalVector[updateForGoal]) << " on limit " << ci << " = " << costUpdate << COLOUR_default << endl;
    }
    
    costToUpdate += costUpdate;
}

void RPGHeuristic::Private::calculateGoalCost(BuildingPayload * const payload)
{
    
    static const int localDebug = 0;
    
    static const string CGC("calculateGoalCost(): ");
    
    if (!payload->tooExpensive) {
        if (localDebug & 1) {
            cout << COLOUR_light_green << CGC << "Cost threshold known to be okay\n" << COLOUR_default;
        }
        return;
    }
    
    if (!NumericAnalysis::areThereAnyIndependentGoalCosts()) {
        if (localDebug & 1) {
            cout << COLOUR_light_green << CGC << "No independent goal costs, so costs known to be okay\n" << COLOUR_default;
        }
        
        payload->tooExpensive = false;
        return;
    }
    
    if (localDebug) {
        cout << COLOUR_light_magenta << CGC << COLOUR_default << endl;
    }
    
    
    const int costCount = currentCosts.size();
        
    assert(costCount); // or there shouldn't be independent goal costs
    
    vector<double> maxCosts(costCount, 0.0);
    vector<double> additiveCosts(costCount, 0.0);
    vector<int> mostExpensiveGID(costCount, -1);
    
    const int lgc = literalGoalVector.size();
    
    int fID;

    const list<double> & literalGoalDeadlines = RPGBuilder::getLiteralGoalDeadlines();
    list<double>::const_iterator deadlineItr = literalGoalDeadlines.begin();
    
    
    for (int gID = 0; gID < lgc; ++gID, ++deadlineItr) {
        
        if (!literalGoalVector[gID]) {
            // static goal
            continue;
        }
        
        fID = literalGoalVector[gID]->getStateID();

        if (payload->startState.first.find(fID) != payload->startState.first.end()) {
            if (localDebug & 2) {
                cout << COLOUR_light_blue << CGC << "goal fact" << *(literalGoalVector[gID]) << " is already true" << endl << COLOUR_default;
            }
            // true in the state being evaluated
            continue;
        }
        

                
                
        
        const list<CostedAchieverDetails> & achievers = achieverDetails[fID];        
        assert(!achievers.empty());
            
        list<CostedAchieverDetails>::const_reverse_iterator accItr = achievers.rbegin();
        const list<CostedAchieverDetails>::const_reverse_iterator accEnd = achievers.rend();
        
        for (; accItr != accEnd && accItr->layer > *deadlineItr; ++accItr) ;
        
        if (accItr == accEnd) {
            // in this case, we don't have an early enough achiever, give up for now
            return;
        }
                    
        
        const CostedAchieverDetails & achiever = *accItr;

        
        
        for (int ci = 0; ci < costCount; ++ci) {
            
            if (achiever.costs[ci] > 0.0) {
                if (mostExpensiveGID[ci] == -1 || maxCosts[ci] < achiever.costs[ci]) {
                    if (localDebug & 2) {
                        cout << COLOUR_light_blue << CGC << "goal fact" << *(literalGoalVector[gID]) << ": is ";
                        if (mostExpensiveGID[ci] == -1) {
                            cout << " the first max on ";
                        } else {
                            cout << " the new max on ";
                        }                        
                        cout << ci << ", " << achiever.costs[ci] << endl << COLOUR_default;
                    }
                    updateAdditiveCosts(payload, ci, *deadlineItr, mostExpensiveGID[ci], additiveCosts[ci]);
                    maxCosts[ci] = achiever.costs[ci];
                    mostExpensiveGID[ci] = gID;
                    
                    
                } else {
                    updateAdditiveCosts(payload, ci, *deadlineItr, gID, additiveCosts[ci]);
                }
            } else if (achiever.costs[ci] < 0.0) {
                if (mostExpensiveGID[ci] == -1 || maxCosts[ci] > achiever.costs[ci]) {
                    if (localDebug & 2) {
                        cout << COLOUR_light_blue << CGC << "goal fact" << *(literalGoalVector[gID]) << ": is ";
                        if (mostExpensiveGID[ci] == -1) {
                            cout << " the first max on ";
                        } else {
                            cout << " the new max on ";
                        }                        
                        cout << ci << ", " << achiever.costs[ci] << endl << COLOUR_default;
                    }
                                        
                                        
                    updateAdditiveCosts(payload, ci, *deadlineItr, mostExpensiveGID[ci], additiveCosts[ci]);
                    maxCosts[ci] = achiever.costs[ci];
                    mostExpensiveGID[ci] = gID;
                    
                } else {
                    updateAdditiveCosts(payload, ci, *deadlineItr, gID, additiveCosts[ci]);
                }
            }
        }
        
    }

    const vector<NumericAnalysis::NumericLimitDescriptor> & limits = NumericAnalysis::getGoalNumericUsageLimits();

    double localCost, limitToUse;
    
    for (int ci = 0; ci < costCount; ++ci) {

        localCost = additiveCosts[ci] + maxCosts[ci];

        if (localCost == 0.0) {
            if (localDebug & 1) {
                cout << COLOUR_light_blue << CGC << "final effect on limit " << ci << " is zero: ignoring it\n" << COLOUR_default;
            }
            
            // can use none of this resource
            continue;
        }
        
        limitToUse = limits[ci].limit;
        
        if (Globals::bestSolutionQuality > -DBL_MAX && limits[ci].optimisationLimit) {
            if (Globals::bestSolutionQuality + 0.001 > limitToUse) {
                limitToUse = Globals::bestSolutionQuality + 0.001;
            }
        }
                                
        if (limitToUse > -DBL_MAX) {
                                                                                                    
            if (localCost > 0.0) {
                switch (limits[ci].op) {
                    case E_GREATER:
                    {
                        if (localCost <= limitToUse) {
                            if (localDebug & 1) {
                                cout << COLOUR_light_red << CGC << "final effect on limit " << ci << " is " << localCost << ", limit is " << limitToUse << ", so too expensive\n" << COLOUR_default;
                            }
                            
                            return;
                        }
                        break;
                    }
                    case E_GREATEQ:
                    {
                        if (localCost < limitToUse) {
                            if (localDebug & 1) {
                                cout << COLOUR_light_red << CGC << "final effect on limit " << ci << " is " << localCost << ", limit is " << limitToUse << ", so too expensive\n" << COLOUR_default;
                            }
                            
                            return;
                        }
                        break;                    
                    }                                
                    default:
                    {
                        cerr << "Internal error: limits should be defined in terms of > or >=\n";
                        exit(1);
                    }
                }
                if (localDebug & 1) {
                    cout << COLOUR_light_green << CGC << "final effect on limit " << ci << " is " << localCost << ", limit is " << limitToUse << ", so okay\n" << COLOUR_default;
                }
                                                            
            } else if (localCost < 0.0) {
                switch (limits[ci].op) {
                    case E_GREATER:
                    {
                        if (localCost <= limitToUse) {
                            if (localDebug & 1) {
                                cout << COLOUR_light_red << CGC << "final effect on limit " << ci << " is " << localCost << ", limit is " << limitToUse << ", so too expensive\n" << COLOUR_default;
                            }
                            
                            return;
                        }
                        break;
                    }
                    case E_GREATEQ:
                    {
                        if (localCost < limitToUse) {
                            if (localDebug & 1) {
                                cout << COLOUR_light_red << CGC << "final effect on limit " << ci << " is " << localCost << ", limit is " << limitToUse << ", so too expensive\n" << COLOUR_default;
                            }
                            
                            return;
                        }
                        break;                    
                    }                                
                    default:
                    {
                        cerr << "Internal error: limits should be defined in terms of > or >=\n";
                        exit(1);
                    }
                }
                if (localDebug & 1) {
                    cout << COLOUR_light_green << CGC << "final effect on limit " << ci << " is " << localCost << ", limit is " << limitToUse << ", so okay\n" << COLOUR_default;
                }
            } 
        }
    
        
    }

    
    // if the for loop didn't 'return' us out of here, the cost limits are okay
    
    if (localDebug & 1) {
        cout << COLOUR_light_red << CGC << "success: RP is cheap enough\n" << COLOUR_default;
    }
                                    
    payload->tooExpensive = false;
    
}
#endif


void RPGHeuristic::findApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions)
{
    d->findApplicableActions(theState, stateTime, applicableActions);
}

bool negativesAreOkay(const list<Literal*> & checkFor, const StateFacts & checkIn)
{

    list<Literal*>::const_iterator cfItr = checkFor.begin();
    const list<Literal*>::const_iterator cfEnd = checkFor.end();

    for (; cfItr != cfEnd; ++cfItr) {
        if (checkIn.find((*cfItr)->getStateID()) != checkIn.end()) return false;
    }

    return true;
}

void RPGHeuristic::Private::findApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions)
{

    static const bool debug = (Globals::globalVerbosity & 16);


    static const set<int> emptyIntSet;


    vector<int> startPreconditionCounts(*initialUnsatisfiedProcessedStartPreconditions);
    vector<int> endPreconditionCounts(*initialUnsatisfiedEndPreconditions);

    list<ActionSegment> toFilter;

    for (int pass = 0; pass < 2; ++pass) {
        list<pair<int, VAL::time_spec> > & dependents = (pass ? *onlyNumericPreconditionActions : *preconditionlessActions);

        if (debug) {
            if (pass) {
                cout << "Considering the " << dependents.size() << " actions with only numeric preconditions\n";
            } else {
                cout << "Considering the " << dependents.size() << " propositionally preconditionless actions\n";
            }
        }


        list<pair<int, VAL::time_spec> >::iterator depItr = dependents.begin();
        const list<pair<int, VAL::time_spec> >::iterator depEnd = dependents.end();

        for (; depItr != depEnd; ++depItr) {
            const int currAct = depItr->first;
            const VAL::time_spec startOrEnd = depItr->second;

            if (!negativesAreOkay((startOrEnd == VAL::E_AT_START
                                   ? RPGBuilder::getProcessedStartNegativePropositionalPreconditions()[currAct]
                                   : RPGBuilder::getEndNegativePropositionalPreconditions()[currAct]) ,
                                  theState.first)) {
                continue;
            }

            ActionSegment candidate(getOp(currAct), startOrEnd, -1, emptyIntSet);
            const bool nonMutex = RPGBuilder::stepNeedsToHaveFinished(candidate, theState, candidate.needToFinish);

            if (nonMutex) {
                toFilter.push_back(candidate);
                if (debug) {
                    cout << *(getOp(currAct));
                    if (startOrEnd == VAL::E_AT_START) {
                        cout << " start is a propositional candidate\n";
                    } else {
                        cout << " end is a propositional candidate\n";
                    }
                }
            } else {
                if (debug) {
                    cout << *(getOp(currAct));
                    if (startOrEnd == VAL::E_AT_START) {
                        cout << " start breaks a propositional invariant\n";
                    } else {
                        cout << " end breaks a propositional invariant\n";
                    }
                }
            }

        }
    }

    {
        StateFacts::const_iterator stateItr = theState.first.begin();
        const StateFacts::const_iterator stateEnd = theState.first.end();

        for (; stateItr != stateEnd; ++stateItr) {

            if (debug) {
                cout << "Considering what benefits from " << *(RPGBuilder::getLiteral(FACTA(stateItr))) << " being true\n";                
            }
            
            list<pair<int, VAL::time_spec> > & dependents = (*processedPreconditionsToActions)[FACTA(stateItr)];

            list<pair<int, VAL::time_spec> >::const_iterator depItr = dependents.begin();
            const list<pair<int, VAL::time_spec> >::const_iterator depEnd = dependents.end();

            for (; depItr != depEnd; ++depItr) {
                const int currAct = depItr->first;
                const VAL::time_spec startOrEnd = depItr->second;
                int & toManipulate = (startOrEnd == VAL::E_AT_START ? startPreconditionCounts[currAct] : endPreconditionCounts[currAct]);

                if (!(--toManipulate)) {

                    if (negativesAreOkay((startOrEnd == VAL::E_AT_START
                                          ? RPGBuilder::getProcessedStartNegativePropositionalPreconditions()[currAct]
                                          : RPGBuilder::getEndNegativePropositionalPreconditions()[currAct]) ,
                                         theState.first)) {
                        ActionSegment candidate(getOp(currAct), startOrEnd, -1, emptyIntSet);
                        const bool nonMutex = RPGBuilder::stepNeedsToHaveFinished(candidate, theState, candidate.needToFinish);

                        if (nonMutex) {
                            if (debug) {
                                cout << "Based on propositions, " << candidate << " is a candidate\n";
                            }
                            toFilter.push_back(candidate);
                        }
                    }
                }
            }
        }

    }


    list<ActionSegment>::iterator fItr = toFilter.begin();
    const list<ActionSegment>::iterator fEnd = toFilter.end();

    for (; fItr != fEnd; ++fItr) {
        /*const pair<int, VAL::time_spec> currAct = *fItr;
        list<RPGBuilder::NumericPrecondition> & currList = (*actionsToNumericPreconditions)[currAct->first];
        bool isApplicable = true;
        list<RPGBuilder::NumericPrecondition>::iterator npItr = currList.begin();
        const list<RPGBuilder::NumericPrecondition>::iterator npEnd = currList.end();

        for (; npItr != npEnd; ++npItr) {
        if (!npItr->isSatisfied(theState.second)) {
        isApplicable = false;
        break;
        }
        }
        if (isApplicable) {
        applicableActions.push_back(currAct);
        }*/

        bool isApplicable = true;

        if (fItr->second == VAL::E_AT_START) {
            isApplicable = TemporalAnalysis::okayToStart(fItr->first->getID(), stateTime);
        } else if (fItr->second == VAL::E_AT_END) {
            isApplicable = TemporalAnalysis::okayToEnd(fItr->first->getID(), stateTime);
        }

        if (debug && !isApplicable) {
            cout << "Not okay to apply " << *fItr << ", according to temporal analysis\n";
        }

        vector<list<int> > * const toQuery = (fItr->second == VAL::E_AT_START ? actionsToProcessedStartNumericPreconditions : actionsToNumericEndPreconditions);
        if (isApplicable) {

            list<int> & nprecs = (*toQuery)[fItr->first->getID()];

            list<int>::iterator npItr = nprecs.begin();
            const list<int>::iterator npEnd = nprecs.end();

            for (; npItr != npEnd; ++npItr) {

                if (!((*rpgNumericPreconditions)[*npItr]).isSatisfiedWCalculate(theState.secondMin, theState.secondMax)) {
                    if (debug) cout << (*rpgNumericPreconditions)[*npItr] << " isn't satisfied, so " << *(fItr->first) << " isn't applicable\n";
                    isApplicable = false;
                    break;
                }

            }

        }

        /*        if (isApplicable && false) { //HACK
                    vector<list<int> > * const checkEffects = (fItr->second == VAL::E_AT_START ? actionsToRPGNumericStartEffects : actionsToRPGNumericEndEffects);

                    {

                        list<int> & neffs = (*checkEffects)[fItr->first->getID()];

                        list<int>::iterator neItr = neffs.begin();
                        const list<int>::iterator neEnd = neffs.end();

                        for (; neItr != neEnd; ++neItr) {

                            if (theState.fluentInvariants.find(((*rpgNumericEffects)[*neItr]).fluentIndex) != theState.fluentInvariants.end()) {
                                isApplicable = false;
                                break;
                            }

                        }


                    }

                }*/

        if (isApplicable) {


            map<int, set<int> >::const_iterator saOneItr = theState.startedActions.find(fItr->first->getID());
            if (fItr->second == VAL::E_AT_START) {
                if (!RPGBuilder::noSelfOverlaps || saOneItr == theState.startedActions.end()) {
                    if (RPGBuilder::isInteresting(fItr->first->getID(), theState.first, theState.startedActions)) {
                        applicableActions.push_back(*fItr);
                    } else {
                        if (debug) {
                            cout << "Not okay to apply " << *fItr << ", as it would not be interesting to do so\n";
                        }                                                            
                    }
                } else {
                    if (debug) {
                        cout << "Not okay to apply " << *fItr << ", as it is currently executing and the planner has been asked to forbid self-overlapping actions\n";
                    }                                    
                }
            } else {
                if (saOneItr != theState.startedActions.end()) {
                    applicableActions.push_back(*fItr);                    
                } else {
                    if (debug) {
                        cout << "Not okay to apply " << *fItr << ", as it hasn't been started\n";
                    }                    
                }
            }
        }
    }

    /*{
        map<int, map<int,int> >::iterator saItr = theState.startedActions.begin();
        const map<int, map<int,int> >::iterator saEnd = theState.startedActions.end();
        for (; saItr != saEnd; ++saItr) {
            RPGBuilder::LinearEffects * const lEffs = RPGBuilder::getLinearDiscretisation()[saItr->first];
            if (lEffs) {
                const int lLim = lEffs->divisions - 1;
                map<int,int>::iterator saOneItr = saItr->second.begin();
                const map<int,int>::iterator saOneEnd = saItr->second.end();

                for (; saOneItr != saOneEnd && saOneItr->first < lLim; ++saOneItr) {
                    applicableActions.push_back(ActionSegment(getOp(saItr->first), VAL::E_OVER_ALL, saOneItr->first, emptyIntList));
                }
            }
        }
    }*/

    {

        const int nextTIL = theState.nextTIL;
        static list<RPGBuilder::FakeTILAction> & tilActs = RPGBuilder::getTILs();
        static const list<RPGBuilder::FakeTILAction>::iterator tilEnd = tilActs.end();

        list<RPGBuilder::FakeTILAction>::iterator tilItr = tilActs.begin();

        int i = 0;

        for (; i < nextTIL; ++i, ++tilItr);

        for (; i == nextTIL && tilItr != tilEnd; ++tilItr, ++i) {

            ActionSegment candidate(0, VAL::E_AT, i, emptyIntSet);

            const bool isApplicable = RPGBuilder::stepNeedsToHaveFinished(candidate, theState, candidate.needToFinish);

            if (isApplicable) {
                //cout << "TIL " << i << " is applicable\n";
                applicableActions.push_back(candidate);
            } else {
                break;
            }

        }

    }

};


bool RPGHeuristic::testApplicability(const MinimalState & theState, const double & stateTime, const ActionSegment & actID, bool fail, bool ignoreDeletes)
{
    return d->testApplicability(theState, stateTime, actID, fail, ignoreDeletes);
}

bool RPGHeuristic::Private::testApplicability(const MinimalState & theState, const double & stateTime, const ActionSegment & actID, const bool & fail, const bool & ignoreDeletes)
{

    if (actID.second == VAL::E_AT_START) {

        if (RPGBuilder::noSelfOverlaps) {

            if (fail) {
                assert(theState.startedActions.find(actID.first->getID()) == theState.startedActions.end());
            } else {
                if (theState.startedActions.find(actID.first->getID()) != theState.startedActions.end()) return false;
            }
        }


        if (fail) {
            assert(TemporalAnalysis::okayToStart(actID.first->getID(), stateTime));
        } else {
            if (!TemporalAnalysis::okayToStart(actID.first->getID(), stateTime)) return false;
        }

        {
            set<int> ntf;
            const bool passesPropositional = RPGBuilder::stepNeedsToHaveFinished(actID, theState, ntf);
            if (!passesPropositional) {
                if (fail) {
                    assert(false);
                } else {
                    return false;
                }
            }
        }

        if (!negativesAreOkay(RPGBuilder::getProcessedStartNegativePropositionalPreconditions()[actID.first->getID()],
                              theState.first)) {
            if (fail) {
                assert(false);
            } else {
                return false;
            }
        }


        if (!ignoreDeletes) {

            list<int> & checkFor = (*actionsToProcessedStartNumericPreconditions)[actID.first->getID()];
            list<int>::iterator fItr = checkFor.begin();
            const list<int>::iterator fEnd = checkFor.end();

            for (; fItr != fEnd; ++fItr) {

                if (!((*rpgNumericPreconditions)[*fItr]).isSatisfiedWCalculate(theState.secondMin, theState.secondMax)) {
                    if (fail) {
                        cerr << "Fatal internal error: precondition " << (*rpgNumericPreconditions)[*fItr] << " not satisfied\n";
                        assert(false);
                    } else {
                        return false;
                    }
                }

            }
        }



    } else if (actID.second == VAL::E_AT_END) {


        if (fail) {
            assert(TemporalAnalysis::okayToEnd(actID.first->getID(), stateTime));
        } else {
            if (!TemporalAnalysis::okayToEnd(actID.first->getID(), stateTime)) return false;
        }


        {
            set<int> ntf;
            const bool passesPropositional = RPGBuilder::stepNeedsToHaveFinished(actID, theState, ntf);
            if (!passesPropositional) {
                if (fail) {
                    assert(false);
                } else {
                    return false;
                }
            }
        }

        if (!negativesAreOkay(RPGBuilder::getEndNegativePropositionalPreconditions()[actID.first->getID()],
                              theState.first)) {
            if (fail) {
                assert(false);
            } else {
                return false;
            }
        }


        if (!ignoreDeletes) {

            list<int> & checkFor = (*actionsToNumericEndPreconditions)[actID.first->getID()];
            list<int>::iterator fItr = checkFor.begin();
            const list<int>::iterator fEnd = checkFor.end();



            for (; fItr != fEnd; ++fItr) {

                if (!((*rpgNumericPreconditions)[*fItr]).isSatisfiedWCalculate(theState.secondMin, theState.secondMax)) {
                    if (fail) {
                        assert(false);
                    } else {
                        return false;
                    }
                }

            }
        }


    } else if (actID.second != VAL::E_AT) {
        assert(false);

    } else { // til action

        const int & nextTIL = theState.nextTIL;

        if (fail) {
            if (actID.divisionID < nextTIL) {
                cout << "Trying to apply " << actID.divisionID << ", but next one is " << nextTIL << "\n";
            }
            assert(actID.divisionID >= nextTIL);
        } else {
            if (actID.divisionID < nextTIL) return false;
        }

        static list<RPGBuilder::FakeTILAction> & tilActs = RPGBuilder::getTILs();
        static const list<RPGBuilder::FakeTILAction>::iterator tilEnd = tilActs.end();

        list<RPGBuilder::FakeTILAction>::iterator tilItr = tilActs.begin();

        int i = 0;

        static const set<int> emptyIntSet;

        for (; i < nextTIL; ++i, ++tilItr);

        for (; i <= actID.divisionID; ++tilItr, ++i) {
            if (tilItr == tilEnd) {
                cout << "Error: " << i << " TILs in the domain, but trying to access the one with index " << i << "\n";
                assert(tilItr != tilEnd);
            }

            ActionSegment candidate(actID.first, VAL::E_AT, i, emptyIntSet);

            const bool passesPropositional = RPGBuilder::stepNeedsToHaveFinished(candidate, theState, candidate.needToFinish);
            if (!passesPropositional) {
                if (fail) {
                    assert(false);
                } else {
                    return false;
                }
            }

        }


    }

    return true;


}
void RPGHeuristic::filterApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions)
{
    return d->filterApplicableActions(theState, stateTime, applicableActions);
}

void RPGHeuristic::Private::filterApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions)
{


    const bool filterDebug = false;

    if (filterDebug) cout << "Filtering applicable actions\n";

    list<ActionSegment> toFilter(applicableActions);

    list<ActionSegment> toNumericFilter;

    applicableActions.clear();

    if (filterDebug) cout << "Input consists of " << toFilter.size() << " actions\n";

    list<ActionSegment>::iterator tfItr = toFilter.begin();
    const list<ActionSegment>::iterator tfEnd = toFilter.end();

    /*for (; fItr != fEnd; ++fItr) {
    const int currAct = *fItr;
    list<RPGBuilder::NumericPrecondition> & currList = (*actionsToNumericPreconditions)[currAct];
    bool isApplicable = true;
    list<RPGBuilder::NumericPrecondition>::iterator npItr = currList.begin();
    const list<RPGBuilder::NumericPrecondition>::iterator npEnd = currList.end();

    for (; npItr != npEnd; ++npItr) {
    if (!npItr->isSatisfied(theState.second)) {
    isApplicable = false;
    break;
    }
    }
    if (isApplicable) {
    applicableActions.push_back(currAct);
    }
    }*/

    for (; tfItr != tfEnd; ++tfItr) {


        bool isApplicable = true;

        if (tfItr->second == VAL::E_AT_START) {

            if (!RPGBuilder::noSelfOverlaps || theState.startedActions.find(tfItr->first->getID()) == theState.startedActions.end()) {

                if (filterDebug) cout << "Considering start of " << *(tfItr->first) << "\n";

                isApplicable = RPGBuilder::isInteresting(tfItr->first->getID(), theState.first, theState.startedActions);
                
                if (isApplicable) isApplicable = TemporalAnalysis::okayToStart(tfItr->first->getID(), stateTime);

                if (isApplicable) isApplicable = RPGBuilder::stepNeedsToHaveFinished(*tfItr, theState, tfItr->needToFinish);

                if (isApplicable) isApplicable = negativesAreOkay(RPGBuilder::getProcessedStartNegativePropositionalPreconditions()[tfItr->first->getID()], theState.first);

            } else {
                isApplicable = false;
            }

        } else if (tfItr->second == VAL::E_AT_END) {
            const map<int, set<int> >::const_iterator saItr = theState.startedActions.find(tfItr->first->getID());
            if (saItr != theState.startedActions.end() && TemporalAnalysis::okayToEnd(tfItr->first->getID(), stateTime)) {
                isApplicable = RPGBuilder::stepNeedsToHaveFinished(*tfItr, theState, tfItr->needToFinish);
            } else {
                isApplicable = false;
            }
            if (isApplicable) isApplicable = negativesAreOkay(RPGBuilder::getEndNegativePropositionalPreconditions()[tfItr->first->getID()], theState.first);

        } else if (tfItr->second != VAL::E_AT) {

            assert(false);
        } else if (tfItr->second == VAL::E_AT) { // TIL action

            static const set<int> emptyIntSet;

            static bool cachedTILs = false;
            static vector<RPGBuilder::FakeTILAction*> tilActs;
            if (!cachedTILs) {
                cachedTILs = true;
                list<RPGBuilder::FakeTILAction> & rpgTILs = RPGBuilder::getTILs();
                tilActs = vector<RPGBuilder::FakeTILAction*>(rpgTILs.size());

                list<RPGBuilder::FakeTILAction>::iterator rItr = rpgTILs.begin();
                const list<RPGBuilder::FakeTILAction>::iterator rEnd = rpgTILs.end();

                for (int i = 0; rItr != rEnd; ++rItr, ++i) {
                    tilActs[i] = &(*rItr);
                }
            }

            const int nextTIL = theState.nextTIL;

            if (tfItr->divisionID >= nextTIL) {

                bool safe = true;

                for (int i = nextTIL; safe && i <= tfItr->divisionID; ++i) {

                    ActionSegment candidate(tfItr->first, VAL::E_AT, i, emptyIntSet);

                    safe = RPGBuilder::stepNeedsToHaveFinished(candidate, theState, tfItr->needToFinish);

                }

                if (safe) {
                    applicableActions.push_back(*tfItr);
                    // skip straight to the destination list - no numeric interactions,
                    // as these are timed literals, not timed fluents
                }

            }
            isApplicable = false;
        } else {
            isApplicable = false;
        }

        if (isApplicable) {
            toNumericFilter.push_back(*tfItr);
        }
    }



    list<ActionSegment>::iterator tnfItr = toNumericFilter.begin();
    const list<ActionSegment>::iterator tnfEnd = toNumericFilter.end();

    for (; tnfItr != tnfEnd; ++tnfItr) {
        bool isApplicable = true;

        if (filterDebug) cout << "Considering numerically " << *(tnfItr->first) << "\n";

        vector<list<int> > * const toQuery = (tnfItr->second == VAL::E_AT_START ? actionsToProcessedStartNumericPreconditions : actionsToNumericEndPreconditions);
        {

            list<int> & nprecs = (*toQuery)[tnfItr->first->getID()];

            list<int>::iterator npItr = nprecs.begin();
            const list<int>::iterator npEnd = nprecs.end();

            for (; npItr != npEnd; ++npItr) {

                if (!((*rpgNumericPreconditions)[*npItr]).isSatisfiedWCalculate(theState.secondMin, theState.secondMax)) {
                    isApplicable = false;
                    break;
                } else {
                    if (filterDebug) cout << "\t" << (*rpgNumericPreconditions)[*npItr] << "satisfied\n";
                }

            }

        }

        if (isApplicable) {
            applicableActions.push_back(*tnfItr);
        }
    }


};


list<instantiatedOp*> * RPGHeuristic::makePlan(list<int> & steps)
{

    list<instantiatedOp*> * toReturn = new list<instantiatedOp*>();

    list<int>::iterator sItr = steps.begin();
    const list<int>::iterator sEnd = steps.end();
    cout << "\n";
    for (; sItr != sEnd; ++sItr) {
        toReturn->push_back(RPGBuilder::getInstantiatedOp(*sItr));
    }

    return toReturn;
}

instantiatedOp* RPGHeuristic::getOp(const int & i)
{

    return RPGBuilder::getInstantiatedOp(i);

};


list<Literal*> & RPGHeuristic::getDeleteEffects(const int & i, const VAL::time_spec & t)
{
    if (t == VAL::E_AT_START) {
        return (*(d->actionsToStartNegativeEffects))[i];
    } else {
        return (*(d->actionsToEndNegativeEffects))[i];
    }
}

list<Literal*> & RPGHeuristic::getAddEffects(const int & i, const VAL::time_spec & t)
{
    if (t == VAL::E_AT_START) {
        return (*(d->actionsToStartEffects))[i];
    } else {
        return (*(d->actionsToEndEffects))[i];
    }
};

list<Literal*> & RPGHeuristic::getPreconditions(const int & i, const VAL::time_spec & t)
{
    if (t == VAL::E_AT_START) {
        return (*(d->actionsToStartPreconditions))[i];
    } else {
        return (*(d->actionsToEndPreconditions))[i];
    }
};


list<int> & RPGHeuristic::getNumericEffects(const int & i, const VAL::time_spec & t)
{
    if (t == VAL::E_AT_START) {
        return (*(d->actionsToRPGNumericStartEffects))[i];
    } else {
        return (*(d->actionsToRPGNumericEndEffects))[i];
    }

};

list<Literal*> & RPGHeuristic::getInvariants(const int & i)
{
    return (*(d->actionsToInvariants))[i];
}

RPGBuilder::RPGNumericEffect & RPGHeuristic::getRPGNE(const int & i)
{
    return (*(d->rpgNumericEffects))[i];
};


bool RPGHeuristic::printRPGAsDot = false;
bool RPGHeuristic::blindSearch = false;
bool RPGHeuristic::makeCTSEffectsInstantaneous = false;    
bool RPGHeuristic::ignoreNumbers = false;
    
void RPGHeuristic::setGuidance(const char * config)
{
    const string asString(config);
    
    if (asString == "blind") {
        blindSearch = true;                
    } else if (asString == "nonumbers") {
        ignoreNumbers = true;
    } else if (asString == "makectsinstantaneous") {
        makeCTSEffectsInstantaneous = true;
    } else {
        cerr << "Possible options for the -g parameter are:\n";
        cerr << "\t-gblind                - use blind search (0 heuristic for goal states, otherwise 1)\n";
        cerr << "\t-gnonumbers            - ignore numeric preconditions and effects\n";
        cerr << "\t-gmakectsinstantaneous - make continuous effects instantaneous (as in the Colin IJCAI paper)\n";
        exit(1);
    }
}
    


};
